#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - Web Configuration Interface
"""

import argparse
import asyncio
import os
import json
import math
import subprocess
import threading
import time
from pathlib import Path
from fastapi import Body, FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from typing import Literal, Optional
import uvicorn

import autopilot_params
import cockpit_widget
import config_schema
import depth_probe
import depth_stream
import dvl_source
import feature_stream
import param_checker as param_checker_module
import pose_stream
import mock_mode
import snapshots
import system_stats
import viewers
import vio_log
import vio_telemetry
from autopilot_params import AutopilotParams
from stereo_config import (DEFAULT_LEVER_ARM_M, DEFAULT_REFRACTIVE_INDEX, clamp_refractive_index, format_lever_arm,
                           parse_lever_arm, stereo_command_args)
from vio_supervisor import VIOSupervisor

WEB_PORT = 8095
SNAPSHOT_DIR = snapshots.SNAPSHOT_DIR
# None: vio_telemetry.DEFAULT_DIR (RAM-backed /dev/shm when available)
TELEMETRY_DIR = None
STATIC_DIR = Path(__file__).resolve().parent / "static"
SNAPSHOT_TIMEOUT = 3.0
# Wait for a depth probe: its frames at the configured fps plus this much
PROBE_EXTRA_TIMEOUT = 3.0
# The three focal-check snapshots go out one per second (SnapshotService.min_interval)
FOCAL_CHECK_TIMEOUT = 3 * SNAPSHOT_TIMEOUT
FOCAL_CHECK_STREAMS = ("rect_left", "rect_right", "depth")
# /api/history: longest window and most points per response
MAX_HISTORY_SECONDS = vio_telemetry.HISTORY_SECONDS
MAX_HISTORY_POINTS = 600
MAX_TAIL_ROWS = 300
# Telemetry older than this (or from another process) counts as stale: VIO stopped or not publishing
TELEMETRY_STALE_S = 3.0
# No processed frame for this long while running: the camera counts as disconnected
CAMERA_FRAME_TIMEOUT_S = 3.0
# /api/stream/*: how often each client checks for a new frame or event (the leases: viewer_registry)
DEPTH_STREAM_POLL_S = 0.05
# /api/stream/features and /api/stream/pose: the widget's status line this often
WIDGET_STATUS_INTERVAL_S = 1.0
# ArduPilot panel: per-request timeout and attempts of each parameter read, and how long a write waits
# for a new HEARTBEAT to prove the vehicle is live and disarmed
AUTOPILOT_TIMEOUT = 1.0
AUTOPILOT_ATTEMPTS = 2
HEARTBEAT_WAIT = 2.5
# Mock mode (mock_mode.py): no camera, no autopilot; set by OAKD_MOCK=1 or --mock (enable_mock_mode)
MOCK = False

app = FastAPI(title="OAK-D VIO Configuration")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# Configuration file path (persistent across restarts)
CONFIG_FILE = Path("/app/data/config.json")

# Default configuration: every field of config_schema.CONFIG_FIELDS
# (log_only: passive logger, no MAVLink output, autopilot params untouched; OAKD_LOG_ONLY=1 forces it on)
DEFAULT_CONFIG = config_schema.default_config()

# The stereo fields come from the same table as pnp_vio.py's arguments
VIOConfig = config_schema.ConfigModel


def load_config() -> dict:
    """Load configuration from file or return defaults (an older file: see config_schema.migrate_saved)"""
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE) as f:
                return config_schema.migrate_saved(json.load(f))
        except Exception as e:
            print(f"Error loading config: {e}")
    return DEFAULT_CONFIG.copy()


def save_config(config: dict):
    """Save configuration to file"""
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f, indent=2)


def log_only_status(config=None) -> dict:
    """Effective log-only state: the saved setting, or forced by OAKD_LOG_ONLY in the environment"""
    config = load_config() if config is None else config
    forced = vio_log.env_forces_log_only()
    return {"log_only": bool(config.get("log_only")) or forced, "log_only_forced_by_env": forced}


def saved_lever_arm(config: dict) -> tuple:
    """The saved lever arm, or the default if a hand-edited config.json holds something unusable."""
    try:
        return parse_lever_arm(config.get("lever_arm_m", DEFAULT_LEVER_ARM_M))
    except (TypeError, ValueError):
        return DEFAULT_LEVER_ARM_M


def saved_number(config: dict, key: str):
    """A saved number forced into its schema range (the default if it isn't a number), like clamp_refractive_index."""
    field = config_schema.FIELDS_BY_KEY[key]
    kind = int if field.kind == "int" else float
    try:
        value = kind(config.get(key, field.default))
    except (TypeError, ValueError, OverflowError):
        return field.default
    if not math.isfinite(value):
        return field.default
    return min(max(value, kind(field.minimum)), kind(field.maximum))


def telemetry_dir() -> Path:
    return Path(TELEMETRY_DIR) if TELEMETRY_DIR is not None else vio_telemetry.DEFAULT_DIR


def _lease(touch, lease_file, with_status=False):
    """(touch, remove) for viewers.ViewerRegistry: remove() spares a lease renewed since (e.g. by /api/pose)."""
    touched = {}

    def renew():
        directory = telemetry_dir()
        touch(directory)
        if with_status:
            vio_telemetry.touch_lease(vio_telemetry.STATUS_LEASE, directory)
        touched["at"] = time.time()

    def remove():
        path = telemetry_dir() / lease_file
        at = touched.pop("at", None)
        try:
            if at is not None and path.stat().st_mtime <= at:
                path.unlink()
        except FileNotFoundError:
            pass

    return renew, remove


def make_viewer_registry(**kwargs):
    """
    The live viewers of the streams: video (/api/stream/depth: makes the camera send images), features and pose
    (/api/stream/features and /api/stream/pose, plus the status lease for their status lines).
    """
    kwargs.setdefault("log", print)
    return viewers.ViewerRegistry({
        "video": _lease(depth_stream.touch_lease, depth_stream.LEASE_FILE),
        "features": _lease(feature_stream.touch_lease, feature_stream.LEASE_FILE, with_status=True),
        "pose": _lease(pose_stream.touch_lease, pose_stream.LEASE_FILE, with_status=True),
    }, **kwargs)


viewer_registry = make_viewer_registry()


def saved_choice(config: dict, key: str) -> str:
    """A saved choice, or its default if a hand-edited config.json has something else."""
    field = config_schema.FIELDS_BY_KEY[key]
    value = config.get(key)
    return value if value in field.choices else field.default


def saved_depth_stream_encoding(config: dict) -> str:
    value = config.get("depth_stream_encoding")
    return value if value in depth_stream.VARIANTS else DEFAULT_CONFIG["depth_stream_encoding"]


def build_vio_command(config: dict) -> list:
    """PnP VIO command line for a configuration"""
    cmd = [
        "python", "-u", "/app/pnp_vio.py",
        "--fps", str(config["fps"]),
        "--camera-angle", str(config["camera_angle"]),
        "--vehicle-ip", config["vehicle_ip"],
        "--quality-rate", str(config["quality_rate"]),
        "--width", str(config["resolution_width"]),
        "--height", str(config["resolution_height"]),
        "--num-features", str(config["num_features"]),
        "--max-features", str(int(config.get("max_features", DEFAULT_CONFIG["max_features"]))),
        "--min-inliers", str(int(config.get("min_inliers", DEFAULT_CONFIG["min_inliers"]))),
        "--recovery-frames", str(int(config["tracking_recovery_frames"])),
        "--max-reference-age", str(float(config["max_reference_age"])),
        "--snapshots" if config["snapshots"] else "--no-snapshots",
        "--depth-stream" if config.get("depth_stream", True) else "--no-depth-stream",
        "--depth-stream-fps", str(saved_number(config, "depth_stream_fps")),
        "--depth-stream-quality", str(saved_number(config, "depth_stream_quality")),
        "--depth-stream-encoding", saved_depth_stream_encoding(config),
        "--depth-stream-debug" if config.get("depth_stream_debug") else "--no-depth-stream-debug",
        "--depth-transport", saved_choice(config, "depth_transport"),
        "--telemetry-dir", str(telemetry_dir()),
        "--intrinsics-from", config.get("intrinsics_from") or "rectified",
        "--depth-scale", str(float(config.get("depth_scale", 1.0))),
        "--refractive-index", str(clamp_refractive_index(config.get("refractive_index", DEFAULT_REFRACTIVE_INDEX))),
        "--lever-arm" if config.get("lever_arm_enabled") else "--no-lever-arm",
        # One token: argparse would take a value starting with "-" (negative x) for an option
        f"--lever-arm-m={format_lever_arm(saved_lever_arm(config))}",
        *stereo_command_args(config),
    ]
    if config.get("intrinsics_override"):
        cmd += ["--intrinsics-override", str(config["intrinsics_override"])]
    if log_only_status(config)["log_only"]:
        cmd.append("--log-only")
    return cmd


def camera_process_env() -> dict:
    """Environment of the processes that use the camera (the VIO, the stereo probe): depthai on the path."""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONPATH"] = "/app/depthai-python"
    return env


def launch_vio() -> subprocess.Popen:
    """Launch the VIO process for the saved configuration (mock_vio.py in mock mode)"""
    config = load_config()
    cmd = build_vio_command(config)

    if MOCK:
        env = {**os.environ, "PYTHONUNBUFFERED": "1"}
        cmd = mock_mode.vio_command(cmd, SNAPSHOT_DIR, vio_log.LOG_DIR)
        cwd = str(mock_mode.APP_DIR)
    else:
        env = camera_process_env()
        cwd = "/app"

    # stdout/stderr are inherited so VIO logs reach the container output
    process = subprocess.Popen(
        cmd,
        env=env,
        cwd=cwd
    )
    mode = (" (LOG-ONLY)" if "--log-only" in cmd else "") + (" (MOCK)" if MOCK else "")
    print(f"Started PnP VIO process with PID {process.pid}{mode}", flush=True)
    return process


def record_supervisor_event(event, **fields):
    """Append a supervisor event to supervisor.jsonl next to the VIO data logs"""
    vio_log.append_event(vio_log.LOG_DIR / "supervisor.jsonl", event, **fields)


# Restarts the VIO process if it exits without being stopped (e.g. a depthai segfault)
supervisor = VIOSupervisor(launch_vio, log=lambda msg: print(msg, flush=True),
                           on_event=record_supervisor_event)
cpu_sampler = system_stats.CpuSampler()
history_reader = vio_telemetry.HistoryReader()
dvl_poller = dvl_source.DvlPoller()
# Armed state from the last parameter check, read or write (param_checker)
vehicle_state = {"armed": None, "detail": "not read yet", "checked_at": None}
vehicle_state_lock = threading.Lock()
# Mock mode's in-memory autopilot (enable_mock_mode)
mock_autopilot = None


def enable_mock_mode(directory=None):
    """
    Mock mode (see mock_mode.py): data under `directory` (default mock_mode.data_dir()), mock_vio.py instead of
    the camera, the mock autopilot and DVL, and urllib refusing every request. Call before serving.
    """
    global MOCK, CONFIG_FILE, SNAPSHOT_DIR, TELEMETRY_DIR, dvl_poller, mock_autopilot
    root = Path(directory) if directory is not None else mock_mode.data_dir()
    for name in ("snapshots", "telemetry", "logs"):
        (root / name).mkdir(parents=True, exist_ok=True)
    MOCK = True
    CONFIG_FILE = root / "config.json"
    SNAPSHOT_DIR = root / "snapshots"
    TELEMETRY_DIR = root / "telemetry"
    vio_log.LOG_DIR = root / "logs"
    if not CONFIG_FILE.exists():
        save_config({**DEFAULT_CONFIG, **mock_mode.INITIAL_CONFIG})
    dvl_poller = dvl_source.DvlPoller(fetch=mock_mode.dvl_fetch)
    mock_mode.block_network()
    mock_autopilot = mock_mode.MockAutopilot(armed=mock_mode.mock_armed())
    remember_vehicle_state(make_autopilot().heartbeat_status())
    print(f"MOCK MODE: no camera, no autopilot; data in {root}", flush=True)


def start_vio():
    """(Re)start the VIO process"""
    supervisor.start()


def stop_vio():
    """Stop the VIO process without restarting it"""
    supervisor.stop()


def render_page(config: dict) -> str:
    """ui.html with ui.css and ui.js inlined (one self-contained page, works offline) and the settings form."""
    html = (STATIC_DIR / "ui.html").read_text()
    parts = {
        "<!--UI_CSS-->": f"<style>\n{(STATIC_DIR / 'ui.css').read_text()}\n</style>",
        "<!--UI_JS-->": "\n".join(f"<script>\n{(STATIC_DIR / name).read_text()}\n</script>"
                                  for name in ("features_overlay.js", "link_panel.js", "param_check.js", "ui.js")),
    }
    for group, _ in config_schema.GROUPS:
        parts[f"<!--FIELDS_{group.upper()}-->"] = config_schema.render_group(group, config)
    for marker, content in parts.items():
        html = html.replace(marker, content)
    return html


# BlueOS's helper GETs every path listed in /openapi.json, every few seconds, reading each body to the end
# (core/services/helper/main.py, detect_service). A GET route in the schema must therefore answer quickly
# with a bounded body and start nothing; streams, waits for the camera and anything that starts work are
# include_in_schema=False, and whatever runs something on the camera or the autopilot is a POST.
# tests/test_openapi_scan.py enforces this.
@app.get("/", response_class=HTMLResponse)
def index():
    """Serve the web UI"""
    return HTMLResponse(content=render_page(load_config()), headers={"Cache-Control": "no-cache"})


def render_widget() -> str:
    """widget.html with widget.css, features_overlay.js, link_panel.js, param_check.js and widget.js inlined, like
    render_page."""
    html = (STATIC_DIR / "widget.html").read_text()
    html = html.replace("<!--WIDGET_CSS-->", f"<style>\n{(STATIC_DIR / 'widget.css').read_text()}\n</style>")
    scripts = "\n".join(f"<script>\n{(STATIC_DIR / name).read_text()}\n</script>"
                        for name in ("features_overlay.js", "link_panel.js", "param_check.js", "widget.js"))
    return html.replace("<!--WIDGET_JS-->", scripts)


@app.get(cockpit_widget.WIDGET_PATH, response_class=HTMLResponse)
def widget_depth():
    """
    The Cockpit widget page: live depth with the tracked features drawn on top and a status line. Always
    200 (Cockpit only mounts the iframe after a successful GET); the page shows why it has no data.
    """
    return HTMLResponse(content=render_widget(), headers={"Cache-Control": "no-cache"})


@app.get(cockpit_widget.POSE_WIDGET_PATH, response_class=HTMLResponse)
def widget_pose():
    """The 3D pose Cockpit widget page (static/pose3d.js on the vendored three.js). Always 200, like /widget/depth."""
    return HTMLResponse(content=(STATIC_DIR / "pose_widget.html").read_text(), headers={"Cache-Control": "no-cache"})


@app.get("/register_service")
def register_service():
    """BlueOS service metadata: name, relative-path support and the Cockpit extras (cockpit_widget.py)"""
    return JSONResponse(cockpit_widget.service_metadata(), headers={"Access-Control-Allow-Origin": "*"})


@app.get(cockpit_widget.EXTRAS_PATH)
def get_cockpit_extras():
    """Cockpit extras: the external widget Cockpit offers in its widget list"""
    return JSONResponse(cockpit_widget.cockpit_extras(),
                        headers={"Access-Control-Allow-Origin": "*", "Cache-Control": "no-cache"})


@app.get("/api/config")
async def get_config():
    """Get current configuration"""
    return JSONResponse(load_config())


@app.get("/api/config/schema")
async def get_config_schema():
    """Every configuration field: name, type, min, max, default, help text, group, choices"""
    return config_schema.schema()


@app.post("/api/config")
async def set_config(body=Body(...)):
    """Validate and save the configuration, then restart VIO. 422 lists every invalid field."""
    config, errors = config_schema.validate_config(body)
    if errors:
        return JSONResponse({"detail": errors, "message": "; ".join(error["msg"] for error in errors)},
                            status_code=422)
    save_config(config)
    start_vio()
    return {"status": "ok", "message": "Configuration saved and VIO restarted"}


@app.post("/api/log_only")
async def set_log_only(body=Body(...)):
    """{"log_only": true|false}: persist the setting and restart VIO, like a config change"""
    if not isinstance(body, dict) or not isinstance(body.get("log_only"), bool):
        raise HTTPException(status_code=400, detail='expected {"log_only": true} or {"log_only": false}')
    save_config({**load_config(), "log_only": body["log_only"]})
    start_vio()
    return {"status": "ok", **log_only_status()}


def live_telemetry(process_status: dict, now: float) -> Optional[dict]:
    """The VIO process's status.json with its age; stale if old, from another process, or VIO is stopped."""
    telemetry = vio_telemetry.read_status(telemetry_dir())
    if telemetry is None:
        return None
    age = now - float(telemetry.get("unix_time") or 0.0)
    same_process = process_status.get("pid") is None or telemetry.get("pid") == process_status.get("pid")
    telemetry["age_s"] = age
    telemetry["stale"] = bool(age > TELEMETRY_STALE_S or not process_status.get("running") or not same_process)
    last = telemetry.get("last_frame") or {}
    telemetry["last_frame_age_s"] = now - last["unix_time"] if last.get("unix_time") else None
    return telemetry


def camera_connected(telemetry: Optional[dict]):
    """True/False from fresh telemetry, None when unknown (VIO stopped or nothing published yet)."""
    if telemetry is None or telemetry["stale"]:
        return None
    if telemetry.get("phase") != "running":
        return False
    age = telemetry.get("last_frame_age_s")
    return age is not None and age <= CAMERA_FRAME_TIMEOUT_S


def pose_status(telemetry: Optional[dict]) -> dict:
    """
    The 3D pose viewer's attitude as the VIO process uses it (pose_stream.PoseStreamService.status: frame,
    attitude_source "autopilot" or "gyro", attitude_age_s of the autopilot's ATTITUDE, switches, the autopilot
    reader's counters); empty while it is not running.
    """
    live = telemetry.get("pose") if telemetry and not telemetry["stale"] else None
    return dict(live or {})


def depth_stream_status(config: dict, process_status: dict, now: float) -> dict:
    """
    The depth stream setting and what the VIO process last published about it (stale if from another run).
    errors counts the camera Script's failed frames; last_error is the latest one (text, unix_time, fatal).
    encoder_stalled: frames go to the encoder but no JPEG comes back (also in last_error). variant is the
    encoding running (encoded_on "camera" or "pi"; fallback says why auto moved to the Pi); device_log the
    camera's recent log lines (depth_stream_debug raises its level while streaming).
    """
    meta = depth_stream.read_meta(telemetry_dir()) or {}
    stale = bool(not meta or not process_status.get("running")
                 or (process_status.get("pid") is not None and meta.get("pid") != process_status.get("pid")))
    return {"errors": 0, "last_error": None, "encoder_stalled": False, "variant": None, "encoded_on": None,
            "fallback": None, "device_log": None, **meta,
            "configured_encoding": saved_depth_stream_encoding(config),
            "debug_configured": bool(config.get("depth_stream_debug")),
            "enabled": bool(config.get("depth_stream", True)),
            "streaming": bool(meta.get("streaming")) and not stale, "stale": stale,
            "age_s": now - float(meta["unix_time"]) if meta.get("unix_time") else None}


HOST_JPEG_FALLBACK = "fallback: raw depth over the link, encoded on the Pi"


def _link_mbps(telemetry: Optional[dict], name: str):
    link = (telemetry or {}).get("link") or {}
    for stream in link.get("streams") or ():
        if stream.get("name") == name:
            return stream.get("mbps")
    return None


def image_streams_status(config: dict, depth: dict, telemetry: Optional[dict], process_status: dict,
                         viewers: dict) -> dict:
    """
    Every image stream that can cross the camera link: codec, quality, encoded_on ("camera" or "pi"), what
    crosses (over_link "jpeg" or "raw"), whether it is being viewed and its link Mbps (telemetry.link's
    per-stream estimate, 10 s mean). Raw frames encoded on the Pi are fallbacks, listed in fallbacks.
    """
    fresh = telemetry if telemetry and not telemetry.get("stale") else None
    streams = []
    if depth.get("enabled"):
        on_pi = depth.get("encoded_on") == "pi"
        reason = ((depth.get("fallback") or {}).get("reason") or depth.get("variant_reason")
                  or ("Depth Stream Encoding is host_jpeg" if depth.get("variant") == "host_jpeg" else None))
        link_name = ("depth_on_demand" if depth.get("depth_frames") == "on_demand" else None) if on_pi \
            else "depth_stream"
        streams.append({
            "name": "live_stream", "label": "Live stream", "variant": depth.get("variant"),
            "codec": depth.get("codec") or "mjpeg", "quality": depth.get("quality"),
            "encoded_on": depth.get("encoded_on"), "over_link": None if depth.get("stale") else (
                "raw" if on_pi else "jpeg"),
            "viewed": bool(depth.get("streaming")) or bool((viewers or {}).get("video")),
            "mbps": _link_mbps(fresh, link_name) if link_name else None,
            "jpeg_kbps": depth.get("kbps"),
            "note": ("the PnP depth frame crosses anyway (full_frame): no extra link traffic"
                     if on_pi and depth.get("depth_frames") == "every_frame" else None),
            "fallback": f"{HOST_JPEG_FALLBACK}: {reason}" if on_pi and not depth.get("stale") else None})
    shots = snapshots.read_encoding_status(SNAPSHOT_DIR) if process_status.get("running") else None
    if config.get("snapshots", True):
        live = (shots or {}).get("streams") or {}
        for stream in snapshots.STREAMS:
            entry = live.get(stream)
            if entry is not None and not entry.get("available"):
                continue
            streams.append({
                "name": f"snapshot_{stream}", "label": f"Snapshot {stream}", **(entry or {}),
                "configured": "camera_jpeg" if stream in snapshots.CAMERA_JPEG_STREAMS else "raw",
                "viewed": None, "mbps": None, "per_request": True,
                "fallback": ((entry or {}).get("fallback") or {}).get("text")})
    return {"streams": streams, "fallbacks": [s["fallback"] for s in streams if s.get("fallback")],
            "snapshot_mbps": _link_mbps(fresh, "snapshot")}


def depth_transport_status(config: dict, telemetry: Optional[dict]) -> dict:
    """
    How PnP gets its depth: configured (depth_transport), plus what the running
    VIO reports (live: depth_points.DepthPointsReceiver.status in points, frames and feature gaps in full_frame,
    and full_depth: the on-demand full depth frames, depth_points.FullDepthGate.status) and the camera's load
    (vio_telemetry.read_camera_load, every 5 s). live and camera_load are null while not running.
    """
    fresh = telemetry if telemetry and not telemetry.get("stale") else {}
    return {"configured": saved_choice(config, "depth_transport"),
            "live": fresh.get("depth_points"), "camera_load": fresh.get("camera_load")}


@app.get("/api/status")
def get_status():
    """
    VIO process status (automatic restarts, log-only mode, the PnP camera matrix), plus the PnP live
    telemetry (rates, failures by reason, output, chip temperature), host CPU load, the last known
    armed state and the last autopilot parameter check (autopilot_params: param_checker.ParamChecker.summary,
    the cached result; this GET never contacts the autopilot). telemetry is null until the PnP VIO publishes;
    stale when it stopped.
    """
    if MOCK:
        remember_vehicle_state(make_autopilot().heartbeat_status())
    now = time.time()
    process = supervisor.status()
    vio_telemetry.touch_lease(vio_telemetry.STATUS_LEASE, telemetry_dir())
    viewer_registry.tick()
    telemetry = live_telemetry(process, now)
    with vehicle_state_lock:
        vehicle = dict(vehicle_state)
    started = telemetry.get("started_at") if telemetry else None
    config = load_config()
    depth = depth_stream_status(config, process, now)
    viewers = viewer_registry.status()
    return {
        "mock": MOCK,
        **process,
        **log_only_status(config),
        "camera": snapshots.read_camera_status(SNAPSHOT_DIR),
        "depth_stream": depth,
        "depth_transport": depth_transport_status(config, telemetry),
        "image_streams": image_streams_status(config, depth, telemetry, process, viewers),
        "viewers": viewers,
        "pose": pose_status(telemetry),
        "telemetry": telemetry,
        "camera_connected": camera_connected(telemetry),
        "chip_temperature_c": telemetry.get("chip_temperature_c") if telemetry and not telemetry["stale"] else None,
        "cpu_percent": cpu_sampler.percent(),
        "uptime_s": now - started if started and process.get("running") else None,
        "vehicle": vehicle,
        "autopilot_params": param_checker.summary(now),
        "server_time": now,
    }


def _column(rows, name, digits=4):
    values = rows[:, vio_telemetry.COLUMN[name]].tolist()
    return [None if not math.isfinite(v) else round(v, digits) for v in values]


def history_columns(rows, now):
    return {
        "t": [round(t - now, 3) for t in rows[:, vio_telemetry.COLUMN["t"]].tolist()],
        **{name: _column(rows, name) for name in ("vx", "vy", "vz", "x", "y", "yaw")},
        "inliers": _column(rows, "inliers", 0),
        "ok": _column(rows, "ok", 0),
    }


@app.get("/api/history", include_in_schema=False)
def get_history(request: Request, seconds: float = Query(60.0, gt=0, le=MAX_HISTORY_SECONDS),
                max_points: int = Query(MAX_HISTORY_POINTS, ge=10, le=MAX_HISTORY_POINTS),
                tail: float = Query(0.0, ge=0, le=10.0)):
    """
    PnP VIO history for the plots, oldest first, t in seconds relative to server_time: velocities (body FRD,
    m/s; null for rejected frames), integrated trajectory x, y (m) and yaw (rad) (integrated, drifts),
    inliers. At most max_points (every k-th frame, newest kept); tail adds the last `tail` seconds at full
    rate (for the velocity trail). dvl: the optional DVL velocity overlay (see dvl_source.py).
    """
    now = time.time()
    directory = telemetry_dir()
    vio_telemetry.touch_lease(vio_telemetry.HISTORY_LEASE, directory)
    config = load_config()
    dvl_poller.configure(f"http://{config.get('vehicle_ip', '127.0.0.1')}/mavlink2rest",
                         config.get("dvl_source", "auto"))
    dvl_poller.touch()
    rows, mtime = history_reader.read(directory)
    etag = f'W/"{mtime}-{seconds:g}-{max_points}-{tail:g}-{dvl_poller.version}"'
    if mtime is not None and request.headers.get("if-none-match") == etag:
        return Response(status_code=304, headers={"ETag": etag})
    picked, stride = vio_telemetry.downsample(rows, seconds, now, max_points)
    tail_rows = vio_telemetry.downsample(rows, tail, now, MAX_TAIL_ROWS)[0] if tail > 0 else picked[:0]
    process = supervisor.status()
    t = vio_telemetry.COLUMN["t"]
    body = {
        "server_time": now,
        "seconds": seconds,
        "points": int(len(picked)),
        "stride": stride,
        "max_points": max_points,
        "available_seconds": float(now - rows[0, t]) if rows is not None and len(rows) else 0.0,
        "published_at": mtime,
        "vio_running": bool(process.get("running")),
        "min_inliers": int(config.get("min_inliers", DEFAULT_CONFIG["min_inliers"])),
        "columns": history_columns(picked, now),
        "tail": history_columns(tail_rows, now),
        "dvl": dvl_poller.snapshot(since=now - seconds, now=now),
    }
    body["dvl"]["t"] = [round(value - now, 3) for value in body["dvl"]["t"]]
    return JSONResponse(body, headers={"ETag": etag, "Cache-Control": "no-cache"})


@app.get("/api/logs")
async def get_logs():
    """List the data logs (name, size in bytes, mtime as unix time), newest first"""
    return vio_log.list_logs(vio_log.LOG_DIR)


@app.get("/api/logs/{name}")
async def download_log(name: str):
    """Download one data log; only plain .jsonl names directly inside the log directory"""
    path = vio_log.resolve_log_file(name, vio_log.LOG_DIR)
    if path is None:
        raise HTTPException(status_code=404, detail="No such log")
    return FileResponse(path, media_type="application/x-ndjson", filename=path.name)


def vio_image_size():
    """(width, height) of the rectified-left grid: what the running PnP VIO reported, else the saved config."""
    camera = snapshots.read_camera_status(SNAPSHOT_DIR)
    if camera and camera.get("image_size"):
        return tuple(int(v) for v in camera["image_size"])
    config = load_config()
    return int(config["resolution_width"]), int(config["resolution_height"])


@app.get("/api/snapshot", include_in_schema=False)
def get_snapshot(stream: Literal[snapshots.STREAMS], raw: bool = False, crosshair: bool = False,
                 x: Optional[float] = None, y: Optional[float] = None, w: Optional[int] = None,
                 h: Optional[int] = None, overlay: bool = False):
    """
    One fresh frame from the running PnP VIO, without stopping it: rect_left, rect_right or rgb (the
    camera's JPEG; PNG if sent raw: X-Snapshot-Fallback says why), depth (PNG, colorized, invalid black;
    raw=1 for 16-bit millimetres). Waits up to ~3 s. The overlays below return a PNG.
    crosshair=1 draws the depth probe's ROI (x, y, w, h as for /api/depth_probe) on rect_left, rect_right or depth.
    overlay=1 (rect_left) draws the tracked features of the last processed frame: inliers green, outliers
    red, unmatched grey, with a line from each feature's previous position; X-Overlay-* headers count them.
    """
    if raw and stream != "depth":
        raise HTTPException(status_code=400, detail="raw=1 only applies to stream=depth")
    if overlay and stream != "rect_left":
        raise HTTPException(status_code=400, detail="overlay=1 only applies to stream=rect_left")
    probe_request = None
    image_size = vio_image_size()
    if crosshair:
        if raw or stream == "rgb":
            raise HTTPException(status_code=400, detail="crosshair=1 works on rect_left, rect_right and depth, without raw=1")
        try:
            probe_request, _ = depth_probe.make_request(x, y, w, h, 1, image_size)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
    requested_at = snapshots.request_snapshot(stream, SNAPSHOT_DIR)
    try:
        data, meta = snapshots.wait_for_snapshot(stream, requested_at, raw=raw, timeout=SNAPSHOT_TIMEOUT,
                                                 snapshot_dir=SNAPSHOT_DIR)
    except snapshots.SnapshotUnavailable as e:
        raise HTTPException(status_code=404, detail=str(e))
    except snapshots.SnapshotTimeout as e:
        raise HTTPException(status_code=504, detail=f"{e}; snapshots need the PnP VIO running")
    headers = {
        "Cache-Control": "no-store",
        "X-Snapshot-Unix-Time": str(meta.get("unix_time")),
        "X-Snapshot-Device-Timestamp": str(meta.get("device_timestamp")),
        "X-Snapshot-Sequence": str(meta.get("sequence")),
        "X-Snapshot-Encoded-On": str(meta.get("encoded_on")),
    }
    if (meta.get("fallback") or {}).get("text"):
        headers["X-Snapshot-Fallback"] = meta["fallback"]["text"]
    media_type = "image/jpeg" if meta.get("format") == "jpeg" and not raw else "image/png"
    if overlay:
        features = meta.get("overlay")
        if features:
            data = snapshots.draw_feature_overlay(data, features, features.get("image_size") or image_size)
            media_type = "image/png"
            headers.update({"X-Overlay": "features", "X-Overlay-Inliers": str(features.get("inliers")),
                            "X-Overlay-Outliers": str(features.get("outliers")),
                            "X-Overlay-Unmatched": str(features.get("unmatched")),
                            "X-Overlay-Frame-Sequence": str(features.get("sequence"))})
        else:
            headers["X-Overlay"] = "unavailable"
    if probe_request is not None:
        data = depth_probe.draw_probe_overlay(data, probe_request, image_size)
        media_type = "image/png"
    return Response(content=data, media_type=media_type, headers=headers)


def require_pnp_running(what):
    config = load_config()
    if not supervisor.status().get("running"):
        raise HTTPException(status_code=503, detail=f"{what} needs the PnP VIO running; it is stopped")
    return config


async def depth_stream_parts(directory: Path, limit: int, client: str = "page"):
    """Multipart parts of each new depth_stream.jpg, as a video viewer (whose lease keeps the camera streaming)."""
    reader = depth_stream.FrameReader(directory)
    sent = 0
    token = viewer_registry.open("video", client)
    try:
        while True:
            viewer_registry.tick()
            frame = reader.read_new()
            if frame is not None:
                yield depth_stream.multipart_part(frame)
                sent += 1
                if limit and sent >= limit:
                    return
            await asyncio.sleep(DEPTH_STREAM_POLL_S)
    finally:
        viewer_registry.close(token)


@app.get("/api/stream/depth", include_in_schema=False)
async def stream_depth(frames: int = Query(0, ge=0, le=100000), client: str = "page"):
    """
    Live depth of the running PnP VIO as multipart/x-mixed-replace MJPEG (for an <img>): the JPEGs the
    camera encodes, forwarded as they are. Gray, linear in depth over stereo_depth_min_mm..max_mm, near
    bright, invalid black (see depth_stream.py; /api/status depth_stream has the scale and the centre
    depth). The camera only streams while a client is connected (and viewers.LINGER_S after the last one
    left); /api/status viewers counts them. client: "page" or "widget", for that count. frames > 0 stops
    after that many.
    """
    config = require_pnp_running("the depth stream")
    if not config.get("depth_stream", True):
        raise HTTPException(status_code=503, detail="the depth stream is off in the settings (Live depth stream)")
    directory = telemetry_dir()
    return StreamingResponse(depth_stream_parts(directory, frames, client),
                             media_type=f"multipart/x-mixed-replace; boundary={depth_stream.BOUNDARY}",
                             headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


def widget_status_now() -> dict:
    now = time.time()
    process = supervisor.status()
    config = load_config()
    telemetry = live_telemetry(process, now)
    depth = depth_stream_status(config, process, now)
    viewers = viewer_registry.status()
    return cockpit_widget.widget_status(cockpit_widget.widget_problem(config, process, telemetry), telemetry,
                                        depth, log_only_status(config)["log_only"], now, mock=MOCK,
                                        viewers=viewers,
                                        image_streams=image_streams_status(config, depth, telemetry, process,
                                                                           viewers),
                                        autopilot_params=param_checker.summary(now))


async def feature_events(directory: Path, limit: int, client: str = "page"):
    """Server-sent events: each new features.json as it is, and the widget status once a second."""
    reader = feature_stream.LatestFile(directory / feature_stream.FEATURES_FILE)
    sent, status_at = 0, float("-inf")
    token = viewer_registry.open("features", client)
    try:
        yield b"retry: 3000\n\n"
        while True:
            now = time.monotonic()
            viewer_registry.tick()
            events = []
            if now - status_at >= WIDGET_STATUS_INTERVAL_S:
                status_at = now
                events.append(feature_stream.sse_event("status", json.dumps(widget_status_now())))
            data = reader.read_new()
            if data is not None:
                events.append(feature_stream.sse_event("features", data))
            for event in events:
                yield event
                sent += 1
                if limit and sent >= limit:
                    return
            await asyncio.sleep(DEPTH_STREAM_POLL_S)
    finally:
        viewer_registry.close(token)


@app.get("/api/stream/features", include_in_schema=False)
async def stream_features(events: int = Query(0, ge=0, le=100000), client: str = "page"):
    """
    text/event-stream for the Cockpit widget. "features": the PnP tracker's features of a recent frame,
    about 5 per second, as the VIO process wrote them (feature_stream.py: points [u, v, state] on the
    depth stream's pixel grid). "status": the widget's status line (cockpit_widget.widget_status) once a
    second, with "problem" set while there is nothing to show. Always 200, so EventSource keeps
    reconnecting. The VIO process only publishes features while a client is connected (a "features" viewer,
    counted in /api/status viewers; client "page" or "widget"); they are not an image, so they never make the
    camera send video. events > 0 stops after that many events.
    """
    directory = telemetry_dir()
    return StreamingResponse(feature_events(directory, events, client), media_type="text/event-stream",
                             headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


async def pose_events(directory: Path, limit: int, client: str = "page"):
    """
    Server-sent events: "trail" when the VIO's trail file is new for this connection (once, and again after a
    VIO restart), each new pose.json as "pose", and the widget status once a second.
    """
    poses = feature_stream.LatestFile(directory / pose_stream.POSE_FILE)
    trails = feature_stream.LatestFile(directory / pose_stream.TRAIL_FILE)
    sent, status_at, trail_epoch = 0, float("-inf"), None
    token = viewer_registry.open("pose", client)
    try:
        yield b"retry: 3000\n\n"
        while True:
            now = time.monotonic()
            viewer_registry.tick()
            events = []
            if now - status_at >= WIDGET_STATUS_INTERVAL_S:
                status_at = now
                events.append(feature_stream.sse_event("status", json.dumps(widget_status_now())))
            trail = trails.read_new()
            if trail is not None:
                try:
                    epoch = json.loads(trail).get("epoch")
                except ValueError:
                    epoch = None
                if epoch is not None and epoch != trail_epoch:
                    trail_epoch = epoch
                    events.append(feature_stream.sse_event("trail", trail))
            pose = poses.read_new()
            if pose is not None:
                events.append(feature_stream.sse_event("pose", pose))
            for event in events:
                yield event
                sent += 1
                if limit and sent >= limit:
                    return
            await asyncio.sleep(DEPTH_STREAM_POLL_S)
    finally:
        viewer_registry.close(token)


@app.get("/api/stream/pose", include_in_schema=False)
async def stream_pose(events: int = Query(0, ge=0, le=100000), client: str = "page"):
    """
    text/event-stream for the 3D pose viewer (main page panel and /widget/pose). "pose": the VIO's integrated
    pose, at most pose_stream.RATE_HZ (10) per second: x, y, z (m), roll, pitch, yaw (radians, ZYX),
    roll_deg, pitch_deg, yaw_deg (the same in degrees) and q [w, x, y, z], in `frame`: "NED" (from the VIO
    start position, x north, y east, z down; once the autopilot's ATTITUDE has been read) or "start-FRD"
    (the body frame at VIO start: x forward, y right, z down). attitude_source is "autopilot" (ArduPilot
    ATTITUDE, attitude_age_s old) or "gyro" (gyro-integrated, drifting); see pose_stream. "trail": the recent
    path, [t, x, y, z] points, sent again when its epoch changes (VIO restart, start-FRD -> NED). "status":
    the widget status line once a second. Always 200; the VIO process only publishes while a client is
    connected (a "pose" viewer in /api/status viewers; client "page" or "widget"). events > 0 stops after that
    many.
    """
    directory = telemetry_dir()
    return StreamingResponse(pose_events(directory, events, client), media_type="text/event-stream",
                             headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@app.get("/api/pose", include_in_schema=False)
def get_pose():
    """The latest pose (as in /api/stream/pose); 404 until the VIO process has published it."""
    require_pnp_running("the pose")
    directory = telemetry_dir()
    pose_stream.touch_lease(directory)
    data = feature_stream.LatestFile(directory / pose_stream.POSE_FILE).read_new()
    if data is None:
        raise HTTPException(status_code=404, detail="no pose published yet: the VIO process publishes it while "
                                                    "asked, from its next frame on; retry")
    return Response(content=data, media_type="application/json", headers={"Cache-Control": "no-store"})


@app.get("/api/features", include_in_schema=False)
def get_features():
    """The latest tracked features (as in /api/stream/features); 404 until the VIO process has published them."""
    require_pnp_running("the tracked features")
    directory = telemetry_dir()
    feature_stream.touch_lease(directory)
    data = feature_stream.LatestFile(directory / feature_stream.FEATURES_FILE).read_new()
    if data is None:
        raise HTTPException(status_code=404, detail="no tracked features published yet: the VIO process publishes "
                                                    "them while asked, from its next frame on; retry")
    return Response(content=data, media_type="application/json", headers={"Cache-Control": "no-store"})


@app.post("/api/depth_probe")
def post_depth_probe(x: Optional[float] = None, y: Optional[float] = None, w: Optional[int] = None,
                     h: Optional[int] = None, frames: Optional[int] = None):
    """
    Depth statistics in a w x h ROI centred on pixel (x, y) of the rectified-left grid, over the next
    `frames` depth frames of the running PnP VIO (defaults: image centre, 41 x 41, 10 frames). POST with
    the parameters in the query: it runs on the VIO's frames.
    """
    config = require_pnp_running("the depth probe")
    try:
        request, clamped = depth_probe.make_request(x, y, w, h, frames, vio_image_size())
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    probe_dir = SNAPSHOT_DIR / "depth_probe"
    request_id = depth_probe.request_probe(request, probe_dir)
    timeout = request.frames / max(float(config.get("fps") or 1), 0.5) + PROBE_EXTRA_TIMEOUT
    try:
        result = depth_probe.wait_for_probe(request_id, timeout, probe_dir)
    except depth_probe.ProbeError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except depth_probe.ProbeTimeout as e:
        raise HTTPException(status_code=504, detail=str(e))
    result["clamped"] = clamped + result.get("clamped", [])
    return JSONResponse(result, headers={"Cache-Control": "no-store"})


@app.post("/api/focal_check")
def post_focal_check(tolerance: float = Query(0.15, gt=0.0, lt=1.0)):
    """
    Compare fx of the camera matrix in use with the focal length implied by the device depth
    (f = Z * d / B at row-consistent rect_left/rect_right feature matches). Takes three snapshots
    about 1 s apart, so the camera and scene must be still. POST (tolerance in the query): it requests
    snapshots from the camera.
    """
    camera = snapshots.read_camera_status(SNAPSHOT_DIR)
    if not camera or not camera.get("intrinsics"):
        raise HTTPException(status_code=503, detail="no camera matrix published: the PnP VIO is not running "
                                                    "or has not received a frame yet")
    if not camera.get("baseline_m"):
        raise HTTPException(status_code=503, detail="the calibration has no stereo baseline")
    import cv2
    import numpy as np
    import rectified_intrinsics  # needs OpenCV; main.py must start without it

    deadline = time.monotonic() + FOCAL_CHECK_TIMEOUT
    requested = {stream: snapshots.request_snapshot(stream, SNAPSHOT_DIR) for stream in FOCAL_CHECK_STREAMS}
    images, metas = {}, {}
    for stream in FOCAL_CHECK_STREAMS:
        try:
            data, metas[stream] = snapshots.wait_for_snapshot(
                stream, requested[stream], raw=stream == "depth", snapshot_dir=SNAPSHOT_DIR,
                timeout=max(deadline - time.monotonic(), 0.0))
        except snapshots.SnapshotUnavailable as e:
            raise HTTPException(status_code=404, detail=str(e))
        except snapshots.SnapshotTimeout as e:
            raise HTTPException(status_code=504, detail=f"{e}; the focal check needs the PnP VIO running")
        images[stream] = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)

    fx_K = float(camera["intrinsics"][0])
    measurement = rectified_intrinsics.measure_rectified_focal(
        images["rect_left"], images["rect_right"], images["depth"], float(camera["baseline_m"]),
        camera.get("depth_range_mm") or (300, 4000))
    result = rectified_intrinsics.focal_check(fx_K, measurement, tolerance,
                                              fx_raw=float(camera["raw"][0]) if camera.get("raw") else None)
    timestamps = {stream: metas[stream].get("device_timestamp") for stream in FOCAL_CHECK_STREAMS}
    known = [t for t in timestamps.values() if isinstance(t, (int, float))]
    result.update(
        candidates=measurement["candidates"], row_consistent=measurement["row_consistent"],
        baseline_m=camera["baseline_m"], intrinsics=camera["intrinsics"], intrinsics_source=camera.get("source"),
        device_timestamps=timestamps, timestamp_spread_s=(max(known) - min(known)) if known else None,
        note="rect_left, rect_right and depth are separate snapshots about 1 s apart, not one stereo pair: keep the "
             "camera and the scene still. This checks fx against the focal length the device used for depth; it "
             "does not check fy, cx or cy. It uses the in-air camera matrix: the device depth is computed with the "
             "in-air focal length, so the refractive index cancels out and the check holds in water too.")
    return JSONResponse(result, headers={"Cache-Control": "no-store"})


def make_autopilot() -> AutopilotParams:
    global mock_autopilot
    if MOCK:
        if mock_autopilot is None:
            mock_autopilot = mock_mode.MockAutopilot(armed=mock_mode.mock_armed())
        return mock_autopilot
    return AutopilotParams(load_config().get("vehicle_ip", "127.0.0.1"), timeout=AUTOPILOT_TIMEOUT)


def remember_vehicle_state(heartbeat: dict):
    with vehicle_state_lock:
        vehicle_state.update(armed=heartbeat.get("armed"), detail=heartbeat.get("detail"), checked_at=time.time())


def make_param_checker() -> param_checker_module.ParamChecker:
    return param_checker_module.ParamChecker(lambda: make_autopilot(), attempts=lambda: AUTOPILOT_ATTEMPTS,
                                             on_heartbeat=remember_vehicle_state,
                                             log=lambda message: print(message, flush=True))


# The recommended-parameter check (main page banner): every 30 s from startup, on the panel's read, after a write
param_checker = make_param_checker()


@app.post("/api/autopilot/params/read")
def read_autopilot_params():
    """
    The recommended parameters (autopilot_params.SUGGESTED_OPTICAL_HOLD_PARAMS) as the autopilot has them, via
    mavlink2rest: current, suggested, status ok/differs/unreadable, rationale, needs_reboot, allowed range; plus
    the HEARTBEAT armed state. Only reads (PARAM_REQUEST_READ); never writes. Also refreshes the parameter check
    in /api/status (it waits for a check already running). POST because it sends MAVLink requests.
    """
    result = param_checker.read()
    summary = param_checker.summary()
    if result is None:
        raise HTTPException(status_code=502, detail=summary["error"])
    return JSONResponse({**result, "mavlink2rest": summary["mavlink2rest"], "read_at": summary["checked_at"]},
                        headers={"Cache-Control": "no-store"})


@app.post("/api/autopilot/params")
def post_autopilot_params(body=Body(...)):
    """
    Write recommended parameters: {"confirm": true, "params": {"NAME": value, ...}}. 400 without confirm, for
    names outside the recommended table or values outside their range; 409 (nothing written) unless a new
    HEARTBEAT arrives and says disarmed. Each written parameter is read back, then the parameter check runs
    again. Returns changed [{name, old, new, readback}], failed, unchanged and reboot_required (never rebooted
    from here). Mock mode writes to the in-memory mock autopilot.
    """
    if not isinstance(body, dict) or body.get("confirm") is not True:
        raise HTTPException(status_code=400, detail='writing needs {"confirm": true, "params": {...}}')
    problems = autopilot_params.validate_param_values(body.get("params"))
    if problems:
        raise HTTPException(status_code=400, detail="; ".join(problems))
    try:
        result = param_checker.write(body["params"], heartbeat_wait=HEARTBEAT_WAIT)
    except autopilot_params.WriteRefused as e:
        return JSONResponse({"detail": str(e), "heartbeat": e.heartbeat}, status_code=409)
    return JSONResponse({**result, "mock": MOCK, "written_at": time.time()}, headers={"Cache-Control": "no-store"})


@app.post("/api/start")
async def api_start():
    """Start VIO"""
    start_vio()
    return {"status": "ok"}


@app.post("/api/stop")
async def api_stop():
    """Stop VIO"""
    stop_vio()
    return {"status": "ok"}


@app.post("/api/reset")
async def api_reset():
    """Reset to default configuration"""
    save_config(DEFAULT_CONFIG.copy())
    start_vio()
    return {"status": "ok"}


@app.on_event("startup")
async def startup_event():
    """Start VIO and the background parameter check on server startup"""
    start_vio()
    param_checker.start()


@app.on_event("shutdown")
async def shutdown_event():
    """Stop VIO, the supervisor thread and the parameter check on server shutdown"""
    supervisor.shutdown()
    param_checker.stop()


if mock_mode.enabled():
    enable_mock_mode()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="OAK-D VIO web server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=WEB_PORT)
    parser.add_argument("--mock", action="store_true",
                        help="mock mode: no camera, no autopilot (same as OAKD_MOCK=1; data in OAKD_MOCK_DIR)")
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    if args.mock and not MOCK:
        enable_mock_mode()
    uvicorn.run(app, host=args.host, port=args.port)
