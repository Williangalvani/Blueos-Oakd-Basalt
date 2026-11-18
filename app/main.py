#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - Web Configuration Interface
"""

import os
import json
import math
import subprocess
import threading
import time
from html import escape
from pathlib import Path
from fastapi import Body, FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from typing import Literal, Optional
import uvicorn

import autopilot_params
import config_schema
import depth_probe
import dvl_source
import snapshots
import system_stats
import vio_log
import vio_telemetry
from autopilot_params import AutopilotParams
from stereo_config import (DEFAULT_LEVER_ARM_M, DEFAULT_REFRACTIVE_INDEX, STEREO_SETTINGS, clamp_refractive_index,
                           format_lever_arm, parse_lever_arm, stereo_command_args)
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
# ArduPilot panel: per-request timeout and attempts of each parameter read, and how long a write waits
# for a new HEARTBEAT to prove the vehicle is live and disarmed
AUTOPILOT_TIMEOUT = 1.0
AUTOPILOT_ATTEMPTS = 2
HEARTBEAT_WAIT = 2.5

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
    """Load configuration from file or return defaults"""
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE) as f:
                config = json.load(f)
                # Merge with defaults for any missing keys
                return {**DEFAULT_CONFIG, **config}
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


def telemetry_dir() -> Path:
    return Path(TELEMETRY_DIR) if TELEMETRY_DIR is not None else vio_telemetry.DEFAULT_DIR


def build_vio_command(config: dict) -> list:
    """VIO command line for a configuration"""
    common_args = [
        "--fps", str(config["fps"]),
        "--camera-angle", str(config["camera_angle"]),
        "--vehicle-ip", config["vehicle_ip"],
        "--quality-rate", str(config["quality_rate"]),
        "--width", str(config["resolution_width"]),
        "--height", str(config["resolution_height"]),
    ]
    if config["algorithm"] == "pnp":
        cmd = [
            "python", "-u", "/app/pnp_vio.py",
            *common_args,
            "--num-features", str(config["num_features"]),
            "--min-inliers", str(int(config.get("min_inliers", DEFAULT_CONFIG["min_inliers"]))),
            "--recovery-frames", str(int(config["tracking_recovery_frames"])),
            "--max-reference-age", str(float(config["max_reference_age"])),
            "--snapshots" if config["snapshots"] else "--no-snapshots",
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
    else:
        cmd = [
            "python", "-u", "/app/basalt_configurable.py",
            *common_args,
            "--grid-size", str(config["optical_flow_grid_size"]),
            "--flow-iterations", str(config["optical_flow_max_iterations"]),
            "--flow-levels", str(config["optical_flow_levels"]),
            "--max-states", str(config["vio_max_states"]),
            "--max-kfs", str(config["vio_max_kfs"]),
            "--vio-iterations", str(config["vio_max_iterations"]),
            "--skip-frames", str(config["optical_flow_skip_frames"]),
        ]
    if log_only_status(config)["log_only"]:
        cmd.append("--log-only")
    return cmd


def launch_vio() -> subprocess.Popen:
    """Launch the VIO process for the saved configuration"""
    config = load_config()
    cmd = build_vio_command(config)

    env = os.environ.copy()
    env["PYTHONPATH"] = "/app/depthai-python"
    env["PYTHONUNBUFFERED"] = "1"
    
    # stdout/stderr are inherited so VIO logs reach the container output
    process = subprocess.Popen(
        cmd,
        env=env,
        cwd="/app"
    )
    mode = " (LOG-ONLY)" if "--log-only" in cmd else ""
    print(f"Started {config['algorithm']} VIO process with PID {process.pid}{mode}", flush=True)
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
# Armed state from the ArduPilot panel's last read or write (no polling of its own)
vehicle_state = {"armed": None, "detail": "not read yet", "checked_at": None}
vehicle_state_lock = threading.Lock()


def start_vio():
    """(Re)start the VIO process"""
    supervisor.start()


def stop_vio():
    """Stop the VIO process without restarting it"""
    supervisor.stop()


def stereo_form_fields(config: dict) -> str:
    """Web form fields for the stereo settings; an empty number field means auto/preset (null)."""
    fields = []
    for setting in STEREO_SETTINGS:
        value = config.get(setting.key, setting.default)
        name = escape(setting.key)
        if setting.kind is bool:
            control = (f'<label class="checkbox"><input type="checkbox" name="{name}"{" checked" if value else ""}> '
                       f'{escape(setting.label)}</label>')
        else:
            label = f'<label>{escape(setting.label)}</label>'
            if setting.choices:
                options = "".join(
                    f'<option value="{escape(str(choice))}"{" selected" if str(choice) == str(value) else ""}>'
                    f'{escape(str(choice))}</option>' for choice in setting.choices)
                control = f'{label}<select name="{name}">{options}</select>'
            else:
                step = "any" if setting.kind is float else "1"
                shown = "" if value is None else escape(str(value))
                control = f'{label}<input type="number" name="{name}" value="{shown}" step="{step}" placeholder="auto">'
        hint = f'<div class="hint" title="{escape(setting.hint)}">{escape(setting.hint)}</div>' if setting.hint else ""
        fields.append(f'                <div class="field" data-field="{name}">{control}{hint}</div>')
    return "\n".join(fields)


def render_page(config: dict) -> str:
    """ui.html with ui.css and ui.js inlined (one self-contained page, works offline) and the settings form."""
    html = (STATIC_DIR / "ui.html").read_text()
    parts = {
        "<!--UI_CSS-->": f"<style>\n{(STATIC_DIR / 'ui.css').read_text()}\n</style>",
        "<!--UI_JS-->": f"<script>\n{(STATIC_DIR / 'ui.js').read_text()}\n</script>",
        "<!--FIELDS_STEREO-->": stereo_form_fields(config),
    }
    for group, _ in config_schema.GROUPS:
        if group != "stereo":
            parts[f"<!--FIELDS_{group.upper()}-->"] = config_schema.render_group(group, config)
    for marker, content in parts.items():
        html = html.replace(marker, content)
    return html


@app.get("/", response_class=HTMLResponse)
def index():
    """Serve the web UI"""
    return HTMLResponse(content=render_page(load_config()), headers={"Cache-Control": "no-cache"})


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
    """True/False from fresh telemetry, None when unknown (VIO stopped, Basalt, or nothing published yet)."""
    if telemetry is None or telemetry["stale"]:
        return None
    if telemetry.get("phase") != "running":
        return False
    age = telemetry.get("last_frame_age_s")
    return age is not None and age <= CAMERA_FRAME_TIMEOUT_S


@app.get("/api/status")
def get_status():
    """
    VIO process status (automatic restarts, log-only mode, the PnP camera matrix), plus the PnP live
    telemetry (rates, failures by reason, output, chip temperature), host CPU load and the last known
    armed state. telemetry is null until the PnP VIO publishes; stale when it stopped.
    """
    now = time.time()
    process = supervisor.status()
    vio_telemetry.touch_lease(vio_telemetry.STATUS_LEASE, telemetry_dir())
    telemetry = live_telemetry(process, now)
    with vehicle_state_lock:
        vehicle = dict(vehicle_state)
    started = telemetry.get("started_at") if telemetry else None
    return {
        **process,
        **log_only_status(),
        "camera": snapshots.read_camera_status(SNAPSHOT_DIR),
        "algorithm": load_config().get("algorithm"),
        "telemetry": telemetry,
        "camera_connected": camera_connected(telemetry),
        "chip_temperature_c": telemetry.get("chip_temperature_c") if telemetry and not telemetry["stale"] else None,
        "cpu_percent": cpu_sampler.percent(),
        "uptime_s": now - started if started and process.get("running") else None,
        "vehicle": vehicle,
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


@app.get("/api/history")
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


@app.get("/api/snapshot")
def get_snapshot(stream: Literal[snapshots.STREAMS], raw: bool = False, crosshair: bool = False,
                 x: Optional[float] = None, y: Optional[float] = None, w: Optional[int] = None,
                 h: Optional[int] = None, overlay: bool = False):
    """
    PNG of one fresh frame from the running PnP VIO, without stopping it: rect_left, rect_right,
    depth (colorized, invalid black; raw=1 for 16-bit millimetres) or rgb. Waits up to ~3 s.
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
        raise HTTPException(status_code=504, detail=f"{e}; snapshots need the PnP algorithm running")
    headers = {
        "Cache-Control": "no-store",
        "X-Snapshot-Unix-Time": str(meta.get("unix_time")),
        "X-Snapshot-Device-Timestamp": str(meta.get("device_timestamp")),
        "X-Snapshot-Sequence": str(meta.get("sequence")),
    }
    if overlay:
        features = meta.get("overlay")
        if features:
            data = snapshots.draw_feature_overlay(data, features, features.get("image_size") or image_size)
            headers.update({"X-Overlay": "features", "X-Overlay-Inliers": str(features.get("inliers")),
                            "X-Overlay-Outliers": str(features.get("outliers")),
                            "X-Overlay-Unmatched": str(features.get("unmatched")),
                            "X-Overlay-Frame-Sequence": str(features.get("sequence"))})
        else:
            headers["X-Overlay"] = "unavailable"
    if probe_request is not None:
        data = depth_probe.draw_probe_overlay(data, probe_request, image_size)
    return Response(content=data, media_type="image/png", headers=headers)


def require_pnp_running(what):
    config = load_config()
    if config.get("algorithm") != "pnp":
        raise HTTPException(status_code=503, detail=f"{what} needs the PnP algorithm (current: {config.get('algorithm')})")
    if not supervisor.status().get("running"):
        raise HTTPException(status_code=503, detail=f"{what} needs the PnP VIO running; it is stopped")
    return config


@app.get("/api/depth_probe")
def get_depth_probe(x: Optional[float] = None, y: Optional[float] = None, w: Optional[int] = None,
                    h: Optional[int] = None, frames: Optional[int] = None):
    """
    Depth statistics in a w x h ROI centred on pixel (x, y) of the rectified-left grid, over the next
    `frames` depth frames of the running PnP VIO (defaults: image centre, 41 x 41, 10 frames).
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


@app.get("/api/focal_check")
def get_focal_check(tolerance: float = Query(0.15, gt=0.0, lt=1.0)):
    """
    Compare fx of the camera matrix in use with the focal length implied by the device depth
    (f = Z * d / B at row-consistent rect_left/rect_right feature matches). Takes three snapshots
    about 1 s apart, so the camera and scene must be still.
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
            raise HTTPException(status_code=504, detail=f"{e}; the focal check needs the PnP algorithm running")
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
    return AutopilotParams(load_config().get("vehicle_ip", "127.0.0.1"), timeout=AUTOPILOT_TIMEOUT)


def remember_vehicle_state(heartbeat: dict):
    with vehicle_state_lock:
        vehicle_state.update(armed=heartbeat.get("armed"), detail=heartbeat.get("detail"), checked_at=time.time())


@app.get("/api/autopilot/params")
def get_autopilot_params():
    """
    The suggested optical-hold parameters (autopilot_params.SUGGESTED_OPTICAL_HOLD_PARAMS) as the autopilot
    has them, via mavlink2rest: current, suggested, status ok/differs/unreadable, rationale, needs_reboot,
    allowed range; plus the HEARTBEAT armed state. Only reads (PARAM_REQUEST_READ); never writes.
    """
    autopilot = make_autopilot()
    result = autopilot_params.read_suggested_params(autopilot, attempts=AUTOPILOT_ATTEMPTS)
    remember_vehicle_state(result["heartbeat"])
    return JSONResponse({**result, "mavlink2rest": autopilot.base_url, "read_at": time.time()},
                        headers={"Cache-Control": "no-store"})


@app.post("/api/autopilot/params")
def post_autopilot_params(body=Body(...)):
    """
    Write suggested parameters: {"confirm": true, "params": {"NAME": value, ...}}. 400 without confirm, for
    names outside the suggested table or values outside their range; 409 (nothing written) unless a new
    HEARTBEAT arrives and says disarmed. Each written parameter is read back. Returns changed
    [{name, old, new, readback}], failed, unchanged and reboot_required.
    """
    if not isinstance(body, dict) or body.get("confirm") is not True:
        raise HTTPException(status_code=400, detail='writing needs {"confirm": true, "params": {...}}')
    problems = autopilot_params.validate_param_values(body.get("params"))
    if problems:
        raise HTTPException(status_code=400, detail="; ".join(problems))
    autopilot = make_autopilot()
    try:
        result = autopilot_params.write_suggested_params(autopilot, body["params"], heartbeat_wait=HEARTBEAT_WAIT,
                                                         attempts=AUTOPILOT_ATTEMPTS)
    except autopilot_params.WriteRefused as e:
        remember_vehicle_state(e.heartbeat)
        return JSONResponse({"detail": str(e), "heartbeat": e.heartbeat}, status_code=409)
    remember_vehicle_state(result["heartbeat"])
    return JSONResponse({**result, "written_at": time.time()}, headers={"Cache-Control": "no-store"})


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
    """Start VIO on server startup"""
    start_vio()


@app.on_event("shutdown")
async def shutdown_event():
    """Stop VIO and the supervisor thread on server shutdown"""
    supervisor.shutdown()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT)
