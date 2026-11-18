#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - Web Configuration Interface
"""

import os
import json
import subprocess
import time
from html import escape
from pathlib import Path
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from pydantic import BaseModel, Field, create_model, field_validator
from typing import List, Literal, Optional
import uvicorn

import depth_probe
import snapshots
import vio_log
from stereo_config import (DEFAULT_LEVER_ARM_M, DEFAULT_REFRACTIVE_INDEX, DEPTH_SCALE_RANGE, INTRINSICS_SOURCES,
                           REFRACTIVE_INDEX_RANGE, STEREO_DEFAULTS, STEREO_SETTINGS, clamp_refractive_index,
                           format_lever_arm, parse_intrinsics_override, parse_lever_arm, stereo_command_args)
from vio_supervisor import VIOSupervisor

WEB_PORT = 8095
SNAPSHOT_DIR = snapshots.SNAPSHOT_DIR
SNAPSHOT_TIMEOUT = 3.0
# Wait for a depth probe: its frames at the configured fps plus this much
PROBE_EXTRA_TIMEOUT = 3.0
# The three focal-check snapshots go out one per second (SnapshotService.min_interval)
FOCAL_CHECK_TIMEOUT = 3 * SNAPSHOT_TIMEOUT
FOCAL_CHECK_STREAMS = ("rect_left", "rect_right", "depth")

app = FastAPI(title="OAK-D VIO Configuration")

# Configuration file path (persistent across restarts)
CONFIG_FILE = Path("/app/data/config.json")

# Default configuration
DEFAULT_CONFIG = {
    "algorithm": "basalt",
    "fps": 3,
    "camera_angle": 0.0,
    "vehicle_ip": "127.0.0.1",
    "quality_rate": 1.0,
    "resolution_width": 320,
    "resolution_height": 200,
    # VIO tuning parameters
    "optical_flow_grid_size": 80,
    "optical_flow_max_iterations": 3,
    "optical_flow_levels": 2,
    "vio_max_states": 2,
    "vio_max_kfs": 4,
    "vio_max_iterations": 3,
    "optical_flow_skip_frames": 1,
    # PnP (option D) parameters
    "num_features": 300,
    "tracking_recovery_frames": 20,
    "max_reference_age": 1.0,
    "snapshots": True,
    "intrinsics_from": "rectified",
    "intrinsics_override": None,
    "depth_scale": 1.0,
    "refractive_index": DEFAULT_REFRACTIVE_INDEX,
    # Diagnostics only: the autopilot applies VISO_POS_X/Y/Z itself
    "lever_arm_enabled": False,
    "lever_arm_m": list(DEFAULT_LEVER_ARM_M),
    **STEREO_DEFAULTS,
    # Passive logger: no MAVLink output, autopilot params untouched (OAKD_LOG_ONLY=1 forces it on)
    "log_only": False,
}


class BaseVIOConfig(BaseModel):
    algorithm: Literal["basalt", "pnp"] = "basalt"
    fps: int = 3
    camera_angle: float = 0.0
    vehicle_ip: str = "127.0.0.1"
    quality_rate: float = 1.0
    resolution_width: int = 320
    resolution_height: int = 200
    optical_flow_grid_size: int = 80
    optical_flow_max_iterations: int = 3
    optical_flow_levels: int = 2
    vio_max_states: int = 2
    vio_max_kfs: int = 4
    vio_max_iterations: int = 3
    optical_flow_skip_frames: int = 1
    num_features: int = 300
    tracking_recovery_frames: int = 20
    max_reference_age: float = 1.0
    snapshots: bool = True
    intrinsics_from: Literal[INTRINSICS_SOURCES] = "rectified"
    intrinsics_override: Optional[str] = None
    depth_scale: float = Field(1.0, ge=DEPTH_SCALE_RANGE[0], le=DEPTH_SCALE_RANGE[1])
    refractive_index: float = Field(DEFAULT_REFRACTIVE_INDEX, ge=REFRACTIVE_INDEX_RANGE[0], le=REFRACTIVE_INDEX_RANGE[1])
    lever_arm_enabled: bool = False
    lever_arm_m: List[float] = Field(default_factory=lambda: list(DEFAULT_LEVER_ARM_M))
    log_only: bool = False

    @field_validator("lever_arm_m", mode="before")
    @classmethod
    def _check_lever_arm(cls, value):
        return list(parse_lever_arm(value))

    @field_validator("intrinsics_override")
    @classmethod
    def _check_intrinsics_override(cls, value):
        if value is None or not value.strip():
            return None
        parse_intrinsics_override(value)
        return value.strip()


def _setting_field(setting):
    kind = Literal[setting.choices] if setting.choices else setting.kind
    return (Optional[kind] if setting.optional else kind), setting.default


# The stereo fields come from the same table as pnp_vio.py's arguments
VIOConfig = create_model("VIOConfig", __base__=BaseVIOConfig,
                         **{setting.key: _setting_field(setting) for setting in STEREO_SETTINGS})


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
            "--recovery-frames", str(int(config["tracking_recovery_frames"])),
            "--max-reference-age", str(float(config["max_reference_age"])),
            "--snapshots" if config["snapshots"] else "--no-snapshots",
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
        hint = f'<div class="hint">{escape(setting.hint)}</div>' if setting.hint else ""
        fields.append(f'                <div class="field">{control}{hint}</div>')
    return "\n".join(fields)


@app.get("/", response_class=HTMLResponse)
async def index():
    """Serve the configuration UI"""
    config = load_config()
    
    html = """
<!DOCTYPE html>
<html>
<head>
    <title>OAK-D VIO Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e; 
            color: #eee; 
            margin: 0; 
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { color: #00d4ff; margin-bottom: 10px; }
        h2 { color: #888; font-size: 14px; margin-top: 30px; margin-bottom: 15px; text-transform: uppercase; }
        .status { 
            padding: 10px 15px; 
            border-radius: 8px; 
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .status.running { background: #1a4d1a; }
        .status.stopped { background: #4d1a1a; }
        .status-dot { 
            width: 12px; 
            height: 12px; 
            border-radius: 50%; 
            animation: pulse 2s infinite;
        }
        .status.running .status-dot { background: #00ff00; }
        .status.stopped .status-dot { background: #ff0000; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .field { 
            background: #16213e; 
            padding: 15px; 
            border-radius: 8px;
        }
        label { 
            display: block; 
            font-size: 12px; 
            color: #888; 
            margin-bottom: 5px;
            text-transform: uppercase;
        }
        input, select { 
            width: 100%; 
            padding: 10px; 
            border: 1px solid #333; 
            border-radius: 4px; 
            background: #0f0f23; 
            color: #fff;
            font-size: 16px;
        }
        input:focus { outline: none; border-color: #00d4ff; }
        .hint { font-size: 11px; color: #666; margin-top: 5px; }
        
        .buttons { 
            display: flex; 
            gap: 10px; 
            margin-top: 20px;
            flex-wrap: wrap;
        }
        button { 
            padding: 12px 24px; 
            border: none; 
            border-radius: 6px; 
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            transition: all 0.2s;
        }
        .btn-primary { background: #00d4ff; color: #000; }
        .btn-primary:hover { background: #00a8cc; }
        .btn-success { background: #00cc66; color: #000; }
        .btn-success:hover { background: #00aa55; }
        .btn-danger { background: #cc3333; color: #fff; }
        .btn-danger:hover { background: #aa2222; }
        .btn-secondary { background: #444; color: #fff; }
        .btn-secondary:hover { background: #555; }
        
        .toast {
            position: fixed;
            bottom: 20px;
            right: 20px;
            padding: 15px 25px;
            border-radius: 8px;
            color: #fff;
            opacity: 0;
            transition: opacity 0.3s;
        }
        .log-only {
            padding: 10px 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            background: #4d3d00;
            color: #ffd54f;
            font-weight: 600;
        }
        .checkbox { display: flex; align-items: center; gap: 10px; text-transform: none; font-size: 14px; color: #eee; }
        .checkbox input { width: auto; }
        .logs { list-style: none; padding: 0; margin: 0; }
        .logs li { padding: 6px 0; border-bottom: 1px solid #222; font-size: 13px; }
        .logs a { color: #00d4ff; }
        .snapshot { max-width: 100%; margin-top: 10px; border-radius: 4px; }
        .probe { font-size: 11px; color: #ccc; background: #0f0f23; padding: 10px; border-radius: 4px; max-height: 300px; overflow: auto; }
        .probe:empty { display: none; }
        .toast.show { opacity: 1; }
        .toast.success { background: #00cc66; }
        .toast.error { background: #cc3333; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎥 OAK-D Visual Odometry</h1>
        
        <div id="status" class="status stopped">
            <div class="status-dot"></div>
            <span id="status-text">Checking...</span>
        </div>
        <div id="camera-status" class="hint"></div>
        <div id="log-only-banner" class="log-only" hidden></div>
        
        <form id="config-form">
            <h2>📷 Camera Settings</h2>
            <div class="grid">
                <div class="field">
                    <label>Algorithm</label>
                    <select name="algorithm" data-value="{algorithm}">
                        <option value="basalt">Basalt VIO</option>
                        <option value="pnp">PnP (on-device stereo + tracking)</option>
                    </select>
                    <div class="hint">PnP ignores the Basalt tuning below</div>
                </div>
                <div class="field">
                    <label>FPS</label>
                    <input type="number" name="fps" value="{fps}" min="1" max="60">
                    <div class="hint">Lower = less CPU usage</div>
                </div>
                <div class="field">
                    <label>Camera Angle (°)</label>
                    <input type="number" name="camera_angle" value="{camera_angle}" step="0.1">
                    <div class="hint">0 = forward, 90 = down</div>
                </div>
                <div class="field">
                    <label>Width</label>
                    <input type="number" name="resolution_width" value="{resolution_width}" min="160" max="1280">
                </div>
                <div class="field">
                    <label>Height</label>
                    <input type="number" name="resolution_height" value="{resolution_height}" min="100" max="800">
                </div>
            </div>
            
            <h2>🔧 VIO Algorithm</h2>
            <div class="grid">
                <div class="field">
                    <label>Optical Flow Grid Size</label>
                    <input type="number" name="optical_flow_grid_size" value="{optical_flow_grid_size}" min="20" max="200">
                    <div class="hint">Higher = fewer features, faster</div>
                </div>
                <div class="field">
                    <label>Flow Iterations</label>
                    <input type="number" name="optical_flow_max_iterations" value="{optical_flow_max_iterations}" min="1" max="10">
                </div>
                <div class="field">
                    <label>Pyramid Levels</label>
                    <input type="number" name="optical_flow_levels" value="{optical_flow_levels}" min="1" max="5">
                </div>
                <div class="field">
                    <label>Skip Frames</label>
                    <input type="number" name="optical_flow_skip_frames" value="{optical_flow_skip_frames}" min="0" max="10">
                    <div class="hint">0 = process all frames</div>
                </div>
                <div class="field">
                    <label>Max States</label>
                    <input type="number" name="vio_max_states" value="{vio_max_states}" min="1" max="10">
                </div>
                <div class="field">
                    <label>Max Keyframes</label>
                    <input type="number" name="vio_max_kfs" value="{vio_max_kfs}" min="2" max="20">
                </div>
                <div class="field">
                    <label>VIO Iterations</label>
                    <input type="number" name="vio_max_iterations" value="{vio_max_iterations}" min="1" max="10">
                </div>
                <div class="field">
                    <label>PnP Features</label>
                    <input type="number" name="num_features" value="{num_features}" min="50" max="1000">
                    <div class="hint">PnP only: features tracked on the camera</div>
                </div>
                <div class="field">
                    <label>PnP Recovery After (frames)</label>
                    <input type="number" name="tracking_recovery_frames" value="{tracking_recovery_frames}" min="0" max="1000">
                    <div class="hint">PnP only: consecutive failed frames before the tracker is re-initialised; 0 disables</div>
                </div>
                <div class="field">
                    <label>PnP Max Reference Age (s)</label>
                    <input type="number" name="max_reference_age" value="{max_reference_age}" step="0.1" min="0" max="10">
                    <div class="hint">PnP only: how long failed frames keep the last good frame as reference, so the next delta covers the gap; 0 = no limit</div>
                </div>
                <div class="field">
                    <label class="checkbox">
                        <input type="checkbox" name="snapshots" {snapshots_checked}>
                        Snapshots
                    </label>
                    <div class="hint">PnP only: on-demand camera snapshots below; no bandwidth until requested</div>
                </div>
                <div class="field">
                    <label>PnP Intrinsics Source</label>
                    <select name="intrinsics_from" data-value="{intrinsics_from}">
                        <option value="rectified">rectified (calibration + alpha)</option>
                        <option value="frame">frame (depth frame matrix)</option>
                        <option value="cam_b">cam_b (raw calibration)</option>
                        <option value="cam_c">cam_c (raw calibration)</option>
                    </select>
                    <div class="hint">PnP only: camera matrix of the rectified-left image. rectified is the matrix the camera rectifies to</div>
                </div>
                <div class="field">
                    <label>PnP Intrinsics Override</label>
                    <input type="text" name="intrinsics_override" value="{intrinsics_override}" placeholder="fx,fy,cx,cy">
                    <div class="hint">PnP only: wins over the source; pixels on the width x height grid. Empty = off</div>
                </div>
                <div class="field">
                    <label>PnP Depth Scale</label>
                    <input type="number" name="depth_scale" value="{depth_scale}" step="0.001" min="0.2" max="5">
                    <div class="hint">PnP only: extra trim on the camera depth on top of the refractive index (e.g. a residual measured with the depth probe); 1 = none</div>
                </div>
                <div class="field">
                    <label>PnP Refractive Index (water)</label>
                    <input type="number" name="refractive_index" value="{refractive_index}" step="0.001" min="1" max="2">
                    <div class="hint">PnP only: water in front of the flat port. The calibration is in air, so fx, fy and the depth are multiplied by it. 1.333 fresh water, ~1.34 sea water, 1.0 = in air (bench tests)</div>
                </div>
                <div class="field">
                    <label class="checkbox">
                        <input type="checkbox" name="lever_arm_enabled" {lever_arm_enabled_checked}>
                        Lever arm diagnostics (log only)
                    </label>
                    <div class="hint">PnP only. Does NOT change what is sent: the autopilot applies the camera offset itself from VISO_POS_X/Y/Z. When on, the data log also gets each delta corrected to the IMU (lever_delta_* fields), for comparison only</div>
                </div>
                <div class="field">
                    <label>Lever Arm x,y,z (m)</label>
                    <input type="text" name="lever_arm_m" value="{lever_arm_m}" placeholder="0.1655,-0.0069,0.18">
                    <div class="hint">Camera relative to the IMU, body FRD (forward, right, down), as VISO_POS_X/Y/Z. Only used by the diagnostics above</div>
                </div>
            </div>

            <h2>🌊 Stereo Depth (PnP)</h2>
            <div class="grid">
{stereo_fields}
            </div>
            
            <h2>📡 MAVLink Output</h2>
            <div class="grid">
                <div class="field">
                    <label>Vehicle IP</label>
                    <input type="text" name="vehicle_ip" value="{vehicle_ip}">
                </div>
                <div class="field">
                    <label>Quality Rate (Hz)</label>
                    <input type="number" name="quality_rate" value="{quality_rate}" step="0.1" min="0.1" max="10">
                </div>
                <div class="field">
                    <label class="checkbox">
                        <input type="checkbox" name="log_only" {log_only_checked}>
                        Log only: do not send anything to the autopilot
                    </label>
                    <div class="hint">No MAVLink output and no autopilot parameter changes; poses only go to the data log</div>
                </div>
            </div>
            
            <div class="buttons">
                <button type="submit" class="btn-primary">💾 Save & Restart</button>
                <button type="button" class="btn-success" onclick="startVIO()">▶️ Start</button>
                <button type="button" class="btn-danger" onclick="stopVIO()">⏹️ Stop</button>
                <button type="button" class="btn-secondary" onclick="resetDefaults()">↩️ Reset Defaults</button>
            </div>
        </form>

        <h2>📸 Snapshots (PnP)</h2>
        <div class="buttons">
            <button type="button" class="btn-secondary" onclick="snapshot('rect_left')">Rectified left</button>
            <button type="button" class="btn-secondary" onclick="snapshot('rect_right')">Rectified right</button>
            <button type="button" class="btn-secondary" onclick="snapshot('depth')">Depth</button>
            <button type="button" class="btn-secondary" onclick="snapshot('rgb')">RGB</button>
        </div>
        <div id="snapshot-status" class="hint"></div>
        <img id="snapshot" class="snapshot" alt="">

        <h2>📏 Depth Probe (PnP)</h2>
        <div class="grid">
            <div class="field"><label>x (px)</label><input type="number" id="probe-x" placeholder="centre"></div>
            <div class="field"><label>y (px)</label><input type="number" id="probe-y" placeholder="centre"></div>
            <div class="field"><label>ROI w (px)</label><input type="number" id="probe-w" placeholder="41" min="1" max="200"></div>
            <div class="field"><label>ROI h (px)</label><input type="number" id="probe-h" placeholder="41" min="1" max="200"></div>
            <div class="field"><label>Frames</label><input type="number" id="probe-frames" placeholder="10" min="1" max="50"></div>
        </div>
        <div class="hint">Pixel coordinates on the rectified-left image (width × height above). Empty uses the default.</div>
        <div class="buttons">
            <button type="button" class="btn-primary" onclick="depthProbe()">Probe depth</button>
            <button type="button" class="btn-secondary" onclick="aimProbe('rect_left')">Aim on rectified left</button>
            <button type="button" class="btn-secondary" onclick="aimProbe('depth')">Aim on depth</button>
            <button type="button" class="btn-secondary" onclick="focalCheck()">Focal check</button>
        </div>
        <div id="probe-summary" class="hint"></div>
        <pre id="probe-result" class="probe"></pre>

        <h2>📝 Data Logs</h2>
        <ul id="logs" class="logs"><li>Loading...</li></ul>
    </div>
    
    <div id="toast" class="toast"></div>
    
    <script>
        function showToast(message, type) {
            const toast = document.getElementById('toast');
            toast.textContent = message;
            toast.className = 'toast show ' + type;
            setTimeout(() => toast.className = 'toast', 3000);
        }
        
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    const el = document.getElementById('status');
                    const text = document.getElementById('status-text');
                    const restarts = data.restart_count
                        ? ' · ' + data.restart_count + ' automatic restart(s), last exit code ' + data.last_exit_code
                        : '';
                    if (data.running) {
                        el.className = 'status running';
                        text.textContent = 'VIO Running (PID: ' + data.pid + ')' + restarts;
                    } else if (data.restart_in !== null) {
                        el.className = 'status stopped';
                        text.textContent = 'VIO exited (code ' + data.last_exit_code + '), restarting in '
                            + Math.ceil(data.restart_in) + ' s';
                    } else {
                        el.className = 'status stopped';
                        text.textContent = 'VIO Stopped';
                    }
                    const banner = document.getElementById('log-only-banner');
                    banner.hidden = !data.log_only;
                    banner.textContent = data.log_only_forced_by_env
                        ? 'LOG-ONLY (forced by OAKD_LOG_ONLY on the container): nothing is sent to the autopilot'
                        : 'LOG-ONLY: nothing is sent to the autopilot';
                    document.getElementById('camera-status').textContent = cameraSummary(data.camera);
                });
        }

        function cameraSummary(camera) {
            if (!camera || !camera.intrinsics) return '';
            const k = values => values.map(v => v.toFixed(1)).join(', ');
            let text = 'PnP camera matrix (' + camera.source + ', in air) fx, fy, cx, cy = ' + k(camera.intrinsics);
            if (camera.refractive_index !== undefined) {
                text += ' · n ' + camera.refractive_index + ' × depth scale ' + camera.depth_scale
                    + ' → effective ' + k(camera.effective_intrinsics) + ', depth × '
                    + camera.total_depth_scale.toFixed(4);
            }
            return text;
        }

        function updateLogs() {
            fetch('/api/logs')
                .then(r => r.json())
                .then(logs => {
                    const list = document.getElementById('logs');
                    list.replaceChildren();
                    if (!logs.length) {
                        const item = document.createElement('li');
                        item.textContent = 'No logs yet';
                        list.appendChild(item);
                    }
                    for (const log of logs) {
                        const item = document.createElement('li');
                        const link = document.createElement('a');
                        link.href = '/api/logs/' + encodeURIComponent(log.name);
                        link.textContent = log.name;
                        link.download = log.name;
                        item.appendChild(link);
                        item.append(' · ' + (log.size / 1024).toFixed(1) + ' KiB · '
                            + new Date(log.mtime * 1000).toLocaleString());
                        list.appendChild(item);
                    }
                });
        }
        
        document.getElementById('config-form').addEventListener('submit', async (e) => {
            e.preventDefault();
            const form = new FormData(e.target);
            const config = Object.fromEntries(form.entries());
            // Convert numbers; an empty field means "auto" (null)
            for (const [key, value] of Object.entries(config)) {
                if (key === 'vehicle_ip') continue;
                if (key === 'intrinsics_override') {
                    config[key] = value.trim() === '' ? null : value.trim();
                    continue;
                }
                if (key === 'lever_arm_m') {
                    config[key] = value.trim();
                    continue;
                }
                if (value === '') {
                    config[key] = null;
                } else if (!isNaN(value)) {
                    config[key] = parseFloat(value);
                }
            }
            // Unchecked checkboxes are missing from FormData
            for (const box of e.target.querySelectorAll('input[type=checkbox]')) {
                config[box.name] = box.checked;
            }
            
            const res = await fetch('/api/config', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(config)
            });
            
            if (res.ok) {
                showToast('Configuration saved! Restarting VIO...', 'success');
                setTimeout(updateStatus, 2000);
            } else {
                showToast('Error saving configuration', 'error');
            }
        });
        
        async function startVIO() {
            const res = await fetch('/api/start', {method: 'POST'});
            if (res.ok) {
                showToast('VIO started', 'success');
                setTimeout(updateStatus, 1000);
            }
        }
        
        async function stopVIO() {
            const res = await fetch('/api/stop', {method: 'POST'});
            if (res.ok) {
                showToast('VIO stopped', 'success');
                setTimeout(updateStatus, 1000);
            }
        }
        
        async function showSnapshot(stream, query) {
            const status = document.getElementById('snapshot-status');
            status.textContent = 'Requesting ' + stream + '...';
            const res = await fetch('/api/snapshot?stream=' + stream + (query ? '&' + query : ''));
            if (!res.ok) {
                const error = await res.json().catch(() => ({}));
                status.textContent = error.detail || ('Error ' + res.status);
                return;
            }
            const img = document.getElementById('snapshot');
            if (img.src) URL.revokeObjectURL(img.src);
            img.src = URL.createObjectURL(await res.blob());
            status.textContent = stream + ' at ' + new Date().toLocaleTimeString();
        }

        function snapshot(stream) {
            return showSnapshot(stream, '');
        }

        function probeParams(names) {
            const params = new URLSearchParams();
            for (const name of names) {
                const value = document.getElementById('probe-' + name).value;
                if (value !== '') params.set(name, value);
            }
            return params;
        }

        async function showProbeResult(url, busy, summarize) {
            const summary = document.getElementById('probe-summary');
            const result = document.getElementById('probe-result');
            summary.textContent = busy;
            const res = await fetch(url);
            const data = await res.json().catch(() => ({}));
            if (!res.ok) {
                summary.textContent = data.detail || ('Error ' + res.status);
                result.textContent = '';
                return;
            }
            summary.textContent = summarize(data);
            result.textContent = JSON.stringify(data, null, 2);
        }

        function depthProbe() {
            return showProbeResult('/api/depth_probe?' + probeParams(['x', 'y', 'w', 'h', 'frames']),
                'Probing depth...', data => data.summary);
        }

        function aimProbe(stream) {
            const params = probeParams(['x', 'y', 'w', 'h']);
            params.set('crosshair', '1');
            return showSnapshot(stream, params.toString());
        }

        function focalCheck() {
            return showProbeResult('/api/focal_check', 'Taking rect_left, rect_right and depth snapshots (keep the camera still)...',
                data => (data.ok ? 'OK: ' : 'CHECK: ') + data.message);
        }

        async function resetDefaults() {
            const res = await fetch('/api/reset', {method: 'POST'});
            if (res.ok) {
                showToast('Reset to defaults. Reloading...', 'success');
                setTimeout(() => location.reload(), 1000);
            }
        }
        
        // Templating only fills value attributes, so select the saved option here
        for (const select of document.querySelectorAll('select[data-value]')) {
            select.value = select.dataset.value;
        }
        
        // Update status every 5 seconds
        updateStatus();
        setInterval(updateStatus, 5000);
        updateLogs();
        setInterval(updateLogs, 15000);
    </script>
</body>
</html>
"""
    html = html.replace("{log_only_checked}", "checked" if config.get("log_only") else "")
    html = html.replace("{snapshots_checked}", "checked" if config.get("snapshots") else "")
    html = html.replace("{lever_arm_enabled_checked}", "checked" if config.get("lever_arm_enabled") else "")
    html = html.replace("{lever_arm_m}", escape(format_lever_arm(saved_lever_arm(config))))
    html = html.replace("{stereo_fields}", stereo_form_fields(config))
    # str.format() would choke on the CSS/JS braces in the template
    for key, value in config.items():
        html = html.replace("{" + key + "}", "" if value is None else escape(str(value)))
    
    return HTMLResponse(content=html)


@app.get("/api/config")
async def get_config():
    """Get current configuration"""
    return JSONResponse(load_config())


@app.post("/api/config")
async def set_config(config: VIOConfig):
    """Save configuration and restart VIO"""
    save_config(config.dict())
    start_vio()
    return {"status": "ok", "message": "Configuration saved and VIO restarted"}


@app.get("/api/status")
async def get_status():
    """Get VIO process status, including automatic restarts, log-only mode and the PnP camera matrix"""
    return {**supervisor.status(), **log_only_status(), "camera": snapshots.read_camera_status(SNAPSHOT_DIR)}


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
                 h: Optional[int] = None):
    """
    PNG of one fresh frame from the running PnP VIO, without stopping it: rect_left, rect_right,
    depth (colorized, invalid black; raw=1 for 16-bit millimetres) or rgb. Waits up to ~3 s.
    crosshair=1 draws the depth probe's ROI (x, y, w, h as for /api/depth_probe) on rect_left, rect_right or depth.
    """
    if raw and stream != "depth":
        raise HTTPException(status_code=400, detail="raw=1 only applies to stream=depth")
    probe_request = None
    if crosshair:
        if raw or stream == "rgb":
            raise HTTPException(status_code=400, detail="crosshair=1 works on rect_left, rect_right and depth, without raw=1")
        image_size = vio_image_size()
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
    if probe_request is not None:
        data = depth_probe.draw_probe_overlay(data, probe_request, image_size)
    headers = {
        "Cache-Control": "no-store",
        "X-Snapshot-Unix-Time": str(meta.get("unix_time")),
        "X-Snapshot-Device-Timestamp": str(meta.get("device_timestamp")),
        "X-Snapshot-Sequence": str(meta.get("sequence")),
    }
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
