#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - Web Configuration Interface
"""

import os
import json
import subprocess
from html import escape
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from pydantic import BaseModel, create_model
from typing import Literal, Optional
import uvicorn

import snapshots
import vio_log
from stereo_config import STEREO_DEFAULTS, STEREO_SETTINGS, stereo_command_args
from vio_supervisor import VIOSupervisor

WEB_PORT = 8095
SNAPSHOT_DIR = snapshots.SNAPSHOT_DIR
SNAPSHOT_TIMEOUT = 3.0

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
    "snapshots": True,
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
    snapshots: bool = True
    log_only: bool = False


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
            "--snapshots" if config["snapshots"] else "--no-snapshots",
            *stereo_command_args(config),
        ]
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
                    <label class="checkbox">
                        <input type="checkbox" name="snapshots" {snapshots_checked}>
                        Snapshots
                    </label>
                    <div class="hint">PnP only: on-demand camera snapshots below; no bandwidth until requested</div>
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
                });
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
        
        async function snapshot(stream) {
            const status = document.getElementById('snapshot-status');
            status.textContent = 'Requesting ' + stream + '...';
            const res = await fetch('/api/snapshot?stream=' + stream);
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
    html = html.replace("{stereo_fields}", stereo_form_fields(config))
    # str.format() would choke on the CSS/JS braces in the template
    for key, value in config.items():
        html = html.replace("{" + key + "}", escape(str(value)))
    
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
    """Get VIO process status, including automatic restarts and log-only mode"""
    return {**supervisor.status(), **log_only_status()}


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


@app.get("/api/snapshot")
def get_snapshot(stream: Literal[snapshots.STREAMS], raw: bool = False):
    """
    PNG of one fresh frame from the running PnP VIO, without stopping it: rect_left, rect_right,
    depth (colorized, invalid black; raw=1 for 16-bit millimetres) or rgb. Waits up to ~3 s.
    """
    if raw and stream != "depth":
        raise HTTPException(status_code=400, detail="raw=1 only applies to stream=depth")
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
    return Response(content=data, media_type="image/png", headers=headers)


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
