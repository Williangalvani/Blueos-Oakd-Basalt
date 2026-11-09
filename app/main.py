#!/usr/bin/env python3
"""
BlueOS OAK-D VIO Extension - Web Configuration Interface
"""

import os
import json
import subprocess
import signal
from html import escape
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import Optional
import uvicorn

app = FastAPI(title="OAK-D VIO Configuration")

# Configuration file path (persistent across restarts)
CONFIG_FILE = Path("/app/data/config.json")

# Default configuration
DEFAULT_CONFIG = {
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
}

# VIO process handle
vio_process: Optional[subprocess.Popen] = None


class VIOConfig(BaseModel):
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


def start_vio():
    """Start the VIO process"""
    global vio_process
    
    config = load_config()
    
    # Kill existing process if running
    stop_vio()
    
    # Build command with config
    cmd = [
        "python", "/app/basalt_configurable.py",
        "--fps", str(config["fps"]),
        "--camera-angle", str(config["camera_angle"]),
        "--vehicle-ip", config["vehicle_ip"],
        "--quality-rate", str(config["quality_rate"]),
        "--width", str(config["resolution_width"]),
        "--height", str(config["resolution_height"]),
        "--grid-size", str(config["optical_flow_grid_size"]),
        "--flow-iterations", str(config["optical_flow_max_iterations"]),
        "--flow-levels", str(config["optical_flow_levels"]),
        "--max-states", str(config["vio_max_states"]),
        "--max-kfs", str(config["vio_max_kfs"]),
        "--vio-iterations", str(config["vio_max_iterations"]),
        "--skip-frames", str(config["optical_flow_skip_frames"]),
    ]
    
    env = os.environ.copy()
    env["PYTHONPATH"] = "/app/depthai-python"
    
    vio_process = subprocess.Popen(
        cmd,
        env=env,
        cwd="/app"
    )
    print(f"Started VIO process with PID {vio_process.pid}")


def stop_vio():
    """Stop the VIO process"""
    global vio_process
    if vio_process and vio_process.poll() is None:
        vio_process.terminate()
        try:
            vio_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            vio_process.kill()
        print("Stopped VIO process")
    vio_process = None


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
        
        <form id="config-form">
            <h2>📷 Camera Settings</h2>
            <div class="grid">
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
            </div>
            
            <div class="buttons">
                <button type="submit" class="btn-primary">💾 Save & Restart</button>
                <button type="button" class="btn-success" onclick="startVIO()">▶️ Start</button>
                <button type="button" class="btn-danger" onclick="stopVIO()">⏹️ Stop</button>
                <button type="button" class="btn-secondary" onclick="resetDefaults()">↩️ Reset Defaults</button>
            </div>
        </form>
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
                    if (data.running) {
                        el.className = 'status running';
                        text.textContent = 'VIO Running (PID: ' + data.pid + ')';
                    } else {
                        el.className = 'status stopped';
                        text.textContent = 'VIO Stopped';
                    }
                });
        }
        
        document.getElementById('config-form').addEventListener('submit', async (e) => {
            e.preventDefault();
            const form = new FormData(e.target);
            const config = Object.fromEntries(form.entries());
            // Convert numbers
            for (const key of Object.keys(config)) {
                if (!isNaN(config[key]) && key !== 'vehicle_ip') {
                    config[key] = parseFloat(config[key]);
                }
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
        
        async function resetDefaults() {
            const res = await fetch('/api/reset', {method: 'POST'});
            if (res.ok) {
                showToast('Reset to defaults. Reloading...', 'success');
                setTimeout(() => location.reload(), 1000);
            }
        }
        
        // Update status every 5 seconds
        updateStatus();
        setInterval(updateStatus, 5000);
    </script>
</body>
</html>
"""
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
    """Get VIO process status"""
    global vio_process
    if vio_process and vio_process.poll() is None:
        return {"running": True, "pid": vio_process.pid}
    return {"running": False, "pid": None}


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
    """Stop VIO on server shutdown"""
    stop_vio()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
