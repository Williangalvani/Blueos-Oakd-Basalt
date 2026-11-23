"""
Mock mode: the web page and the Cockpit widget on a PC without an OAK-D or an autopilot.

OAKD_MOCK=1 in the environment (or main.py --mock) makes main.py:
- keep its data (config.json, snapshots, telemetry, logs) under OAKD_MOCK_DIR (default
  <tmp>/oakd-vio-mock-<uid>), and start from a PnP configuration there;
- launch mock_vio.py instead of pnp_vio.py / basalt_configurable.py, through the same VIOSupervisor. The
  fake VIO publishes through the real telemetry, depth stream, feature stream, snapshot and depth probe
  services, so every web server code path runs as with a camera;
- use MockAutopilot for the ArduPilot panel: a disarmed HEARTBEAT and fixed parameter values, read only;
  POST /api/autopilot/params is refused before anything else;
- feed the DVL overlay from dvl_fetch (a fake mavlink2rest answering in-process);
- make urllib refuse every URL (block_network), so no request ever leaves the process.

Nothing here imports depthai.
"""

import datetime
import math
import os
import random
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

ENV = "OAKD_MOCK"
DIR_ENV = "OAKD_MOCK_DIR"
TRUE_VALUES = {"1", "true", "yes", "on"}
APP_DIR = Path(__file__).resolve().parent
MOCK_VIO = APP_DIR / "mock_vio.py"
# Saved on first start in mock mode: the PnP algorithm at a resolution the widget shows well
INITIAL_CONFIG = {"algorithm": "pnp", "fps": 10, "resolution_width": 640, "resolution_height": 400}
AUTOPILOT_URL = "mock://autopilot"
WRITE_REFUSED = "mock mode: the autopilot is simulated and read-only; nothing was written"
# What the mock autopilot reports; VISO_POS_* and EK3_SRC1_POSXY differ from the suggestions on purpose
PARAM_VALUES = {
    "VISO_TYPE": 1.0, "VISO_POS_X": 0.0, "VISO_POS_Y": 0.0, "VISO_POS_Z": 0.0, "VISO_ORIENT": 0.0,
    "VISO_DELAY_MS": 10.0, "EK3_SRC1_VELXY": 6.0, "EK3_SRC1_POSXY": 6.0, "EK3_SRC1_POSZ": 1.0, "EK3_SRC1_YAW": 1.0,
}
DVL_SYSTEM, DVL_COMPONENT = 1, 197
DVL_RATE_HZ = 5.0


def enabled(environ=None):
    environ = os.environ if environ is None else environ
    return environ.get(ENV, "").strip().lower() in TRUE_VALUES


def data_dir(environ=None):
    environ = os.environ if environ is None else environ
    if environ.get(DIR_ENV):
        return Path(environ[DIR_ENV]).expanduser()
    return Path(tempfile.gettempdir()) / f"oakd-vio-mock-{os.getuid()}"


def vio_command(command, snapshot_dir, log_dir, python=sys.executable):
    """main.build_vio_command's command line, running mock_vio.py instead (it ignores the options it lacks)."""
    algorithm = "pnp" if command[2].endswith("pnp_vio.py") else "basalt"
    return [python, "-u", str(MOCK_VIO), "--algorithm", algorithm, *command[3:],
            "--snapshot-dir", str(snapshot_dir), "--log-dir", str(log_dir)]


def block_network():
    """Every urllib request fails at once: in mock mode nothing may reach a vehicle."""
    def urlopen(request, *args, **kwargs):
        url = request if isinstance(request, str) else request.full_url
        raise urllib.error.URLError(f"mock mode: not contacting {url}")

    urllib.request.urlopen = urlopen


def motion(t):
    """The mock vehicle's motion at wall time t: body FRD velocity (m/s) and yaw rate (rad/s)."""
    tau = 2.0 * math.pi
    return (0.12 * math.sin(tau * t / 37.0) + 0.04 * math.sin(tau * t / 11.0),
            0.08 * math.sin(tau * t / 23.0 + 1.0),
            0.02 * math.sin(tau * t / 17.0),
            0.05 * math.sin(tau * t / 29.0))


class MockAutopilot:
    """autopilot_params.AutopilotParams for mock mode: disarmed, fixed parameter values, never writes."""

    def __init__(self, values=None, clock=time.time):
        self.base_url = AUTOPILOT_URL
        self.values = dict(PARAM_VALUES if values is None else values)
        self.clock = clock

    def heartbeat_status(self, now=None):
        now = self.clock() if now is None else now
        return {"reachable": True, "armed": False, "counter": int(now), "age_s": now % 1.0,
                "detail": "disarmed (mock autopilot)"}

    def live_armed_state(self, wait=2.5, poll=0.1):
        return self.heartbeat_status()

    def is_armed(self):
        return False

    def read(self, name, attempts=3):
        value = self.values.get(name)
        return None if value is None else (value, "MAV_PARAM_TYPE_REAL32")

    def write(self, name, value, param_type, attempts=3):
        raise PermissionError(WRITE_REFUSED)

    def configure_visual_odometry(self):
        pass


def _iso(t):
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat()


def dvl_fetch(url, timeout=None, clock=time.time):
    """
    dvl_source.DvlPoller's fetch in mock mode: a mavlink2rest with a DVL sending VISION_POSITION_DELTA at
    DVL_RATE_HZ, its velocity the mock motion plus a few percent and noise. Never touches the network.
    """
    now = clock()
    counter = int(now * DVL_RATE_HZ)
    sample_t = counter / DVL_RATE_HZ
    status = {"time": {"counter": counter, "last_update": _iso(sample_t)}}
    if url.rstrip("/").endswith("/mavlink/vehicles"):
        heartbeat = {"message": {"type": "HEARTBEAT", "base_mode": {"bits": 0}}, "status": status}
        delta = {"message": {"type": "VISION_POSITION_DELTA"}, "status": status}
        return {"vehicles": {str(DVL_SYSTEM): {"components": {
            "1": {"messages": {"HEARTBEAT": heartbeat}},
            str(DVL_COMPONENT): {"messages": {"VISION_POSITION_DELTA": delta}}}}}}
    if url.endswith(f"/vehicles/{DVL_SYSTEM}/components/{DVL_COMPONENT}/messages/VISION_POSITION_DELTA"):
        dt = 1.0 / DVL_RATE_HZ
        rng = random.Random(counter)
        velocity = [1.03 * v + rng.gauss(0.0, 0.01) for v in motion(sample_t)[:3]]
        return {"status": status, "message": {
            "type": "VISION_POSITION_DELTA", "time_usec": int(sample_t * 1e6), "time_delta_usec": int(dt * 1e6),
            "angle_delta": [0.0, 0.0, 0.0], "position_delta": [v * dt for v in velocity], "confidence": 90.0}}
    raise urllib.error.URLError(f"mock mavlink2rest has no {url}")
