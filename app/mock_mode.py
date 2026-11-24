"""
Mock mode: the web page and the Cockpit widget on a PC without an OAK-D or an autopilot.

OAKD_MOCK=1 in the environment (or main.py --mock) makes main.py:
- keep its data (config.json, snapshots, telemetry, logs) under OAKD_MOCK_DIR (default
  <tmp>/oakd-vio-mock-<uid>), and start from a PnP configuration there;
- launch mock_vio.py instead of pnp_vio.py / basalt_configurable.py, through the same VIOSupervisor. The
  fake VIO publishes through the real telemetry, depth stream, feature stream, snapshot and depth probe
  services, so every web server code path runs as with a camera;
- use one MockAutopilot for the parameter checker and the ArduPilot panel: a disarmed HEARTBEAT (ARMED with
  OAKD_MOCK_ARMED=1) and the dev vehicle's parameter values, which a confirmed write changes in memory;
- feed the DVL overlay from dvl_fetch (a fake mavlink2rest answering in-process), and the mock VIO's 3D pose
  attitude from attitude_fetch (the same, in the VIO process; with gaps, OAKD_MOCK_ATTITUDE);
- make urllib refuse every URL (block_network), so no request ever leaves the process.

Nothing here imports depthai.
"""

import datetime
import math
import os
import random
import struct
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
# OAKD_MOCK_ARMED=1: the mock autopilot reports ARMED, so the page's writes are refused
ARMED_ENV = "OAKD_MOCK_ARMED"
# What the mock autopilot starts with: the dev vehicle's values, so VISO_POS_* and EK3_SRC1_POSXY differ from the
# recommended set
PARAM_VALUES = {
    "AHRS_EKF_TYPE": 3.0, "EK3_ENABLE": 1.0,
    "VISO_TYPE": 1.0, "VISO_POS_X": 0.0, "VISO_POS_Y": 0.0, "VISO_POS_Z": 0.0, "VISO_ORIENT": 0.0,
    "VISO_DELAY_MS": 10.0, "EK3_SRC1_VELXY": 6.0, "EK3_SRC1_POSXY": 6.0, "EK3_SRC1_POSZ": 1.0, "EK3_SRC1_VELZ": 0.0,
    "EK3_SRC1_YAW": 1.0,
}
DVL_SYSTEM, DVL_COMPONENT = 1, 197
DVL_RATE_HZ = 5.0
# The mock autopilot's ATTITUDE (attitude_fetch): heading at the VIO start, rate, and (period, length) in s of
# its gaps
ATTITUDE_ENV = "OAKD_MOCK_ATTITUDE"
ATTITUDE_MODES = ("gaps", "on", "off")
ATTITUDE_HEADING_OFFSET = math.radians(30.0)
ATTITUDE_RATE_HZ = 10.0
ATTITUDE_GAP = (120.0, 8.0)
# Gyro bias (rad/s, body z) the mock VIO adds to the 3D pose's rotation deltas, so the gyro-integrated
# attitude drifts visibly like a real one (the VIO output and the trajectory panel do not get it)
GYRO_BIAS = math.radians(0.3)


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


def _survey_block(first, second, vz):
    """Two lanes of a lawnmower survey: lane, two `first` turns around a step, lane back, two `second` turns."""
    turn = math.pi / 2 / TURN_S
    return [(LANE_S, LANE_SPEED, 0.0, 0.0), (TURN_S, TURN_SPEED, 0.0, first * turn), (STEP_S, STEP_SPEED, vz, 0.0),
            (TURN_S, TURN_SPEED, 0.0, first * turn), (LANE_S, LANE_SPEED, 0.0, 0.0),
            (TURN_S, TURN_SPEED, 0.0, second * turn), (STEP_S, STEP_SPEED, vz, 0.0),
            (TURN_S, TURN_SPEED, 0.0, second * turn)]


# The mock vehicle surveys a lawnmower pattern: 24 s lanes at 0.35 m/s (about 8.4 m), 90-degree turns, and a
# 1.5 m step between lanes that also changes the depth by STEP_DEPTH_M. The first block steps right and
# down, the second (mirrored) left and up, so the path closes after SURVEY_PERIOD_S.
LANE_S, TURN_S, STEP_S = 24.0, 6.0, 5.0
LANE_SPEED, TURN_SPEED, STEP_SPEED = 0.35, 0.15, 0.3
STEP_DEPTH_M = 0.4
SURVEY = (_survey_block(1, -1, STEP_DEPTH_M / STEP_S) + _survey_block(-1, 1, -STEP_DEPTH_M / STEP_S))
SURVEY_PERIOD_S = sum(segment[0] for segment in SURVEY)
# Small roll and pitch oscillation (amplitude rad, period s)
ROLL_WOBBLE = (math.radians(2.0), 7.0)
PITCH_WOBBLE = (math.radians(1.5), 11.0)


def motion(t):
    """The mock vehicle's motion at wall time t: body FRD velocity (m/s) and yaw rate (rad/s)."""
    phase = t % SURVEY_PERIOD_S
    for duration, forward, down, yaw_rate in SURVEY:
        if phase < duration:
            return forward, 0.0, down, yaw_rate
        phase -= duration
    return SURVEY[-1][1], 0.0, SURVEY[-1][2], SURVEY[-1][3]


def heading(t):
    """The mock vehicle's yaw (rad) at wall time t: motion's yaw rate integrated, 0 at the start of each survey."""
    phase, yaw = t % SURVEY_PERIOD_S, 0.0
    for duration, _, _, yaw_rate in SURVEY:
        if phase < duration:
            return yaw + yaw_rate * phase
        yaw += yaw_rate * duration
        phase -= duration
    return yaw


def attitude(t):
    """Roll and pitch (rad) of the mock vehicle's small oscillation (as angles, so they stay bounded)."""
    tau = 2.0 * math.pi
    return tuple(amplitude * math.sin(tau * t / period) for amplitude, period in (ROLL_WOBBLE, PITCH_WOBBLE))


def mock_armed(environ=None):
    environ = os.environ if environ is None else environ
    return environ.get(ARMED_ENV, "").strip().lower() in TRUE_VALUES


def _float32(value):
    return struct.unpack("f", struct.pack("f", float(value)))[0]


class MockAutopilot:
    """
    autopilot_params.AutopilotParams for mock mode, in memory: a HEARTBEAT (disarmed unless `armed`), the
    parameters in `values` (float32 like a real autopilot), PARAM_SET changes them. Never touches the network.
    """

    def __init__(self, values=None, armed=False, clock=time.time):
        self.base_url = AUTOPILOT_URL
        self.values = {name: _float32(value) for name, value in (PARAM_VALUES if values is None else values).items()}
        self.armed = armed
        self.clock = clock
        self.booted_at = clock()

    def heartbeat_status(self, now=None):
        now = self.clock() if now is None else now
        state = "ARMED" if self.armed else "disarmed"
        return {"reachable": True, "armed": self.armed, "counter": int(now), "age_s": now % 1.0,
                "detail": f"{state} (mock autopilot)"}

    def live_armed_state(self, wait=2.5, poll=0.1):
        return self.heartbeat_status()

    def is_armed(self):
        return self.armed

    def boot_time(self, now=None):
        return self.booted_at

    def read(self, name, attempts=3):
        value = self.values.get(name)
        return None if value is None else (value, "MAV_PARAM_TYPE_REAL32")

    def write(self, name, value, param_type, attempts=3):
        if name not in self.values:
            return None
        self.values[name] = _float32(value)
        return self.values[name], param_type


def _iso(t):
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat()


def attitude_mode(environ=None):
    environ = os.environ if environ is None else environ
    value = environ.get(ATTITUDE_ENV, "").strip().lower()
    return value if value in ATTITUDE_MODES else ATTITUDE_MODES[0]


def attitude_fetch(started, mode=None, clock=time.time):
    """
    The 3D pose's autopilot_attitude.AttitudeSource fetch in mock mode (in the mock VIO process): mavlink2rest's
    ATTITUDE of the mock vehicle started at `started`, consistent with the motion the mock VIO measures:
    yaw ATTITUDE_HEADING_OFFSET + heading(t) - heading(started), roll and pitch attitude(t - started). mode
    (OAKD_MOCK_ATTITUDE): "gaps" freezes the message for the last ATTITUDE_GAP[1] s of every ATTITUDE_GAP[0] s
    (stale: the viewer falls back to the gyro), "on" never does, "off" has none. Never touches the network.
    """
    mode = attitude_mode() if mode is None else mode

    def fetch():
        if mode == "off":
            raise urllib.error.URLError("mock mavlink2rest: no ATTITUDE (OAKD_MOCK_ATTITUDE=off)")
        since = max(0.0, clock() - started)
        period, gap = ATTITUDE_GAP
        if mode == "gaps" and since % period >= period - gap:
            since -= since % period - (period - gap)
        counter = int(since * ATTITUDE_RATE_HZ)
        sample = counter / ATTITUDE_RATE_HZ
        roll, pitch = attitude(sample)
        yaw = math.remainder(ATTITUDE_HEADING_OFFSET + heading(started + sample) - heading(started), math.tau)
        return {"message": {"type": "ATTITUDE", "time_boot_ms": int(sample * 1000), "roll": roll, "pitch": pitch,
                            "yaw": yaw, "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0},
                "status": {"time": {"counter": counter, "last_update": _iso(started + sample)}}}
    return fetch


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
