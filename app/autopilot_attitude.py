"""
The autopilot's attitude for the 3D pose viewer: ArduPilot's ATTITUDE message (roll, pitch, yaw in rad, the
vehicle body FRD relative to NED), read from mavlink2rest by the VIO process. Read only: one GET of
<base>/mavlink/vehicles/1/components/1/messages/ATTITUDE (autopilot_params.AutopilotParams.vehicle_message,
a few hundred bytes) every POLL_INTERVAL_S on a daemon thread, with a TIMEOUT_S timeout, and every
RETRY_INTERVAL_S while mavlink2rest or the message is missing. Nothing is sent to the autopilot, and the
VISION_POSITION_DELTA output does not use any of this.

Sample time: mavlink2rest's status.time.last_update (when it received the message; the same clock as this
process on BlueOS). Without it, the time this process first saw the message counter at that value, and the
first read is ignored (it may be an old message). A sample older than STALE_S counts as stale: the autopilot
stopped sending (mavlink2rest keeps the last message forever), or the vehicle is gone.

depthai-free so it can be unit tested.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from autopilot_params import message_time

MESSAGE = "ATTITUDE"
POLL_INTERVAL_S = 0.5
RETRY_INTERVAL_S = 5.0
TIMEOUT_S = 0.4
STALE_S = 3.0
SOURCE_TEXT = "ArduPilot ATTITUDE (mavlink2rest)"


@dataclass(frozen=True)
class AttitudeSample:
    """One ATTITUDE: roll, pitch, yaw (rad, ZYX: body FRD -> NED) and when mavlink2rest received it (unix s)."""
    roll: float
    pitch: float
    yaw: float
    unix_time: float
    counter: Optional[int] = None
    time_boot_ms: Optional[int] = None

    def matrix(self):
        """Body FRD -> NED rotation Rz(yaw) Ry(pitch) Rx(roll)."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])


def parse_attitude(entry, received_at):
    """
    AttitudeSample from a mavlink2rest message entry ({"message": {...}, "status": {"time": ...}}). The sample
    time is status.time.last_update, else received_at. Raises ValueError for anything that is not a usable
    ATTITUDE.
    """
    try:
        message = entry["message"]
        roll, pitch, yaw = (float(message[key]) for key in ("roll", "pitch", "yaw"))
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"not an ATTITUDE message ({type(e).__name__}: {e})")
    if message.get("type", MESSAGE) != MESSAGE:
        raise ValueError(f"expected {MESSAGE}, got {message.get('type')}")
    if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
        raise ValueError("ATTITUDE with a non-finite angle")
    if abs(roll) > math.pi + 1e-3 or abs(pitch) > math.pi / 2 + 1e-3 or abs(yaw) > 2 * math.pi + 1e-3:
        raise ValueError(f"ATTITUDE out of range: roll {roll}, pitch {pitch}, yaw {yaw}")
    status = entry.get("status", {}).get("time", {}) if isinstance(entry.get("status"), dict) else {}
    counter = status.get("counter") if isinstance(status, dict) else None
    stamp = message_time(entry)
    boot = message.get("time_boot_ms")
    return AttitudeSample(roll, pitch, math.remainder(yaw, math.tau), stamp if stamp is not None else received_at,
                          counter if isinstance(counter, int) else None,
                          int(boot) if isinstance(boot, (int, float)) else None)


class AttitudeSource:
    """
    Polls ATTITUDE with fetch() (returns the mavlink2rest entry, raises on errors) on its own thread (start(),
    close()), or step by step with poll_once() in tests. latest() hands the newest sample to the VIO loop.
    """

    def __init__(self, fetch, clock=time.time, poll_interval=POLL_INTERVAL_S, retry_interval=RETRY_INTERVAL_S,
                 stale_s=STALE_S, description=SOURCE_TEXT, log=print):
        self.fetch = fetch
        self.clock = clock
        self.poll_interval = poll_interval
        self.retry_interval = retry_interval
        self.stale_s = stale_s
        self.description = description
        self.log = log
        self.sample = None
        self.reads = self.samples = self.errors = 0
        self.last_error = None
        self._counter = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._reported = None

    def poll_once(self, now=None):
        """One read; True if it gave a new sample."""
        now = self.clock() if now is None else now
        self.reads += 1
        try:
            entry = self.fetch()
            sample = parse_attitude(entry, now)
        except Exception as e:
            self._failed(str(e) or type(e).__name__)
            return False
        with self._lock:
            first, same = self._counter is None, sample.counter is not None and sample.counter == self._counter
            self._counter = sample.counter
            self.last_error = None
            if same:
                return False
            if message_time(entry) is None and (first or sample.counter is None):
                return False  # unknown age: wait for the counter to move
            self.sample = sample
            self.samples += 1
        if self._reported != "ok":
            self._reported = "ok"
            self.log(f"Autopilot attitude: reading {self.description}")
        return True

    def _failed(self, error):
        with self._lock:
            self.errors += 1
            self.last_error = error
        if self._reported != "error":
            self._reported = "error"
            self.log(f"Autopilot attitude: not available ({error}); the 3D pose uses the gyro-integrated attitude")

    def latest(self, now=None):
        """(sample, age in s) of the newest sample if it is fresh (at most stale_s old), else (None, age or None)."""
        now = self.clock() if now is None else now
        with self._lock:
            sample = self.sample
        if sample is None:
            return None, None
        age = max(0.0, now - sample.unix_time)
        return (sample if age <= self.stale_s else None), age

    def state(self, now=None):
        now = self.clock() if now is None else now
        sample, age = self.latest(now)
        return {"message": MESSAGE, "description": self.description, "fresh": sample is not None, "age_s": age,
                "stale_after_s": self.stale_s, "reads": self.reads, "samples": self.samples, "errors": self.errors,
                "last_error": self.last_error}

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="autopilot-attitude", daemon=True)
            self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            self.poll_once()
            self._stop.wait(self.retry_interval if self.last_error else self.poll_interval)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def from_mavlink2rest(vehicle_ip, log=print, **kwargs):
    """A started AttitudeSource reading the autopilot (system 1, component 1) through BlueOS's mavlink2rest."""
    from autopilot_params import AutopilotParams
    client = AutopilotParams(vehicle_ip, timeout=TIMEOUT_S)
    return AttitudeSource(lambda: client.vehicle_message(MESSAGE), log=log,
                          description=f"{SOURCE_TEXT} at {vehicle_ip}", **kwargs).start()
