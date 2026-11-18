"""
Live status and plot history of the PnP VIO for the web page, published through files like snapshots.py.

VIO process side (Telemetry): the loop calls record_frame() once per frame. It only appends one row to a
preallocated numpy ring buffer, updates a bounded rolling window of failure reasons and a few counters,
under an uncontended lock (a few microseconds; see test_vio_telemetry's benchmark). A daemon thread
publishes at a low rate, and only while main.py has asked recently (a "lease" file it touches on each
/api/status or /api/history request), so nothing is serialized or written while nobody watches:

- status.json (a few KB) every STATUS_INTERVAL_S while the status lease is fresh,
- history.npy (the ring buffer, oldest row first) every HISTORY_INTERVAL_S while the history lease is fresh.

Memory bound: HISTORY_SECONDS of frames at min(fps, MAX_HISTORY_FPS), +10 %, len(HISTORY_COLUMNS) float64
each: at most 19800 rows x 12 x 8 B = 1.9 MB (10 fps: 6600 rows, 0.63 MB). The failure window holds
FAILURE_WINDOW_S of (time, reason) tuples, at most 300 s x 30 fps.

The camera chip temperature is sampled on the publisher thread at most every TEMPERATURE_INTERVAL_S
(an RPC to the device), never on the VIO loop.

main.py side: touch_lease(), read_status(), HistoryReader, downsample(). depthai-free so it can be unit tested.
"""

import io
import json
import math
import os
import threading
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np

import snapshots

# RAM-backed and shared by the processes of the container; main.py passes it to pnp_vio.py
DEFAULT_DIR = Path("/dev/shm/oakd-vio-telemetry") if Path("/dev/shm").is_dir() else snapshots.SNAPSHOT_DIR / "telemetry"
STATUS_FILE = "status.json"
HISTORY_FILE = "history.npy"
STATUS_LEASE = "status.lease"
HISTORY_LEASE = "history.lease"

HISTORY_COLUMNS = ("t", "ok", "inliers", "vx", "vy", "vz", "x", "y", "yaw", "proc_ms", "features", "valid_depth")
COLUMN = {name: index for index, name in enumerate(HISTORY_COLUMNS)}
HISTORY_SECONDS = 600.0
MAX_HISTORY_FPS = 30.0
FAILURE_WINDOW_S = 300.0
RECENT_WINDOW_S = 10.0
FPS_WINDOW_S = 5.0
SEND_RATE_WINDOW_S = 5.0
STATUS_INTERVAL_S = 0.5
HISTORY_INTERVAL_S = 1.0
LEASE_S = 10.0
TEMPERATURE_INTERVAL_S = 5.0


def history_capacity(fps):
    return int(math.ceil(round(HISTORY_SECONDS * min(max(float(fps), 1.0), MAX_HISTORY_FPS) * 1.1, 6)))


def normalize_reason(reason):
    # "pnp_error: <OpenCV message>" would make one key per message
    return str(reason).split(":", 1)[0] if reason else "unknown"


class RingBuffer:
    """Fixed-size float64 rows; append() overwrites the oldest row once full."""

    def __init__(self, capacity, columns):
        self.data = np.full((capacity, columns), np.nan)
        self.capacity = capacity
        self.next = 0
        self.count = 0

    def append(self, row):
        self.data[self.next] = row
        self.next = (self.next + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def ordered(self, last=None):
        """Copy of the rows, oldest first (only the newest `last` if given)."""
        n = self.count if last is None else min(last, self.count)
        start = (self.next - n) % self.capacity
        if start + n <= self.capacity:
            return self.data[start:start + n].copy()
        return np.concatenate((self.data[start:], self.data[:self.next]))

    @property
    def nbytes(self):
        return self.data.nbytes


class FailureWindow:
    """Frames and failure reasons over the last window_s seconds, counted incrementally."""

    def __init__(self, window_s=FAILURE_WINDOW_S, max_frames=int(FAILURE_WINDOW_S * MAX_HISTORY_FPS)):
        self.window_s = window_s
        self.frames = deque()
        self.max_frames = max_frames
        self.counts = Counter()

    def add(self, t, reason):
        """reason None for a good frame."""
        if len(self.frames) >= self.max_frames:
            self._drop()
        self.frames.append((t, reason))
        if reason is not None:
            self.counts[reason] += 1
        self.prune(t)

    def prune(self, now):
        while self.frames and now - self.frames[0][0] > self.window_s:
            self._drop()

    def _drop(self):
        _, reason = self.frames.popleft()
        if reason is not None:
            self.counts[reason] -= 1
            if not self.counts[reason]:
                del self.counts[reason]

    def summary(self):
        total = len(self.frames)
        failed = sum(self.counts.values())
        reasons = sorted(self.counts.items(), key=lambda item: -item[1])
        return {
            "window_s": self.window_s, "frames": total, "failed": failed,
            "failed_percent": 100.0 * failed / total if total else None,
            "by_reason": [{"reason": reason, "count": count,
                           "percent_of_frames": 100.0 * count / total,
                           "percent_of_failures": 100.0 * count / failed} for reason, count in reasons],
        }


class TrajectoryIntegrator:
    """
    x-y path from the body-frame deltas: each delta is in the body frame of its reference, so it is rotated by
    the yaw integrated so far (from the gyro prior), then the delta's own yaw is added. Starts at the origin
    facing +x; it drifts, and frames whose delta was dropped (reference resets) are missing from it.
    """

    def __init__(self):
        self.x = self.y = self.yaw = 0.0

    def step(self, dx, dy, dyaw):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        if dyaw is not None and math.isfinite(dyaw):
            self.yaw = math.remainder(self.yaw + dyaw, math.tau)
        return self.x, self.y, self.yaw


def _finite(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _lease_fresh(path, now, lease_s=LEASE_S):
    try:
        return now - path.stat().st_mtime <= lease_s
    except OSError:
        return False


class Telemetry:
    """
    VIO process side. record_frame() per frame; start() runs the publisher thread, close() stops it.
    temperature() returns the chip temperature in °C (or raises); output_error() the MAVLink sender's
    last error. Both are called on the publisher thread only.
    """

    def __init__(self, directory=DEFAULT_DIR, fps=10, min_inliers=15, mode="sending", temperature=None,
                 output_error=None, clock=time.time, log=print):
        self.directory = Path(directory)
        self.fps = fps
        self.min_inliers = min_inliers
        self.mode = mode
        self.temperature = temperature
        self.output_error = output_error
        self.clock = clock
        self.log = log
        self.phase = "starting"
        self.started_at = clock()
        self.history = RingBuffer(history_capacity(fps), len(HISTORY_COLUMNS))
        self.failures = FailureWindow()
        self.trajectory = TrajectoryIntegrator()
        self.sent_times = deque(maxlen=int(SEND_RATE_WINDOW_S * MAX_HISTORY_FPS) + 1)
        self.frames_total = self.ok_total = self.sent_total = 0
        self.reference_resets = self.bridged = self.gap_frames_total = self.recoveries = 0
        self.last = None
        self.chip_temperature = None
        self.chip_temperature_at = None
        self.extra = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._last_temperature = float("-inf")
        self._last_status = self._last_history = float("-inf")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            for name in (STATUS_FILE, HISTORY_FILE):
                (self.directory / name).unlink(missing_ok=True)  # never show a previous run's data
        except OSError as e:
            log(f"Telemetry directory {self.directory} unusable, the web page gets no live status: {e}")

    # --- VIO loop (hot path) ---

    def record_frame(self, t, ok, reason, features, valid_depth, matches, inliers, proc_ms, dt=None,
                     position_delta=None, yaw_delta=None, gap_frames=0, reference_reset=False, sent=False):
        """t: wall time (s). position_delta: body FRD (m) over dt (s), for ok frames. yaw_delta: rad, body z."""
        with self._lock:
            self.frames_total += 1
            vx = vy = vz = math.nan
            if ok and position_delta is not None and dt:
                dx, dy, dz = float(position_delta[0]), float(position_delta[1]), float(position_delta[2])
                vx, vy, vz = dx / dt, dy / dt, dz / dt
                x, y, yaw = self.trajectory.step(dx, dy, yaw_delta)
            else:
                x, y, yaw = self.trajectory.x, self.trajectory.y, self.trajectory.yaw
            self.history.append((t, 1.0 if ok else 0.0, inliers, vx, vy, vz, x, y, yaw, proc_ms, features,
                                 valid_depth))
            self.failures.add(t, None if ok else normalize_reason(reason))
            if ok:
                self.ok_total += 1
                if gap_frames:
                    self.bridged += 1
                    self.gap_frames_total += gap_frames
            if reference_reset:
                self.reference_resets += 1
            if sent:
                self.sent_total += 1
                self.sent_times.append(t)
            self.last = (t, ok, reason, features, valid_depth, matches, inliers, proc_ms, gap_frames)

    def record_recovery(self):
        with self._lock:
            self.recoveries += 1

    def set_phase(self, phase, **extra):
        with self._lock:
            self.phase = phase
            self.extra.update(extra)

    # --- publisher thread ---

    def start(self, interval=0.1):
        self._thread = threading.Thread(target=self._run, args=(interval,), name="vio-telemetry", daemon=True)
        self._thread.start()

    def close(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def _run(self, interval):
        while not self._stop.wait(interval):
            try:
                self.publish_due()
            except Exception as e:  # never let a full disk or a device error kill telemetry
                self.log(f"Telemetry publish failed: {e}")

    def publish_due(self, now=None):
        """One publisher step: sample the temperature and write whatever is due and leased."""
        now = self.clock() if now is None else now
        if self.temperature is not None and now - self._last_temperature >= TEMPERATURE_INTERVAL_S:
            self._last_temperature = now
            self.sample_temperature(now)
        if now - self._last_status >= STATUS_INTERVAL_S and _lease_fresh(self.directory / STATUS_LEASE, now):
            self._last_status = now
            self.write_status(now)
        if now - self._last_history >= HISTORY_INTERVAL_S and _lease_fresh(self.directory / HISTORY_LEASE, now):
            self._last_history = now
            self.write_history()

    def sample_temperature(self, now=None):
        try:
            value = self.temperature()
            value = float(getattr(value, "average", value))
            self.chip_temperature = value if math.isfinite(value) else None
        except Exception:
            self.chip_temperature = None
        self.chip_temperature_at = self.clock() if now is None else now
        return self.chip_temperature

    def status(self, now=None):
        now = self.clock() if now is None else now
        with self._lock:
            recent = self.history.ordered(last=int(RECENT_WINDOW_S * MAX_HISTORY_FPS) + 1)
            failures = self.failures.summary()
            sent_recent = [t for t in self.sent_times if now - t <= SEND_RATE_WINDOW_S]
            last = self.last
            counters = {"frames_total": self.frames_total, "ok_total": self.ok_total, "sent_total": self.sent_total,
                        "reference_resets": self.reference_resets, "bridged_frames": self.bridged,
                        "gap_frames_total": self.gap_frames_total, "recoveries": self.recoveries}
            phase, extra = self.phase, dict(self.extra)
            position = (self.trajectory.x, self.trajectory.y, self.trajectory.yaw)
        try:
            output_error = self.output_error() if self.output_error is not None else None
        except Exception as e:
            output_error = str(e)
        return {
            "pid": os.getpid(),
            "unix_time": now,
            "started_at": self.started_at,
            "phase": phase,
            **extra,
            "fps_configured": self.fps,
            "min_inliers": self.min_inliers,
            **counters,
            "last_frame": None if last is None else {
                "unix_time": last[0], "ok": bool(last[1]), "reason": last[2] or None, "features": int(last[3]),
                "valid_depth": int(last[4]), "matches": int(last[5]), "inliers": int(last[6]),
                "proc_ms": _finite(last[7]), "gap_frames": int(last[8])},
            "recent": recent_summary(recent, now),
            "failures": failures,
            "output": {"mode": self.mode, "messages_sent": counters["sent_total"],
                       "rate_hz": len(sent_recent) / SEND_RATE_WINDOW_S, "last_error": output_error},
            "chip_temperature_c": self.chip_temperature,
            "chip_temperature_at": self.chip_temperature_at,
            "trajectory": {"x": position[0], "y": position[1], "yaw": position[2]},
            "history": {"rows": self.history.count, "capacity": self.history.capacity, "bytes": self.history.nbytes,
                        "seconds": HISTORY_SECONDS},
        }

    def write_status(self, now=None):
        snapshots.atomic_write(self.directory / STATUS_FILE, json.dumps(self.status(now)).encode())

    def write_history(self):
        with self._lock:
            rows = self.history.ordered()
        buffer = io.BytesIO()
        np.save(buffer, rows, allow_pickle=False)
        snapshots.atomic_write(self.directory / HISTORY_FILE, buffer.getvalue())


def recent_summary(rows, now, window_s=RECENT_WINDOW_S, fps_window_s=FPS_WINDOW_S):
    """Frame rate, ok %, mean inliers and processing time of the rows (oldest first) in the last window_s."""
    if rows.size:
        rows = rows[now - rows[:, COLUMN["t"]] <= window_s]
    if not rows.size:
        return {"window_s": window_s, "frames": 0, "fps": 0.0, "ok_percent": None, "inliers_mean": None,
                "proc_ms_mean": None, "proc_ms_max": None}
    t = rows[:, COLUMN["t"]]
    fps_rows = t[now - t <= fps_window_s]
    span = min(fps_window_s, max(now - float(t[0]), 1e-6))
    ok = rows[:, COLUMN["ok"]] > 0.5
    inliers = rows[ok, COLUMN["inliers"]]
    proc = rows[:, COLUMN["proc_ms"]]
    return {
        "window_s": window_s,
        "frames": int(len(rows)),
        "fps": len(fps_rows) / span if len(fps_rows) > 1 else 0.0,
        "ok_percent": 100.0 * float(ok.mean()),
        "inliers_mean": float(inliers.mean()) if inliers.size else None,
        "proc_ms_mean": _finite(np.nanmean(proc)) if np.isfinite(proc).any() else None,
        "proc_ms_max": _finite(np.nanmax(proc)) if np.isfinite(proc).any() else None,
    }


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(name, directory=DEFAULT_DIR):
    """Ask the VIO process to keep publishing `name` (STATUS_LEASE or HISTORY_LEASE) for LEASE_S."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / name).touch()
    except OSError:
        pass


def read_status(directory=DEFAULT_DIR):
    try:
        return json.loads((Path(directory) / STATUS_FILE).read_text())
    except (OSError, ValueError):
        return None


class HistoryReader:
    """history.npy, parsed again only when the file changes."""

    def __init__(self):
        self._key = None
        self._rows = None

    def read(self, directory=DEFAULT_DIR):
        """(rows oldest first, file mtime) or (None, None)."""
        path = Path(directory) / HISTORY_FILE
        try:
            stat = path.stat()
            key = (str(path), stat.st_mtime_ns, stat.st_size)
            if key != self._key:
                rows = np.load(path, allow_pickle=False)
                if rows.ndim != 2 or rows.shape[1] != len(HISTORY_COLUMNS):
                    return None, None
                self._key, self._rows = key, rows
            return self._rows, stat.st_mtime
        except (OSError, ValueError):
            return None, None


def downsample(rows, seconds, now, max_points):
    """Rows of the last `seconds`, every k-th so at most max_points remain, always keeping the newest."""
    if rows is None or not len(rows):
        return np.zeros((0, len(HISTORY_COLUMNS))), 1
    rows = rows[rows[:, COLUMN["t"]] >= now - seconds]
    stride = max(1, int(math.ceil(len(rows) / max_points)))
    return rows[::-1][::stride][::-1], stride
