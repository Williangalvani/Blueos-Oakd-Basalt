"""
3D pose of the PnP VIO for the pose viewer (the main page's panel and the /widget/pose Cockpit widget),
published through files like feature_stream.py.

Frame "start-FRD": the vehicle body frame at VIO start, x forward, y right, z down (m). With the vehicle
level at the start it is NED with north turned to the initial heading; there is no absolute heading or
gravity reference, so the attitude is relative to the start too. roll, pitch, yaw (rad) are the ZYX
(aerospace) Euler angles of the body in that frame; q is the same rotation as a quaternion [w, x, y, z].

PoseIntegrator chains the deltas VISION_POSITION_DELTA carries, in body FRD: the rotation vector of the
current body frame relative to the reference and the displacement expressed in the current body frame:
    R <- R @ exp(rotation_delta)        p <- p + R @ position_delta
It drifts, like any dead reckoning; frames without a delta (rejected, reference resets) add nothing. The
x-y trajectory panel (vio_telemetry.TrajectoryIntegrator) is the yaw-only version of the same chain.

VIO process side (PoseStreamService): record() once per frame, always, so the pose and the trail are
current when a viewer opens. While main.py holds the lease (pose.lease, touched while a client is
connected to /api/stream/pose) it writes pose.json at most RATE_HZ times a second, and pose_trail.json
(the trail: [t, x, y, z] points TRAIL_STEP_M or TRAIL_STEP_S apart, at most TRAIL_POINTS, thinned to every
other point when full) at most once a second when it changed, and every TRAIL_REFRESH_S anyway. main.py
sends the trail once per connection (and again if the VIO restarted), then each pose; the viewer extends
its own trail from the poses.

depthai-free so it can be unit tested.
"""

import json
import math
import os
import time
from pathlib import Path

import numpy as np

import snapshots

POSE_FILE = "pose.json"
TRAIL_FILE = "pose_trail.json"
LEASE_FILE = "pose.lease"
LEASE_S = 4.0
RATE_HZ = 10.0
TRAIL_INTERVAL_S = 1.0
# Rewritten this often even without changes, so main.py's freshness check (LatestFile, 3 s) accepts it
TRAIL_REFRESH_S = 2.0
TRAIL_STEP_M = 0.05
TRAIL_STEP_S = 2.0
TRAIL_POINTS = 1500
FRAME = "start-FRD"
FRAME_TEXT = ("body frame at VIO start: x forward, y right, z down (m); NED with north at the initial heading "
              "if the vehicle started level")


def rotvec_to_matrix(rotvec):
    """Rodrigues: rotation vector (rad) -> 3x3 rotation matrix."""
    v = np.asarray(rotvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(v))
    k = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    if theta < 1e-12:
        return np.eye(3) + k
    return np.eye(3) + math.sin(theta) / theta * k + (1.0 - math.cos(theta)) / theta ** 2 * (k @ k)


def euler_zyx(R):
    """(roll, pitch, yaw) in rad of a body -> world rotation matrix, ZYX order."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def quaternion(R):
    """[w, x, y, z] of a rotation matrix, w >= 0."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    return q if q[0] >= 0 else [-c for c in q]


class PoseIntegrator:
    """Position (m) and attitude (body -> start-FRD rotation) from per-frame body FRD deltas."""

    def __init__(self):
        self.position = np.zeros(3)
        self.rotation = np.eye(3)

    def step(self, position_delta, rotation_delta=None, yaw_delta=None):
        """rotation_delta: body FRD rotation vector (rad); without it, a rotation of yaw_delta about body z."""
        if rotation_delta is not None:
            delta = np.asarray(rotation_delta, dtype=np.float64).reshape(3)
        elif yaw_delta is not None and math.isfinite(yaw_delta):
            delta = np.array([0.0, 0.0, float(yaw_delta)])
        else:
            delta = np.zeros(3)
        if np.all(np.isfinite(delta)) and np.any(delta):
            u, _, vt = np.linalg.svd(self.rotation @ rotvec_to_matrix(delta))
            self.rotation = u @ vt  # keep it a rotation as the steps accumulate
        if position_delta is not None:
            d = np.asarray(position_delta, dtype=np.float64).reshape(3)
            if np.all(np.isfinite(d)):
                self.position = self.position + self.rotation @ d
        return self.pose()

    def pose(self):
        roll, pitch, yaw = euler_zyx(self.rotation)
        x, y, z = (float(v) for v in self.position)
        return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw, "q": quaternion(self.rotation)}


def _lease_fresh(path, now, lease_s):
    try:
        return now - path.stat().st_mtime <= lease_s
    except OSError:
        return False


def _rounded(pose):
    return {key: ([round(c, 6) for c in value] if key == "q" else round(value, 4)) for key, value in pose.items()}


class PoseStreamService:
    """VIO process side; see the module docstring."""

    def __init__(self, directory, camera_angle_deg=0.0, rate_hz=RATE_HZ, lease_s=LEASE_S, poll_interval=0.5,
                 trail_points=TRAIL_POINTS, trail_step_m=TRAIL_STEP_M, trail_step_s=TRAIL_STEP_S,
                 trail_interval=TRAIL_INTERVAL_S, clock=time.time, log=print):
        self.directory = Path(directory)
        self.camera_angle_deg = float(camera_angle_deg)
        self.interval = 1.0 / rate_hz
        self.lease_s = lease_s
        self.poll_interval = poll_interval
        self.trail_points = max(int(trail_points), 4)
        self.trail_step_m = trail_step_m
        self.trail_step_s = trail_step_s
        self.trail_interval = trail_interval
        self.clock = clock
        self.log = log
        self.integrator = PoseIntegrator()
        self.epoch = f"{os.getpid()}-{int(clock() * 1000)}"
        self.trail = []
        self.trail_changed = False
        self.frames = self.ok_frames = 0
        self.last_ok = None
        self.wanted = False
        self.published = 0
        self.errors = 0
        self._last_poll = self._last_write = self._last_trail = float("-inf")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            for name in (POSE_FILE, TRAIL_FILE):
                (self.directory / name).unlink(missing_ok=True)  # never show a previous run's pose
        except OSError as e:
            log(f"Pose stream directory {self.directory} unusable: {e}")
        self._add_trail_point(clock())

    def record(self, t, ok, position_delta=None, rotation_delta=None, yaw_delta=None):
        """One VIO frame (t: wall time). Returns True if it wrote pose.json."""
        self.frames += 1
        self.last_ok = bool(ok)
        if ok and position_delta is not None:
            self.ok_frames += 1
            self.integrator.step(position_delta, rotation_delta, yaw_delta)
            self._add_trail_point(t)
        return self.tick(t)

    def _add_trail_point(self, t):
        x, y, z = (float(v) for v in self.integrator.position)
        if self.trail:
            last = self.trail[-1]
            moved = math.dist((x, y, z), last[1:])
            if moved < self.trail_step_m and not (moved > 0 and t - last[0] >= self.trail_step_s):
                return
        self.trail.append([round(t, 2), round(x, 3), round(y, 3), round(z, 3)])
        if len(self.trail) > self.trail_points:
            self.trail = self.trail[:-1:2] + [self.trail[-1]]  # every other point, always keeping the newest
        self.trail_changed = True

    def payload(self, now):
        return {"pid": os.getpid(), "epoch": self.epoch, "unix_time": now, "frame": FRAME,
                **_rounded(self.integrator.pose()), "ok": self.last_ok, "frames": self.frames,
                "ok_frames": self.ok_frames, "rate_hz": round(1.0 / self.interval, 2),
                "camera_angle_deg": self.camera_angle_deg}

    def trail_payload(self, now):
        return {"pid": os.getpid(), "epoch": self.epoch, "unix_time": now, "frame": FRAME,
                "step_m": self.trail_step_m, "max_points": self.trail_points, "points": self.trail}

    def tick(self, now=None):
        now = self.clock() if now is None else now
        if now - self._last_poll >= self.poll_interval:
            self._last_poll = now
            wanted = _lease_fresh(self.directory / LEASE_FILE, now, self.lease_s)
            if wanted != self.wanted:
                self.log(f"Pose stream {'started: a viewer is open' if wanted else 'stopped: no viewer'}")
                self.wanted = wanted
                self._last_trail = float("-inf")
                self.trail_changed = True
        # 10 % slack: frames exactly one interval apart (10 fps) must not drop to every other one on jitter
        if not self.wanted or now - self._last_write < 0.9 * self.interval:
            return False
        self._last_write = now
        try:
            since = now - self._last_trail
            if (self.trail_changed and since >= self.trail_interval) or since >= TRAIL_REFRESH_S:
                self._last_trail = now
                self.trail_changed = False
                snapshots.atomic_write(self.directory / TRAIL_FILE, json.dumps(self.trail_payload(now)).encode())
            snapshots.atomic_write(self.directory / POSE_FILE, json.dumps(self.payload(now)).encode())
        except Exception as e:  # never let the viewer's feed stop the VIO
            self.errors += 1
            if self.errors == 1 or self.errors % 100 == 0:
                self.log(f"Pose stream: {e} ({self.errors} errors)")
            return False
        self.published += 1
        return True


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(directory):
    """Ask the VIO process to publish the pose for LEASE_S."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / LEASE_FILE).touch()
    except OSError:
        pass
