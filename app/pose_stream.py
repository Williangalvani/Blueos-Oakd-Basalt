"""
3D pose of the PnP VIO for the pose viewer (the main page's panel and the /widget/pose Cockpit widget),
published through files like feature_stream.py.

Position: PoseIntegrator chains the displacements VISION_POSITION_DELTA carries (body FRD, expressed in the
body frame of the current frame), rotated into the world frame by the attitude in use:
    p <- p + R @ position_delta
Frames without a delta (rejected, reference resets) add nothing, so the position drifts like any dead
reckoning. The x-y trajectory panel (vio_telemetry.TrajectoryIntegrator) is a yaw-only gyro version.

Attitude R (body FRD -> world), from one of two sources (attitude_source):
- "autopilot": ArduPilot's ATTITUDE (autopilot_attitude.AttitudeSource, read from mavlink2rest), while a
  sample is at most autopilot_attitude.STALE_S old. Between samples (polled every 0.5 s) it is carried
  forward with the frames' gyro rotation since the sample's time (CHAIN_HISTORY_S of them are kept), so the
  view moves smoothly and the sample's age does not lag turns; each new sample replaces it.
- "gyro": the frames' gyro rotation deltas chained, R <- R @ exp(rotation_delta). Drifts with the gyro bias
  (about 0.65 deg/s on an OAK-D-W BMI270 at rest), and rotation during failed frames is not measured.
Neither is a filter: no gyro bias estimate and no gravity correction.

Frames:
- "start-FRD": the body frame at VIO start, x forward, y right, z down (m); NED with north at the initial
  heading if the vehicle started level. Used until the first autopilot sample (all the time without one).
- "NED": x north, y east, z down (m), from the VIO start position, heading from the autopilot. The first
  autopilot sample switches to it: the position and the trail so far are rotated about the start by
  T = R_autopilot @ R_gyro^T (a re-expression of start-FRD in NED, as good as the gyro attitude was), and
  the trail gets a new epoch so the viewers redraw it. With an autopilot from the start (the usual case:
  it answers before the camera has opened) the trail is still at the origin and nothing moves.
Switching sources later never moves the position or the trail: autopilot -> gyro (stale ATTITUDE) continues
from the last autopilot attitude, so nothing jumps and the frame stays NED while the heading drifts;
gyro -> autopilot snaps the attitude back by whatever the gyro drifted meanwhile (a few degrees after a
short loss), and later displacements go in the right direction again.

roll, pitch, yaw are the ZYX (aerospace) Euler angles of R in radians; roll_deg, pitch_deg, yaw_deg the same
in degrees; q is R as a quaternion [w, x, y, z].

VIO process side (PoseStreamService): record() once per frame, always, so the pose and the trail are
current when a viewer opens. While main.py holds the lease (pose.lease, touched while a client is
connected to /api/stream/pose) it writes pose.json at most RATE_HZ times a second, and pose_trail.json
(the trail: [t, x, y, z] points TRAIL_STEP_M or TRAIL_STEP_S apart, at most TRAIL_POINTS, thinned to every
other point when full) at most once a second when it changed, and every TRAIL_REFRESH_S anyway. main.py
sends the trail once per connection (and again if the VIO restarted or the frame changed), then each pose;
the viewer extends its own trail from the poses. status() is the attitude part for /api/status.

depthai-free so it can be unit tested.
"""

import json
import math
import os
import time
from collections import deque
from pathlib import Path

import numpy as np

import leases
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
# Gyro rotation kept to carry an autopilot sample forward from its own time
CHAIN_HISTORY_S = 2.0
FRAME_START, FRAME_NED = "start-FRD", "NED"
FRAME_TEXTS = {
    FRAME_START: ("body frame at VIO start: x forward, y right, z down (m); NED with north at the initial heading "
                  "if the vehicle started level"),
    FRAME_NED: "NED from the VIO start position: x north, y east, z down (m), heading from the autopilot",
}
AUTOPILOT, GYRO = "autopilot", "gyro"
ATTITUDE_TEXTS = {AUTOPILOT: "ArduPilot ATTITUDE", GYRO: "gyro-integrated, drifting"}
UNITS = {"x, y, z": "m", "roll, pitch, yaw": "rad", "roll_deg, pitch_deg, yaw_deg": "deg",
         "attitude_age_s": "s"}


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


def _orthonormal(R):
    u, _, vt = np.linalg.svd(R)
    return u @ vt  # keep it a rotation as the steps accumulate


class PoseIntegrator:
    """
    Position (m) and attitude (body -> world) from per-frame body FRD deltas and, when given, the autopilot's
    attitude; see the module docstring. frame and source say which frame and attitude are in use.
    """

    def __init__(self, history_s=CHAIN_HISTORY_S):
        self.position = np.zeros(3)
        self.rotation = np.eye(3)  # in use
        self.gyro = np.eye(3)  # the gyro chain; continues from the last autopilot attitude
        self.chain = np.eye(3)  # every delta since the start, never reset: only its differences are used
        self.history = deque()  # (t, chain) of the last history_s
        self.history_s = history_s
        self.frame = FRAME_START
        self.source = GYRO
        self.anchor = None  # (autopilot sample, chain at its time)
        self.realigned = None  # T of the change to NED, until the caller takes it

    def step(self, position_delta, rotation_delta=None, yaw_delta=None, t=None, attitude=None):
        """
        rotation_delta: body FRD rotation vector (rad); without it, a rotation of yaw_delta about body z.
        attitude: a fresh autopilot sample (autopilot_attitude.AttitudeSample) or None. t: wall time (s).
        """
        if rotation_delta is not None:
            delta = np.asarray(rotation_delta, dtype=np.float64).reshape(3)
        elif yaw_delta is not None and math.isfinite(yaw_delta):
            delta = np.array([0.0, 0.0, float(yaw_delta)])
        else:
            delta = np.zeros(3)
        if np.all(np.isfinite(delta)) and np.any(delta):
            D = rotvec_to_matrix(delta)
            self.chain = _orthonormal(self.chain @ D)
            self.gyro = _orthonormal(self.gyro @ D)
            if t is not None:
                self.history.append((float(t), self.chain))
                while self.history and t - self.history[0][0] > self.history_s:
                    self.history.popleft()
        self.set_attitude(attitude)
        if position_delta is not None:
            d = np.asarray(position_delta, dtype=np.float64).reshape(3)
            if np.all(np.isfinite(d)):
                self.position = self.position + self.rotation @ d
        return self.pose()

    def _chain_at(self, t):
        chain = self.chain
        for when, value in reversed(self.history):
            if when <= t:
                return value
            chain = value
        return chain  # older than the history: the oldest kept

    def set_attitude(self, attitude):
        """Use a fresh autopilot sample, or the gyro chain if None (between frames too). Returns the source."""
        if attitude is None:
            self.source = GYRO
            self.rotation = self.gyro
            return self.source
        if self.anchor is None or self.anchor[0] != attitude:
            self.anchor = (attitude, self._chain_at(attitude.unix_time))
        sample, chain_then = self.anchor
        R = _orthonormal(sample.matrix() @ chain_then.T @ self.chain)
        if self.frame == FRAME_START:
            T = R @ self.gyro.T
            self.position = T @ self.position
            self.frame, self.realigned = FRAME_NED, T
        self.source = AUTOPILOT
        self.rotation = self.gyro = R
        return self.source

    def take_realignment(self):
        T, self.realigned = self.realigned, None
        return T

    def pose(self):
        roll, pitch, yaw = euler_zyx(self.rotation)
        x, y, z = (float(v) for v in self.position)
        return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw,
                "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch), "yaw_deg": math.degrees(yaw),
                "q": quaternion(self.rotation)}


def _rounded(pose):
    return {key: ([round(c, 6) for c in value] if key == "q" else round(value, 4)) for key, value in pose.items()}


def frame_text(frame, source):
    text = FRAME_TEXTS[frame]
    if frame == FRAME_NED and source == GYRO:
        text += "; the autopilot attitude is lost, so the heading drifts with the gyro until it is back"
    return text


class PoseStreamService:
    """
    VIO process side; see the module docstring. attitude: an autopilot_attitude.AttitudeSource (its latest()
    is read once per frame), or None for the gyro only (attitude_mode "gyro"; the VIO always passes one).
    """

    def __init__(self, directory, camera_angle_deg=0.0, rate_hz=RATE_HZ, lease_s=LEASE_S, poll_interval=0.5,
                 trail_points=TRAIL_POINTS, trail_step_m=TRAIL_STEP_M, trail_step_s=TRAIL_STEP_S,
                 trail_interval=TRAIL_INTERVAL_S, clock=time.time, log=print, attitude=None):
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
        self.attitude = attitude
        self.attitude_mode = AUTOPILOT if attitude is not None else GYRO
        self.attitude_age = None
        self.integrator = PoseIntegrator()
        self._epoch_base = f"{os.getpid()}-{int(clock() * 1000)}"
        self.epoch = self._epoch_base
        self.trail = []
        self.trail_changed = False
        self.frames = self.ok_frames = 0
        self.last_ok = None
        self.wanted = False
        self.published = 0
        self.errors = 0
        self.switches = 0
        self.last_switch = None
        self.realigned = None
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
        sample = None
        if self.attitude is not None:
            sample, self.attitude_age = self.attitude.latest(t)
        before = self.integrator.source
        if ok and position_delta is not None:
            self.ok_frames += 1
            self.integrator.step(position_delta, rotation_delta, yaw_delta, t=t, attitude=sample)
        else:
            self.integrator.set_attitude(sample)
        T = self.integrator.take_realignment()
        if T is not None:
            self._realign(T, t)
        if self.integrator.source != before and self.attitude is not None:
            self._switched(before, self.integrator.source, t, quiet=T is not None)
        if ok and position_delta is not None:
            self._add_trail_point(t)
        return self.tick(t)

    def _realign(self, T, t):
        moved = any(any(point[1:]) for point in self.trail)
        for point in self.trail:
            point[1:] = [round(float(v), 3) for v in T @ np.asarray(point[1:], dtype=np.float64)]
        self.realigned = {"unix_time": t, "yaw_deg": round(math.degrees(math.atan2(T[1, 0], T[0, 0])), 2),
                          "trail_points": len(self.trail) if moved else 0}
        if moved:  # the viewers' own trails are in start-FRD: a new epoch makes them take this one
            self.epoch = f"{self._epoch_base}-ned"
            self.trail_changed = True
            self._last_trail = float("-inf")
        turned = f" (the trail so far turned by {self.realigned['yaw_deg']:.1f} deg)" if moved else ""
        self.log(f"Pose stream: autopilot attitude found, the pose is now in NED from the VIO start{turned}")

    def _switched(self, before, now_source, t, quiet=False):
        self.switches += 1
        age = self.attitude_age
        self.last_switch = {"unix_time": t, "from": before, "to": now_source,
                            "attitude_age_s": None if age is None else round(age, 2)}
        if quiet:
            return
        if now_source == GYRO:
            self.log(f"Pose stream: autopilot ATTITUDE {'lost' if age is None else f'{age:.1f} s old'}; the attitude "
                     f"is gyro-integrated from its last value and drifts until it is back")
        else:
            self.log("Pose stream: autopilot ATTITUDE back; the attitude follows the autopilot again")

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

    def status(self, now=None):
        """The attitude source for /api/status (telemetry "pose") and the viewers."""
        now = self.clock() if now is None else now
        source = self.integrator.source
        age = self.attitude_age
        return {"frame": self.integrator.frame, "frame_text": frame_text(self.integrator.frame, source),
                "attitude_mode": self.attitude_mode, "attitude_source": source,
                "attitude_text": ATTITUDE_TEXTS[source], "attitude_age_s": None if age is None else round(age, 2),
                "switches": self.switches, "last_switch": self.last_switch, "realigned": self.realigned,
                "autopilot": self.attitude.state(now) if self.attitude is not None else None}

    def payload(self, now):
        return {"pid": os.getpid(), "epoch": self.epoch, "unix_time": now,
                **_rounded(self.integrator.pose()), "ok": self.last_ok, "frames": self.frames,
                "ok_frames": self.ok_frames, "rate_hz": round(1.0 / self.interval, 2),
                "camera_angle_deg": self.camera_angle_deg, **self.status(now), "units": UNITS}

    def trail_payload(self, now):
        return {"pid": os.getpid(), "epoch": self.epoch, "unix_time": now, "frame": self.integrator.frame,
                "step_m": self.trail_step_m, "max_points": self.trail_points, "points": self.trail}

    def tick(self, now=None):
        now = self.clock() if now is None else now
        if now - self._last_poll >= self.poll_interval:
            self._last_poll = now
            wanted = leases.fresh(self.directory / LEASE_FILE, now, self.lease_s)
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

    def close(self):
        if self.attitude is not None:
            self.attitude.close()


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(directory):
    """Ask the VIO process to publish the pose for LEASE_S."""
    leases.touch(directory, LEASE_FILE)
