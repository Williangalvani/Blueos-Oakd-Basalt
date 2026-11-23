"""
Tracked features of the PnP VIO for the Cockpit depth widget, published through a file like depth_stream.py.

VIO process side (FeatureStreamService): tick() once per frame. With nobody watching that is a clock read
and a comparison, plus one stat of the lease at most every poll_interval. While main.py holds the lease
(it touches DIRECTORY/features.lease while a client is connected to /api/stream/features), at most every
`interval` it writes DIRECTORY/features.json atomically: that frame's features as one line of JSON (about
4 KB for 300 features). main.py forwards the bytes as they are, as server-sent events.

Positions are on the depth image's pixel grid, which is the grid of the depth stream's JPEG. The tracker
works on the rectified-left image at image_size, and the depth is aligned to that image, at the same size
or decimated. Positions are scaled like pnp_odometry.scale_to_depth_grid.

State of each feature, best first (its index in STATES is what each point carries):
    inlier     matched to the reference and kept by RANSAC
    outlier    matched, rejected by RANSAC
    matched    matched, no RANSAC result (too few matches, or the solve failed before RANSAC)
    depth      not in the reference, valid depth in this frame (it joins the next reference)
    no_depth   neither

main.py side: touch_lease(), LatestFile, sse_event(). depthai-free so it can be unit tested.
"""

import json
import os
import time
from pathlib import Path

import numpy as np

import snapshots

FEATURES_FILE = "features.json"
LEASE_FILE = "features.lease"
LEASE_S = 4.0
PUBLISH_INTERVAL_S = 0.2
STATES = ("no_depth", "depth", "matched", "outlier", "inlier")
NO_DEPTH, DEPTH, MATCHED, OUTLIER, INLIER = range(len(STATES))


def feature_states(count, depth_valid=None, matched_index=None, inlier_index=None):
    """uint8 state per feature (see STATES). inlier_index refers to matched_index, which refers to the features."""
    states = np.full(count, NO_DEPTH, np.uint8)
    if depth_valid is not None:
        valid = np.asarray(depth_valid, bool).ravel()
        if len(valid) == count:
            states[valid] = DEPTH
    if matched_index is not None:
        matched = np.asarray(matched_index, np.int64).ravel()
        matched = matched[(matched >= 0) & (matched < count)]
        if inlier_index is None:
            states[matched] = MATCHED
        else:
            states[matched] = OUTLIER
            inliers = np.asarray(inlier_index, np.int64).ravel()
            states[matched[inliers[(inliers >= 0) & (inliers < len(matched))]]] = INLIER
    return states


def to_frame_grid(uv, image_size, frame_size):
    """Pixel positions on the image_size (width, height) grid -> the frame_size grid."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if image_size is None or tuple(image_size) == tuple(frame_size):
        return uv
    return uv * np.array([frame_size[0] / image_size[0], frame_size[1] / image_size[1]])


def features_payload(uv, states, image_size, frame_size, sequence=None, ok=None, reason=None, now=None):
    """
    The published JSON object: points [u, v, state] on the frame_size (width, height) depth grid, u, v
    rounded to 0.1 px; counts per state; the frame's PnP result.
    """
    points = np.round(to_frame_grid(uv, image_size, frame_size), 1).tolist()
    states = np.asarray(states, np.uint8)
    for point, state in zip(points, states.tolist()):
        point.append(state)
    counts = np.bincount(states, minlength=len(STATES)).tolist()
    return {"pid": os.getpid(), "unix_time": time.time() if now is None else now, "sequence": sequence,
            "frame_size": [int(frame_size[0]), int(frame_size[1])],
            "image_size": None if image_size is None else [int(image_size[0]), int(image_size[1])],
            "ok": None if ok is None else bool(ok), "reason": reason or None, "states": list(STATES),
            "counts": dict(zip(STATES, counts)), "points": points}


def _lease_fresh(path, now, lease_s):
    try:
        return now - path.stat().st_mtime <= lease_s
    except OSError:
        return False


class FeatureStreamService:
    """
    VIO process side. tick(...) once per frame with the frame's features, references only: nothing is
    copied or converted unless a client holds the lease and `interval` has passed since the last write.
    """

    def __init__(self, directory, image_size, interval=PUBLISH_INTERVAL_S, lease_s=LEASE_S, poll_interval=0.5,
                 clock=time.time, log=print):
        self.directory = Path(directory)
        self.image_size = None if image_size is None else tuple(image_size)
        self.interval = interval
        self.lease_s = lease_s
        self.poll_interval = poll_interval
        self.clock = clock
        self.log = log
        self.wanted = False
        self.published = 0
        self.errors = 0
        self._last_poll = self._last_write = float("-inf")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            (self.directory / FEATURES_FILE).unlink(missing_ok=True)  # never show a previous run's features
        except OSError as e:
            log(f"Feature stream directory {self.directory} unusable: {e}")

    def tick(self, uv, frame_shape, sequence=None, ok=None, reason=None, depth_valid=None, matched_index=None,
             inlier_index=None):
        """frame_shape: the depth image's shape (height, width). Returns True if it wrote the features."""
        now = self.clock()
        if now - self._last_poll >= self.poll_interval:
            self._last_poll = now
            wanted = _lease_fresh(self.directory / LEASE_FILE, now, self.lease_s)
            if wanted != self.wanted:
                self.log(f"Feature stream {'started: a client is watching' if wanted else 'stopped: no client'}")
                self.wanted = wanted
        if not self.wanted or now - self._last_write < self.interval:
            return False
        self._last_write = now
        try:
            count = len(uv)
            states = feature_states(count, depth_valid, matched_index, inlier_index)
            payload = features_payload(uv, states, self.image_size, (frame_shape[1], frame_shape[0]), sequence, ok,
                                       reason, now)
            snapshots.atomic_write(self.directory / FEATURES_FILE, json.dumps(payload).encode())
        except Exception as e:  # never let the widget's feed stop the VIO
            self.errors += 1
            if self.errors == 1 or self.errors % 100 == 0:
                self.log(f"Feature stream: {e} ({self.errors} errors)")
            return False
        self.published += 1
        return True


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(directory):
    """Ask the VIO process to publish the features for LEASE_S."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / LEASE_FILE).touch()
    except OSError:
        pass


class LatestFile:
    """The bytes of a file the VIO process rewrites atomically, returned once per new file and only if fresh."""

    def __init__(self, path, max_age_s=3.0, clock=time.time):
        self.path = Path(path)
        self.max_age_s = max_age_s
        self.clock = clock
        self._key = None

    def read_new(self):
        try:
            stat = self.path.stat()
        except OSError:
            return None
        key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        if key == self._key:
            return None
        self._key = key
        if self.clock() - stat.st_mtime > self.max_age_s:
            return None  # left over from a VIO process that stopped
        try:
            return self.path.read_bytes()
        except OSError:
            return None


def sse_event(event, data):
    """One server-sent event; data is one line (JSON from json.dumps never holds a raw newline)."""
    if isinstance(data, str):
        data = data.encode()
    return b"event: " + event.encode() + b"\ndata: " + data.replace(b"\n", b"") + b"\n\n"
