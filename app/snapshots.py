"""
On-demand snapshots of the PnP VIO camera streams, taken without stopping or slowing VIO.

1. GET /api/snapshot (main.py) writes SNAPSHOT_DIR/requests/<stream> holding its request time.
2. The VIO process polls that directory at most every poll_interval (one scandir), and sends the
   device a one-message request. A Script node on the device forwards a single frame of that stream
   (it keeps only the latest frame of each stream, in non-blocking size-1 queues), so nothing crosses
   the PoE link until a snapshot is asked for. One request is in flight at a time, at most one per
   min_interval.
3. A background thread converts and encodes the frame and atomically writes <stream>.png (depth:
   colorized, invalid black, plus 16-bit depth_raw.png) and then <stream>.json, which echoes the
   request time it serves.
4. The endpoint waits up to ~3 s for a <stream>.json that serves its request.

rect_left snapshots also carry the tracked features of the last processed frame ("overlay" in the JSON:
position, inlier/outlier/unmatched and the position in the previous frame), built on the writer thread
from references the VIO loop keeps anyway, so it costs nothing until a snapshot is asked for.
/api/snapshot?overlay=1 draws it (draw_feature_overlay).

Requests for the same stream coalesce in one file. This module is depthai-free so it can be unit tested.
"""

import json
import os
import queue
import threading
import time
from pathlib import Path

import numpy as np

SNAPSHOT_DIR = Path("/app/data/snapshots")
STREAMS = ("rect_left", "rect_right", "depth", "rgb")
# Camera matrix and calibration the running PnP VIO uses, for /api/status and /api/focal_check
CAMERA_STATUS = "camera.json"


class SnapshotUnavailable(Exception):
    pass


class SnapshotTimeout(Exception):
    pass


def atomic_write(path, data):
    path = Path(path)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def write_camera_status(status, snapshot_dir=SNAPSHOT_DIR):
    Path(snapshot_dir).mkdir(parents=True, exist_ok=True)
    atomic_write(Path(snapshot_dir) / CAMERA_STATUS, json.dumps(status).encode())


def clear_camera_status(snapshot_dir=SNAPSHOT_DIR):
    (Path(snapshot_dir) / CAMERA_STATUS).unlink(missing_ok=True)


def read_camera_status(snapshot_dir=SNAPSHOT_DIR):
    try:
        return json.loads((Path(snapshot_dir) / CAMERA_STATUS).read_text())
    except (OSError, ValueError):
        return None


def _check_stream(stream):
    if stream not in STREAMS:
        raise ValueError(f"Unknown stream {stream!r}; expected one of {', '.join(STREAMS)}")


# --- main.py side ---------------------------------------------------------------------------------

def request_snapshot(stream, snapshot_dir=SNAPSHOT_DIR, clock=time.time):
    """Ask the VIO process for a fresh frame of stream; returns the request time to wait on."""
    _check_stream(stream)
    request_dir = Path(snapshot_dir) / "requests"
    request_dir.mkdir(parents=True, exist_ok=True)
    requested_at = clock()
    atomic_write(request_dir / stream, json.dumps({"stream": stream, "requested_at": requested_at}).encode())
    return requested_at


def read_metadata(stream, snapshot_dir=SNAPSHOT_DIR):
    try:
        return json.loads((Path(snapshot_dir) / f"{stream}.json").read_text())
    except (OSError, ValueError):
        return None


def snapshot_path(stream, raw=False, snapshot_dir=SNAPSHOT_DIR):
    return Path(snapshot_dir) / (f"{stream}_raw.png" if raw else f"{stream}.png")


def wait_for_snapshot(stream, requested_at, raw=False, timeout=3.0, snapshot_dir=SNAPSHOT_DIR,
                      poll=0.05, clock=time.monotonic, sleep=time.sleep):
    """
    PNG bytes and metadata of the first snapshot of stream that serves a request made at
    requested_at. Raises SnapshotUnavailable if the VIO process answered with an error, or
    SnapshotTimeout after timeout seconds.
    """
    deadline = clock() + timeout
    while True:
        meta = read_metadata(stream, snapshot_dir)
        if meta is not None and meta.get("requested_at", float("-inf")) >= requested_at:
            if meta.get("error"):
                raise SnapshotUnavailable(meta["error"])
            try:
                return snapshot_path(stream, raw, snapshot_dir).read_bytes(), meta
            except OSError:
                pass  # replaced by a newer snapshot between the two reads; retry
        if clock() >= deadline:
            raise SnapshotTimeout(f"No {stream} snapshot within {timeout:.0f} s")
        sleep(poll)


OVERLAY_COLORS = {"inlier": (113, 204, 46), "outlier": (60, 76, 231), "unmatched": (166, 163, 143)}  # BGR


def draw_feature_overlay(png, overlay, image_size=None):
    """
    PNG bytes with the overlay's features drawn: inliers green, outliers (matched, rejected by RANSAC) red,
    features without a match or depth grey, and a line from each feature's previous-frame position.
    """
    import cv2  # main.py must start even if OpenCV is broken
    image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint8:
        raise ValueError("the feature overlay needs an 8-bit image")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    width, height = image_size or overlay.get("image_size") or (image.shape[1], image.shape[0])
    sx, sy = image.shape[1] / float(width), image.shape[0] / float(height)
    for u, v, status, pu, pv in overlay.get("points", ()):
        color = OVERLAY_COLORS.get(status, OVERLAY_COLORS["unmatched"])
        centre = (int(round(u * sx)), int(round(v * sy)))
        if pu is not None and pv is not None:
            cv2.line(image, (int(round(pu * sx)), int(round(pv * sy))), centre, color, 1, cv2.LINE_AA)
        cv2.circle(image, centre, 2 if status == "unmatched" else 4, color, 1, cv2.LINE_AA)
    return encode_png(image)


# --- VIO process side -----------------------------------------------------------------------------

def build_overlay(ids, uv, matched_index, inlier_index, previous_ids, previous_uv, sequence=None, image_size=None,
                  ok=None, reason=None):
    """
    Feature overlay of one processed frame: [u, v, status, previous u, previous v] per feature, status
    inlier / outlier / unmatched. inlier_index refers to matched_index, which refers to ids/uv.
    """
    ids = np.asarray(ids).ravel().tolist()
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    status = ["unmatched"] * len(ids)
    matched = [] if matched_index is None else np.asarray(matched_index).ravel().tolist()
    for k in matched:
        status[k] = "outlier"
    if inlier_index is not None:
        for j in np.asarray(inlier_index).ravel().tolist():
            if 0 <= j < len(matched):
                status[matched[j]] = "inlier"
    previous = {}
    if previous_ids is not None and previous_uv is not None:
        previous_uv = np.asarray(previous_uv, dtype=np.float64).reshape(-1, 2)
        previous = dict(zip(np.asarray(previous_ids).ravel().tolist(), previous_uv.tolist()))
    points = []
    for k, feature_id in enumerate(ids):
        prev = previous.get(feature_id)
        points.append([round(float(uv[k, 0]), 1), round(float(uv[k, 1]), 1), status[k],
                       None if prev is None else round(prev[0], 1), None if prev is None else round(prev[1], 1)])
    return {"sequence": sequence, "image_size": None if image_size is None else list(image_size), "ok": ok,
            "reason": reason or None, "points": points,
            "inliers": status.count("inlier"), "outliers": status.count("outlier"),
            "unmatched": status.count("unmatched")}


def colorize_depth(depth_mm, min_mm=300, max_mm=4000):
    """uint16 depth (mm) -> BGR image, near = red, far = blue, invalid (0) = black."""
    depth = np.asarray(depth_mm)
    scaled = (depth.astype(np.float32) - min_mm) / float(max(max_mm - min_mm, 1))
    gray = (255.0 * (1.0 - np.clip(scaled, 0.0, 1.0))).astype(np.uint8)
    import cv2  # only the VIO process encodes; main.py must start even if OpenCV is broken
    color = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    color[depth == 0] = 0
    return color


def encode_png(image):
    import cv2
    ok, data = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("PNG encoding failed")
    return data.tobytes()


def write_snapshot(snapshot_dir, stream, image, meta, depth_range_mm=(300, 4000), clock=time.time):
    """Write the PNG(s) for one frame, then <stream>.json (so the metadata never points at an older PNG)."""
    snapshot_dir = Path(snapshot_dir)
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    image = np.asarray(image)
    files = [f"{stream}.png"]
    if stream == "depth":
        atomic_write(snapshot_dir / "depth_raw.png", encode_png(image.astype(np.uint16)))
        atomic_write(snapshot_dir / "depth.png", encode_png(colorize_depth(image, *depth_range_mm)))
        files.append("depth_raw.png")
    else:
        atomic_write(snapshot_dir / f"{stream}.png", encode_png(image))
    record = {**meta, "stream": stream, "width": int(image.shape[1]), "height": int(image.shape[0]),
              "written_at": clock(), "files": files}
    atomic_write(snapshot_dir / f"{stream}.json", json.dumps(record).encode())


def write_error(snapshot_dir, stream, requested_at, error, clock=time.time):
    snapshot_dir = Path(snapshot_dir)
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    record = {"stream": stream, "requested_at": requested_at, "error": error, "written_at": clock()}
    atomic_write(snapshot_dir / f"{stream}.json", json.dumps(record).encode())


class SnapshotWriter:
    """Background thread that converts and writes snapshots, so the VIO loop never encodes PNGs."""

    def __init__(self, snapshot_dir=SNAPSHOT_DIR, depth_range_mm=(300, 4000), log=print):
        self.snapshot_dir = Path(snapshot_dir)
        self.depth_range_mm = depth_range_mm
        self.log = log
        self._queue = queue.Queue(maxsize=4)
        self._thread = threading.Thread(target=self._run, name="snapshot-writer", daemon=True)
        self._thread.start()

    def submit(self, stream, load, meta):
        """load() returns the frame as a numpy array; it runs on the writer thread. False if busy."""
        return self._put(("frame", stream, load, meta))

    def submit_error(self, stream, requested_at, error):
        return self._put(("error", stream, requested_at, error))

    def _put(self, item):
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            return False

    def close(self, timeout=2.0):
        self._put(None)
        self._thread.join(timeout)

    def _run(self):
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                if item[0] == "frame":
                    _, stream, load, meta = item
                    meta = {key: value() if callable(value) else value for key, value in meta.items()}
                    write_snapshot(self.snapshot_dir, stream, load(), meta, self.depth_range_mm)
                else:
                    _, stream, requested_at, error = item
                    write_error(self.snapshot_dir, stream, requested_at, error)
            except Exception as e:
                self.log(f"Snapshot of {item[1]} failed: {e}")


class SnapshotService:
    """
    VIO-side request handling. Call tick() once per VIO frame (cheap: at most one scandir every
    poll_interval) and on_frame() with each message from the device's snapshot queue.

    send_request(stream) must ask the device for one frame of stream without blocking.
    unavailable maps streams that can't be served (e.g. no CAM_A) to the error to answer with.
    overlay(), if given, is called when a rect_left frame arrives and returns a zero-argument function
    (run on the writer thread) that builds that snapshot's feature overlay, or None.
    """

    def __init__(self, available, send_request, writer, snapshot_dir=SNAPSHOT_DIR, unavailable=None,
                 min_interval=1.0, poll_interval=0.2, reply_timeout=2.0, clock=time.monotonic, log=print,
                 overlay=None):
        self.available = set(available)
        self.unavailable = dict(unavailable or {})
        self.send_request = send_request
        self.writer = writer
        self.overlay = overlay
        self.request_dir = Path(snapshot_dir) / "requests"
        self.min_interval = min_interval
        self.poll_interval = poll_interval
        self.reply_timeout = reply_timeout
        self.clock = clock
        self.log = log
        self.pending = None  # (stream, requested_at, sent_at)
        self.last_poll = float("-inf")
        self.last_request = float("-inf")
        self.served = 0

    def tick(self):
        now = self.clock()
        if self.pending is not None and now - self.pending[2] > self.reply_timeout:
            stream, requested_at, _ = self.pending
            self.pending = None
            self.log(f"No {stream} snapshot from the device within {self.reply_timeout:.0f} s")
            self.writer.submit_error(stream, requested_at, "the device did not return a frame")
        if now - self.last_poll < self.poll_interval:
            return
        self.last_poll = now
        for stream in self._requests():
            if stream not in self.available:
                requested_at = self._claim(stream)
                if requested_at is not None:
                    error = self.unavailable.get(stream, f"{stream} is not available")
                    self.writer.submit_error(stream, requested_at, error)
                continue
            if self.pending is not None or now - self.last_request < self.min_interval:
                continue  # left in place for a later tick
            requested_at = self._claim(stream)
            if requested_at is None:
                continue
            self.send_request(stream)
            self.pending = (stream, requested_at, now)
            self.last_request = now

    def on_frame(self, load, device_timestamp=None, sequence=None):
        """load(stream) returns the frame as a numpy array; it runs on the writer thread."""
        if self.pending is None:
            return False  # late reply to a request that already timed out
        stream, requested_at, _ = self.pending
        self.pending = None
        meta = {"requested_at": requested_at, "unix_time": time.time(),
                "device_timestamp": device_timestamp, "sequence": sequence}
        if stream == "rect_left" and self.overlay is not None:
            build = self.overlay()
            if build is not None:
                meta["overlay"] = build
        self.served += 1
        return self.writer.submit(stream, lambda: load(stream), meta)

    def _requests(self):
        """Streams with a pending request file, oldest request first."""
        try:
            entries = [entry for entry in os.scandir(self.request_dir) if entry.name in STREAMS]
        except FileNotFoundError:
            return []
        mtimes = {}
        for entry in entries:
            try:
                mtimes[entry.name] = entry.stat().st_mtime
            except FileNotFoundError:
                pass
        return sorted(mtimes, key=mtimes.get)

    def _claim(self, stream):
        """
        Take a request file and return its request time (None if it vanished or is unreadable).
        Renaming before reading means a request written meanwhile lands in a new file, served later.
        """
        claimed = self.request_dir / f".claimed-{stream}"
        try:
            os.replace(self.request_dir / stream, claimed)
            return float(json.loads(claimed.read_text())["requested_at"])
        except (OSError, ValueError, KeyError, TypeError):
            return None
