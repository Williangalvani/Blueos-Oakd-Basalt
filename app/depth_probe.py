"""
Depth probe: statistics of the device depth in a small region of interest (ROI) over the next N
frames, measured inside the running PnP VIO without slowing it.

1. GET /api/depth_probe (main.py) writes PROBE_DIR/requests/<id>.json with the ROI and frame count.
2. The VIO loop calls DepthProbeService.tick() every frame (at most one scandir every poll_interval)
   and on_depth() with the depth frame it already holds. While a request is active, on_depth()
   copies the ROI (at most 200x200 uint16) of each frame; there is no other per-frame work.
3. After N frames a short-lived thread computes the statistics and atomically writes
   PROBE_DIR/results/<id>.json. The endpoint waits for it, then deletes it.

One request is served at a time, oldest first. depthai-free; OpenCV only for the snapshot overlay.
"""

import json
import math
import os
import re
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

import snapshots

PROBE_DIR = snapshots.SNAPSHOT_DIR / "depth_probe"
DEFAULT_ROI_PX = 41
MAX_ROI_PX = 200
DEFAULT_FRAMES = 10
MAX_FRAMES = 50
# Requests older than this when the VIO process sees them are dropped (nobody waits any more)
REQUEST_MAX_AGE_S = 30.0
# Results nobody collected (the endpoint timed out) are deleted after this
RESULT_MAX_AGE_S = 300.0
REQUEST_NAME = re.compile(r"^[0-9a-f]{32}\.json$")

NOTE = ("z is the depth along the optical axis of the rectified left camera (StereoDepth outputs Z, not the "
        "distance along the ray). range is the straight-line distance from the camera's optical centre to the "
        "ROI-centre point, sqrt(X^2 + Y^2 + Z^2) with the camera matrix in use; at the image centre they are equal. "
        "To compare with a tape measure, aim at a flat target square to the camera and measure from the left "
        "camera's lens (the optical centre is a few mm behind the front of the lens, plus the port or window "
        "thickness), not from the housing. raw is the device depth; scaled is raw x depth_scale, what PnP uses. "
        "Pixels outside the device's min/max depth are invalid (0) and only count against valid_fraction.")


class ProbeTimeout(Exception):
    pass


class ProbeError(Exception):
    pass


@dataclass(frozen=True)
class ProbeRequest:
    x: float  # ROI centre on the rectified-left image grid (px)
    y: float
    w: int  # ROI size (px)
    h: int
    frames: int


def make_request(x=None, y=None, w=None, h=None, frames=None, image_size=(640, 400)):
    """
    Validated probe request on the image_size (width, height) grid, and the list of values clamped.
    Missing values get the defaults (image centre, 41 x 41 px, 10 frames). w, h and frames above
    their maximum are clamped; anything else unusable raises ValueError.
    """
    width, height = image_size
    x = width / 2 if x is None else float(x)
    y = height / 2 if y is None else float(y)
    if not (math.isfinite(x) and math.isfinite(y) and 0 <= x < width and 0 <= y < height):
        raise ValueError(f"x, y must be pixel coordinates inside the {width}x{height} rectified-left image, "
                         f"got ({x:g}, {y:g})")
    clamped = []

    def bounded(name, value, default, maximum):
        value = default if value is None else int(value)
        if value < 1:
            raise ValueError(f"{name} must be at least 1, got {value}")
        if value > maximum:
            clamped.append(f"{name} {value} -> {maximum}")
            value = maximum
        return value

    request = ProbeRequest(x, y, bounded("w", w, DEFAULT_ROI_PX, MAX_ROI_PX), bounded("h", h, DEFAULT_ROI_PX, MAX_ROI_PX),
                           bounded("frames", frames, DEFAULT_FRAMES, MAX_FRAMES))
    return request, clamped


def roi_bounds(request, shape, image_size):
    """(x0, y0, x1, y1), end exclusive, of the request's ROI on an array of shape (rows, cols, ...) whose grid
    may be a downscale of image_size (decimated depth); clipped to the array."""
    rows, cols = shape[:2]
    sx, sy = cols / image_size[0], rows / image_size[1]
    w, h = max(1, int(round(request.w * sx))), max(1, int(round(request.h * sy)))
    x0 = int(math.floor(request.x * sx + 0.5)) - w // 2
    y0 = int(math.floor(request.y * sy + 0.5)) - h // 2
    return max(x0, 0), max(y0, 0), min(x0 + w, cols), min(y0 + h, rows)


def ray_point(u, v, z, intrinsics):
    """3D point (X, Y, Z) at pixel (u, v) with depth z, and its range from the optical centre."""
    X = (u - intrinsics.cx) / intrinsics.fx * z
    Y = (v - intrinsics.cy) / intrinsics.fy * z
    return [float(X), float(Y), float(z)], float(math.sqrt(X * X + Y * Y + z * z))


def _finite(value):
    return None if value is None or not math.isfinite(value) else float(value)


def compute_result(request, rois, depth_range_mm=(300, 4000), depth_scale=1.0, intrinsics=None,
                   intrinsics_source=None, timestamps=(), sequences=()):
    """Statistics of the ROI crops (uint16 mm, one per frame); every depth in metres also comes scaled."""
    low, high = depth_range_mm
    valid_parts, frame_medians, frame_valid = [], [], []
    for roi in rois:
        roi = np.asarray(roi)
        values = roi[(roi > 0) & (roi >= low) & (roi <= high)]
        valid_parts.append(values)
        frame_valid.append(values.size / roi.size if roi.size else 0.0)
        frame_medians.append(float(np.median(values)) if values.size else None)
    values = np.concatenate(valid_parts).astype(np.float64) if valid_parts else np.zeros(0)
    n_px = int(sum(np.asarray(roi).size for roi in rois))
    medians = [m for m in frame_medians if m is not None]

    result = {
        "roi": {"x": request.x, "y": request.y, "w": request.w, "h": request.h},
        "frames": len(rois),
        "n_px": n_px,
        "n_valid_px": int(values.size),
        "valid_fraction": values.size / n_px if n_px else 0.0,
        "valid_range_mm": [low, high],
        "median_mm": None, "p10_mm": None, "p90_mm": None, "mean_mm": None,
        "median_of_frame_medians_mm": float(np.median(medians)) if medians else None,
        "frame_medians_mm": frame_medians,
        "frame_valid_fractions": frame_valid,
        "depth_scale": depth_scale,
        "intrinsics": None if intrinsics is None else {
            "fx": float(intrinsics.fx), "fy": float(intrinsics.fy), "cx": float(intrinsics.cx), "cy": float(intrinsics.cy)},
        "intrinsics_source": intrinsics_source,
        "raw_m": None, "scaled_m": None,
        "range_m": {"raw": None, "scaled": None},
        "point_m": {"raw": None, "scaled": None},
        "device_timestamps": [_finite(t) if t is not None else None for t in timestamps],
        "sequences": list(sequences),
        "note": NOTE,
    }
    if values.size:
        p10, median, p90 = np.percentile(values, [10, 50, 90])
        result.update(median_mm=float(median), p10_mm=float(p10), p90_mm=float(p90), mean_mm=float(values.mean()))
    if result["median_of_frame_medians_mm"] is not None:
        raw = result["median_of_frame_medians_mm"] / 1000.0
        result["raw_m"], result["scaled_m"] = raw, raw * depth_scale
        if intrinsics is not None:
            for key, z in (("raw", raw), ("scaled", raw * depth_scale)):
                result["point_m"][key], result["range_m"][key] = ray_point(request.x, request.y, z, intrinsics)
    result["summary"] = summary(result)
    return result


def summary(result):
    valid = f"valid {100 * result['valid_fraction']:.0f}%"
    if result["raw_m"] is None:
        return f"no valid depth in the ROI, {valid}"
    return f"median {result['raw_m']:.3f} m (scaled {result['scaled_m']:.3f} m), {valid}"


# --- main.py side ---------------------------------------------------------------------------------

def request_probe(request, probe_dir=PROBE_DIR, clock=time.time):
    """Queue a probe for the VIO process; returns the request id to wait on."""
    request_dir = Path(probe_dir) / "requests"
    request_dir.mkdir(parents=True, exist_ok=True)
    request_id = uuid.uuid4().hex
    record = {"id": request_id, "requested_at": clock(), **asdict(request)}
    snapshots.atomic_write(request_dir / f"{request_id}.json", json.dumps(record).encode())
    return request_id


def wait_for_probe(request_id, timeout, probe_dir=PROBE_DIR, poll=0.05, clock=time.monotonic, sleep=time.sleep):
    """
    The result for request_id. Raises ProbeError if the VIO process answered with an error and
    ProbeTimeout after timeout seconds (withdrawing the request if it was not picked up).
    """
    probe_dir = Path(probe_dir)
    result_path = probe_dir / "results" / f"{request_id}.json"
    deadline = clock() + timeout
    while True:
        try:
            result = json.loads(result_path.read_text())
        except (OSError, ValueError):
            result = None
        if result is not None:
            result_path.unlink(missing_ok=True)
            if result.get("error"):
                raise ProbeError(result["error"])
            return result
        if clock() >= deadline:
            request_path = probe_dir / "requests" / f"{request_id}.json"
            if request_path.exists():
                request_path.unlink(missing_ok=True)
                raise ProbeTimeout(f"the VIO process did not pick up the probe within {timeout:.0f} s")
            raise ProbeTimeout(f"the VIO process took the probe but did not finish it within {timeout:.0f} s "
                               "(no depth frames?)")
        sleep(poll)


def draw_probe_overlay(png, request, image_size):
    """PNG bytes with a crosshair at the request's centre and its ROI box (8-bit images only)."""
    import cv2  # main.py must start even if OpenCV is broken
    image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint8:
        raise ValueError("the crosshair needs an 8-bit image")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    x0, y0, x1, y1 = roi_bounds(request, image.shape, image_size)
    sx, sy = image.shape[1] / image_size[0], image.shape[0] / image_size[1]
    centre = (int(math.floor(request.x * sx + 0.5)), int(math.floor(request.y * sy + 0.5)))
    magenta = (255, 0, 255)
    cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), magenta, 1)
    cv2.drawMarker(image, centre, magenta, cv2.MARKER_CROSS, 2 * max(x1 - x0, y1 - y0, 20), 1)
    return snapshots.encode_png(image)


# --- VIO process side -----------------------------------------------------------------------------

def _run_in_thread(fn):
    threading.Thread(target=fn, name="depth-probe", daemon=True).start()


class DepthProbeService:
    """
    VIO-side handling. Call tick() once per frame and on_depth() with each depth frame (uint16 mm, on
    the image_size grid or a downscale of it). set_intrinsics() once the camera matrix is chosen.
    run(fn) runs the final statistics and file write off the VIO thread (a new thread by default).
    """

    def __init__(self, probe_dir=PROBE_DIR, depth_range_mm=(300, 4000), depth_scale=1.0, image_size=(640, 400),
                 poll_interval=0.2, clock=time.monotonic, wall_clock=time.time, run=_run_in_thread, log=print):
        self.probe_dir = Path(probe_dir)
        self.depth_range_mm = tuple(depth_range_mm)
        self.depth_scale = depth_scale
        self.image_size = tuple(image_size)
        self.poll_interval = poll_interval
        self.clock = clock
        self.wall_clock = wall_clock
        self.run = run
        self.log = log
        self.intrinsics = None
        self.intrinsics_source = None
        self.active = None
        self.last_poll = float("-inf")
        self.served = 0

    def set_intrinsics(self, intrinsics, source):
        self.intrinsics, self.intrinsics_source = intrinsics, source

    def tick(self):
        if self.active is not None:
            return
        now = self.clock()
        if now - self.last_poll < self.poll_interval:
            return
        self.last_poll = now
        for name in self._requests():
            record = self._claim(name)
            if record is None:
                continue
            request_id = name[:-len(".json")]
            requested_at = record.get("requested_at")
            if not isinstance(requested_at, (int, float)) or self.wall_clock() - requested_at > REQUEST_MAX_AGE_S:
                continue
            try:
                request, clamped = make_request(record.get("x"), record.get("y"), record.get("w"), record.get("h"),
                                                record.get("frames"), self.image_size)
            except (TypeError, ValueError) as e:
                self._write(request_id, {"id": request_id, "requested_at": requested_at, "error": str(e)})
                continue
            self.active = {"id": request_id, "requested_at": requested_at, "request": request, "clamped": clamped,
                           "bounds": None, "rois": [], "timestamps": [], "sequences": [], "started": self.clock()}
            return

    def on_depth(self, depth, device_timestamp=None, sequence=None):
        """Per-frame work while a probe is active: one ROI copy. True if the frame was used."""
        active = self.active
        if active is None:
            return False
        if active["bounds"] is None:
            active["bounds"] = roi_bounds(active["request"], depth.shape, self.image_size)
        x0, y0, x1, y1 = active["bounds"]
        active["rois"].append(np.array(depth[y0:y1, x0:x1], copy=True))
        active["timestamps"].append(device_timestamp)
        active["sequences"].append(sequence)
        if len(active["rois"]) >= active["request"].frames:
            self.active = None
            self.served += 1
            intrinsics, source, scale, shape = self.intrinsics, self.intrinsics_source, self.depth_scale, depth.shape
            self.run(lambda: self._finish(active, intrinsics, source, scale, shape))
        return True

    def _finish(self, active, intrinsics, source, scale, depth_shape):
        try:
            result = compute_result(active["request"], active["rois"], self.depth_range_mm, scale, intrinsics, source,
                                    active["timestamps"], active["sequences"])
            x0, y0, x1, y1 = active["bounds"]
            result.update(id=active["id"], requested_at=active["requested_at"], clamped=active["clamped"],
                          roi_depth_px={"x0": x0, "y0": y0, "x1": x1, "y1": y1},
                          depth_shape=[int(depth_shape[1]), int(depth_shape[0])], image_size=list(self.image_size),
                          duration_s=self.clock() - active["started"], unix_time=self.wall_clock())
        except Exception as e:
            self.log(f"Depth probe failed: {e}")
            result = {"id": active["id"], "requested_at": active["requested_at"], "error": f"probe failed: {e}"}
        self._write(active["id"], result)

    def _write(self, request_id, result):
        try:
            result_dir = self.probe_dir / "results"
            result_dir.mkdir(parents=True, exist_ok=True)
            snapshots.atomic_write(result_dir / f"{request_id}.json", json.dumps(result).encode())
        except OSError as e:
            self.log(f"Depth probe result not written: {e}")

    def _requests(self):
        """Request file names, oldest first."""
        try:
            entries = [entry for entry in os.scandir(self.probe_dir / "requests") if REQUEST_NAME.match(entry.name)]
        except FileNotFoundError:
            return []
        mtimes = {}
        for entry in entries:
            try:
                mtimes[entry.name] = entry.stat().st_mtime
            except FileNotFoundError:
                pass
        return sorted(mtimes, key=mtimes.get)

    def _claim(self, name):
        """Take a request file (rename, then read) and return its record, or None."""
        claimed = self.probe_dir / "requests" / f".claimed-{name}"
        try:
            os.replace(self.probe_dir / "requests" / name, claimed)
            record = json.loads(claimed.read_text())
        except (OSError, ValueError):
            return None
        finally:
            claimed.unlink(missing_ok=True)
        self._clean_results()
        return record if isinstance(record, dict) else None

    def _clean_results(self):
        try:
            entries = list(os.scandir(self.probe_dir / "results"))
        except FileNotFoundError:
            return
        cutoff = time.time() - RESULT_MAX_AGE_S
        for entry in entries:
            try:
                if entry.stat().st_mtime < cutoff:
                    os.unlink(entry.path)
            except OSError:
                pass
