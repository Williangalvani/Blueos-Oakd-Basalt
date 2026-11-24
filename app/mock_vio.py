#!/usr/bin/env python3
"""
Fake PnP VIO for mock mode (mock_mode.py): what pnp_vio.py publishes, from a synthetic camera, without
depthai. main.py launches it through the VIOSupervisor with pnp_vio.py's command line (unknown options are
ignored) plus --snapshot-dir and --log-dir.

Scene: a seabed texture larger than the view, panned by the trajectory integrated from mock_mode.motion;
depth is a tilted plane whose distance drifts, with three blobs (nearer) moving across the view, no depth
where the texture is dark (rock shadows), in the left edge band and outside the stereo depth range. The
right image is the left one shifted by the disparity fx * baseline / depth, so snapshots, the depth probe
and the focal check give consistent results.

Tracked features follow the texture. Their state is what the PnP would give: no depth, depth (new, joins
the next reference), matched, rejected (most features on a moving blob) or inlier. For the last TURBID_S of
every FAILURE_PERIOD_S after the start the water turns turbid: depth drops out and the frames fail, which
also exercises the tracking recovery count.

Motion: mock_mode.motion, a closed lawnmower survey with depth steps and 90-degree yaw turns, plus a small
roll and pitch oscillation (mock_mode.attitude). The texture pans with the x-y trajectory, folded back
at its edges.

Published through the real services, at the configured fps:
- vio_telemetry.Telemetry: status.json, history.npy (lease-gated), the chip temperature;
- depth_stream.DepthStreamService: depth_stream.jpg (8-bit depth as the camera's Script encodes it, JPEG at
  --depth-stream-quality) every round(fps / depth-stream-fps)-th frame, only while the lease is held. The
  camera variants are emulated here; host_jpeg runs the real depth_stream.HostJpegEncoder. With
  --depth-stream-debug a stand-in device takes the log level and logs mock encoder lines;
- feature_stream.FeatureStreamService: features.json while its lease is held;
- pose_stream.PoseStreamService: pose.json and pose_trail.json while its lease is held; its attitude from
  mock_mode.attitude_fetch (an in-process mavlink2rest ATTITUDE with gaps), and its gyro deltas carry
  mock_mode.GYRO_BIAS so the fallback drifts;
- snapshots.SnapshotService and depth_probe.DepthProbeService; camera.json;
- link_stats.LinkMonitor (telemetry.link): the per-stream counts the real VIO makes (full_frame: RAW16 depth,
  tracked features, Sync headers; points: tracked features, the depth points, the ROI configs up, full depth
  frames only on demand; both: IMU at IMU_RATE_HZ, snapshots, the camera's JPEGs), and stand-in XLink counters
  (MockLinkCounters: those bytes plus a little protocol traffic), all marked mock;
- --depth-transport points (the default, as in pnp_vio.py): the real depth_points.DepthPointsReceiver and
  FullDepthGate, against MockPointsCamera, which answers each request as the camera's Script and
  SpatialLocationCalculator would (5x5 window, every pixel, thresholds exclusive, average) on the synthetic
  depth of that frame; the depth probe and the host_jpeg stream get full depth frames only through the gate;
- the camera's load (vio_telemetry.read_camera_load on MockCameraLoad: Leon CPU and memory, synthetic);
- a JSONL data log under --log-dir.
Nothing is sent anywhere.
"""

import argparse
import math
import os
import random
import signal
import sys
import time
import warnings
from collections import deque
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import autopilot_attitude
import depth_points
import depth_probe
import depth_stream
import feature_stream
import link_stats
import mock_mode
import pose_stream
import snapshots
import vio_log
import vio_telemetry
from stereo_config import add_stereo_arguments, disparity_format, resolve_stereo_settings

BASELINE_M = 0.075
# fx at 640 px wide, scaled with the width
FOCAL_AT_640 = 400.0
FAILURE_PERIOD_S = 50.0
TURBID_S = 2.5
STARTUP_PHASE_S = 0.3
STATS_EVERY = 150
SNAPSHOT_HISTORY_S = 10.0
# pnp_vio's IMU: accelerometer and gyroscope at 200 Hz, one packet per message (batch threshold 1)
IMU_RATE_HZ = 200.0
MOCK_LINK_SOURCE = "mock (synthetic XLink counters)"
# Mock XLink traffic beyond the counted streams: a share of them plus RPC / timesync / watchdog bytes
MOCK_LINK_OVERHEAD = 0.01
MOCK_LINK_BACKGROUND_DOWN = 1500.0
MOCK_LINK_BACKGROUND_UP = 800.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Fake PnP VIO (mock mode)", allow_abbrev=False)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--num-features", type=int, default=300)
    parser.add_argument("--max-features", type=int, default=0)
    parser.add_argument("--min-inliers", type=int, default=15)
    parser.add_argument("--recovery-frames", type=int, default=20)
    parser.add_argument("--max-reference-age", type=float, default=1.0)
    parser.add_argument("--snapshots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--snapshot-encoding", choices=snapshots.ENCODINGS, default=snapshots.CAMERA_JPEG)
    parser.add_argument("--depth-stream", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--depth-stream-fps", type=float, default=5.0)
    parser.add_argument("--depth-stream-quality", type=int, default=70)
    parser.add_argument("--depth-stream-encoding", choices=depth_stream.VARIANTS, default=depth_stream.DEFAULT_VARIANT)
    parser.add_argument("--depth-stream-debug", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--pose-attitude-source", choices=pose_stream.ATTITUDE_MODES, default=pose_stream.AUTOPILOT)
    parser.add_argument("--depth-transport", choices=depth_points.TRANSPORTS, default=depth_points.DEFAULT_TRANSPORT)
    parser.add_argument("--depth-points-rois", choices=depth_points.ROI_SOURCES,
                        default=depth_points.DEFAULT_ROI_SOURCE)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=15.0)
    parser.add_argument("--camera-angle", type=float, default=0.0)
    parser.add_argument("--depth-scale", type=float, default=1.0)
    parser.add_argument("--refractive-index", type=float, default=1.333)
    parser.add_argument("--log-only", action="store_true")
    parser.add_argument("--telemetry-dir", type=Path, default=None)
    parser.add_argument("--snapshot-dir", type=Path, default=snapshots.SNAPSHOT_DIR)
    parser.add_argument("--log-dir", type=Path, default=vio_log.LOG_DIR)
    parser.add_argument("--seed", type=int, default=None)
    add_stereo_arguments(parser)
    args, _ = parser.parse_known_args(argv)
    return args


def make_texture(width, height, rng):
    """Seabed-like texture (blurred noise, ripples, rocks), three times the view in each direction."""
    import cv2
    size = (3 * width, 3 * height)
    noise = rng.normal(0, 1, (size[1], size[0])).astype(np.float32)

    def blurred(sigma):
        layer = cv2.GaussianBlur(noise, (0, 0), sigma)
        return layer / max(float(layer.std()), 1e-6)

    texture = 22 * blurred(1.5) + 32 * blurred(8.0)
    yy, xx = np.mgrid[0:size[1], 0:size[0]].astype(np.float32)
    texture += 8 * np.sin(xx / 9.0 + 0.6 * np.sin(yy / 40.0))
    image = np.clip(texture + 115, 0, 255).astype(np.uint8)
    for _ in range(int(60 * size[0] * size[1] / (1920 * 1200)) + 20):
        centre = (int(rng.integers(0, size[0])), int(rng.integers(0, size[1])))
        radius = int(rng.integers(4, 22) * width / 640) + 2
        shade = int(rng.integers(25, 200))
        cv2.circle(image, centre, radius, shade, -1, cv2.LINE_AA)
    return image


class Scene:
    """The synthetic camera: texture, depth and disparity as functions of time and the pan offset."""

    def __init__(self, width, height, depth_range_mm, rng):
        self.width, self.height = width, height
        self.min_mm, self.max_mm = depth_range_mm
        self.fx = FOCAL_AT_640 * width / 640.0
        self.intrinsics = [self.fx, self.fx, width / 2.0, height / 2.0]
        self.texture = make_texture(width, height, rng)
        self.edge_band = max(1, int(round(12 * width / 640)))
        self.blobs = [(rng.uniform(0.15, 0.3), rng.uniform(0.15, 0.3), rng.uniform(19, 31), rng.uniform(13, 23),
                       rng.uniform(0, math.tau), rng.uniform(0.08, 0.13), rng.uniform(250, 450)) for _ in range(3)]
        self.grid = np.mgrid[0:height, 0:width].astype(np.float32)

    def clamp_offset(self, x, y):
        return (min(max(x, 0.0), 2.0 * self.width - 1), min(max(y, 0.0), 2.0 * self.height - 1))

    def view(self, offset):
        x0, y0 = int(offset[0]), int(offset[1])
        return self.texture[y0:y0 + self.height, x0:x0 + self.width]

    def blob_weight(self, u, v, t):
        """Largest blob influence (0..1) at pixels u, v, and the depth (mm) the blobs take off there."""
        nu, nv = u / self.width, v / self.height
        weight = np.zeros(np.shape(u), np.float32)
        lift = np.zeros(np.shape(u), np.float32)
        for ax, ay, px, py, phase, sigma, height in self.blobs:
            cx = 0.5 + ax * math.sin(math.tau * t / px + phase)
            cy = 0.5 + ay * math.sin(math.tau * t / py + 2.0 * phase)
            w = np.exp(-((nu - cx) ** 2 + ((nv - cy) * self.height / self.width) ** 2) / (2.0 * sigma ** 2))
            weight = np.maximum(weight, w)
            lift = np.maximum(lift, height * w)
        return weight, lift

    def geometric_depth(self, u, v, t):
        """Depth in mm without holes (float)."""
        base = 1600.0 + 250.0 * math.sin(math.tau * t / 45.0)
        plane = base + 350.0 * (v / self.height - 0.5) + 150.0 * (u / self.width - 0.5)
        return plane - self.blob_weight(u, v, t)[1]

    def depth_mm(self, u, v, t, offset, turbid=0.0, rng=None):
        """Device-like depth (uint16 mm, 0 invalid) at pixels u, v."""
        z = self.geometric_depth(u, v, t)
        texture = self.view(offset)[np.clip(v.astype(int), 0, self.height - 1), np.clip(u.astype(int), 0, self.width - 1)]
        valid = (texture >= 50) & (u >= self.edge_band) & (z >= self.min_mm) & (z <= self.max_mm)
        if turbid and rng is not None:
            valid &= rng.random(np.shape(u)) >= turbid
        return np.where(valid, np.round(z), 0).astype(np.uint16)

    def depth_image(self, t, offset, turbid=0.0):
        yy, xx = self.grid
        rng = np.random.default_rng(int(t * 1000)) if turbid else None
        return self.depth_mm(xx, yy, t, offset, turbid, rng)

    def frame(self, stream, t, offset, turbid=0.0):
        """rect_left, rect_right (uint8) or depth (uint16 mm) at time t."""
        import cv2
        if stream == "depth":
            return self.depth_image(t, offset, turbid)
        if stream == "rect_left":
            return self.view(offset).copy()
        yy, xx = self.grid
        disparity = self.fx * BASELINE_M * 1000.0 / np.maximum(self.geometric_depth(xx, yy, t), 1.0)
        map_x = (xx + int(offset[0]) + disparity).astype(np.float32)
        map_y = (yy + int(offset[1])).astype(np.float32)
        return cv2.remap(self.texture, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)


def body_rotation_delta(t0, t1, yaw_delta):
    """
    Body FRD rotation vector from t0 to t1 of the mock attitude Rz(yaw) Ry(pitch) Rx(roll): the survey's yaw
    turn (yaw_delta) with mock_mode.attitude's roll and pitch at both ends. t0, t1 in s since the VIO start,
    so the vehicle starts level and the start-FRD frame is too.
    """
    from pnp_odometry import matrix_to_rotvec

    def world(roll, pitch, yaw):
        return (pose_stream.rotvec_to_matrix([0.0, 0.0, yaw]) @ pose_stream.rotvec_to_matrix([0.0, pitch, 0.0])
                @ pose_stream.rotvec_to_matrix([roll, 0.0, 0.0]))

    (roll0, pitch0), (roll1, pitch1) = mock_mode.attitude(t0), mock_mode.attitude(t1)
    return [float(v) for v in matrix_to_rotvec(world(roll0, pitch0, 0.0).T @ world(roll1, pitch1, yaw_delta))]


def fold(value, half):
    """value folded into [-half, half] like a triangle wave: continuous, so the view never jumps."""
    phase = (value + half) % (4.0 * half)
    return (phase if phase <= 2.0 * half else 4.0 * half - phase) - half


class MockLogDevice:
    """Stands in for the camera's log level in mock mode (DeviceLogDebug talks to it)."""

    def __init__(self, device_log):
        self.device_log = device_log
        self.level = "WARN"

    def getLogLevel(self):
        return self.level

    def setLogLevel(self, level):
        self.level = level
        self.say(f"log level {level} (mock: no camera)")

    def say(self, text, node="VideoEncoder(5)", level="DEBUG"):
        self.device_log.on_message(type("LogMessage", (), {"nodeIdName": node, "level": level, "payload": text}))


class MockLinkCounters:
    """Stands in for device.getProfilingData(): the counted stream bytes, MOCK_LINK_OVERHEAD and a background."""

    def __init__(self, streams, clock):
        self.streams, self.clock, self.started = streams, clock, clock()

    def __call__(self):
        elapsed = self.clock() - self.started
        totals = self.streams.totals()
        down = sum(nbytes for name, (_, nbytes) in totals.items() if name not in link_stats.UP_STREAMS)
        up = sum(nbytes for name, (_, nbytes) in totals.items() if name in link_stats.UP_STREAMS)
        return type("ProfilingData", (), {
            "numBytesRead": int(down * (1.0 + MOCK_LINK_OVERHEAD) + MOCK_LINK_BACKGROUND_DOWN * elapsed),
            "numBytesWritten": int(up * (1.0 + MOCK_LINK_OVERHEAD) + MOCK_LINK_BACKGROUND_UP * elapsed)})()


def mock_jpeg(image, quality):
    """What the camera's MJPEG encoder would send for image (bytes)."""
    import cv2
    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode returned no image")
    return jpeg.tobytes()


class MockQueue:
    """The host side of an XLink output queue (what depth_points.PointsMatcher uses); never blocks."""

    def __init__(self):
        self.items = deque()

    def tryGetAll(self):
        items, self.items = list(self.items), deque()
        return items

    def get(self, timeout=None):
        return self.items.popleft() if self.items else None


def slc_rows(depth_at, windows, thresholds, center_thresholds=depth_points.CENTER_THRESHOLDS):
    """
    What the SpatialLocationCalculator returns for these windows (x, y, w, h in depth pixels): per window the
    echoed rect, the depth_points.ALGORITHM depth of every STEPS[ALGORITHM]-th pixel in each direction strictly
    inside the thresholds (the other algorithm's column 0, as the firmware leaves it) and their count. The last
    EXTRA_ROIS windows use center_thresholds (as make_config does). depth_at(u, v): uint16 depth (0 invalid) at
    integer pixel arrays.
    """
    windows = np.asarray(windows, dtype=np.int64).reshape(-1, 4)
    n = len(windows)
    if not n:
        return np.zeros((0, 7))
    step = depth_points.STEPS[depth_points.ALGORITHM]
    steps = np.arange(0, int(max(windows[:, 2].max(), windows[:, 3].max())), step)
    x, y, w, h = (windows[:, k][:, None, None] for k in range(4))
    dx, dy = steps[None, :, None], steps[None, None, :]
    inside = (dx < w) & (dy < h)
    depth = depth_at(np.broadcast_to(x + dx, inside.shape), np.broadcast_to(y + dy, inside.shape)).astype(np.float64)
    lo = np.full(n, float(thresholds[0]))
    hi = np.full(n, float(thresholds[1]))
    extra = min(depth_points.EXTRA_ROIS, n)
    lo[n - extra:], hi[n - extra:] = center_thresholds
    valid = inside & (depth > lo[:, None, None]) & (depth < hi[:, None, None])
    count = valid.sum(axis=(1, 2))
    masked = np.where(valid, depth, np.nan).reshape(n, -1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        if depth_points.ALGORITHM == depth_points.MEDIAN:
            median, average = np.nan_to_num(np.round(np.nanmedian(masked, axis=1))), np.zeros(n)
        else:
            median, average = np.zeros(n), np.nan_to_num(np.nanmean(masked, axis=1))
    return np.column_stack([windows.astype(np.float64), median, average, count])


class MockPointsCamera:
    """
    Points mode's camera side in mock mode (depth_points.device_script + the SpatialLocationCalculator): it holds
    the latest depth frame (its time, pan offset and turbidity) and answers a request for that frame's
    sequence number with what the SLC computes on it (slc_rows), re-tagged with the frame's sequence number and
    timestamp, into `queue`. rois "host": requests are the Pi's tag and ROI config (receive(): a config takes the
    tag before it, as the camera Script does); "camera": the features themselves (frame()). Its status reports
    (size, tagging, stats) go to on_status.
    """

    def __init__(self, scene, roi_source, image_size, thresholds, on_status, rng, link_streams):
        self.scene = scene
        self.roi_source = roi_source
        self.image_size = tuple(image_size)
        self.depth_size = tuple(image_size)
        self.thresholds = thresholds
        self.on_status = on_status
        self.rng = rng
        self.link_streams = link_streams
        self.queue = MockQueue()
        self.held = None
        self.tag = None
        self.tagging_reported = False
        self.sent = self.misses = 0
        self.busy = []

    def status(self, kind, text):
        report = f"{kind}\t0\t{self.sent}\t{self.misses}\t{text}".encode()
        self.link_streams.add("depth_points_status", link_stats.buffer_bytes(len(report)))
        self.on_status(report)

    def start(self):
        self.status("size", f"{self.depth_size[0]} {self.depth_size[1]}")

    def frame(self, sequence, t, offset, turbid, uv):
        self.held = (int(sequence), t, tuple(offset), turbid)
        if self.roi_source == depth_points.ROIS_CAMERA:
            self.answer(depth_points.all_windows(uv, self.image_size, self.depth_size), self.thresholds, sequence)

    def receive(self, message):
        if message[0] == "tag":
            self.tag = message[1]
            return
        _, windows, thresholds = message
        sequence, self.tag = self.tag, None
        if sequence is None:
            return
        if not self.tagging_reported:
            # The camera reads 0 as the config's own sequence number (test16b)
            self.tagging_reported = True
            self.status("tagging", f"{sequence} 0")
        self.answer(windows, thresholds, sequence)

    def answer(self, windows, thresholds, sequence):
        if self.held is None or self.held[0] != int(sequence):
            self.misses += 1
            self.status("miss", f"request {sequence}, depth frame {self.held[0] if self.held else None}")
            return
        _, t, offset, turbid = self.held
        rng = np.random.default_rng(int(t * 1000)) if turbid else None
        start = time.perf_counter()
        rows = slc_rows(lambda u, v: self.scene.depth_mm(u, v, t, offset, turbid, rng), windows, thresholds)
        took = time.perf_counter() - start
        locations = [SimpleNamespace(config=SimpleNamespace(roi=SimpleNamespace(x=r[0], y=r[1], width=r[2], height=r[3])),
                                     depthMedian=r[4], depthAverage=r[5], depthAveragePixelCount=int(r[6]))
                     for r in rows]
        self.queue.items.append(SimpleNamespace(getSequenceNum=lambda s=int(sequence): s,
                                                getTimestamp=lambda t=t: timedelta(seconds=t),
                                                getSpatialLocations=lambda loc=locations: loc))
        self.sent += 1
        self.busy.append(took)
        if self.sent % depth_points.STATS_EVERY == 0:
            busy, self.busy = np.array(self.busy) * 1000.0, []
            self.status("stats", f"0.1 0.2 {busy.mean():.1f} {busy.max():.1f} {self.depth_size[0]} "
                                 f"{self.depth_size[1]}")


class MockCameraLoad:
    """Stands in for the camera's load RPCs (vio_telemetry.read_camera_load); its Leon CPU follows the work."""

    def __init__(self, vio):
        self.vio = vio

    @staticmethod
    def _cpu(value):
        return SimpleNamespace(average=min(max(value + random.gauss(0.0, 0.01), 0.0), 1.0))

    @staticmethod
    def _memory(used, total):
        return SimpleNamespace(used=int(used), total=int(total))

    def getLeonCssCpuUsage(self):
        vio = self.vio
        value = 0.18 + (0.06 if vio.points_mode else 0.0)
        if vio.stream is not None and vio.stream.enabled and not vio.stream.on_host:
            value += 0.08
        return self._cpu(value)

    def getLeonMssCpuUsage(self):
        vio = self.vio
        value = 0.31 + (0.02 if vio.points_mode else 0.0)
        if vio.points_mode and vio.args.depth_points_rois == depth_points.ROIS_CAMERA:
            # test16b: the camera Script's build took 111 ms of Leon MSS per frame of ~255 features
            value += min(0.111 / 255 * vio.args.num_features * vio.args.fps, 0.67)
        return self._cpu(value)

    def getDdrMemoryUsage(self):
        return self._memory(180e6, 343e6)

    def getCmxMemoryUsage(self):
        return self._memory(1.9e6, 2.5e6)

    def getLeonCssHeapUsage(self):
        return self._memory(44e6, 78e6)

    def getLeonMssHeapUsage(self):
        return self._memory(12e6, 46e6)


class Features:
    """Tracked features on the view: ids kept while visible, replaced when lost (like the LK tracker)."""

    def __init__(self, count, width, height, rng):
        self.count, self.width, self.height, self.rng = count, width, height, rng
        self.margin = 4.0
        self.uv = self._spawn(count)
        self.ids = np.arange(count, dtype=np.int64)
        self.next_id = count

    def _spawn(self, n):
        return self.rng.uniform((10, 10), (self.width - 10, self.height - 10), (n, 2))

    def step(self, shift):
        self.uv = self.uv - shift + self.rng.normal(0, 0.3, self.uv.shape)
        lost = ((self.uv < self.margin) | (self.uv > (self.width - self.margin, self.height - self.margin))).any(axis=1)
        lost |= self.rng.random(len(self.uv)) < 0.02
        n = int(lost.sum())
        if n:
            self.uv[lost] = self._spawn(n)
            self.ids = self.ids.copy()
            self.ids[lost] = self.next_id + np.arange(n)
            self.next_id += n


class FakeVio:
    def __init__(self, args, rng, clock=time.time):
        self.args = args
        self.rng = rng
        self.clock = clock
        self.started = clock()
        self.width, self.height = args.width, args.height
        stereo = resolve_stereo_settings(args)
        self.depth_range = (int(stereo["stereo_depth_min_mm"]), int(stereo["stereo_depth_max_mm"]))
        self.stereo = stereo
        self.scene = Scene(self.width, self.height, self.depth_range, rng)
        count = min(args.num_features, args.max_features) if args.max_features > 0 else args.num_features
        self.features = Features(max(count, 1), self.width, self.height, rng)
        self.telemetry_dir = args.telemetry_dir or args.snapshot_dir / "telemetry"
        self.ppm = self.scene.fx / 1.6  # pixels per metre of motion at the mean depth
        self.home = np.array([self.width, self.height], np.float64)
        self.offset = self.home.copy()
        self.reference = None  # (ids, depth valid) of the last good frame
        self.reference_t = None
        self.previous = (None, None)
        self.failed_run = 0
        self.last_ok_t = None
        self.sequence = 0
        self.last_depth = None
        self.stream = None
        self.log_device = None
        self.stream_count = 0
        self.stream_busy = []
        self.stream_sent = 0
        self.snapshot_overlay = None
        # (t, pan offset, overlay builder) of recent frames: a snapshot shows the frame of its request time
        self.frames = deque(maxlen=int(SNAPSHOT_HISTORY_S * max(args.fps, 1)) + 1)
        self.link_stats = link_stats
        self.link = link_stats.LinkMonitor(clock=clock, source=MOCK_LINK_SOURCE, mock=True)
        self.imu_due = 0.0
        self.points_mode = args.depth_transport == depth_points.POINTS
        self.points = self.points_camera = self.gate = None
        self.full_frames = 0

    # --- per-frame model ---

    def turbidity(self, t):
        """Fraction of depth lost: TURBID_S at the end of every FAILURE_PERIOD_S since the start."""
        phase = (t - self.started) % FAILURE_PERIOD_S
        return 0.97 if phase >= FAILURE_PERIOD_S - TURBID_S else 0.0

    def proc_ms(self, t):
        value = 14.0 + 3.0 * math.sin(math.tau * t / 30.0) + self.rng.normal(0, 1.2)
        if self.rng.random() < 0.03:
            value += self.rng.uniform(8, 20)
        return max(value, 3.0)

    def temperature(self):
        # Called on the telemetry publisher thread: the module-level random, not self.rng
        t = self.clock()
        return 46.0 + 3.0 * math.sin(math.tau * t / 240.0) + random.gauss(0.0, 0.15)

    def solve(self, t):
        """One frame of fake PnP: feature states, the result and its reason."""
        ids, uv = self.features.ids, self.features.uv
        depth = self.scene.depth_mm(uv[:, 0], uv[:, 1], t, self.offset, self.turbidity(t), self.rng)
        valid = depth > 0
        matched_index = inlier_index = None
        if self.reference is None:
            ok, reason, matches, inliers = False, "no_reference", 0, 0
        else:
            ref_ids, ref_valid = self.reference
            in_reference = np.isin(ids, ref_ids[ref_valid])
            matched_index = np.flatnonzero(in_reference)
            matches = len(matched_index)
            if matches < 6:
                ok, reason, inliers = False, "too_few_matches", 0
            else:
                weight, _ = self.scene.blob_weight(uv[matched_index, 0], uv[matched_index, 1], t)
                outlier = self.rng.random(matches) < 0.05 + 0.9 * weight ** 2
                inlier_index = np.flatnonzero(~outlier)
                inliers = len(inlier_index)
                if inliers < self.args.min_inliers:
                    ok, reason = False, "too_few_inliers"
                elif self.rng.random() < 0.01:
                    ok, reason = False, "gyro_mismatch"
                else:
                    ok, reason = True, None
        return valid, matched_index, inlier_index, ok, reason, matches, inliers

    def step(self, t, dt):
        vx, vy, vz, _ = mock_mode.motion(t)
        x, y = self.telemetry.trajectory.x, self.telemetry.trajectory.y
        target = self.scene.clamp_offset(self.home[0] + fold(y * self.ppm, self.width - 1),
                                         self.home[1] - fold(x * self.ppm, self.height - 1))
        shift = np.array(target) - self.offset
        self.offset = np.array(target)
        self.previous = (self.features.ids, self.features.uv)
        self.features.step(shift)
        valid, matched_index, inlier_index, ok, reason, matches, inliers = self.solve(t)
        ids, uv = self.features.ids, self.features.uv

        gap_frames, position_delta, yaw_delta, delta_t, reference_reset = 0, None, None, None, False
        rotation_delta = None
        if ok:
            delta_t = t - self.last_ok_t if self.last_ok_t is not None else dt
            gap_frames = self.failed_run
            position_delta = [v * delta_t + self.rng.normal(0, 0.002) for v in (vx, vy, vz)]
            # The true turn since the reference, as the PnP measures it across a bridged gap too
            yaw_delta = mock_mode.heading(t) - mock_mode.heading(t - delta_t)
            yaw_delta = (yaw_delta + math.pi) % (2.0 * math.pi) - math.pi
            rotation_delta = body_rotation_delta(t - delta_t - self.started, t - self.started, yaw_delta)
            rotation_delta[2] += mock_mode.GYRO_BIAS * delta_t
            self.failed_run = 0
            self.last_ok_t = t
        elif reason != "no_reference":
            self.failed_run += 1
            if self.args.recovery_frames and self.failed_run >= self.args.recovery_frames:
                self.telemetry.record_recovery()
                self.data_log.event("recovery", count=self.telemetry.recoveries, failed_frames=self.failed_run,
                                    reason=reason, mock=True)
                self.failed_run, self.reference, self.last_ok_t, reference_reset = 0, None, None, True
            elif self.args.max_reference_age > 0 and t - self.reference_t > self.args.max_reference_age:
                self.reference, self.reference_t, self.last_ok_t, reference_reset = (ids, valid), t, t, True
        if ok or reason == "no_reference":
            self.reference = (ids, valid)
            self.reference_t = self.last_ok_t = t
        return {"ids": ids, "uv": uv, "valid": valid, "matched_index": matched_index, "inlier_index": inlier_index,
                "ok": ok, "reason": reason, "matches": matches, "inliers": inliers, "gap_frames": gap_frames,
                "position_delta": position_delta, "yaw_delta": yaw_delta, "rotation_delta": rotation_delta,
                "dt": delta_t, "reference_reset": reference_reset}

    # --- camera link ---

    def start_link(self):
        self.link.set_device(MockLinkCounters(self.link.streams, self.clock), usb_speed="SUPER", protocol="USB",
                             protocol_raw="X_LINK_USB_VSC", platform="RVC2 (mock)")

    def count_link_frame(self, features):
        """What one VIO frame takes from the host queues (pnp_vio's counts), and the IMU messages since the last."""
        streams = self.link.streams
        streams.add("features", link_stats.features_bytes(features))
        if not self.points_mode:
            streams.add("depth", link_stats.frame_bytes(self.width * self.height * 2))
            streams.add("sync_group", link_stats.MESSAGE_GROUP_OVERHEAD)
        self.imu_due += IMU_RATE_HZ / max(self.args.fps, 1)
        messages = int(self.imu_due)
        if messages:
            self.imu_due -= messages
            streams.add("imu", messages * link_stats.imu_bytes(1), messages)

    # --- depth transport ---

    def start_points(self):
        """points: the real DepthPointsReceiver (pnp_vio.PointsSource's settings) and FullDepthGate, on MockPointsCamera."""
        args, streams = self.args, self.link.streams
        thresholds = depth_points.thresholds_mm(args.min_depth, args.max_depth)
        host = args.depth_points_rois == depth_points.ROIS_HOST

        def send(message):
            self.points_camera.receive(message)

        def counted(rows):
            streams.add("depth_points", link_stats.spatial_data_bytes(0 if rows is None else len(rows)))

        def send_gate_control(every):
            streams.add("control", link_stats.BUFFER_OVERHEAD)

        self.points_camera = MockPointsCamera(self.scene, args.depth_points_rois, (self.width, self.height),
                                              thresholds, lambda data: self.points.on_status(data), self.rng, streams)
        self.points = depth_points.DepthPointsReceiver(
            self.points_camera.queue, (self.width, self.height), args.depth_points_rois,
            wait_s=depth_points.wait_s(args.fps, args.depth_points_rois), send=send if host else None,
            build_config=lambda windows, limits, sequence: ("config", windows, limits),
            build_tag=lambda sequence: ("tag", sequence), thresholds=thresholds,
            frame_period_s=1.0 / max(args.fps, 1), log=print, on_message=counted)
        self.gate = depth_points.FullDepthGate(send_gate_control, clock=self.clock, log=print)
        self.points_camera.start()

    def points_frame(self, t, frame, sequence):
        """points: this frame's depth at its features (DepthPoints or None) and its validity in frame["valid"]."""
        uv = frame["uv"]
        self.points_camera.frame(sequence, t, self.offset, self.turbidity(t), uv)
        requested = self.points.request(sequence, uv)
        if requested and self.args.depth_points_rois == depth_points.ROIS_HOST:
            self.link.streams.add("depth_points_rois",
                                  link_stats.points_request_bytes(len(uv) + depth_points.EXTRA_ROIS))
        points, problem = self.points.take(sequence, uv, t, requested=requested)
        frame["valid"] = points.depth_mm > 0 if points is not None else np.zeros(len(uv), bool)
        frame["depth_problem"] = problem
        return points

    def full_depth(self, t, sequence, needs):
        """points: a full depth frame if the gate forwards this one (counted as depth_on_demand), else None."""
        every = self.gate.update(needs)
        if not every or sequence % every:
            return None
        nbytes = link_stats.frame_bytes(self.width * self.height * 2)
        self.link.streams.add("depth_on_demand", nbytes)
        self.gate.on_frame(nbytes)
        return self.scene.depth_image(t, self.offset, self.turbidity(t))

    def depth_transport_status(self):
        if self.points is not None:
            return {**self.points.status(), "full_depth": self.gate.status(), "mock": True}
        return {"transport": depth_points.FULL_FRAME, "frames": self.full_frames, "feature_gaps": 0,
                "feature_gap_frames": 0, "full_depth": None, "mock": True}

    # --- services ---

    def camera_status(self):
        k = self.scene.intrinsics
        n, scale = self.args.refractive_index, self.args.depth_scale
        return {"pid": os.getpid(), "unix_time": self.clock(), "image_size": [self.width, self.height],
                "source": "rectified", "requested_source": "rectified", "intrinsics": k,
                "effective_intrinsics": [k[0] * n, k[1] * n, k[2], k[3]], "raw": k, "rectified": k, "frame": k,
                "frame_matches": True, "override": None, "alpha": self.stereo.get("stereo_alpha"),
                "distortion_correction": self.stereo.get("stereo_distortion_correction"),
                "baseline_m": BASELINE_M, "baseline_spec_m": BASELINE_M, "depth_scale": scale,
                "refractive_index": n, "total_depth_scale": scale * n, "depth_range_mm": list(self.depth_range),
                "warnings": ["mock mode: synthetic camera"], "mock": True}

    def frame_at(self, requested_at):
        """(t, offset, overlay builder, sequence) of the first recent frame at or after requested_at, else the newest."""
        for entry in self.frames:
            if entry[0] >= requested_at:
                return entry
        return self.frames[-1]

    def depth_stream_info(self):
        every = depth_stream.stream_divisor(self.args.fps, self.args.depth_stream_fps)
        min_mm, max_mm = self.depth_range
        requested = self.args.depth_stream_encoding
        problem = depth_stream.size_problem(self.width, self.height)
        fmt = disparity_format(None, self.stereo)
        disparity_problem = depth_stream.disparity_problem(fmt)
        variant, reason = depth_stream.resolve_variant(requested, disparity_problem, problem)
        info = {"encoding": depth_stream.encoding_for(variant), "codec": "mjpeg",
                "fps_target": self.args.fps / every, "every": every, "camera_fps": self.args.fps,
                "quality": self.args.depth_stream_quality, "image_size": [self.width, self.height],
                "depth_range_mm": [min_mm, max_mm], "quantization_mm": 1 << depth_stream.depth_shift(max_mm),
                "gray_far": depth_stream.GRAY_FAR, "gray_near": depth_stream.GRAY_NEAR,
                "gray_invalid_below": depth_stream.GRAY_INVALID_BELOW, "requested_variant": requested,
                "variant": variant or depth_stream.HOST_JPEG, "variant_reason": reason, "size_problem": problem,
                "source": depth_stream.source_for(variant or depth_stream.HOST_JPEG), "disparity_format": fmt,
                "disparity_problem": disparity_problem, "mock": True,
                "depth_frames": "on_demand" if self.points_mode else "every_frame"}
        if variant == depth_stream.DISPARITY:
            info["disparity_max"] = int(fmt["max_disparity"])
            info["disparity"] = depth_stream.disparity_scale(self.scene.fx, BASELINE_M, min_mm, max_mm,
                                                             info["disparity_max"])
        return info

    def make_depth_stream(self):
        info = self.depth_stream_info()
        host = device_log = debug = None
        if info["variant"] == depth_stream.HOST_JPEG or info["requested_variant"] == depth_stream.AUTO:
            # points: the gate forwards every n-th frame already
            host = depth_stream.HostJpegEncoder(*self.depth_range, self.args.depth_stream_quality,
                                                1 if self.points_mode else info["every"])
        if info["variant"] != depth_stream.HOST_JPEG:
            names = ("Script(4)", "VideoEncoder(5)")
            if info["variant"] == depth_stream.MANIP:
                names += ("ImageManip(6)",)
            device_log = depth_stream.DeviceLog(names)
            if self.args.depth_stream_debug:
                self.log_device = MockLogDevice(device_log)
                debug = depth_stream.DeviceLogDebug(self.log_device, "DEBUG", log=print)
        self.stream = depth_stream.DepthStreamService(self.telemetry_dir, lambda on: None, info, log=print,
                                                      host_encoder=host, device_log=device_log, debug=debug)
        return self.stream

    def encode_depth(self, t):
        """
        What the camera's Script + VideoEncoder produce (mono_features: the synthetic rectified-left image the
        mock features are on), then the device's timing report every STATS_EVERY.
        """
        import cv2
        start = time.perf_counter()
        depth = self.scene.depth_image(t, self.offset, self.turbidity(t))
        if self.stream.variant == depth_stream.MONO_FEATURES:
            gray = self.scene.frame("rect_left", t, self.offset)
        elif self.stream.variant == depth_stream.DISPARITY:
            gray = depth_stream.gray_disparity_reference(depth, self.scene.fx, BASELINE_M,
                                                         self.stream.info.get("disparity_max",
                                                                              depth_stream.DISPARITY_MAX))
        else:
            gray = depth_stream.gray_reference(depth, *self.depth_range)
        ok, jpeg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, int(self.args.depth_stream_quality)])
        self.last_depth = depth
        if not ok:
            return None
        self.link.streams.add("depth_stream", link_stats.encoded_bytes(len(jpeg)))
        took = time.perf_counter() - start
        self.stream_busy.append(took)
        self.stream_sent += 1
        if self.log_device is not None and self.log_device.level == "DEBUG":
            self.log_device.say(f"VideoEncoder took '{1000 * took:.1f}' ms. (mock)")
        if self.stream_sent % STATS_EVERY == 0:
            busy, self.stream_busy = self.stream_busy, []
            report = (f"stats\t0\t{self.stream_sent}\t{1000 * sum(busy) / len(busy):.1f} {1000 * max(busy):.1f} "
                      f"{self.width} {self.height}").encode()
            self.link.streams.add("depth_stream_status", link_stats.buffer_bytes(len(report)))
            self.stream.on_device_status(report)
        return jpeg.tobytes()

    def run(self, stop):
        args = self.args
        self.data_log = vio_log.open_vio_log(args.log_dir)
        self.data_log.event("start", algorithm="pnp", mock=True, args=vars(args), stereo=self.stereo,
                            log_only=args.log_only)
        self.telemetry = vio_telemetry.Telemetry(self.telemetry_dir, fps=args.fps, min_inliers=args.min_inliers,
                                                 mode="log-only" if args.log_only else "sending",
                                                 temperature=self.temperature, log=print, link=self.link,
                                                 camera_load=lambda: vio_telemetry.read_camera_load(MockCameraLoad(self)),
                                                 depth_points_status=self.depth_transport_status)
        writer = stream = None
        try:
            self.telemetry.set_phase("opening_device")
            self.telemetry.start()
            stop.wait(STARTUP_PHASE_S)
            self.start_link()
            if self.points_mode:
                self.start_points()
            self.data_log.event("depth_transport", transport=args.depth_transport,
                                rois=args.depth_points_rois if self.points_mode else None, mock=True)
            self.telemetry.set_phase("configuring")
            writer = snapshots.SnapshotWriter(args.snapshot_dir, self.depth_range, log=print)
            pending = []
            camera_jpeg = args.snapshot_encoding == snapshots.CAMERA_JPEG
            snapshot_service = snapshots.SnapshotService(
                ("rect_left", "rect_right", "depth") if args.snapshots else (),
                lambda stream, raw: pending.append(raw), writer,
                args.snapshot_dir, unavailable={"rgb": "mock mode: the synthetic camera has no CAM_A (RGB)"}
                if args.snapshots else {s: "snapshots are disabled (--no-snapshots)" for s in snapshots.STREAMS},
                log=print, overlay=lambda: self.snapshot_overlay,
                encoded=("rect_left", "rect_right") if camera_jpeg else (),
                not_encoded={} if camera_jpeg else {s: "Snapshot Encoding is raw" for s in ("rect_left", "rect_right")})
            probe = depth_probe.DepthProbeService(args.snapshot_dir / "depth_probe", self.depth_range,
                                                  args.depth_scale, (self.width, self.height), log=print,
                                                  refractive_index=args.refractive_index)
            from pnp_odometry import Intrinsics
            probe.set_intrinsics(Intrinsics(*self.scene.intrinsics), "rectified")
            if args.depth_stream:
                self.stream = stream = self.make_depth_stream()
            features_out = feature_stream.FeatureStreamService(self.telemetry_dir, (self.width, self.height),
                                                               log=print)
            attitude = None
            if args.pose_attitude_source == pose_stream.AUTOPILOT:
                attitude = autopilot_attitude.AttitudeSource(
                    mock_mode.attitude_fetch(self.started, clock=self.clock), clock=self.clock, log=print,
                    description=f"mock autopilot ATTITUDE ({mock_mode.attitude_mode()})").start()
            self.pose_out = pose_stream.PoseStreamService(self.telemetry_dir, args.camera_angle, log=print,
                                                          attitude=attitude)
            self.telemetry.pose_status = self.pose_out.status
            snapshots.write_camera_status(self.camera_status(), args.snapshot_dir)
            stop.wait(STARTUP_PHASE_S)
            self.telemetry.set_phase("running")
            print(f"✅ Mock PnP VIO running: {self.width}x{self.height} @ {args.fps} fps, synthetic camera", flush=True)
            self.loop(stop, snapshot_service, pending, probe, stream, features_out)
        finally:
            if stream is not None:
                stream.close()
            if getattr(self, "pose_out", None) is not None:
                self.pose_out.close()
            if writer is not None:
                writer.close()
            self.telemetry.close()
            snapshots.clear_camera_status(args.snapshot_dir)
            self.data_log.event("exit", exit_code=0)
            self.data_log.close()

    def loop(self, stop, snapshot_service, pending, probe, stream, features_out):
        args = self.args
        interval = 1.0 / max(args.fps, 1)
        every = depth_stream.stream_divisor(args.fps, args.depth_stream_fps)
        next_time = self.clock()
        parent = os.getppid()
        while not stop.is_set():
            if os.getppid() != parent:
                print("Mock VIO: the web server is gone, exiting", flush=True)
                return
            t = self.clock()
            self.sequence += 1
            frame = self.step(t, interval)
            sequence = self.sequence
            points = self.points_frame(t, frame, sequence) if self.points_mode else None
            self.full_frames += not self.points_mode
            built = (frame["ids"], frame["uv"], frame["matched_index"], frame["inlier_index"], *self.previous,
                     sequence, (self.width, self.height), frame["ok"], frame["reason"])
            self.frames.append((t, tuple(self.offset), lambda values=built: snapshots.build_overlay(*values), sequence))
            features_out.tick(frame["uv"], (self.height, self.width), sequence, frame["ok"], frame["reason"],
                              frame["valid"], frame["matched_index"], frame["inlier_index"])
            self.count_link_frame(len(frame["ids"]))

            snapshot_service.tick()
            turbid = self.turbidity(t)
            while pending:  # the "device" answers each request with the frame of its request time
                raw = pending.pop(0)
                self.link.streams.add("control", link_stats.BUFFER_OVERHEAD)
                stream_name, requested_at = snapshot_service.pending[0], snapshot_service.pending[1]
                shot_t, offset, self.snapshot_overlay, shot_sequence = self.frame_at(requested_at)
                image = self.scene.frame(stream_name, shot_t, offset, self.turbidity(shot_t))
                if raw:
                    nbytes = link_stats.frame_bytes(image.nbytes)
                else:  # the camera's MJPEG encoder
                    image = mock_jpeg(image, snapshots.JPEG_QUALITY)
                    nbytes = link_stats.encoded_bytes(len(image))
                self.link.streams.add("snapshot", nbytes)
                snapshot_service.on_frame(lambda s, image=image: image, shot_t, shot_sequence, encoded=not raw,
                                          nbytes=nbytes)
            probe.tick()
            if self.points_mode:
                # Full depth frames only through the gate (the probe: every frame; host_jpeg: every n-th)
                watched_host_jpeg = stream is not None and stream.on_host and stream.enabled
                full = self.full_depth(t, sequence, {"depth_probe": 1 if probe.active is not None else 0,
                                                     "host_jpeg": every if watched_host_jpeg else 0})
                if probe.active is not None and full is not None:
                    probe.on_depth(full, t, sequence)
                center, sample = (points.center, depth_points.depth_sample(points)) if points else (None, None)
                if stream is not None:
                    stream.tick(full if stream.on_host else None, sequence if full is not None else None,
                                center=center, sample=sample)
            else:
                if probe.active is not None:
                    probe.on_depth(self.scene.depth_image(t, self.offset, turbid), t, sequence)
                if stream is not None and stream.on_host:
                    if stream.enabled:
                        self.last_depth = self.scene.depth_image(t, self.offset, turbid)
                    stream.tick(self.last_depth, sequence)
                elif stream is not None:
                    stream.tick(self.last_depth)
            if stream is not None and not stream.on_host:
                if stream.enabled:
                    self.stream_count += 1
                    if self.stream_count >= every:
                        self.stream_count = 0
                        jpeg = self.encode_depth(t)
                        if (jpeg is not None and stream.on_frame(jpeg, sequence)
                                and stream.variant == depth_stream.MONO_FEATURES):
                            features_out.publish_for(sequence)
                else:
                    self.stream_count = every - 1

            sent = frame["ok"] and not args.log_only
            proc_ms = self.proc_ms(t)
            valid_depth = int(np.count_nonzero(frame["valid"]))
            self.data_log.step(sequence=sequence, time_delta_us=None if frame["dt"] is None else frame["dt"] * 1e6,
                               position_delta=frame["position_delta"], features=len(frame["ids"]),
                               valid_depth=valid_depth, matches=frame["matches"], inliers=frame["inliers"],
                               proc_ms=proc_ms, status="ok" if frame["ok"] else "rejected", reason=frame["reason"],
                               sent=sent, gap_frames=frame["gap_frames"], reference_reset=frame["reference_reset"],
                               depth_source="points" if self.points_mode else "frame",
                               depth_problem=frame.get("depth_problem"), mock=True)
            self.telemetry.record_frame(t, frame["ok"], frame["reason"], len(frame["ids"]), valid_depth,
                                        frame["matches"], frame["inliers"], proc_ms, frame["dt"],
                                        frame["position_delta"], frame["yaw_delta"], frame["gap_frames"],
                                        frame["reference_reset"], sent)
            self.pose_out.record(t, frame["ok"], frame["position_delta"], frame["rotation_delta"])
            next_time += interval
            now = self.clock()
            if next_time < now - 1.0:
                next_time = now  # fell far behind (suspended): don't race to catch up
            stop.wait(max(0.0, next_time - now))


def main(argv=None):
    import threading
    args = parse_args(argv)
    stop = threading.Event()

    def on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    rng = np.random.default_rng(args.seed)
    FakeVio(args, rng).run(stop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
