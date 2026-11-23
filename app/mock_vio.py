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

Published through the real services, at the configured fps:
- vio_telemetry.Telemetry: status.json, history.npy (lease-gated), the chip temperature;
- depth_stream.DepthStreamService: depth_stream.jpg (8-bit depth as the camera's Script encodes it, JPEG at
  --depth-stream-quality) every round(fps / depth-stream-fps)-th frame, only while the lease is held;
- feature_stream.FeatureStreamService: features.json while its lease is held;
- snapshots.SnapshotService and depth_probe.DepthProbeService; camera.json;
- a JSONL data log under --log-dir.
Nothing is sent anywhere. --algorithm basalt only idles, as Basalt publishes none of this.
"""

import argparse
import math
import os
import random
import signal
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

import depth_probe
import depth_stream
import feature_stream
import mock_mode
import snapshots
import vio_log
import vio_telemetry
from stereo_config import add_stereo_arguments, resolve_stereo_settings

BASELINE_M = 0.075
# fx at 640 px wide, scaled with the width
FOCAL_AT_640 = 400.0
FAILURE_PERIOD_S = 50.0
TURBID_S = 2.5
STARTUP_PHASE_S = 0.3
STATS_EVERY = 150
SNAPSHOT_HISTORY_S = 10.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Fake PnP VIO (mock mode)", allow_abbrev=False)
    parser.add_argument("--algorithm", choices=("pnp", "basalt"), default="pnp")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--num-features", type=int, default=300)
    parser.add_argument("--min-inliers", type=int, default=15)
    parser.add_argument("--recovery-frames", type=int, default=20)
    parser.add_argument("--max-reference-age", type=float, default=1.0)
    parser.add_argument("--snapshots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--depth-stream", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--depth-stream-fps", type=float, default=5.0)
    parser.add_argument("--depth-stream-quality", type=int, default=70)
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
        self.features = Features(max(args.num_features, 1), self.width, self.height, rng)
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
        self.stream_count = 0
        self.stream_busy = []
        self.stream_sent = 0
        self.snapshot_overlay = None
        # (t, pan offset, overlay builder) of recent frames: a snapshot shows the frame of its request time
        self.frames = deque(maxlen=int(SNAPSHOT_HISTORY_S * max(args.fps, 1)) + 1)

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
        vx, vy, vz, yaw_rate = mock_mode.motion(t)
        x, y = self.telemetry.trajectory.x, self.telemetry.trajectory.y
        target = self.scene.clamp_offset(self.home[0] + y * self.ppm, self.home[1] - x * self.ppm)
        shift = np.array(target) - self.offset
        self.offset = np.array(target)
        self.previous = (self.features.ids, self.features.uv)
        self.features.step(shift)
        valid, matched_index, inlier_index, ok, reason, matches, inliers = self.solve(t)
        ids, uv = self.features.ids, self.features.uv

        gap_frames, position_delta, yaw_delta, delta_t, reference_reset = 0, None, None, None, False
        if ok:
            delta_t = t - self.last_ok_t if self.last_ok_t is not None else dt
            gap_frames = self.failed_run
            position_delta = [v * delta_t + self.rng.normal(0, 0.002) for v in (vx, vy, vz)]
            yaw_delta = yaw_rate * delta_t
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
                "position_delta": position_delta, "yaw_delta": yaw_delta, "dt": delta_t,
                "reference_reset": reference_reset}

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
        return {"encoding": "gray8-depth-linear", "codec": "mjpeg", "fps_target": self.args.fps / every,
                "every": every, "camera_fps": self.args.fps, "quality": self.args.depth_stream_quality,
                "image_size": [self.width, self.height], "depth_range_mm": [min_mm, max_mm],
                "quantization_mm": 1 << depth_stream.depth_shift(max_mm), "gray_far": depth_stream.GRAY_FAR,
                "gray_near": depth_stream.GRAY_NEAR, "gray_invalid_below": depth_stream.GRAY_INVALID_BELOW,
                "mock": True}

    def encode_depth(self, t):
        """What the camera's Script + VideoEncoder produce, then the device's timing report every STATS_EVERY."""
        import cv2
        start = time.perf_counter()
        depth = self.scene.depth_image(t, self.offset, self.turbidity(t))
        gray = depth_stream.gray_reference(depth, *self.depth_range)
        ok, jpeg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, int(self.args.depth_stream_quality)])
        self.last_depth = depth
        if not ok:
            return None
        self.stream_busy.append(time.perf_counter() - start)
        self.stream_sent += 1
        if self.stream_sent % STATS_EVERY == 0:
            busy, self.stream_busy = self.stream_busy, []
            self.stream.on_device_status(
                f"stats\t0\t{self.stream_sent}\t{1000 * sum(busy) / len(busy):.1f} {1000 * max(busy):.1f} "
                f"{self.width} {self.height}".encode())
        return jpeg.tobytes()

    def run(self, stop):
        args = self.args
        self.data_log = vio_log.open_vio_log(args.log_dir)
        self.data_log.event("start", algorithm="pnp", mock=True, args=vars(args), stereo=self.stereo,
                            log_only=args.log_only)
        self.telemetry = vio_telemetry.Telemetry(self.telemetry_dir, fps=args.fps, min_inliers=args.min_inliers,
                                                 mode="log-only" if args.log_only else "sending",
                                                 temperature=self.temperature, log=print)
        writer = stream = None
        try:
            self.telemetry.set_phase("opening_device")
            self.telemetry.start()
            stop.wait(STARTUP_PHASE_S)
            self.telemetry.set_phase("configuring")
            writer = snapshots.SnapshotWriter(args.snapshot_dir, self.depth_range, log=print)
            pending = []
            snapshot_service = snapshots.SnapshotService(
                ("rect_left", "rect_right", "depth") if args.snapshots else (), pending.append, writer,
                args.snapshot_dir, unavailable={"rgb": "mock mode: the synthetic camera has no CAM_A (RGB)"}
                if args.snapshots else {s: "snapshots are disabled (--no-snapshots)" for s in snapshots.STREAMS},
                log=print, overlay=lambda: self.snapshot_overlay)
            probe = depth_probe.DepthProbeService(args.snapshot_dir / "depth_probe", self.depth_range,
                                                  args.depth_scale, (self.width, self.height), log=print,
                                                  refractive_index=args.refractive_index)
            from pnp_odometry import Intrinsics
            probe.set_intrinsics(Intrinsics(*self.scene.intrinsics), "rectified")
            if args.depth_stream:
                self.stream = stream = depth_stream.DepthStreamService(self.telemetry_dir, lambda on: None,
                                                                       self.depth_stream_info(), log=print)
            features_out = feature_stream.FeatureStreamService(self.telemetry_dir, (self.width, self.height),
                                                               log=print)
            snapshots.write_camera_status(self.camera_status(), args.snapshot_dir)
            stop.wait(STARTUP_PHASE_S)
            self.telemetry.set_phase("running")
            print(f"✅ Mock PnP VIO running: {self.width}x{self.height} @ {args.fps} fps, synthetic camera", flush=True)
            self.loop(stop, snapshot_service, pending, probe, stream, features_out)
        finally:
            if stream is not None:
                stream.close()
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
            built = (frame["ids"], frame["uv"], frame["matched_index"], frame["inlier_index"], *self.previous,
                     sequence, (self.width, self.height), frame["ok"], frame["reason"])
            self.frames.append((t, tuple(self.offset), lambda values=built: snapshots.build_overlay(*values), sequence))
            features_out.tick(frame["uv"], (self.height, self.width), sequence, frame["ok"], frame["reason"],
                              frame["valid"], frame["matched_index"], frame["inlier_index"])

            snapshot_service.tick()
            turbid = self.turbidity(t)
            while pending:  # the "device" answers each request with the frame of its request time
                pending.pop(0)
                shot_t, offset, self.snapshot_overlay, shot_sequence = self.frame_at(snapshot_service.pending[1])
                snapshot_service.on_frame(lambda s, t=shot_t, o=offset: self.scene.frame(s, t, o, self.turbidity(t)),
                                          shot_t, shot_sequence)
            probe.tick()
            if probe.active is not None:
                probe.on_depth(self.scene.depth_image(t, self.offset, turbid), t, sequence)
            if stream is not None:
                stream.tick(self.last_depth)
                if stream.enabled:
                    self.stream_count += 1
                    if self.stream_count >= every:
                        self.stream_count = 0
                        jpeg = self.encode_depth(t)
                        if jpeg is not None:
                            stream.on_frame(jpeg, sequence)
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
                               mock=True)
            self.telemetry.record_frame(t, frame["ok"], frame["reason"], len(frame["ids"]), valid_depth,
                                        frame["matches"], frame["inliers"], proc_ms, frame["dt"],
                                        frame["position_delta"], frame["yaw_delta"], frame["gap_frames"],
                                        frame["reference_reset"], sent)
            next_time += interval
            now = self.clock()
            if next_time < now - 1.0:
                next_time = now  # fell far behind (suspended): don't race to catch up
            stop.wait(max(0.0, next_time - now))


def idle(stop):
    """Basalt in mock mode: a running process that publishes nothing, like basalt_configurable.py."""
    print("Mock Basalt VIO: idle (mock mode publishes live data for PnP only)", flush=True)
    parent = os.getppid()
    while not stop.wait(0.5):
        if os.getppid() != parent:
            return


def main(argv=None):
    import threading
    args = parse_args(argv)
    stop = threading.Event()

    def on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    if args.algorithm == "basalt":
        idle(stop)
        return 0
    rng = np.random.default_rng(args.seed)
    FakeVio(args, rng).run(stop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
