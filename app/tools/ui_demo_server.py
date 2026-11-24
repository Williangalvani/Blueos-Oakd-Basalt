#!/usr/bin/env python3
"""
Development server for the web UI: the real FastAPI app (main.py) with everything outside it faked, so the
page can be developed and screenshotted on a laptop. It never contacts a real host.

- VIO: a recorded PnP data log (JSONL) is replayed at real time into a real vio_telemetry.Telemetry (the
  same ring buffer, failure window and lease-gated publisher pnp_vio.py uses); the first --prefill seconds
  are replayed at once so the plots start full. The supervisor is a stand-in: Start/Stop/Reset VIO restart
  the replay.
- Camera: generated rect_left / rect_right / depth frames served through the real SnapshotService (with the
  feature overlay) and DepthProbeService; camera.json from the log's intrinsics event.
- mavlink2rest: tools/fake_mavlink2rest.py on 127.0.0.1 (an autopilot with parameters, HEARTBEAT, and a
  DVL whose VISION_POSITION_DELTA follows the replayed velocity plus noise). --armed arms it.
- Everything lives in a temporary directory; urllib refuses any host but 127.0.0.1.

    python app/tools/ui_demo_server.py --log ~/oakd-stage1/flight3/vio-20261118-190503-10.jsonl
    open http://127.0.0.1:8099/
"""

import argparse
import json
import math
import os
import random
import shutil
import signal
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import depth_probe  # noqa: E402
import snapshots  # noqa: E402
import vio_log  # noqa: E402
import vio_telemetry  # noqa: E402
from fake_mavlink2rest import FakeVehicle, serve  # noqa: E402

DEFAULT_LOG = Path.home() / "oakd-stage1/flight3/vio-20261118-190503-10.jsonl"
WIDTH, HEIGHT = 640, 400


def guard_network():
    """Only 127.0.0.1 may be reached through urllib (the fake mavlink2rest)."""
    real = urllib.request.urlopen

    def urlopen(request, *args, **kwargs):
        url = request if isinstance(request, str) else request.full_url
        host = urllib.parse.urlparse(url).hostname
        if host not in ("127.0.0.1", "localhost"):
            raise urllib.error.URLError(f"ui_demo_server: refusing to contact {host}")
        return real(request, *args, **kwargs)

    urllib.request.urlopen = urlopen


def load_log(path):
    steps, events = [], {}
    with open(path) as f:
        for line in f:
            record = json.loads(line)
            if record.get("type") == "step":
                steps.append(record)
            elif record.get("type") == "event":
                events.setdefault(record.get("event"), record)
    if not steps:
        raise SystemExit(f"{path}: no step records")
    return steps, events


# --- generated camera frames ----------------------------------------------------------------------

def make_scene(seed=3):
    """A seabed-like texture (blurred noise, sand ripples, a few rocks) larger than the view, for panning."""
    import cv2
    rng = np.random.default_rng(seed)
    big = (2 * WIDTH, 2 * HEIGHT)
    noise = rng.normal(0, 1, (big[1], big[0])).astype(np.float32)
    texture = cv2.GaussianBlur(noise, (0, 0), 3) * 22 + cv2.GaussianBlur(noise, (0, 0), 12) * 60
    yy, xx = np.mgrid[0:big[1], 0:big[0]].astype(np.float32)
    texture += 14 * np.sin(xx / 9.0 + 0.6 * np.sin(yy / 40.0))
    image = np.clip(texture + 105, 0, 255).astype(np.uint8)
    for _ in range(60):
        centre = (int(rng.integers(0, big[0])), int(rng.integers(0, big[1])))
        radius = int(rng.integers(6, 26))
        shade = int(rng.integers(40, 200))
        cv2.circle(image, centre, radius, shade, -1, cv2.LINE_AA)
        cv2.circle(image, (centre[0] - radius // 3, centre[1] - radius // 3), max(radius // 3, 2),
                   min(shade + 40, 255), -1, cv2.LINE_AA)
    return image


class Camera:
    """Frames that pan with the replayed trajectory, a sloped depth plane with holes, and tracked features."""

    def __init__(self, seed=3):
        self.scene = make_scene(seed)
        self.rng = np.random.default_rng(seed + 1)
        self.offset = np.array([WIDTH / 2, HEIGHT / 2])
        self.features = self.rng.uniform((20, 20), (WIDTH - 20, HEIGHT - 20), (220, 2))
        self.ids = np.arange(len(self.features))
        self.previous = None
        self.lock = threading.Lock()

    def move(self, x, y):
        # body forward (x) moves the image down, right (y) moves it left; ~300 px per metre at 1.5 m
        with self.lock:
            new = np.array([WIDTH / 2 + y * 300.0, HEIGHT / 2 - x * 300.0]) % (np.array(self.scene.shape[::-1]) -
                                                                                np.array([WIDTH, HEIGHT]))
            shift = new - self.offset
            self.offset = new
            self.previous = (self.ids.copy(), self.features.copy())
            self.features = self.features - shift + self.rng.normal(0, 0.4, self.features.shape)
            lost = ((self.features < 5) | (self.features > (WIDTH - 5, HEIGHT - 5))).any(axis=1)
            lost |= self.rng.random(len(self.features)) < 0.03
            count = int(lost.sum())
            if count:
                self.features[lost] = self.rng.uniform((20, 20), (WIDTH - 20, HEIGHT - 20), (count, 2))
                self.ids[lost] = self.ids.max() + 1 + np.arange(count)

    def frame(self, stream):
        with self.lock:
            x0, y0 = int(self.offset[0]), int(self.offset[1])
            view = self.scene[y0:y0 + HEIGHT, x0:x0 + WIDTH]
            if stream == "rect_left":
                return view.copy()
            if stream == "rect_right":
                return self.scene[y0:y0 + HEIGHT, x0 + 24:x0 + 24 + WIDTH].copy()
            yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
            depth = (1450 + 0.9 * (yy - HEIGHT / 2) + 0.3 * (xx - WIDTH / 2)).astype(np.uint16)
            depth[view < 60] = 0  # dark rock shadows: no stereo match
            depth[:, :12] = 0
            return depth

    def overlay(self, matches, inliers, ok, reason, sequence):
        """Overlay builder for the last frame: the replayed match/inlier counts on the generated features."""
        with self.lock:
            ids, uv, previous = self.ids.copy(), self.features.copy(), self.previous

        def build():
            n = len(ids)
            matched = sorted(self.rng.choice(n, size=min(matches, n), replace=False).tolist()) if matches else []
            inlier_index = sorted(self.rng.choice(len(matched), size=min(inliers, len(matched)),
                                                  replace=False).tolist()) if matched and inliers else []
            return snapshots.build_overlay(ids, uv, matched, inlier_index, None if previous is None else previous[0],
                                           None if previous is None else previous[1], sequence, (WIDTH, HEIGHT),
                                           ok, reason)
        return build


# --- replayed VIO ---------------------------------------------------------------------------------

class Replay:
    def __init__(self, steps, events, telemetry_dir, snapshot_dir, camera, start_s, prefill, speed, min_inliers):
        self.steps = steps
        self.events = events
        self.telemetry_dir = telemetry_dir
        self.snapshot_dir = snapshot_dir
        self.camera = camera
        self.start_s = start_s
        self.prefill = prefill
        self.speed = speed
        self.min_inliers = min_inliers
        self.velocity = None  # latest VIO velocity, for the fake DVL
        self.last = {"matches": 0, "inliers": 0, "ok": False, "reason": "no_reference", "sequence": 0}
        self.running = False
        self.pid = os.getpid()
        self.started = 0
        self._stop = threading.Event()
        self._thread = None
        self.telemetry = None
        self.services = None

    # supervisor interface used by main.py
    def status(self):
        return {"running": self.running, "pid": self.pid if self.running else None, "restart_count": 0,
                "last_exit_code": None, "restart_in": None}

    def start(self):
        self.stop()
        self._stop = threading.Event()
        self.running = True
        self._thread = threading.Thread(target=self._run, args=(self._stop,), name="replay", daemon=True)
        self._thread.start()

    def stop(self):
        if self._thread is not None:
            self._stop.set()
            self._thread.join(3)
            self._thread = None
        if self.telemetry is not None:
            self.telemetry.close()
            self.telemetry = None
        if self.services is not None:
            self.services[0].writer.close()
            self.services = None
        self.running = False
        snapshots.clear_camera_status(self.snapshot_dir)

    def shutdown(self):
        self.stop()

    def write_camera_status(self, refractive_index=1.333, depth_scale=1.0):
        event = self.events.get("intrinsics") or {}
        intrinsics = event.get("intrinsics") or [163.1, 237.1, 329.4, 199.0]
        status = {key: event.get(key) for key in ("source", "requested_source", "raw", "rectified", "frame",
                                                  "frame_matches", "override", "alpha", "distortion_correction",
                                                  "baseline_m")}
        status.update(pid=self.pid, unix_time=time.time(), image_size=[WIDTH, HEIGHT], intrinsics=intrinsics,
                      effective_intrinsics=[intrinsics[0] * refractive_index, intrinsics[1] * refractive_index,
                                            intrinsics[2], intrinsics[3]],
                      depth_scale=depth_scale, refractive_index=refractive_index,
                      total_depth_scale=depth_scale * refractive_index, depth_range_mm=[300, 4000],
                      warnings=event.get("warnings") or [], baseline_spec_m=0.075)
        snapshots.write_camera_status(status, self.snapshot_dir)

    def _record(self, telemetry, step, t, sequence):
        ok = step.get("status") == "ok"
        dt = (step.get("time_delta_us") or 0) * 1e-6 or 0.1
        delta = step.get("position_delta")
        angle = step.get("angle_delta")
        telemetry.record_frame(t, ok, step.get("reason"), step.get("features") or 0, step.get("valid_depth") or 0,
                               step.get("matches") or 0, step.get("inliers") or 0, step.get("proc_ms"), dt,
                               delta if ok else None, angle[2] if ok and angle else None, step.get("gap_frames") or 0,
                               bool(step.get("reference_reset")), bool(step.get("sent")))
        self.velocity = [v / dt for v in delta] if ok and delta else None
        self.last = {"matches": step.get("matches") or 0, "inliers": step.get("inliers") or 0, "ok": ok,
                     "reason": step.get("reason"), "sequence": sequence}
        trajectory = telemetry.trajectory
        self.camera.move(trajectory.x, trajectory.y)

    def _make_services(self):
        writer = snapshots.SnapshotWriter(self.snapshot_dir, log=lambda m: None)
        pending = []
        service = snapshots.SnapshotService(
            ("rect_left", "rect_right", "depth"), lambda stream, raw: pending.append(stream), writer, self.snapshot_dir,
            unavailable={"rgb": "this device has no CAM_A (RGB) camera"}, poll_interval=0.1, log=print,
            overlay=lambda: self.camera.overlay(**self.last))
        probe = depth_probe.DepthProbeService(self.snapshot_dir / "depth_probe", (300, 4000), 1.0, (WIDTH, HEIGHT),
                                              refractive_index=1.333, log=print)
        try:
            from pnp_odometry import Intrinsics
            k = (self.events.get("intrinsics") or {}).get("intrinsics") or [163.1, 237.1, 329.4, 199.0]
            probe.set_intrinsics(Intrinsics(*k), "rectified")
        except Exception as e:
            print(f"depth probe without intrinsics: {e}")
        return service, probe, pending

    def _run(self, stop):
        telemetry = vio_telemetry.Telemetry(self.telemetry_dir, fps=10, min_inliers=self.min_inliers, mode="log-only",
                                            temperature=lambda: 46.0 + random.uniform(-0.4, 0.4),
                                            output_error=lambda: None, log=print)
        self.telemetry = telemetry
        telemetry.set_phase("opening_device")
        telemetry.start()
        self.services = service, probe, pending = self._make_services()
        self.write_camera_status()
        telemetry.set_phase("running")
        steps = self.steps
        t0 = steps[0]["unix_time"]
        live = next((i for i, s in enumerate(steps) if s["unix_time"] - t0 >= self.start_s), 0)
        first = next((i for i, s in enumerate(steps) if s["unix_time"] - t0 >= self.start_s - self.prefill), 0)
        rate = 10.0
        now = time.time()
        for index in range(first, live):  # the minutes before --start at once, ending now
            self._record(telemetry, steps[index], now - (live - index) / rate, index)
        index = live
        next_time = time.time()
        while not stop.is_set():
            step = steps[index % len(steps)]
            self._record(telemetry, step, time.time(), index)
            service.tick()
            probe.tick()
            while pending:
                stream = pending.pop(0)
                service.on_frame(lambda s, f=self.camera.frame(stream): f, device_timestamp=step.get("device_timestamp"),
                                 sequence=index)
            if probe.active is not None:
                probe.on_depth(self.camera.frame("depth"), step.get("device_timestamp"), index)
            index += 1
            next_time += 1.0 / (rate * self.speed)
            stop.wait(max(0.0, next_time - time.time()))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG, help="PnP data log (JSONL) to replay")
    parser.add_argument("--port", type=int, default=8099)
    parser.add_argument("--start", type=float, default=575.0,
                        help="seconds into the log where the live replay starts (flight 3 moves around 540-620 s)")
    parser.add_argument("--prefill", type=float, default=180.0,
                        help="seconds of the log before --start replayed at once, so the plots start full")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--armed", action="store_true", help="the fake autopilot reports ARMED")
    parser.add_argument("--no-dvl", action="store_true", help="no DVL VISION_POSITION_DELTA in the fake mavlink2rest")
    parser.add_argument("--keep", action="store_true", help="keep the temporary directory")
    args = parser.parse_args()

    guard_network()
    steps, events = load_log(args.log)
    root = Path(tempfile.mkdtemp(prefix="oakd-ui-demo-"))
    (root / "logs").mkdir()
    shutil.copy(args.log, root / "logs" / args.log.name)

    replay = None

    def dvl_velocity(t):
        velocity = replay.velocity if replay is not None else None
        if velocity is None:
            return None
        return [1.04 * v + random.gauss(0, 0.012) for v in velocity]

    vehicle = FakeVehicle(armed=args.armed, dvl_velocity=None if args.no_dvl else dvl_velocity)
    server, address = serve(vehicle)
    vehicle.host = address

    import uvicorn
    import main as app_main
    vio_log.LOG_DIR = root / "logs"
    app_main.CONFIG_FILE = root / "config.json"
    app_main.SNAPSHOT_DIR = root / "snapshots"
    app_main.TELEMETRY_DIR = root / "telemetry"
    start_args = (events.get("start") or {}).get("args") or {}
    config = {**app_main.DEFAULT_CONFIG, "algorithm": "pnp", "vehicle_ip": address, "fps": 10,
              "resolution_width": WIDTH, "resolution_height": HEIGHT, "log_only": True,
              "min_inliers": int(start_args.get("min_inliers", 15)), "num_features": 300}
    app_main.save_config(config)
    camera = Camera()
    replay = Replay(steps, events, root / "telemetry", root / "snapshots", camera, args.start, args.prefill,
                    args.speed, config["min_inliers"])
    app_main.supervisor = replay

    print(f"UI demo: http://127.0.0.1:{args.port}/  (replaying {args.log.name}, {len(steps)} frames; "
          f"fake mavlink2rest at http://{address}/mavlink2rest, armed={args.armed}; data in {root})", flush=True)
    # uvicorn re-raises SIGTERM after its shutdown; exit normally then so the cleanup below runs
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        uvicorn.run(app_main.app, host="127.0.0.1", port=args.port, log_level="warning")
    finally:
        replay.stop()
        server.shutdown()
        if not args.keep:
            shutil.rmtree(root, ignore_errors=True)


if __name__ == "__main__":
    main()
