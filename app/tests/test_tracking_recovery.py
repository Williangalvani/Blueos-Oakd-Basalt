import importlib
import json
import os
import signal
import sys
import time
from collections import Counter
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_probe  # noqa: E402
import depth_stream  # noqa: E402
import feature_stream  # noqa: E402
import snapshots  # noqa: E402
import vio_log  # noqa: E402
from pnp_odometry import Intrinsics, PnPOdometry, PnPParams, back_project, camera_to_body_matrix  # noqa: E402
from rectified_intrinsics import StereoCalibration  # noqa: E402
from tracking_recovery import FeatureTrackerReinit, TrackingRecovery, feature_id_counts  # noqa: E402


# --- state machine ------------------------------------------------------------------------------

def test_recovers_after_n_consecutive_failures():
    recovery = TrackingRecovery(frames=5)
    assert [recovery.update(False) for _ in range(5)] == [False, False, False, False, True]
    assert recovery.count == 1 and recovery.failures == 0


def test_success_resets_the_counter():
    recovery = TrackingRecovery(frames=5)
    for _ in range(4):
        assert not recovery.update(False)
    assert not recovery.update(True)
    assert [recovery.update(False) for _ in range(5)][-1] is True
    assert recovery.count == 1


def test_repeated_recoveries_back_off_until_success():
    recovery = TrackingRecovery(frames=5, max_backoff=4)
    triggers = [i for i in range(1, 200) if recovery.update(False)]
    # 5, then 10, 20, 20, ... failures apart
    assert np.diff([0] + triggers[:5]).tolist() == [5, 10, 20, 20, 20]
    recovery.update(True)
    assert recovery.threshold == 5
    assert [recovery.update(False) for _ in range(5)][-1] is True


def test_zero_frames_disables_recovery():
    recovery = TrackingRecovery(frames=0)
    assert not any(recovery.update(False) for _ in range(1000))


def test_reinit_sends_maintainer_off_then_normal_config_next_frame():
    sent = []
    reinit = FeatureTrackerReinit(sent.append)
    reinit.step()
    assert sent == []
    reinit.start()
    assert sent == [False]
    reinit.step()
    reinit.step()
    assert sent == [False, True]


def test_feature_id_counts():
    assert feature_id_counts(set(), [1, 2, 3]) == (3, 0)
    assert feature_id_counts({1, 2, 3}, [2, 3, 4, 5]) == (2, 2)
    assert feature_id_counts({1}, []) == (0, 0)


WIDTH, HEIGHT = 640, 400
K = Intrinsics(fx=300.0, fy=300.0, cx=320.0, cy=200.0)
GOOD_UV = np.array([(x, y) for x in range(80, 600, 60) for y in range(60, 360, 60)], dtype=np.float64)
# The same ids at shuffled pixels: every feature matches the reference, but no pose fits
SHUFFLED_UV = GOOD_UV[np.random.default_rng(0).permutation(len(GOOD_UV))]
DEPTH = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
DEPTH[:, :40] = 0  # a strip without depth, like the underwater frames with streaky depth


def test_recovery_clears_a_reference_held_across_failures():
    recovery = TrackingRecovery(frames=5)
    odom = PnPOdometry(K, PnPParams(max_reference_age_s=0.0))
    ids = np.arange(len(GOOD_UV))
    odom.process(ids, GOOD_UV, DEPTH, 1.0)
    triggered = []
    for n in range(1, 6):
        frame = odom.process(ids, SHUFFLED_UV, DEPTH, 1.0 + 0.1 * n)
        assert not frame.pose.ok and not frame.reference_reset
        assert odom.prev_timestamp == 1.0 and frame.gap_frames == n - 1
        if recovery.update(frame.pose.ok):
            triggered.append(n)
            odom.reset()
            assert odom.prev_points is None and odom.prev_timestamp is None and odom.gap_frames == 0
    assert triggered == [5]
    # The frame after the recovery starts a new reference instead of matching the old one
    assert odom.process(ids, GOOD_UV, DEPTH, 1.6).pose.reason == "no_reference"
    frame = odom.process(ids, GOOD_UV, DEPTH, 1.7)
    assert frame.pose.ok and frame.dt == pytest.approx(0.1) and frame.gap_frames == 0


# --- pnp_vio.run() end to end with a fake device ------------------------------------------------


class FakeTransformation:
    def getIntrinsicMatrix(self):
        return [[310.0, 0.0, 321.0], [0.0, 310.0, 201.0], [0.0, 0.0, 1.0]]

    def getSize(self):
        return (WIDTH, HEIGHT)


class FakeMsg:
    def __init__(self, sequence, t, frame=None, features=()):
        self.sequence = sequence
        self.t = t
        self.frame = frame
        self.trackedFeatures = features

    def getSequenceNum(self):
        return self.sequence

    def getTimestampDevice(self, *offset):
        return timedelta(seconds=self.t)

    def getFrame(self):
        return self.frame

    def getData(self):
        return np.zeros(0, np.uint8) if self.frame is None else self.frame.view(np.uint8).reshape(-1)

    def getCvFrame(self):
        return self.frame

    def getTransformation(self):
        return FakeTransformation()


def features(ids, uv):
    return [SimpleNamespace(id=int(i), position=SimpleNamespace(x=float(u), y=float(v))) for i, (u, v) in zip(ids, uv)]


def scenario(n_good=10, n_lost=25, n_back=10):
    """Static scene; then the tracker only reports a few features, all in the no-depth strip."""
    groups = []
    lost_uv = np.array([(10.0, 50.0 + 30 * k) for k in range(8)])
    for n in range(n_good + n_lost + n_back):
        if n_good <= n < n_good + n_lost:
            feats = features(range(1000, 1008), lost_uv)
        elif n >= n_good + n_lost:
            feats = features(range(2000, 2000 + len(GOOD_UV)), GOOD_UV)
        else:
            feats = features(range(len(GOOD_UV)), GOOD_UV)
        t = 1.0 + 0.1 * n
        groups.append({"features": FakeMsg(n, t, features=feats), "depth": FakeMsg(n, t, frame=DEPTH)})
    return groups


def pattern_scenario(pattern):
    """Static scene, one frame per character: G good, F the same ids at shuffled pixels (fails)."""
    groups = []
    for n, kind in enumerate(pattern):
        feats = features(range(len(GOOD_UV)), GOOD_UV if kind == "G" else SHUFFLED_UV)
        t = 1.0 + 0.1 * n
        groups.append({"features": FakeMsg(n, t, features=feats), "depth": FakeMsg(n, t, frame=DEPTH)})
    return groups


def still_gyro(t0, t1, rate=200.0):
    """One IMU message with zero-rate gyro packets covering [t0, t1]."""
    packets = [SimpleNamespace(gyroscope=SimpleNamespace(getTimestampDevice=lambda t=t: timedelta(seconds=t),
                                                         x=0.0, y=0.0, z=0.0))
               for t in np.arange(t0, t1, 1.0 / rate)]
    return SimpleNamespace(packets=packets)


class FakeQueue:
    def __init__(self, items=()):
        self.items = list(items)
        self.sent = []

    def get(self):
        return self.items.pop(0)

    def tryGet(self):
        return self.items.pop(0) if self.items else None

    def tryGetAll(self):
        items, self.items = self.items, []
        return items

    def send(self, msg):
        self.sent.append(msg)


class FakePipeline:
    def __init__(self, sync_queue):
        self.sync_queue = sync_queue

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def start(self):
        pass

    def isRunning(self):
        return bool(self.sync_queue.items)


def fake_calibration(R_cam_imu=None):
    return StereoCalibration(raw=K, distortion=[0.0] * 14, sockets={"cam_b": K}, baseline_m=0.075,
                             R_cam_imu=R_cam_imu)


@pytest.fixture
def pnp_vio(monkeypatch, tmp_path):
    dai = mock.MagicMock()
    monkeypatch.setitem(sys.modules, "depthai", dai)
    monkeypatch.delenv(vio_log.LOG_ONLY_ENV, raising=False)
    sys.modules.pop("pnp_vio", None)
    module = importlib.import_module("pnp_vio")
    monkeypatch.setattr(module, "open_vio_log", lambda: vio_log.open_vio_log(tmp_path / "logs"))
    monkeypatch.setattr(module, "open_oak_device", lambda timeout, log: mock.MagicMock())
    monkeypatch.setattr(module, "load_calibration", lambda device, args: fake_calibration())
    module.attitude_sources = []

    def no_autopilot(vehicle_ip, log=print):  # the 3D pose's ATTITUDE reader, without mavlink2rest

        def fetch():
            raise OSError(f"test: no mavlink2rest at {vehicle_ip}")
        source = module.autopilot_attitude.AttitudeSource(fetch, log=log)
        module.attitude_sources.append(source)
        return source

    monkeypatch.setattr(module.autopilot_attitude, "from_mavlink2rest", no_autopilot)
    previous = signal.getsignal(signal.SIGTERM)
    yield module
    signal.signal(signal.SIGTERM, previous)
    sys.modules.pop("pnp_vio", None)


def run_fake(module, monkeypatch, tmp_path, groups, argv, imu=(), log_only=True, **handle_fields):
    sync_queue = FakeQueue(groups)
    snapshot_queue = FakeQueue()
    request_queue = FakeQueue()
    request_queue.send = lambda msg: (request_queue.sent.append(msg), snapshot_queue.items.append(
        FakeMsg(99, 1.5, frame=DEPTH)))
    handles = module.PipelineHandles(
        sync_queue=sync_queue, imu_queue=FakeQueue(imu), feature_config_queue=FakeQueue(),
        stereo_effective={"decimation": 1}, snapshot_request_queue=request_queue, snapshot_queue=snapshot_queue,
        snapshot_streams=("rect_left", "rect_right", "depth"), snapshot_unavailable={"rgb": "no CAM_A"},
        **handle_fields)
    monkeypatch.setattr(module, "build_pipeline", lambda p, device, args, settings: handles)
    monkeypatch.setattr(module.dai, "Pipeline", lambda device: FakePipeline(sync_queue))
    monkeypatch.setattr(module.dai, "FeatureTrackerConfig", lambda: mock.MagicMock())
    monkeypatch.setattr(module.dai, "Buffer", lambda: mock.MagicMock())
    module.main([*(["--log-only"] if log_only else []), "--gyro-gate", "0", "--snapshot-dir",
                 str(tmp_path / "snapshots"), *argv])
    (log_file,) = (tmp_path / "logs").iterdir()
    lines = [json.loads(line) for line in log_file.read_text().splitlines()]
    return lines, handles


def test_run_recovers_from_feature_collapse(pnp_vio, monkeypatch, tmp_path):
    snapshots.request_snapshot("depth", tmp_path / "snapshots")
    lines, handles = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_lost=35),
                              ["--recovery-frames", "10", "--intrinsics-from", "frame"])
    steps = [line for line in lines if line["type"] == "step"]
    events = {line["event"]: line for line in lines if line["type"] == "event"}

    assert [s["status"] for s in steps[1:10]] == ["ok"] * 9
    # Recovery after 10 failures (frame 19) and 20 more (frame 40); each is followed by a fresh reference
    reasons = [s["reason"] for s in steps[10:45]]
    assert reasons == ["too_few_matches"] * 10 + ["no_reference"] + ["too_few_matches"] * 20 + [
        "no_reference"] + ["too_few_matches"] * 3
    # No reference feature survives the collapse, so each lost frame replaces the reference
    assert steps[10]["reference_reset"] is True and steps[10]["gap_frames"] == 0
    assert all(s["reference_age_s"] == pytest.approx(0.1) for s in steps[11:20])

    recoveries = [line for line in lines if line.get("event") == "recovery"]
    assert len(recoveries) == 2
    assert recoveries[0]["failed_frames"] == 10 and recoveries[0]["next_after"] == 20
    assert recoveries[0]["features"] == 8 and recoveries[0]["reason"] == "too_few_matches"

    # Each recovery sends maintainer-off then the normal config one frame later
    configs = handles.feature_config_queue.sent
    maintainer = [c.setFeatureMaintainer.call_args[0][0] for c in configs]
    assert maintainer == [False, True, False, True]
    assert all(c.setNumTargetFeatures.call_args[0][0] == 300 for c in configs)

    # Tracking comes back once the tracker reports features with depth again
    assert [s["status"] for s in steps[-8:]] == ["ok"] * 8

    # New vs tracked ids
    assert (steps[0]["new_features"], steps[0]["tracked_features"]) == (len(GOOD_UV), 0)
    assert (steps[1]["new_features"], steps[1]["tracked_features"]) == (0, len(GOOD_UV))
    assert (steps[10]["new_features"], steps[10]["tracked_features"]) == (8, 0)

    # --intrinsics-from frame: the depth frame matrix is used, as it is not the raw calibration
    assert events["intrinsics"]["source"] == "frame"
    assert events["intrinsics"]["intrinsics"] == [310.0, 310.0, 321.0, 201.0]
    assert events["stereo_config"]["requested"]["stereo_decimation"] == 1
    assert events["start"]["stereo"]["stereo_confidence_threshold"] == 50

    # The snapshot requested before the run was served from the fake device
    assert [m.setSequenceNum.call_args[0][0] for m in handles.snapshot_request_queue.sent] == [
        snapshots.request_number("depth", raw=True)]
    assert (tmp_path / "snapshots" / "depth_raw.png").exists()
    assert snapshots.read_metadata("depth", tmp_path / "snapshots")["sequence"] == 99


def test_run_feeds_the_web_telemetry(pnp_vio, monkeypatch, tmp_path):
    created = []
    real = pnp_vio.vio_telemetry.Telemetry
    monkeypatch.setattr(pnp_vio.vio_telemetry, "Telemetry",
                        lambda *a, **kw: created.append(real(*a, **kw)) or created[-1])
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_lost=35),
                        ["--recovery-frames", "10", "--telemetry-dir", str(tmp_path / "telemetry")])
    steps = [line for line in lines if line["type"] == "step"]
    (telemetry,) = created
    assert telemetry.directory == tmp_path / "telemetry" and telemetry.mode == "log-only"

    telemetry.write_status()
    telemetry.write_history()
    status = pnp_vio.vio_telemetry.read_status(tmp_path / "telemetry")
    rows, _ = pnp_vio.vio_telemetry.HistoryReader().read(tmp_path / "telemetry")
    assert status["phase"] == "running" and status["frames_total"] == len(steps)
    assert status["ok_total"] == sum(s["status"] == "ok" for s in steps) and status["recoveries"] == 2
    reasons = {r["reason"]: r["count"] for r in status["failures"]["by_reason"]}
    assert reasons == Counter(s["reason"] for s in steps if s["status"] != "ok")
    assert reasons["too_few_matches"] > 30 and reasons["no_reference"] >= 2
    assert len(rows) == len(steps)
    ok = rows[:, pnp_vio.vio_telemetry.COLUMN["ok"]]
    assert list(ok) == [1.0 if s["status"] == "ok" else 0.0 for s in steps]


def test_run_without_recovery_never_reinitialises(pnp_vio, monkeypatch, tmp_path):
    lines, handles = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(), ["--recovery-frames", "0"])
    assert not any(line.get("event") == "recovery" for line in lines)
    assert handles.feature_config_queue.sent == []


def test_run_bridges_failed_frames_and_still_recovers(pnp_vio, monkeypatch, tmp_path):
    lines, handles = run_fake(pnp_vio, monkeypatch, tmp_path, pattern_scenario("GGGGG" "FF" "GGG" "FFFFF" "GGG"),
                              ["--recovery-frames", "5"])
    steps = [line for line in lines if line["type"] == "step"]
    assert [s["status"] for s in steps] == ["rejected"] + ["ok"] * 4 + ["rejected"] * 2 + ["ok"] * 3 + [
        "rejected"] * 5 + ["rejected", "ok", "ok"]

    # Failed frames keep frame 4 as the reference, and log how long it has been held
    for n, step in enumerate(steps[5:7]):
        assert step["reason"] in ("too_few_inliers", "pnp_failed") and step["matches"] == len(GOOD_UV)
        assert step["gap_frames"] == n and step["reference_age_s"] == pytest.approx(0.1 * (n + 1))
        assert step["reference_reset"] is False and step["time_delta_us"] is None
    # The next good frame's delta spans 4 -> 7: time and motion over the same interval
    bridged = steps[7]
    assert bridged["gap_frames"] == 2 and bridged["reference_age_s"] == pytest.approx(0.3)
    assert bridged["time_delta_us"] == pytest.approx(0.3e6)
    np.testing.assert_allclose(bridged["position_delta"], 0.0, atol=1e-6)
    assert steps[8]["time_delta_us"] == pytest.approx(0.1e6) and steps[8]["gap_frames"] == 0

    # 5 consecutive failures still trigger the recovery, and the held reference goes with it
    (recovery,) = [line for line in lines if line.get("event") == "recovery"]
    assert recovery["failed_frames"] == 5 and recovery["matches"] == len(GOOD_UV)
    assert [s["gap_frames"] for s in steps[10:15]] == [0, 1, 2, 3, 4]
    assert steps[15]["reason"] == "no_reference" and steps[15]["reference_age_s"] is None
    assert steps[16]["time_delta_us"] == pytest.approx(0.1e6) and steps[16]["gap_frames"] == 0
    assert [c.setFeatureMaintainer.call_args[0][0] for c in handles.feature_config_queue.sent] == [False, True]


def test_run_gyro_prior_covers_a_long_gap(pnp_vio, monkeypatch, tmp_path):
    """A 2.6 s gap, longer than the old fixed gyro history: the prior must still span it."""
    monkeypatch.setattr(pnp_vio, "load_calibration", lambda device, args: fake_calibration(np.eye(3)))
    pattern = "GGG" + "F" * 25 + "GGG"
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, pattern_scenario(pattern),
                        ["--recovery-frames", "0", "--max-reference-age", "0", "--gyro-gate", "5"],
                        imu=[still_gyro(0.9, 1.0 + 0.1 * len(pattern))])
    steps = [line for line in lines if line["type"] == "step"]
    assert [s["status"] for s in steps[3:28]] == ["rejected"] * 25
    assert not any(s["reference_reset"] for s in steps)
    bridged = steps[28]
    assert bridged["status"] == "ok" and bridged["gap_frames"] == 25
    assert bridged["time_delta_us"] == pytest.approx(2.6e6)
    # Logged as null when no gyro prior was available
    assert bridged["gyro_disagreement_deg"] is not None and bridged["gyro_disagreement_deg"] < 0.1


def test_run_max_reference_age_from_the_command_line(pnp_vio, monkeypatch, tmp_path):
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, pattern_scenario("GG" + "F" * 6),
                        ["--recovery-frames", "0", "--max-reference-age", "0.35"])
    steps = [line for line in lines if line["type"] == "step"]
    assert [s["reference_reset"] for s in steps[2:6]] == [False, False, False, True]
    # The shuffled frame that took over is a valid reference for the next shuffled frames
    assert [s["status"] for s in steps[6:]] == ["ok", "ok"]
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["start"]["args"]["max_reference_age"] == 0.35


def test_intrinsics_fall_back_to_calibration(pnp_vio, monkeypatch, tmp_path):
    # No rectified K without alpha, and an unusable frame matrix: the raw calibration is all that is left
    monkeypatch.setattr(FakeTransformation, "getSize", lambda self: (1280, 720))
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(5, 0, 0),
                        ["--intrinsics-from", "frame", "--stereo-alpha", "none"])
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["intrinsics"]["source"] == "cam_b (fallback)"
    assert events["intrinsics"]["intrinsics"] == [300.0, 300.0, 320.0, 200.0]
    assert any("implausible" in line.get("message", "") for line in lines if line.get("event") == "warning")


class ScriptStop(Exception):
    pass


def run_snapshot_script(code, requests, inputs):
    """Run a snapshot Script's code on a strict fake node; returns [(output, frame), ...] it sent."""
    sent, pending = [], list(requests)

    class Input:
        def __init__(self, name):
            self.name = name

        def get(self):
            if self.name == "request":
                if not pending:
                    raise ScriptStop()
                return SimpleNamespace(getSequenceNum=lambda n=pending.pop(0): n)
            return inputs[self.name]

    class Output:
        def __init__(self, name):
            self.name = name

        def send(self, msg):
            sent.append((self.name, msg))

    class Ports(dict):
        def __init__(self, kind, allowed):
            super().__init__()
            self.kind, self.allowed = kind, allowed

        def __missing__(self, name):
            assert name in self.allowed, f"the Script used {name!r}, which the gate does not link"
            self[name] = self.kind(name)
            return self[name]

    node = SimpleNamespace(inputs=Ports(Input, {"request", *inputs}),
                           outputs=Ports(Output, {"snapshot", "mono", "color"}))
    with pytest.raises(ScriptStop):
        exec(compile(code, "snapshot_script", "exec"), {"node": node})
    return sent


def test_snapshot_script_sends_jpegs_to_the_encoders_and_raw_on_request(pnp_vio):
    code = pnp_vio.snapshot_script(["rect_left", "rect_right", "depth", "rgb"],
                                   {"rect_left": "mono", "rect_right": "mono", "rgb": "color"})
    inputs = {name: f"{name} frame" for name in ("rect_left", "rect_right", "depth", "rgb")}
    number = snapshots.request_number
    requests = [number("rect_left"), number("rect_right"), number("rgb"), number("depth"), number("depth", True),
                number("rect_left", True), number("rgb", True), 99]
    assert run_snapshot_script(code, requests, inputs) == [
        ("mono", "rect_left frame"), ("mono", "rect_right frame"), ("color", "rgb frame"),
        ("snapshot", "depth frame"), ("snapshot", "depth frame"), ("snapshot", "rect_left frame"),
        ("snapshot", "rgb frame")]  # 99: out of range, ignored


def test_snapshot_script_without_encoders_sends_everything_raw(pnp_vio):
    code = pnp_vio.snapshot_script(["rect_left", "depth"])
    assert "available = ['depth', 'rect_left']" in code
    inputs = {"rect_left": "left", "depth": "depth"}
    sent = run_snapshot_script(code, [snapshots.request_number("rect_left"), snapshots.request_number("rgb"),
                                      snapshots.request_number("depth")], inputs)
    assert sent == [("snapshot", "left"), ("snapshot", "depth")]  # rgb: not available


def stream_encoders(module, created):
    """The VideoEncoders that are not the snapshot gate's (fed from its 'mono' / 'color' outputs)."""
    return [e for e in nodes_of(created, module.dai.node.VideoEncoder)
            if not any(mock.call(e.input) in s.outputs[name].link.call_args_list
                       for s in nodes_of(created, module.dai.node.Script) for name in ("mono", "color"))]


def snapshot_gate(module, created):
    (script,) = [s for s in nodes_of(created, module.dai.node.Script)
                 if "streams = " in s.setScript.call_args[0][0]]
    return script


def test_build_pipeline_jpeg_encodes_the_snapshots_on_the_camera(pnp_vio):
    dai = pnp_vio.dai
    p, created = fake_dai_pipeline()
    device = mock.MagicMock()
    device.getConnectedCameras.return_value = [dai.CameraBoardSocket.CAM_A]
    args = pnp_vio.parse_args(["--depth-transport", "full_frame", "--no-depth-stream"])
    handles = pnp_vio.build_pipeline(p, device, args, pnp_vio.resolve_stereo_settings(args))
    script = snapshot_gate(pnp_vio, created)
    mono, color = nodes_of(created, dai.node.VideoEncoder)
    script.outputs["mono"].link.assert_called_once_with(mono.input)
    script.outputs["color"].link.assert_called_once_with(color.input)
    for encoder, fps in ((mono, float(args.fps)), (color, pnp_vio.SNAPSHOT_RGB_FPS)):
        encoder.setDefaultProfilePreset.assert_called_once_with(fps, dai.VideoEncoderProperties.Profile.MJPEG)
        encoder.setQuality.assert_called_once_with(snapshots.JPEG_QUALITY)
        encoder.input.setBlocking.assert_called_with(False)
        encoder.input.setMaxSize.assert_called_with(1)
    assert handles.snapshot_encoded == ("rect_left", "rect_right", "rgb")
    assert handles.snapshot_encoded_queues == (mono.out.createOutputQueue.return_value,
                                               color.out.createOutputQueue.return_value)
    assert handles.snapshot_queue is script.outputs["snapshot"].createOutputQueue.return_value
    assert handles.snapshot_not_encoded == {} and handles.snapshot_unavailable == {}
    code = script.setScript.call_args[0][0]
    assert "encoders = {'rect_left': 'mono', 'rect_right': 'mono', 'rgb': 'color'}" in code


@pytest.mark.parametrize("argv, reason", [(["--snapshot-encoding", "raw"], "Snapshot Encoding is raw"),
                                          (["--width", "600"], "multiple of 32")])
def test_build_pipeline_sends_snapshots_raw_when_set_or_the_encoder_cannot(pnp_vio, argv, reason):
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, ["--no-depth-stream", *argv])
    assert nodes_of(created, dai.node.VideoEncoder) == [] and handles.snapshot_encoded == ()
    assert reason in handles.snapshot_not_encoded["rect_left"]
    assert "encoders = {}" in snapshot_gate(pnp_vio, created).setScript.call_args[0][0]


# --- live depth stream in pnp_vio ---------------------------------------------------------------

def test_parse_args_depth_stream(pnp_vio, capsys):
    args = pnp_vio.parse_args([])
    assert (args.depth_stream, args.depth_stream_fps, args.depth_stream_quality) == (True, 5.0, 70)
    args = pnp_vio.parse_args(["--no-depth-stream", "--depth-stream-fps", "2.5", "--depth-stream-quality", "40"])
    assert (args.depth_stream, args.depth_stream_fps, args.depth_stream_quality) == (False, 2.5, 40)
    for bad in (["--depth-stream-fps", "0"], ["--depth-stream-fps", "61"], ["--depth-stream-fps", "nan"],
                ["--depth-stream-quality", "0"], ["--depth-stream-quality", "101"],
                ["--depth-stream-quality", "x"], ["--depth-stream-encoding", "h264"]):
        with pytest.raises(SystemExit):
            pnp_vio.parse_args(bad)
    assert "depth stream fps" in capsys.readouterr().err
    assert (args.depth_stream_encoding, args.depth_stream_debug) == ("auto", False)
    args = pnp_vio.parse_args(["--depth-stream-encoding", "host_jpeg", "--depth-stream-debug"])
    assert (args.depth_stream_encoding, args.depth_stream_debug) == ("host_jpeg", True)


def fake_dai_pipeline():
    """The strict fake pipeline (pipeline_fakes): p.create() returns a distinct MagicMock node per call."""
    import pipeline_fakes
    p = pipeline_fakes.StrictPipeline()
    return p, p.created


def nodes_of(created, kind):
    return [node for k, node in created if k is kind]


def build_fake_pipeline(module, argv, cameras=(), start=False):
    """build_pipeline on fakes; full_frame (the previous pipeline, unchanged) unless argv picks a transport."""
    p, created = fake_dai_pipeline()
    device = mock.MagicMock()
    device.getConnectedCameras.return_value = list(cameras)
    if "--depth-transport" not in argv:
        argv = ["--depth-transport", "full_frame", *argv]
    args = module.parse_args(argv)
    handles = module.build_pipeline(p, device, args, module.resolve_stereo_settings(args))
    if start:
        p.start()
    return handles, created


STEREO_VARIANTS = {"subpixel": [], "subpixel-off": ["--no-stereo-subpixel", "--no-stereo-speckle-filter"]}


def pipeline_combinations():
    """Each transport with each depth stream variant, then each other option changed on its own for both."""
    base = {"snapshot_encoding": snapshots.CAMERA_JPEG, "rgb": True, "stereo": "subpixel"}
    cases = [dict(base, transport=transport, stream=stream)
             for transport in ("points", "full_frame") for stream in depth_stream.VARIANTS]
    for transport in ("points", "full_frame"):
        for change in ({"snapshot_encoding": snapshots.RAW}, {"rgb": False}, {"stereo": "subpixel-off"}):
            cases.append(dict(base, transport=transport, stream="auto", **change))
    return [pytest.param(case, id="-".join(str(value) for value in case.values())) for case in cases]


@pytest.mark.parametrize("case", pipeline_combinations())
def test_build_pipeline_starts_in_every_combination(pnp_vio, case):
    """
    test16 (cb60cc4): points did not start, "'Script.io["depth"]' redefined": the pairing Script and the full
    depth gate each had an input and an output named 'depth'. The strict fake pipeline makes depthai's start
    checks (unique io names per node, link datatypes, stream names, the Scripts' ports).
    """
    dai = pnp_vio.dai
    transport = case["transport"]
    argv = ["--depth-transport", transport, "--depth-stream-encoding", case["stream"],
            "--snapshot-encoding", case["snapshot_encoding"], *STEREO_VARIANTS[case["stereo"]]]
    handles, created = build_fake_pipeline(pnp_vio, argv, cameras=[dai.CameraBoardSocket.CAM_A] if case["rgb"] else [],
                                           start=True)
    scripts = nodes_of(created, dai.node.Script)
    for script in scripts:
        assert not set(script.inputs) & set(script.outputs)
    expected = 1 + (transport == "points") * 2 + (handles.depth_stream_queue is not None)
    assert len(scripts) == expected
    if transport == "points":
        assert handles.points_queue is not None and handles.full_depth_queue is not None
        assert handles.rois_queue is not None


@pytest.mark.parametrize("argv", [["--no-depth-stream"], ["--no-snapshots"], ["--no-depth-stream", "--no-snapshots"]])
@pytest.mark.parametrize("transport", ["points", "full_frame"])
def test_build_pipeline_starts_without_the_optional_parts(pnp_vio, transport, argv):
    build_fake_pipeline(pnp_vio, ["--depth-transport", transport, *argv], start=True)


def test_build_pipeline_streams_the_rectified_left_image_by_default(pnp_vio):
    """auto: stereo.rectifiedLeft (pooled RAW8, the tracker's image) -> gate Script (forwards it) -> encoder."""
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, [])
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    (tracker,) = nodes_of(created, dai.node.FeatureTracker)
    snapshot_script, stream_script = nodes_of(created, dai.node.Script)
    (encoder,) = stream_encoders(pnp_vio, created)
    assert nodes_of(created, dai.node.ImageManip) == []
    mono_in = stream_script.inputs["depth"]
    assert mock.call(mono_in) in stereo.rectifiedLeft.link.call_args_list
    assert mock.call(tracker.inputImage) in stereo.rectifiedLeft.link.call_args_list  # the same image as the features
    assert mock.call(mono_in) not in stereo.depth.link.call_args_list + stereo.disparity.link.call_args_list
    mono_in.setBlocking.assert_called_with(False)
    mono_in.setMaxSize.assert_called_with(1)
    stream_script.outputs["gray"].link.assert_called_once_with(encoder.input)
    encoder.input.setBlocking.assert_called_with(False)
    encoder.input.setMaxSize.assert_called_with(1)
    encoder.setDefaultProfilePreset.assert_called_once_with(5.0, dai.VideoEncoderProperties.Profile.MJPEG)
    code = stream_script.setScript.call_args[0][0]
    compile(code, "depth_stream_script", "exec")
    assert "out = frame" in code and "ImgFrame(" not in code and "to_gray" not in code
    info = handles.depth_stream_info
    assert (info["requested_variant"], info["variant"], info["variant_reason"]) == ("auto", "mono_features", None)
    assert info["encoding"] == "gray8-mono-rectified-left" and info["source"] == "stereo.rectifiedLeft"
    assert len(handles.depth_stream_nodes) == 2  # the Script and the encoder
    # snapshots 2 (the gate's queue, a rectified frame in the mono encoder), and 3 for the rectified frame in
    # the stream Script's queue, in the Script, in the encoder's queue
    stereo.setNumFramesPool.assert_called_once_with(pnp_vio.STEREO_FRAMES_POOL + 5)


def test_build_pipeline_encodes_the_depth_stream_on_the_camera(pnp_vio):
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, ["--stereo-depth-max-mm", "4000"])
    snapshot_script, stream_script = nodes_of(created, dai.node.Script)
    (encoder,) = stream_encoders(pnp_vio, created)

    # The gate's inputs are non-blocking and small
    depth_in = stream_script.inputs["depth"]
    depth_in.setBlocking.assert_called_with(False)
    depth_in.setMaxSize.assert_called_with(1)
    stream_script.inputs["control"].setBlocking.assert_called_with(False)
    code = stream_script.setScript.call_args[0][0]
    assert "every = 6" in code
    encoder.setQuality.assert_called_once_with(70)

    stream_script.inputs["control"].createInputQueue.assert_called_once_with(maxSize=4, blocking=False)
    encoder.out.createOutputQueue.assert_called_once_with(maxSize=2, blocking=False)
    stream_script.outputs["status"].createOutputQueue.assert_called_once_with(maxSize=8, blocking=False)
    assert handles.depth_stream_control_queue is stream_script.inputs["control"].createInputQueue.return_value
    assert handles.depth_stream_queue is encoder.out.createOutputQueue.return_value
    assert handles.depth_stream_status_queue is stream_script.outputs["status"].createOutputQueue.return_value
    info = handles.depth_stream_info
    assert info["fps_target"] == 5.0 and info["every"] == 6 and info["quality"] == 70
    assert info["depth_range_mm"][1] == 4000 and info["quantization_mm"] == 16
    assert info["image_size"] == [640, 400] and info["size_problem"] is None
    assert snapshot_script is not stream_script


def test_build_pipeline_host_jpeg_adds_nothing_on_the_camera(pnp_vio):
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, ["--depth-stream-encoding", "host_jpeg"])
    assert len(nodes_of(created, dai.node.Script)) == 1  # the snapshot gate only
    assert stream_encoders(pnp_vio, created) == [] and nodes_of(created, dai.node.ImageManip) == []
    assert handles.depth_stream_control_queue is None and handles.depth_stream_queue is None
    assert handles.depth_stream_info["variant"] == "host_jpeg" and handles.depth_stream_nodes == ()
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    stereo.setNumFramesPool.assert_called_once_with(pnp_vio.STEREO_FRAMES_POOL + 2)


def test_build_pipeline_auto_goes_to_the_pi_for_a_size_the_encoder_rejects(pnp_vio):
    handles, created = build_fake_pipeline(pnp_vio, ["--width", "600", "--height", "400"])
    assert stream_encoders(pnp_vio, created) == []
    info = handles.depth_stream_info
    assert info["variant"] == "host_jpeg" and "32" in info["size_problem"]


def test_build_pipeline_without_the_depth_stream(pnp_vio):
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, ["--no-depth-stream", "--depth-stream-fps", "1"])
    assert not stream_encoders(pnp_vio, created) and len(nodes_of(created, dai.node.Script)) == 1
    assert handles.depth_stream_control_queue is None and handles.depth_stream_info == {}
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    stereo.setNumFramesPool.assert_called_once_with(pnp_vio.STEREO_FRAMES_POOL + 2)
    handles, created = build_fake_pipeline(pnp_vio, ["--no-depth-stream", "--snapshot-encoding", "raw"])
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    stereo.setNumFramesPool.assert_called_once_with(pnp_vio.STEREO_FRAMES_POOL + 1)

    handles, created = build_fake_pipeline(pnp_vio, ["--no-depth-stream", "--no-snapshots"])
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    stereo.setNumFramesPool.assert_not_called()

    handles, created = build_fake_pipeline(pnp_vio, ["--no-snapshots", "--depth-stream-fps", "1",
                                                     "--depth-stream-quality", "55", "--no-stereo-subpixel"])
    (encoder,) = stream_encoders(pnp_vio, created)
    encoder.setDefaultProfilePreset.assert_called_once_with(1.0, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality.assert_called_once_with(55)
    assert handles.depth_stream_info["every"] == 30


def test_run_streams_depth_only_while_a_client_watches(pnp_vio, monkeypatch, tmp_path):
    telemetry_dir = tmp_path / "snapshots" / "telemetry"
    depth_stream.touch_lease(telemetry_dir)
    encoded = FakeQueue([SimpleNamespace(getData=lambda: np.frombuffer(b"\xff\xd8jpeg\xff\xd9", np.uint8),
                                         getSequenceNum=lambda: 42)])
    control = FakeQueue()
    report = b"error\t3\t0\tconvert: TypeError: incompatible constructor arguments"
    status = FakeQueue([SimpleNamespace(getData=lambda: np.frombuffer(report, np.uint8))])
    info = {"fps_target": 5.0, "quality": 70, "depth_range_mm": [300, 4000]}
    lines, handles = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=6, n_lost=0, n_back=0), [],
                              depth_stream_control_queue=control, depth_stream_queue=encoded,
                              depth_stream_status_queue=status, depth_stream_info=info)
    # The lease enabled the camera's stream (sequence 1); the run's end leaves it to the device shutdown
    assert [m.setSequenceNum.call_args[0][0] for m in control.sent] == [1]
    assert (telemetry_dir / depth_stream.FRAME_FILE).read_bytes() == b"\xff\xd8jpeg\xff\xd9"
    meta = depth_stream.read_meta(telemetry_dir)
    assert meta["frames_total"] == 1 and meta["last_frame"]["sequence"] == 42 and meta["streaming"] is False
    # The Script's error report reaches the metadata /api/status serves
    assert meta["errors"] == 3 and "incompatible constructor" in meta["last_error"]["text"]
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["depth_stream"]["enabled"] is True and events["depth_stream"]["fps_target"] == 5.0
    # VIO output unchanged
    assert [line["status"] for line in lines if line["type"] == "step"][1:] == ["ok"] * 5


def test_run_publishes_the_features_of_the_mono_image_s_own_frame(pnp_vio, monkeypatch, tmp_path):
    """mono_features: each camera image published also publishes the features of the frame it shows."""
    telemetry_dir = tmp_path / "snapshots" / "telemetry"
    depth_stream.touch_lease(telemetry_dir)
    feature_stream.touch_lease(telemetry_dir)
    encoded = FakeQueue([SimpleNamespace(getData=lambda: np.frombuffer(b"\xff\xd8mono\xff\xd9", np.uint8),
                                         getSequenceNum=lambda: 0)])
    info = {"fps_target": 5.0, "quality": 70, "depth_range_mm": [300, 4000], "variant": "mono_features",
            "requested_variant": "mono_features"}
    run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=1, n_lost=0, n_back=0), [],
             depth_stream_control_queue=FakeQueue(), depth_stream_queue=encoded,
             depth_stream_status_queue=FakeQueue(), depth_stream_info=info)
    features = json.loads((telemetry_dir / feature_stream.FEATURES_FILE).read_text())
    assert (features["sequence"], features["image_sequence"], features["synced"]) == (0, 0, True)
    assert len(features["points"]) == len(GOOD_UV)
    meta = depth_stream.read_meta(telemetry_dir)
    assert meta["source"] == "stereo.rectifiedLeft" and meta["encoding"] == "gray8-mono-rectified-left"


def recorded_telemetry(module, monkeypatch):
    created = []
    real = module.vio_telemetry.Telemetry
    monkeypatch.setattr(module.vio_telemetry, "Telemetry", lambda *a, **kw: created.append(real(*a, **kw)) or created[-1])
    return created


def test_run_counts_each_host_queue_for_the_link_breakdown(pnp_vio, monkeypatch, tmp_path):
    link_stats = pnp_vio.link_stats
    created = recorded_telemetry(pnp_vio, monkeypatch)
    telemetry_dir = tmp_path / "snapshots" / "telemetry"
    depth_stream.touch_lease(telemetry_dir)
    snapshots.request_snapshot("depth", tmp_path / "snapshots")
    jpeg = np.frombuffer(b"\xff\xd8" + b"j" * 1000 + b"\xff\xd9", np.uint8)
    encoded = FakeQueue([SimpleNamespace(getData=lambda: jpeg, getSequenceNum=lambda: 3)] * 2)
    report = np.frombuffer(b"stats\t0\t1\t2.0 3.0 640 400", np.uint8)
    status = FakeQueue([SimpleNamespace(getData=lambda: report)])
    imu = [still_gyro(0.9, 0.95), still_gyro(0.95, 1.0)]
    groups = scenario(n_good=6, n_lost=0, n_back=0)
    run_fake(pnp_vio, monkeypatch, tmp_path, groups, ["--telemetry-dir", str(telemetry_dir)], imu=imu,
             depth_stream_control_queue=FakeQueue(), depth_stream_queue=encoded, depth_stream_status_queue=status,
             depth_stream_info={"fps_target": 5.0, "depth_range_mm": [300, 4000], "variant": "mono_features",
                                "requested_variant": "mono_features"})
    (telemetry,) = created
    totals = telemetry.link.streams.totals()
    frames = len(groups)
    assert totals["depth"] == (frames, frames * link_stats.frame_bytes(DEPTH.nbytes))
    # Counted from the ids (1 byte each below 128)
    assert totals["features"] == (frames, frames * link_stats.features_bytes(np.arange(len(GOOD_UV))))
    assert totals["sync_group"] == (frames, frames * link_stats.MESSAGE_GROUP_OVERHEAD)
    assert totals["imu"] == (2, sum(link_stats.imu_bytes(len(m.packets)) for m in imu))
    assert totals["depth_stream"] == (2, 2 * link_stats.encoded_bytes(len(jpeg)))
    assert totals["depth_stream_status"] == (1, link_stats.buffer_bytes(len(report)))
    assert totals["snapshot"] == (1, link_stats.frame_bytes(DEPTH.nbytes))  # the fake device's reply


def test_run_gives_the_link_monitor_each_camera_and_counts_its_log(pnp_vio, monkeypatch, tmp_path):
    created = recorded_telemetry(pnp_vio, monkeypatch)
    device = mock.MagicMock()
    device.getUsbSpeed.return_value = "UsbSpeed.SUPER"
    device.getDeviceInfo.return_value = SimpleNamespace(protocol=SimpleNamespace(name="X_LINK_USB_VSC"))
    device.getPlatformAsString.return_value = "RVC2"
    device.getProfilingData.return_value = SimpleNamespace(numBytesRead=1000, numBytesWritten=10)
    monkeypatch.setattr(pnp_vio, "open_oak_device", lambda timeout, log: device)
    run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=3, n_lost=0, n_back=0), [])
    (telemetry,) = created
    telemetry.link.sample()
    status = telemetry.link.status()
    assert status["available"] is True and status["totals_bytes"] == {"down": 1000, "up": 10}
    assert (status["usb_speed"], status["protocol"], status["platform"]) == ("SUPER", "USB", "RVC2")
    # A log callback counts the camera's log messages; it is removed with the session
    (callback,), _ = device.addLogCallback.call_args
    callback(SimpleNamespace(payload="x" * 36, level="WARN", nodeIdName="Script(4)"))
    assert telemetry.link.streams.totals()["device_log"] == (1, 36 + pnp_vio.link_stats.LOG_MESSAGE_OVERHEAD)
    device.removeLogCallback.assert_called_once_with(device.addLogCallback.return_value)


def test_run_clears_the_link_while_the_camera_reconnects(pnp_vio, monkeypatch, tmp_path):
    calls = []
    real = pnp_vio.link_stats.LinkMonitor

    class Recorded(real):
        def set_device(self, profiling, **info):
            calls.append(("set", profiling))
            super().set_device(profiling, **info)

        def clear_device(self, reason="no camera open"):
            calls.append(("clear", reason))
            super().clear_device(reason)

    monkeypatch.setattr(pnp_vio.link_stats, "LinkMonitor", Recorded)
    first = session_handles(pnp_vio, scenario(n_good=8, n_lost=0, n_back=0))
    second = session_handles(pnp_vio, scenario(n_good=3, n_lost=0, n_back=0))
    devices = []
    real_get = first.sync_queue.get

    def get():
        if len(first.sync_queue.items) == 4:
            devices[0].setMaxReconnectionAttempts.call_args[0][1](SimpleNamespace(name="RECONNECTED"))
        return real_get()

    first.sync_queue.get = get
    run_sessions(pnp_vio, monkeypatch, tmp_path, [first, second], devices=devices)
    assert [kind for kind, _ in calls] == ["set", "clear", "set"]
    assert calls[0][1] is devices[0].getProfilingData and calls[2][1] is devices[1].getProfilingData
    assert "reconnect" in calls[1][1]


def test_the_periodic_log_line_has_the_link_summary(pnp_vio, monkeypatch):
    lines = []
    sink = pnp_vio.logger.add(lines.append, format="{message}")
    try:
        stats = pnp_vio.Stats(lambda: None, link=lambda: "link down 61.9 up 0.1 Mbps")
        stats.frames, stats.ok, stats.proc_ms, stats.features = 1, 1, [1.0], [300]
        stats.start -= pnp_vio.SUMMARY_INTERVAL
        stats.maybe_report()
        stats.start -= pnp_vio.SUMMARY_INTERVAL
        stats.maybe_report()  # no frames
    finally:
        pnp_vio.logger.remove(sink)
    assert len(lines) == 2 and all(", link down 61.9 up 0.1 Mbps" in line for line in lines)


def test_run_without_a_watcher_keeps_the_stream_off(pnp_vio, monkeypatch, tmp_path):
    control = FakeQueue()
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=4, n_lost=0, n_back=0), [],
                        depth_stream_control_queue=control, depth_stream_queue=FakeQueue(),
                        depth_stream_info={"fps_target": 5.0})
    assert [m.setSequenceNum.call_args[0][0] for m in control.sent] == [0]
    assert not (tmp_path / "snapshots" / "telemetry" / depth_stream.FRAME_FILE).exists()


def test_run_publishes_the_tracked_features_while_leased(pnp_vio, monkeypatch, tmp_path):
    telemetry_dir = tmp_path / "snapshots" / "telemetry"
    feature_stream.touch_lease(telemetry_dir)
    written = []
    real = pnp_vio.feature_stream.FeatureStreamService

    class EveryFrame(real):
        def __init__(self, directory, image_size, **kwargs):
            super().__init__(directory, image_size, **{**kwargs, "interval": 0.0})

        def tick(self, *args, **kwargs):
            if super().tick(*args, **kwargs):
                written.append(json.loads((self.directory / feature_stream.FEATURES_FILE).read_text()))

    monkeypatch.setattr(pnp_vio.feature_stream, "FeatureStreamService", EveryFrame)
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=4, n_lost=3, n_back=0), [])
    steps = [line for line in lines if line["type"] == "step"]
    assert len(written) == len(steps)
    for step, features in zip(steps, written):
        counts = features["counts"]
        assert features["frame_size"] == [WIDTH, HEIGHT] and features["image_size"] == [WIDTH, HEIGHT]
        assert sum(counts.values()) == step["features"] == len(features["points"])
        assert counts["inlier"] == step["inliers"] and features["ok"] == (step["status"] == "ok")
        assert features["reason"] == step["reason"]
    # The first frame has no reference: valid depth only; good frames: every feature an inlier
    assert written[0]["counts"]["depth"] == len(GOOD_UV) and written[1]["counts"]["inlier"] == len(GOOD_UV)
    # The lost frames' features are in the strip without depth (x < 40), drawn where the tracker saw them
    lost = written[4]
    assert lost["counts"]["no_depth"] == 8 and all(p[0] < 40 for p in lost["points"])


def test_run_without_a_feature_watcher_writes_nothing(pnp_vio, monkeypatch, tmp_path):
    run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=4, n_lost=0, n_back=0), [])
    assert not (tmp_path / "snapshots" / "telemetry" / feature_stream.FEATURES_FILE).exists()


def session_handles(module, groups, snapshot_reply=True):
    """One pipeline's queues. snapshot_reply=False: the host -> device queue is stuck, as after an XLink reconnect."""
    snapshot_queue, request_queue = FakeQueue(), FakeQueue()
    if snapshot_reply:
        request_queue.send = lambda msg: (request_queue.sent.append(msg), snapshot_queue.items.append(
            FakeMsg(99, 1.5, frame=DEPTH)))
    return module.PipelineHandles(
        sync_queue=FakeQueue(groups), imu_queue=FakeQueue(), feature_config_queue=FakeQueue(),
        stereo_effective={"decimation": 1}, snapshot_request_queue=request_queue, snapshot_queue=snapshot_queue,
        snapshot_streams=("rect_left", "rect_right", "depth"), snapshot_unavailable={"rgb": "no CAM_A"},
        depth_stream_control_queue=FakeQueue(), depth_stream_queue=FakeQueue(), depth_stream_status_queue=FakeQueue(),
        depth_stream_info={"fps_target": 5.0, "depth_range_mm": [300, 4000], "variant": "mono_features",
                           "requested_variant": "mono_features"})


def run_sessions(module, monkeypatch, tmp_path, sessions, between=None, devices=None):
    """main() over several pipelines: each open_oak_device returns a new device, build_pipeline its handles."""
    devices = [] if devices is None else devices
    telemetries = []

    def open_device(timeout, log):
        if devices and between is not None:
            between(len(devices))
        devices.append(mock.MagicMock(name=f"device{len(devices)}"))
        return devices[-1]

    real = module.vio_telemetry.Telemetry

    class Recorded(real):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            telemetries.append(self)

    monkeypatch.setattr(module.vio_telemetry, "Telemetry", Recorded)
    monkeypatch.setattr(module, "open_oak_device", open_device)
    monkeypatch.setattr(module, "build_pipeline", lambda p, device, args, settings: sessions[len(devices) - 1])
    monkeypatch.setattr(module.dai, "Pipeline", lambda device: FakePipeline(sessions[len(devices) - 1].sync_queue))
    monkeypatch.setattr(module.dai, "FeatureTrackerConfig", lambda: mock.MagicMock())
    monkeypatch.setattr(module.dai, "Buffer", lambda: mock.MagicMock())
    module.main(["--log-only", "--gyro-gate", "0", "--snapshot-dir", str(tmp_path / "snapshots")])
    (log_file,) = (tmp_path / "logs").iterdir()
    return [json.loads(line) for line in log_file.read_text().splitlines()], devices, telemetries[0]


def test_run_restarts_the_pipeline_when_the_camera_restarts(pnp_vio, monkeypatch, tmp_path):
    """
    test11: after an XLink drop depthai reconnected by itself, VIO frames came back, but the snapshot
    request queue was stuck (404 until a VIO stop/start). Now the first frame of the restarted camera ends
    the session; the camera is reopened with a new pipeline whose queues work.
    """
    telemetry_dir = tmp_path / "snapshots" / "telemetry"
    depth_stream.touch_lease(telemetry_dir)

    def renumbered(groups, first_sequence, first_t):
        return [{key: FakeMsg(first_sequence + n, first_t + 0.1 * n, frame=msg.frame, features=msg.trackedFeatures)
                 for key, msg in group.items()} for n, group in enumerate(groups)]

    first = renumbered(scenario(n_good=6, n_lost=0, n_back=0), 500, 60.0)  # running for a minute
    restarted = renumbered(scenario(n_good=5, n_lost=0, n_back=0), 0, 0.2)
    stuck = session_handles(pnp_vio, first + restarted[:1], snapshot_reply=False)
    fresh = session_handles(pnp_vio, restarted[1:])

    def between(opened):
        snapshots.request_snapshot("depth", tmp_path / "snapshots")
        depth_stream.touch_lease(telemetry_dir)

    lines, devices, telemetry = run_sessions(pnp_vio, monkeypatch, tmp_path, [stuck, fresh], between)
    steps = [line for line in lines if line["type"] == "step"]
    # Session 1: 6 frames; the restarted camera's first frame is dropped; session 2: 4 frames from a new reference
    assert [s["sequence"] for s in steps] == [500, 501, 502, 503, 504, 505, 1, 2, 3, 4]
    assert [s["reason"] for s in steps].count("no_reference") == 2
    (reconnect,) = [line for line in lines if line.get("event") == "reconnect"]
    assert reconnect["count"] == 1 and "the camera restarted: frame sequence 505 -> 0" in reconnect["reason"]
    assert len(devices) == 2 and devices[0].close.called and not devices[1].close.called
    # Every session registers for depthai's reconnection callback
    assert all(d.setMaxReconnectionAttempts.call_args[0][0] == 1 for d in devices)
    # The new pipeline's queues: the snapshot is served, the depth stream switched on again
    assert [m.setSequenceNum.call_args[0][0] for m in fresh.snapshot_request_queue.sent] == [
        snapshots.request_number("depth", raw=True)]
    assert snapshots.read_metadata("depth", tmp_path / "snapshots")["sequence"] == 99
    assert [m.setSequenceNum.call_args[0][0] for m in fresh.depth_stream_control_queue.sent] == [1]
    assert [m.setSequenceNum.call_args[0][0] for m in stuck.depth_stream_control_queue.sent] == [1]
    assert telemetry.extra["reconnects"] == 1 and "restarted" in telemetry.extra["last_reconnect"]["reason"]
    assert sum(1 for line in lines if line.get("event") == "depth_stream") == 2


def test_run_restarts_the_pipeline_on_depthais_reconnected_callback(pnp_vio, monkeypatch, tmp_path):
    first = session_handles(pnp_vio, scenario(n_good=8, n_lost=0, n_back=0))
    second = session_handles(pnp_vio, scenario(n_good=3, n_lost=0, n_back=0))
    devices = []
    real_get = first.sync_queue.get

    def get():  # depthai's monitor thread reports RECONNECTED while the host waits for frame 4
        if len(first.sync_queue.items) == 4:
            devices[0].setMaxReconnectionAttempts.call_args[0][1](SimpleNamespace(name="RECONNECTED"))
        return real_get()

    first.sync_queue.get = get
    lines, _, _ = run_sessions(pnp_vio, monkeypatch, tmp_path, [first, second], devices=devices)
    steps = [line for line in lines if line["type"] == "step"]
    # Frame 4 arrived after the reconnect: not used; the next session starts over on the new pipeline
    assert [s["sequence"] for s in steps] == [0, 1, 2, 3, 0, 1, 2]
    (reconnect,) = [line for line in lines if line.get("event") == "reconnect"]
    assert "depthai reconnected" in reconnect["reason"] and len(devices) == 2


def test_run_with_the_depth_stream_disabled_logs_it(pnp_vio, monkeypatch, tmp_path):
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=4, n_lost=0, n_back=0), [])
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["depth_stream"]["enabled"] is False
    assert not (tmp_path / "snapshots" / "telemetry" / depth_stream.META_FILE).exists()


# --- camera matrix selection, depth scale and depth probe in pnp_vio ---------------------------

CALIB = json.loads((Path(__file__).parent / "data" / "calib_oakd_w_poe.json").read_text())
K_RAW = Intrinsics.from_matrix(CALIB["cam_b"]["K"])


def device_calibration(R_cam_imu=None):
    return StereoCalibration(raw=K_RAW, distortion=CALIB["cam_b"]["distortion"], sockets={"cam_b": K_RAW},
                             baseline_m=CALIB["baseline_cm"] / 100, baseline_spec_m=CALIB["baseline_spec_cm"] / 100,
                             R_cam_imu=R_cam_imu)


def depth_msg_with(matrix, size=(WIDTH, HEIGHT)):
    transformation = SimpleNamespace(getIntrinsicMatrix=lambda: matrix, getSize=lambda: size)
    return SimpleNamespace(getTransformation=lambda: transformation)


RAW_MATRIX = CALIB["cam_b"]["K"]


def test_parse_args_intrinsics_and_depth_scale(pnp_vio, capsys):
    args = pnp_vio.parse_args([])
    assert args.intrinsics_from == "rectified" and args.intrinsics_override is None and args.depth_scale == 1.0
    args = pnp_vio.parse_args(["--intrinsics-override", "163.1,237.1,329.4,199", "--depth-scale", "1.33",
                               "--intrinsics-from", "frame"])
    assert args.intrinsics_override == (163.1, 237.1, 329.4, 199.0) and args.depth_scale == 1.33
    for bad in (["--depth-scale", "0.1"], ["--depth-scale", "x"], ["--intrinsics-override", "1,2,3"],
                ["--intrinsics-from", "sideways"]):
        with pytest.raises(SystemExit):
            pnp_vio.parse_args(bad)
    assert "depth scale must be between" in capsys.readouterr().err


def test_parse_args_refractive_index(pnp_vio, capsys):
    assert pnp_vio.parse_args([]).refractive_index == 1.333
    assert pnp_vio.parse_args(["--refractive-index", "1.0"]).refractive_index == 1.0
    assert pnp_vio.parse_args(["--refractive-index", "2"]).refractive_index == 2.0
    for bad in ("0.99", "2.01", "nan", "water"):
        with pytest.raises(SystemExit):
            pnp_vio.parse_args(["--refractive-index", bad])
    assert "refractive index must be between 1.0 and 2.0" in capsys.readouterr().err


@pytest.fixture
def plain_pnp_vio(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", mock.MagicMock())
    sys.modules.pop("pnp_vio", None)
    yield importlib.import_module("pnp_vio")
    sys.modules.pop("pnp_vio", None)


def test_load_calibration_reads_distortion_and_baseline(plain_pnp_vio):
    pnp_vio = plain_pnp_vio
    calib = mock.MagicMock()
    calib.getCameraIntrinsics.side_effect = lambda socket, w, h: (
        RAW_MATRIX if socket is pnp_vio.dai.CameraBoardSocket.CAM_B else CALIB["cam_c"]["K"])
    calib.getDistortionCoefficients.return_value = CALIB["cam_b"]["distortion"]
    calib.getBaselineDistance.side_effect = lambda a, b, spec: 7.5 if spec else 7.496
    calib.getImuToCameraExtrinsics.return_value = np.eye(4).tolist()
    device = SimpleNamespace(readCalibration=lambda: calib)

    calibration = pnp_vio.load_calibration(device, pnp_vio.parse_args(["--intrinsics-from", "cam_c"]))
    assert calibration.raw == K_RAW and calibration.distortion == CALIB["cam_b"]["distortion"]
    assert calibration.sockets["cam_c"] == Intrinsics.from_matrix(CALIB["cam_c"]["K"])
    assert calibration.baseline_m == pytest.approx(0.07496) and calibration.baseline_spec_m == pytest.approx(0.075)
    np.testing.assert_array_equal(calibration.R_cam_imu, np.eye(3))
    calib.getDistortionCoefficients.assert_called_once_with(pnp_vio.dai.CameraBoardSocket.CAM_B)

    calib.getDistortionCoefficients.side_effect = RuntimeError("no intrinsic data")
    calib.getBaselineDistance.side_effect = RuntimeError("no extrinsics")
    calibration = pnp_vio.load_calibration(device, pnp_vio.parse_args(["--gyro-gate", "0"]))
    assert calibration.distortion == [] and calibration.baseline_m is None and calibration.R_cam_imu is None
    assert "cam_c" not in calibration.sockets


@pytest.mark.parametrize("argv, depth_msg, source, fx", [
    ([], depth_msg_with(RAW_MATRIX), "rectified", 163.1),
    (["--intrinsics-from", "frame"], depth_msg_with(RAW_MATRIX), "rectified (frame matrix is raw)", 163.1),
    (["--intrinsics-from", "frame"], depth_msg_with([[170, 0, 320], [0, 240, 200], [0, 0, 1]]), "frame", 170.0),
    (["--intrinsics-from", "frame"], SimpleNamespace(getTransformation=lambda: 1 / 0), "rectified (fallback)", 163.1),
    (["--intrinsics-from", "cam_b"], depth_msg_with(RAW_MATRIX), "cam_b", K_RAW.fx),
    (["--intrinsics-override", "150,220,330,200"], depth_msg_with(RAW_MATRIX), "override", 150.0),
])
def test_select_intrinsics(pnp_vio, argv, depth_msg, source, fx):
    args = pnp_vio.parse_args(argv)
    calibration = device_calibration()
    rectified, problem = pnp_vio.rectified_for_settings(calibration, pnp_vio.resolve_stereo_settings(args),
                                                        (args.width, args.height))
    choice = pnp_vio.select_intrinsics(args, calibration, depth_msg, rectified, problem)
    assert choice.source == source
    assert choice.intrinsics.fx == pytest.approx(fx, rel=0.005)


def test_select_intrinsics_without_alpha_falls_back_to_the_frame_with_a_warning(pnp_vio):
    args = pnp_vio.parse_args(["--stereo-alpha", "none"])
    calibration = device_calibration()
    rectified, problem = pnp_vio.rectified_for_settings(calibration, pnp_vio.resolve_stereo_settings(args), (640, 400))
    assert rectified is None
    choice = pnp_vio.select_intrinsics(args, calibration, depth_msg_with(RAW_MATRIX), rectified, problem)
    assert choice.source == "frame (fallback)" and choice.intrinsics == K_RAW
    (warning,) = choice.warnings
    assert "alpha scaling is not set" in warning and "unverified" in warning


def test_run_uses_the_rectified_matrix_and_serves_a_depth_probe(pnp_vio, monkeypatch, tmp_path):
    """Default config, a device whose depth frames carry the raw K (as the OAK-D-W-PoE does)."""
    monkeypatch.setattr(pnp_vio, "load_calibration", lambda device, args: device_calibration())
    monkeypatch.setattr(FakeTransformation, "getIntrinsicMatrix", lambda self: RAW_MATRIX)
    probe_dir = tmp_path / "snapshots" / "depth_probe"
    request_id = depth_probe.request_probe(depth_probe.make_request(320, 200, 41, 41, 5, (WIDTH, HEIGHT))[0], probe_dir)
    monkeypatch.setattr(depth_probe, "_run_in_thread", lambda fn: fn())
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(12, 0, 0),
                        ["--depth-scale", "1.25", "--refractive-index", "1.0"])
    events = {line["event"]: line for line in lines if line["type"] == "event"}

    intrinsics = events["intrinsics"]
    assert intrinsics["source"] == "rectified" and intrinsics["requested_source"] == "rectified"
    np.testing.assert_allclose(intrinsics["intrinsics"], [163.1, 237.1, 329.4, 199.0], rtol=0.005)
    assert intrinsics["effective_intrinsics"] == intrinsics["intrinsics"] and intrinsics["refractive_index"] == 1.0
    assert intrinsics["raw"] == pytest.approx(RAW_MATRIX[0][:1] + [RAW_MATRIX[1][1], RAW_MATRIX[0][2], RAW_MATRIX[1][2]])
    assert intrinsics["frame_matches"] == "raw" and intrinsics["warnings"] == []
    assert intrinsics["alpha"] == 0.0 and intrinsics["depth_scale"] == 1.25
    assert intrinsics["baseline_m"] == pytest.approx(0.07496, abs=1e-5)
    assert events["calibration"]["rectified"] == intrinsics["intrinsics"]
    assert len(events["calibration"]["distortion"]) == 14
    assert events["start"]["depth_scale"] == 1.25 and events["start"]["intrinsics_from"] == "rectified"
    assert not [line for line in lines if line.get("event") == "warning" and "Intrinsics" in line["message"]]

    camera = snapshots.read_camera_status(tmp_path / "snapshots")
    assert camera["source"] == "rectified" and camera["intrinsics"] == intrinsics["intrinsics"]
    assert camera["image_size"] == [WIDTH, HEIGHT] and camera["depth_range_mm"] == [300, 4000]

    result = depth_probe.wait_for_probe(request_id, 0, probe_dir)
    assert result["frames"] == 5 and result["median_mm"] == pytest.approx(2000.0)
    assert result["raw_m"] == pytest.approx(2.0) and result["scaled_m"] == pytest.approx(2.5)
    assert result["intrinsics_source"] == "rectified" and result["sequences"] == [0, 1, 2, 3, 4]


def test_run_with_frame_source_ignores_a_raw_frame_matrix_loudly(pnp_vio, monkeypatch, tmp_path):
    monkeypatch.setattr(pnp_vio, "load_calibration", lambda device, args: device_calibration())
    monkeypatch.setattr(FakeTransformation, "getIntrinsicMatrix", lambda self: RAW_MATRIX)
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(3, 0, 0), ["--intrinsics-from", "frame"])
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["intrinsics"]["source"] == "rectified (frame matrix is raw)"
    warnings = [line["message"] for line in lines if line.get("event") == "warning"]
    assert any("unrectified K; ignored" in message for message in warnings)


# --- lever arm diagnostics ------------------------------------------------------------------------

LEVER_ARM = np.array([0.1655, -0.0069, 0.180])
SPIN_RATE = np.radians(20.0)  # rad/s, nose right
SPIN_DT = 0.1
FLOOR = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)


def yaw_frd(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def spin_scenario(frames=8, lever_arm=LEVER_ARM, rate=SPIN_RATE):
    """
    A level vehicle spinning in place about its IMU, camera looking straight down (--camera-angle 90) at a
    floor 2 m below it, at lever_arm from the IMU. The floor stays at 2 m, so the depth is uniform; the camera
    swings around the IMU, so the feature motion includes a translation. Returns (sync groups, gyro message):
    the gyro in the camera frame (R_cam_imu = I).
    """
    M = camera_to_body_matrix(90.0)  # optical -> body
    floor = M @ back_project(GOOD_UV, np.full(len(GOOD_UV), 2.0), K).T + lever_arm[:, None]  # world = body at t0
    groups = []
    for n in range(frames):
        R_wb = yaw_frd(rate * SPIN_DT * n)
        R_wc, p_c = R_wb @ M, R_wb @ lever_arm
        X = R_wc.T @ (floor - p_c[:, None])
        uv = np.stack([K.fx * X[0] / X[2] + K.cx, K.fy * X[1] / X[2] + K.cy], axis=1)
        t = 1.0 + SPIN_DT * n
        groups.append({"features": FakeMsg(n, t, features=features(range(len(uv)), uv)),
                       "depth": FakeMsg(n, t, frame=FLOOR)})
    omega_cam = M.T @ np.array([0.0, 0.0, rate])
    packets = [SimpleNamespace(gyroscope=SimpleNamespace(getTimestampDevice=lambda t=t: timedelta(seconds=t),
                                                         x=omega_cam[0], y=omega_cam[1], z=omega_cam[2]))
               for t in np.arange(0.9, 1.0 + SPIN_DT * frames, 0.005)]
    return groups, SimpleNamespace(packets=packets)


SPIN_ARGS = ["--camera-angle", "90", "--refractive-index", "1.0", "--intrinsics-override", "300,300,320,200",
             "--gyro-gate", "5", "--recovery-frames", "0"]


def expected_spin_delta(rate=SPIN_RATE):
    """Camera displacement per frame in body FRD for the spin: r - R^T r."""
    return LEVER_ARM - yaw_frd(rate * SPIN_DT).T @ LEVER_ARM


def test_run_lever_arm_diagnostics_cancel_a_pure_spin(pnp_vio, monkeypatch, tmp_path):
    monkeypatch.setattr(pnp_vio, "load_calibration", lambda device, args: fake_calibration(np.eye(3)))
    groups, gyro = spin_scenario()
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, groups, [*SPIN_ARGS, "--lever-arm"], imu=[gyro])
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["start"]["lever_arm"] is True and events["start"]["lever_arm_m"] == LEVER_ARM.tolist()
    steps = [line for line in lines if line["type"] == "step"]
    assert steps[0]["reason"] == "no_reference" and steps[0]["lever_delta_position"] is None
    for step in steps[1:]:
        assert step["status"] == "ok", step["reason"]
        # What is (or would be) sent stays the camera's own motion, swinging on the lever arm
        np.testing.assert_allclose(step["position_delta"], expected_spin_delta(), atol=1e-3)
        np.testing.assert_allclose(step["angle_delta"], [0.0, 0.0, SPIN_RATE * SPIN_DT], atol=np.radians(0.05))
        # The diagnostics take the gyro rotation and find the IMU did not move
        assert step["lever_delta_rotation"] == "gyro" and step["gyro_disagreement_deg"] < 0.1
        np.testing.assert_allclose(step["lever_delta_correction"], expected_spin_delta(), atol=1e-4)
        np.testing.assert_allclose(step["lever_delta_position"], 0.0, atol=1e-3)
    assert np.linalg.norm(expected_spin_delta()) > 0.005


def test_run_without_gyro_uses_the_pnp_rotation_and_lever_arm_off_logs_nothing(pnp_vio, monkeypatch, tmp_path):
    groups, _ = spin_scenario(5)
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, groups,
                        [*SPIN_ARGS, "--gyro-gate", "0", "--lever-arm", "--lever-arm-m=0.1655,-0.0069,0.18"])
    steps = [line for line in lines if line["type"] == "step"]
    assert all(s["lever_delta_rotation"] == "pnp" for s in steps[1:])
    np.testing.assert_allclose([s["lever_delta_position"] for s in steps[1:]], 0.0, atol=1e-3)

    next((tmp_path / "logs").iterdir()).unlink()  # run_fake expects a single log
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, spin_scenario(3)[0], [*SPIN_ARGS, "--gyro-gate", "0"])
    steps = [line for line in lines if line["type"] == "step"]
    assert not any(key.startswith("lever_") for step in steps for key in step)
    assert {line["event"]: line for line in lines if line["type"] == "event"}["start"]["lever_arm"] is False


# --- MAVLink output (log_only off) against a fake mavlink2rest websocket ------------------------

class FakeWebSocketApp:
    """Stands in for websocket.WebSocketApp: opens at once and records what is sent."""
    instances = []

    def __init__(self, url, on_open=None, on_close=None, on_error=None):
        self.url, self.on_open, self.sent, self.closed = url, on_open, [], False
        FakeWebSocketApp.instances.append(self)

    def run_forever(self):
        self.on_open(self)

    def send(self, data):
        self.sent.append(json.loads(data))

    def close(self):
        self.closed = True


@pytest.fixture
def mavlink_out(pnp_vio, monkeypatch):
    """Real MavlinkSender over a fake websocket; any autopilot parameter request fails the test."""
    import autopilot_params
    import mavlink_sender
    FakeWebSocketApp.instances = []
    monkeypatch.setattr(mavlink_sender.websocket, "WebSocketApp", FakeWebSocketApp)
    for name in ("_post", "read", "write"):
        monkeypatch.setattr(autopilot_params.AutopilotParams, name,
                            lambda *args, **kwargs: pytest.fail("the VIO must not touch autopilot parameters"))
    monkeypatch.setattr(pnp_vio, "load_calibration", lambda device, args: fake_calibration(np.eye(3)))
    return pnp_vio


def test_run_sends_vision_position_delta_matching_the_log(mavlink_out, monkeypatch, tmp_path):
    pnp_vio = mavlink_out
    groups, gyro = spin_scenario(12)
    before_us = time.time() * 1e6
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, groups,
                        [*SPIN_ARGS, "--lever-arm", "--vehicle-ip", "10.0.0.9", "--quality-rate", "0"],
                        imu=[gyro], log_only=False)
    after_us = time.time() * 1e6
    (ws,) = FakeWebSocketApp.instances
    assert ws.url == "ws://10.0.0.9/mavlink2rest/ws/mavlink?filter=SEND_ONLY" and ws.closed
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["start"]["log_only"] is False

    steps = [line for line in lines if line["type"] == "step"]
    sent = [m for m in ws.sent if m["message"]["type"] == "VISION_POSITION_DELTA"]
    assert [s["sent"] for s in steps] == [False] + [True] * 11 and len(sent) == 11
    for step, packet in zip(steps[1:], sent):
        assert packet["header"] == {"system_id": 1, "component_id": 195, "sequence": 0}
        message = packet["message"]
        assert set(message) == {"type", "time_usec", "time_delta_usec", "angle_delta", "position_delta", "confidence"}
        # Exactly the logged body-FRD deltas: metres and radians; the camera's own delta, not the lever-arm one
        assert message["position_delta"] == pytest.approx(step["position_delta"], abs=1e-12)
        assert message["angle_delta"] == pytest.approx(step["angle_delta"], abs=1e-12)
        np.testing.assert_allclose(message["position_delta"], expected_spin_delta(), atol=1e-3)
        assert np.linalg.norm(np.subtract(message["position_delta"], step["lever_delta_position"])) > 0.004
        np.testing.assert_allclose(message["angle_delta"], [0.0, 0.0, SPIN_RATE * SPIN_DT], atol=np.radians(0.05))
        # Microseconds of device time, rounded (not truncated: 1.2 - 1.1 is 0.0999999... s)
        assert isinstance(message["time_delta_usec"], int) and message["time_delta_usec"] == 100000
        assert isinstance(message["time_usec"], int) and before_us <= message["time_usec"] <= after_us
        assert 0.0 < message["confidence"] <= 100.0 and message["confidence"] == pytest.approx(step["confidence"])

    names = ["".join(m["message"]["name"]).rstrip("\u0000") for m in ws.sent
             if m["message"]["type"] == "NAMED_VALUE_FLOAT"]
    # quality_rate 0: only the tracking change goes out
    assert names == ["VIO_TRACK"]


def test_sender_message_fields_and_disconnected_drop(monkeypatch):
    import mavlink_sender

    class NeverOpens(FakeWebSocketApp):
        def run_forever(self):
            pass
    FakeWebSocketApp.instances = []
    monkeypatch.setattr(mavlink_sender.websocket, "WebSocketApp", NeverOpens)
    sender = mavlink_sender.MavlinkSender("ws://fake/mavlink2rest/ws/mavlink", reconnect_interval=0.0)
    # Not connected: dropped (False) without blocking, with a reconnect attempt
    assert sender.send_vision_position_delta([0.0, 0.0, 0.01], [0.1, 0.0, 0.0], 99999.9999, 42.0) is False
    (ws,) = FakeWebSocketApp.instances
    assert ws.sent == []
    ws.on_open(ws)
    assert sender.send_vision_position_delta(np.array([0.0, 0.0, 0.01]), np.array([0.1, -0.2, 0.05]),
                                             99999.9999, np.float64(42.5), timestamp_us=1.7e15) is True
    (packet,) = ws.sent
    assert packet["message"] == {"type": "VISION_POSITION_DELTA", "time_usec": 1700000000000000,
                                 "time_delta_usec": 100000, "angle_delta": [0.0, 0.0, 0.01],
                                 "position_delta": [0.1, -0.2, 0.05], "confidence": 42.5}


def test_run_counts_a_failed_send_as_not_sent(mavlink_out, monkeypatch, tmp_path):
    pnp_vio = mavlink_out

    def broken_send(self, data):
        raise ConnectionError("mavlink2rest went away")
    monkeypatch.setattr(FakeWebSocketApp, "send", broken_send)
    groups, gyro = spin_scenario(4)
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, groups, SPIN_ARGS, imu=[gyro], log_only=False)
    steps = [line for line in lines if line["type"] == "step"]
    assert [s["status"] for s in steps] == ["rejected", "ok", "ok", "ok"]
    assert not any(s["sent"] for s in steps)


def test_parse_args_lever_arm(pnp_vio, capsys):
    args = pnp_vio.parse_args([])
    assert args.lever_arm is False and args.lever_arm_m == (0.1655, -0.0069, 0.180)
    args = pnp_vio.parse_args(["--lever-arm", "--lever-arm-m=-0.1,0,0.25"])
    assert args.lever_arm is True and args.lever_arm_m == (-0.1, 0.0, 0.25)
    for bad in (["--lever-arm-m", "1,2"], ["--lever-arm-m", "0,0,6"], ["--lever-arm-m", "a,b,c"]):
        with pytest.raises(SystemExit):
            pnp_vio.parse_args(bad)
    assert "lever arm" in capsys.readouterr().err


def test_run_applies_the_refractive_index_after_choosing_the_matrix(pnp_vio, monkeypatch, tmp_path):
    """Default n 1.333 on top of an override: fx, fy x n for PnP; the in-air K stays what the focal check reads."""
    probe_dir = tmp_path / "snapshots" / "depth_probe"
    request_id = depth_probe.request_probe(depth_probe.make_request(320, 200, 41, 41, 3, (WIDTH, HEIGHT))[0], probe_dir)
    monkeypatch.setattr(depth_probe, "_run_in_thread", lambda fn: fn())
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(6, 0, 0),
                        ["--intrinsics-override", "160,230,330,199", "--depth-scale", "1.1"])
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["start"]["refractive_index"] == 1.333 and events["start"]["depth_scale"] == 1.1

    intrinsics = events["intrinsics"]
    assert intrinsics["source"] == "override" and intrinsics["intrinsics"] == [160.0, 230.0, 330.0, 199.0]
    np.testing.assert_allclose(intrinsics["effective_intrinsics"], [160 * 1.333, 230 * 1.333, 330.0, 199.0])
    assert intrinsics["refractive_index"] == 1.333 and intrinsics["depth_scale"] == 1.1
    assert intrinsics["total_depth_scale"] == pytest.approx(1.333 * 1.1)

    camera = snapshots.read_camera_status(tmp_path / "snapshots")
    assert camera["intrinsics"] == [160.0, 230.0, 330.0, 199.0]
    assert camera["effective_intrinsics"] == intrinsics["effective_intrinsics"]
    assert camera["refractive_index"] == 1.333 and camera["total_depth_scale"] == pytest.approx(1.4663)

    result = depth_probe.wait_for_probe(request_id, 0, probe_dir)
    assert result["raw_m"] == pytest.approx(2.0) and result["scaled_m"] == pytest.approx(2.0 * 1.333 * 1.1)
    assert result["intrinsics"]["fx"] == 160.0 and result["refractive_index"] == 1.333
    # Static scene: still no motion in water
    steps = [line for line in lines if line["type"] == "step"]
    assert all(s["status"] == "ok" for s in steps[1:])
    np.testing.assert_allclose([s["position_delta"] for s in steps[1:]], 0.0, atol=1e-6)


# --- depth_transport points: the build and the run -------------------------------------------------------------

def test_build_pipeline_points_sends_no_depth_frames(pnp_vio):
    """points (the default): no Sync; FeatureTracker -> host; depth -> Script -> SLC (lockstep) -> Script -> host."""
    import depth_points
    dai = pnp_vio.dai
    handles, created = build_fake_pipeline(pnp_vio, ["--depth-transport", "points"])
    assert handles.depth_transport == "points" and handles.sync_queue is None
    assert nodes_of(created, dai.node.Sync) == []
    (stereo,) = nodes_of(created, dai.node.StereoDepth)
    (tracker,) = nodes_of(created, dai.node.FeatureTracker)
    (slc,) = nodes_of(created, dai.node.SpatialLocationCalculator)
    points_script, gate_script = nodes_of(created, dai.node.Script)[:2]
    # The SLC runs on what the Script hands it, one config per depth frame, both inputs blocking size 1
    stereo.depth.link.assert_any_call(points_script.inputs["depth"])
    stereo.depth.link.assert_any_call(gate_script.inputs["depth"])
    assert mock.call(slc.inputDepth) not in stereo.depth.link.call_args_list
    points_script.outputs["config"].link.assert_called_once_with(slc.inputConfig)
    points_script.outputs[depth_points.SLC_DEPTH_OUT].link.assert_called_once_with(slc.inputDepth)
    gate_script.outputs[depth_points.GATE_OUT].createOutputQueue.assert_called_once_with(maxSize=2, blocking=False)
    slc.out.link.assert_called_once_with(points_script.inputs["spatial"])
    slc.passthroughDepth.link.assert_called_once_with(points_script.inputs["passthrough"])
    slc.inputConfig.setWaitForMessage.assert_called_once_with(True)
    for port in (slc.inputConfig, slc.inputDepth):
        port.setBlocking.assert_called_with(True)
        port.setMaxSize.assert_called_with(1)
    points_script.inputs["depth"].setBlocking.assert_called_with(False)
    points_script.inputs["depth"].setMaxSize.assert_called_with(depth_points.DEPTH_BUFFER)
    points_script.inputs["rois"].setMaxSize.assert_called_with(depth_points.ROIS_QUEUE)
    # The Pi's configs go up into the Script; the tracker's features only go to the Pi
    assert handles.rois_queue is not None and handles.features_queue is not None
    assert mock.call(points_script.inputs["features"]) not in tracker.outputFeatures.link.call_args_list
    compile(points_script.setScript.call_args[0][0], "depth_points_script", "exec")
    compile(gate_script.setScript.call_args[0][0], "gate_script", "exec")
    assert handles.full_depth_control_queue is not None and handles.full_depth_queue is not None


class FeaturesMsg(FakeMsg):
    def getTimestamp(self):
        return timedelta(seconds=self.t)


class PointsQueue(FakeQueue):
    """The Pi's points queue: get(timeout) waits (briefly) like depthai's and returns None when nothing came."""

    def get(self, timeout=None):
        if self.items:
            return self.items.pop(0)
        time.sleep(min(timeout.total_seconds(), 0.02) if timeout else 0.0)
        return None


class KeptUpQueue(FakeQueue):
    """The features queue of a Pi that keeps up: nothing more is queued when it asks (FakePipeline runs on it)."""

    def tryGetAll(self):
        return []


def run_fake_points(module, monkeypatch, tmp_path, frames, argv=(), drop=(), corrupt=(), full_depth=(), **fields):
    """
    pnp_vio in points mode on fakes: the tracker's features (frames: (sequence, t, features)), and a camera that
    gets what the Pi sends through slc_fakes.over_link (a config's own sequence number reads 0), pairs each
    config with the tag before it as the Script does, and answers with the strict fake SLC on DEPTH, re-tagged
    with the frame, except for the sequences in drop (no answer) and corrupt (ROIs moved by 10 px).
    """
    import slc_fakes
    for name in ("SpatialLocationCalculatorConfig", "SpatialLocationCalculatorConfigData",
                 "SpatialLocationCalculatorAlgorithm", "Rect", "Buffer"):
        monkeypatch.setattr(module.dai, name, getattr(slc_fakes.FAKE_DAI, name))
    times = {sequence: t for sequence, t, _ in frames}
    features_queue = KeptUpQueue([FeaturesMsg(s, t, features=f) for s, t, f in frames])
    points_queue = PointsQueue()
    rois_queue = FakeQueue()
    rois_queue.tags = []

    def camera(message):
        rois_queue.sent.append(message)
        received = slc_fakes.over_link(message)
        if not isinstance(received, slc_fakes.Config):
            rois_queue.tags.append(received.getSequenceNum())
            return
        assert received.getSequenceNum() == 0 and rois_queue.tags, "a config without its tag"
        sequence = rois_queue.tags.pop()
        if sequence in drop:
            return
        if sequence in corrupt:
            for roi in received.rois:
                roi.roi.x = min(roi.roi.x + 10, WIDTH - roi.roi.width)
        points_queue.items.append(slc_fakes.answer(received, DEPTH, times[sequence], sequence))

    rois_queue.send = camera
    # The gate forwards full depth frames once told to (control sequence number > 0)
    gate_out, gate_control = FakeQueue(), FakeQueue()
    waiting = list(full_depth)

    def control(msg):
        gate_control.sent.append(msg)
        if msg.getSequenceNum() > 0:
            gate_out.items.extend(waiting)
            waiting.clear()

    gate_control.send = control
    size = np.frombuffer(f"size\t0\t0\t0\t{WIDTH} {HEIGHT}".encode(), np.uint8)
    handles = module.PipelineHandles(
        sync_queue=None, imu_queue=FakeQueue(), feature_config_queue=FakeQueue(), stereo_effective={"decimation": 1},
        snapshot_request_queue=FakeQueue(), snapshot_queue=FakeQueue(), snapshot_streams=("rect_left",),
        snapshot_unavailable={}, depth_transport="points", features_queue=features_queue, points_queue=points_queue,
        points_status_queue=FakeQueue([SimpleNamespace(getData=lambda: size)]), rois_queue=rois_queue,
        full_depth_control_queue=gate_control, full_depth_queue=gate_out, **fields)
    monkeypatch.setattr(module, "build_pipeline", lambda p, device, args, settings: handles)
    monkeypatch.setattr(module.dai, "Pipeline", lambda device: FakePipeline(features_queue))
    monkeypatch.setattr(module.dai, "FeatureTrackerConfig", lambda: mock.MagicMock())
    module.main(["--log-only", "--gyro-gate", "0", "--snapshot-dir", str(tmp_path / "snapshots"),
                 "--depth-transport", "points", *argv])
    (log_file,) = (tmp_path / "logs").iterdir()
    return [json.loads(line) for line in log_file.read_text().splitlines()], handles


def static_frames(n):
    return [(s, 1.0 + 0.1 * s, features(range(len(GOOD_UV)), GOOD_UV)) for s in range(n)]


def test_run_points_pairs_each_frame_and_a_missing_one_has_no_depth(pnp_vio, monkeypatch, tmp_path):
    import depth_points
    import link_stats
    created = recorded_telemetry(pnp_vio, monkeypatch)
    lines, handles = run_fake_points(pnp_vio, monkeypatch, tmp_path, static_frames(8), drop={3}, corrupt={5},
                                     argv=["--telemetry-dir", str(tmp_path / "telemetry")])
    steps = [line for line in lines if line["type"] == "step"]
    assert len(steps) == 8 and all(s["depth_source"] == "points" for s in steps)
    assert [s["depth_problem"] for s in steps] == [None, None, None, "timeout", None, "roi", None, None]
    # Frames with depth: every feature off the no-depth strip has it; the missing ones have none (never stale)
    full = [s["valid_depth"] for s in steps if s["depth_problem"] is None]
    assert set(full) == {int((GOOD_UV[:, 0] >= 40 + 2).sum())}
    assert steps[3]["valid_depth"] == 0 and steps[5]["valid_depth"] == 0
    # A frame without depth still solves against the reference, which it carries on (moved into its camera),
    # so the next frame solves too: no delta lost
    assert [s["status"] for s in steps[1:]] == ["ok"] * 7
    assert [s["reference_carried"] for s in steps] == [False] * 3 + [True, False, True, False, False]
    assert not any(s["reference_reset"] for s in steps)
    events = {line["event"]: line for line in lines if line["type"] == "event"}
    assert events["depth_transport"]["transport"] == "points" and "rois" not in events["depth_transport"]

    (telemetry,) = created
    status = telemetry.depth_points_status()
    assert status["transport"] == "points" and status["frames"] == 8 and status["paired"] == 6
    assert status["missing_by_reason"] == {"timeout": 1, "roi": 1} and status["configs_sent"] == 8
    assert status["full_depth"]["on"] is False
    totals = telemetry.link.streams.totals()
    n = len(GOOD_UV) + 3
    assert "depth" not in totals and "sync_group" not in totals
    assert totals["depth_points"] == (7, 7 * link_stats.spatial_data_bytes(n))
    assert totals["depth_points_rois"] == (8, 8 * link_stats.points_request_bytes(n))
    ids = np.arange(len(GOOD_UV))
    assert totals["features"] == (8, 8 * link_stats.features_bytes(ids))
    # Tag then config per frame: the configs' own numbers read 0 on the camera, the tags carried the frames
    sent = handles.rois_queue.sent
    assert [m.getSequenceNum() for m in sent[0::2]] == list(range(8)) and not handles.rois_queue.tags
    assert all(isinstance(m, pnp_vio.dai.SpatialLocationCalculatorConfig) for m in sent[1::2])
    assert status["requested_ahead_max"] == 1 and status["host_behind_dropped"] == 0
    assert status["wait_limit_ms"] == round(depth_points.wait_s(30) * 1000.0, 1)
    # The gate: told "off" once, nothing forwarded
    assert [m.getSequenceNum() for m in handles.full_depth_control_queue.sent] == [0]


def test_max_features_caps_the_tracker_only_when_set(pnp_vio, monkeypatch):
    detectors = []

    class CornerDetector:
        def __init__(self):
            detectors.append(self)

    monkeypatch.setattr(pnp_vio.dai.FeatureTrackerConfig, "CornerDetector", CornerDetector)
    harris = pnp_vio.CORNER_DETECTORS["harris"]
    config = mock.MagicMock()
    pnp_vio.configure_feature_tracker(config, pnp_vio.parse_args([]))
    # 0 (the default): depthai's automatic maximum, the two setters as before
    assert detectors == [] and config.setCornerDetector.call_args_list == [mock.call(harris)]
    config = mock.MagicMock()
    pnp_vio.configure_feature_tracker(config, pnp_vio.parse_args(["--num-features", "250", "--max-features", "200"]))
    (detector,) = detectors
    assert (detector.type, detector.numTargetFeatures, detector.numMaxFeatures) == (harris, 250, 200)
    assert config.setCornerDetector.call_args_list[-1] == mock.call(detector)
    config.setNumTargetFeatures.assert_called_once_with(250)


def test_points_source_requests_what_is_queued_but_never_more_than_the_camera_holds(pnp_vio, monkeypatch):
    import depth_points
    import link_stats
    import slc_fakes
    for name in ("SpatialLocationCalculatorConfig", "SpatialLocationCalculatorConfigData",
                 "SpatialLocationCalculatorAlgorithm", "Rect", "Buffer"):
        monkeypatch.setattr(pnp_vio.dai, name, getattr(slc_fakes.FAKE_DAI, name))
    size = np.frombuffer(f"size\t0\t0\t0\t{WIDTH} {HEIGHT}".encode(), np.uint8)
    queue = FakeQueue()
    sent = []
    rois_queue = FakeQueue()
    rois_queue.send = sent.append
    handles = SimpleNamespace(features_queue=queue, rois_queue=rois_queue, points_queue=PointsQueue(),
                              points_status_queue=FakeQueue([SimpleNamespace(getData=lambda: size)]))
    source = pnp_vio.PointsSource(handles, link_stats.StreamCounter(), pnp_vio.parse_args(["--fps", "30"]))

    def arrive(sequences):
        queue.items += [FeaturesMsg(s, 1.0 + s / 30, features=features(range(len(GOOD_UV)), GOOD_UV))
                        for s in sequences]

    def tags():
        return [m.getSequenceNum() for m in sent if isinstance(m, slc_fakes.HostBuffer)]

    # The Pi stalled for 7 frames: the oldest is taken, the newest that fit are requested behind it, the rest
    # are dropped unrequested
    arrive(range(7))
    assert source.get().msg.getSequenceNum() == 0
    assert tags() == [0, 4, 5, 6] and depth_points.REQUESTS_AHEAD == 4
    assert [type(m) for m in sent] == [slc_fakes.HostBuffer, slc_fakes.Config] * 4
    assert source.get().msg.getSequenceNum() == 4
    arrive(range(7, 10))
    assert source.get().msg.getSequenceNum() == 5 and tags()[4:] == [8, 9]
    status = source.status()
    assert status["requested_ahead_max"] == depth_points.REQUESTS_AHEAD and status["host_behind_dropped"] == 4
    assert status["configs_sent"] == 6 and status["wait_limit_ms"] == round(1000.0 * 2.5 / 30, 1)


def test_run_points_depth_probe_turns_full_depth_on_and_off(pnp_vio, monkeypatch, tmp_path):
    probe_dir = tmp_path / "snapshots" / "depth_probe"
    request, _ = depth_probe.make_request(320, 200, 21, 21, 2, (WIDTH, HEIGHT))
    request_id = depth_probe.request_probe(request, probe_dir)
    full = [FakeMsg(100 + k, 1.0 + 0.1 * k, frame=DEPTH) for k in range(2)]
    lines, handles = run_fake_points(pnp_vio, monkeypatch, tmp_path, static_frames(8), full_depth=full)
    controls = [m.getSequenceNum() for m in handles.full_depth_control_queue.sent]
    # On for the probe (picked up on the first frame; every frame), off once it is done
    assert controls == [1, 0]
    result = json.loads((probe_dir / "results" / f"{request_id}.json").read_text())
    assert result["frames"] == 2 and result["sequences"] == [100, 101] and result["median_mm"] == 2000.0


def test_run_full_frame_is_unchanged(pnp_vio, monkeypatch, tmp_path):
    lines, handles = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_good=5, n_lost=0, n_back=0),
                              ["--depth-transport", "full_frame"])
    steps = [line for line in lines if line["type"] == "step"]
    assert all(s["depth_source"] == "frame" and s["depth_problem"] is None for s in steps)
    assert [s["status"] for s in steps[1:]] == ["ok"] * 4
