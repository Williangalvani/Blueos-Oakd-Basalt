import importlib
import json
import os
import signal
import sys
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_probe  # noqa: E402
import snapshots  # noqa: E402
import vio_log  # noqa: E402
from pnp_odometry import Intrinsics, PnPOdometry, PnPParams  # noqa: E402
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
    previous = signal.getsignal(signal.SIGTERM)
    yield module
    signal.signal(signal.SIGTERM, previous)
    sys.modules.pop("pnp_vio", None)


def run_fake(module, monkeypatch, tmp_path, groups, argv, imu=()):
    sync_queue = FakeQueue(groups)
    snapshot_queue = FakeQueue()
    request_queue = FakeQueue()
    request_queue.send = lambda msg: (request_queue.sent.append(msg), snapshot_queue.items.append(
        FakeMsg(99, 1.5, frame=DEPTH)))
    handles = module.PipelineHandles(
        sync_queue=sync_queue, imu_queue=FakeQueue(imu), feature_config_queue=FakeQueue(),
        stereo_effective={"decimation": 1}, snapshot_request_queue=request_queue, snapshot_queue=snapshot_queue,
        snapshot_streams=("rect_left", "rect_right", "depth"), snapshot_unavailable={"rgb": "no CAM_A"})
    monkeypatch.setattr(module, "build_pipeline", lambda p, device, args, settings: handles)
    monkeypatch.setattr(module.dai, "Pipeline", lambda device: FakePipeline(sync_queue))
    monkeypatch.setattr(module.dai, "FeatureTrackerConfig", lambda: mock.MagicMock())
    monkeypatch.setattr(module.dai, "Buffer", lambda: mock.MagicMock())
    module.main(["--log-only", "--gyro-gate", "0", "--snapshot-dir", str(tmp_path / "snapshots"), *argv])
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
        snapshots.STREAMS.index("depth")]
    assert (tmp_path / "snapshots" / "depth_raw.png").exists()
    assert snapshots.read_metadata("depth", tmp_path / "snapshots")["sequence"] == 99


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


def test_snapshot_script_gates_on_requests(pnp_vio):
    code = pnp_vio.snapshot_script(["rect_left", "depth"])
    compile(code, "snapshot_script", "exec")
    assert "available = ['depth', 'rect_left']" in code
    assert "node.inputs['request'].get()" in code


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
    lines, _ = run_fake(pnp_vio, monkeypatch, tmp_path, scenario(12, 0, 0), ["--depth-scale", "1.25"])
    events = {line["event"]: line for line in lines if line["type"] == "event"}

    intrinsics = events["intrinsics"]
    assert intrinsics["source"] == "rectified" and intrinsics["requested_source"] == "rectified"
    np.testing.assert_allclose(intrinsics["intrinsics"], [163.1, 237.1, 329.4, 199.0], rtol=0.005)
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
