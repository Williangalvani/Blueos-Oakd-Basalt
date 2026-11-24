"""
stereo_probe.py --mode mjpeg_loss and its analysis (mjpeg_loss.py): JPEG round trips of synthetic disparity,
the VIO's depth rules at the features, binning, flips, pairing, the subpixel comparison, the whole mode on a fake
camera, and POST /api/stereo_probe?mode=mjpeg_loss.
"""
import importlib
import json
import os
import sys
from datetime import timedelta
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_points  # noqa: E402
import mjpeg_loss  # noqa: E402
import stereo_probe  # noqa: E402
import vio_log  # noqa: E402
from pnp_odometry import Intrinsics, PnPParams, depth_from_points, lookup_depth  # noqa: E402
from rectified_intrinsics import StereoCalibration, rectified_for_settings  # noqa: E402
from slc_fakes import FAKE_DAI, slc_compute  # noqa: E402
from stereo_config import STEREO_SETTINGS, resolve_stereo_settings  # noqa: E402

STEREO_DEFAULTS = {setting.key: setting.default for setting in STEREO_SETTINGS}
from test_stereo_probe import FakeCamera, FakeDevice, FakeFrame, FakeInput, FakeStereoDepth, fake_dai  # noqa: E402

cv2 = pytest.importorskip("cv2")

SAVED = resolve_stereo_settings(STEREO_DEFAULTS)
K_MM = 163.1 * 0.075 * 1000.0  # OAK-D-W rectified fx at 640x400 (README) x 75 mm
PARAMS = PnPParams()


def jpeg(image, quality):
    return cv2.imencode(".jpg", np.asarray(image, np.uint8), [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def scene_depth(width=128, height=64):
    """True depth (mm, 0 = none): a plane at 0.8 m on the left, a ramp 1.5-3 m on the right, no depth on top."""
    yy, xx = np.mgrid[0:height, 0:width]
    z = np.where(xx < width // 2, 800.0, 1500.0 + (xx - width // 2) * 1500.0 / (width // 2))
    z[:8, :] = 0.0
    return z


def integer_disparity(z, k_mm=K_MM):
    with np.errstate(divide="ignore"):
        d = np.where(z > 0, np.round(k_mm / np.maximum(z, 1.0)), 0)
    return np.clip(d, 0, 95).astype(np.uint8)


# --- pure helpers ---------------------------------------------------------------------------------------

def test_parse_qualities():
    assert mjpeg_loss.parse_qualities(None) == [70, 85, 95, 100]
    assert mjpeg_loss.parse_qualities(" ") == [70, 85, 95, 100]
    assert mjpeg_loss.parse_qualities("95, 70,95,") == [95, 70]
    for bad, match in (("0", "outside"), ("101", "outside"), ("x", "whole number"), (",", "no quality"),
                       ("1,2,3,4,5,6,7", "at most")):
        with pytest.raises(ValueError, match=match):
            mjpeg_loss.parse_qualities(bad)


def test_disparity_to_depth():
    d = np.array([[0, 1, 2, 3, 95]], np.uint8)
    z = mjpeg_loss.disparity_depth_mm(d, K_MM)
    np.testing.assert_allclose(z, [[0.0, K_MM, K_MM / 2, K_MM / 3, K_MM / 95]])
    np.testing.assert_allclose(mjpeg_loss.disparity_depth_mm(d.astype(np.float64), K_MM), z)
    # The depth stream viewer's rule: gray below 3 is invalid
    np.testing.assert_allclose(mjpeg_loss.disparity_depth_mm(d, K_MM, 3), [[0, 0, 0, K_MM / 3, K_MM / 95]])
    assert mjpeg_loss.depth_times_disparity_mm(163.1, 0.075) == pytest.approx(K_MM)
    assert mjpeg_loss.depth_times_disparity_mm(163.1, None) is None
    assert mjpeg_loss.depth_times_disparity_mm(0.0, 0.075) is None


def test_points_rule_is_the_camera_slc_average_and_full_frame_is_lookup_depth():
    rng = np.random.default_rng(3)
    depth = rng.uniform(150, 16000, (64, 128))
    depth[rng.random(depth.shape) < 0.3] = 0  # holes
    depth = np.round(depth).astype(np.uint16)
    uv = np.vstack([rng.uniform((0, 0), (127, 63), (40, 2)), [[0, 0], [127, 63], [1.5, 62.4]]])
    params = PnPParams(min_depth_m=0.2, max_depth_m=15.0)
    sampled = mjpeg_loss.sample_rules(depth, uv, (128, 64), params)
    config = depth_points.make_config(FAKE_DAI, depth_points.all_windows(uv, (128, 64), (128, 64)),
                                      depth_points.thresholds_mm(0.2, 15.0), 1)
    points, problem = depth_points.resolve(depth_points.decode(slc_compute(config, depth)), uv, (128, 64), (128, 64))
    assert problem is None
    camera = depth_from_points(points.depth_mm, points.pixels, points.samples, params) * 1000.0
    np.testing.assert_allclose(sampled["points"], camera, rtol=1e-9, equal_nan=True)
    assert np.isnan(camera).any() and np.isfinite(camera).any()
    np.testing.assert_allclose(sampled["full_frame"], lookup_depth(depth, uv, params) * 1000.0, equal_nan=True)
    x, y = mjpeg_loss.centre_pixels(uv, (128, 64), (128, 64))
    centre = depth[y, x].astype(np.float64)
    expected = np.where((centre >= 200) & (centre <= 15000), centre, np.nan)
    np.testing.assert_allclose(sampled["centre"], expected, equal_nan=True)
    assert mjpeg_loss.sample_rules(depth, np.zeros((0, 2)), (128, 64))["points"].size == 0


def test_features_on_a_decimated_grid_are_scaled():
    depth = np.zeros((32, 64))
    depth[10, 20] = 1000.0
    sampled = mjpeg_loss.sample_rules(depth, [[40.0, 20.0]], (128, 64))
    assert sampled["centre"][0] == 1000.0
    # one valid pixel of 25: below the 30 % the VIO needs
    assert np.isnan(sampled["points"][0]) and np.isnan(sampled["full_frame"][0])


def test_compare_counts_errors_flips_and_bins():
    ref = np.array([400.0, 450.0, 800.0, 1500.0, 3000.0, 5000.0, np.nan, np.nan, 900.0])
    tst = np.array([410.0, 450.0, 760.0, 1650.0, np.nan, 5500.0, 2000.0, np.nan, 900.0])
    out = mjpeg_loss.compare(ref, tst)
    assert (out["points"], out["reference_valid"], out["both_valid"]) == (9, 7, 6)
    assert out["valid_to_invalid"] == 1 and out["valid_to_invalid_pct"] == pytest.approx(100 / 7, abs=0.01)
    assert out["invalid_to_valid"] == 1 and out["invalid_to_valid_pct"] == 50.0
    assert out["error_mm"]["max"] == 500.0 and out["error_mm"]["median"] == 25.0
    assert out["error_pct"]["max"] == 10.0 and out["exact_pct"] == pytest.approx(100 * 2 / 6, abs=0.01)
    assert out["bias_mm"] == pytest.approx((10 + 0 - 40 + 150 + 500 + 0) / 6, abs=0.1)
    bins = out["by_distance"]
    assert list(bins) == ["<0.5", "0.5-1", "1-2", "2-4", ">=4"]
    assert bins["<0.5"]["both_valid"] == 2 and bins["<0.5"]["error_mm"]["max"] == 10.0
    assert bins["0.5-1"]["both_valid"] == 2 and bins["0.5-1"]["error_mm"]["max"] == 40.0
    assert bins["2-4"] == {"reference_valid": 1, "both_valid": 0, "valid_to_invalid_pct": 100.0,
                           "error_mm": {"median": None, "p95": None, "max": None, "mean": None},
                           "error_pct": {"median": None, "p95": None, "max": None, "mean": None}}
    assert bins[">=4"]["error_pct"]["max"] == 10.0
    json.dumps(out)
    empty = mjpeg_loss.compare([], [])
    assert empty["both_valid"] == 0 and empty["valid_to_invalid_pct"] is None and empty["bias_mm"] is None


def test_distance_bins_edges():
    assert mjpeg_loss.distance_bins([0, 499.9, 500, 999, 1000, 2000, 3999, 4000, 9000]).tolist() == \
        [0, 0, 1, 1, 2, 3, 3, 4, 4]
    assert mjpeg_loss.distance_labels((0.25, 1.5)) == ["<0.25", "0.25-1.5", ">=1.5"]


def test_gray_errors_count_levels_and_ringing():
    raw = np.array([[0, 0, 0, 10, 10, 10, 10, 10]])
    dec = np.array([[0, 1, 4, 10, 11, 8, 20, 10]])
    errors = mjpeg_loss.GrayErrors()
    errors.add(raw, dec)
    out = errors.result()
    assert out["valid_pixels"] == 5 and out["max_levels"] == 10
    assert out["levels_pct"] == {"0": 40.0, "1": 20.0, "2": 20.0, ">=3": 20.0}
    assert out["invalid_pixels"] == 3 and out["invalid_decoded_nonzero_pct"] == pytest.approx(66.67)
    assert out["invalid_decoded_ge3_pct"] == pytest.approx(33.33)


def test_jpeg_round_trip_of_synthetic_disparity():
    """q100 keeps almost every gray level; lower qualities are smaller and lose more, mostly at depth edges."""
    disparity = integer_disparity(scene_depth(256, 128))
    records = {"disparity": [{"seq": s, "ts": s / 10, "type": "RAW8", "image": disparity} for s in range(3)],
               "features": [{"seq": s, "ts": s / 10, "uv": np.array([[30.0, 60.0], [128.0, 60.0], [200.0, 90.0],
                                                                     [40.0, 3.0], [64.0, 64.0]])}
                            for s in range(3)],
               "jpeg": {q: [{"seq": s, "ts": s / 10, "data": jpeg(disparity, q)} for s in range(3)]
                        for q in (50, 100)}}
    out = mjpeg_loss.analyse_mjpeg(records, [50, 100], K_MM, (256, 128), PARAMS, 10)
    assert out["frames"]["disparity_with_features"] == 3 and out["pairing"]["q100"]["method"] == "sequence"
    q100, q50 = out["qualities"][100], out["qualities"][50]
    assert q100["frames_compared"] == 3 and q100["rate"]["jpegs"] == 3
    assert q50["rate"]["bytes_mean"] < q100["rate"]["bytes_mean"]
    assert q100["rate"]["mbps_at_fps"] == pytest.approx(q100["rate"]["bytes_mean"] * 8 * 10 / 1e6, abs=0.001)
    assert q100["gray_error_frame"]["levels_pct"]["0"] > 95
    assert q50["gray_error_frame"]["levels_pct"]["0"] < q100["gray_error_frame"]["levels_pct"]["0"]
    primary = q100["decode"]["zero"]["points"]
    assert primary["points"] == 15 and primary["reference_valid"] == 12  # the 3 in the top band have no depth
    assert primary["error_mm"]["median"] == 0.0
    # Every rule and decode rule is there
    assert set(q50["decode"]) == {"zero", "viewer"} and set(q50["decode"]["zero"]) == set(mjpeg_loss.RULES)
    steps = out["raw"]["steps"]
    assert steps["points"] == 12 and steps["distinct_disparities"] >= 3
    assert out["raw"]["valid_pct"]["centre"] == 80.0


def test_a_jpeg_that_does_not_decode_is_counted():
    disparity = integer_disparity(scene_depth())
    records = {"disparity": [{"seq": 1, "ts": None, "type": "RAW8", "image": disparity}],
               "features": [{"seq": 1, "ts": None, "uv": np.array([[30.0, 30.0]])}],
               "jpeg": {70: [{"seq": 1, "ts": None, "data": b"\xff\xd8 broken"}],
                        95: [{"seq": 1, "ts": None, "data": jpeg(disparity[:, :64], 95)}]}}
    out = mjpeg_loss.analyse_mjpeg(records, [70, 95], K_MM, (128, 64), PARAMS, 10)
    assert out["qualities"][70]["decode_failed"] == 1 and out["qualities"][70]["frames_compared"] == 0
    assert out["qualities"][95]["size_mismatch"] == 1
    assert out["qualities"][70]["decode"]["zero"]["points"]["points"] == 0


def test_pairing_by_sequence_checked_by_timestamp():
    ref = [{"seq": s, "ts": 1.0 + s / 10} for s in range(10)]
    same = [{"seq": s, "ts": 1.0 + s / 10} for s in (2, 3, 5)]
    pairs, info = mjpeg_loss.pair(ref, same, 0.1)
    assert pairs == {0: 2, 1: 3, 2: 5} and info["method"] == "sequence" and info["timestamps_agree"] == 3
    # An encoder counting its own frames: the sequence numbers point at other frames, the timestamps do not
    own = [{"seq": s - 2, "ts": 1.0 + s / 10} for s in (2, 3, 5)]
    pairs, info = mjpeg_loss.pair(ref, own, 0.1)
    assert pairs == {0: 2, 1: 3, 2: 5} and info["method"] == "timestamp" and info["timestamps_agree"] == 0
    # ... or numbers the reference never had
    pairs, info = mjpeg_loss.pair(ref, [{"seq": 100 + s, "ts": 1.0 + s / 10} for s in (4, 6)], 0.1)
    assert pairs == {0: 4, 1: 6} and info["method"] == "timestamp"
    # No timestamps: the sequence numbers are taken as they are
    pairs, info = mjpeg_loss.pair([{"seq": 1, "ts": None}], [{"seq": 1, "ts": None}, {"seq": 7, "ts": None}], 0.1)
    assert pairs == {0: 0} and info["method"] == "sequence" and info["unpaired"] == 1


def test_quantisation_theory_and_measured_steps():
    rows = {row["z_m"]: row for row in mjpeg_loss.quantisation_table(K_MM, 3)}
    assert rows[1.0]["step_mm"] == pytest.approx(1e6 / K_MM, abs=0.05)
    assert rows[1.0]["step_subpixel_mm"] == pytest.approx(1e6 / K_MM / 8, abs=0.01)
    assert rows[2.0]["step_mm"] == pytest.approx(4 * rows[1.0]["step_mm"], abs=0.2)
    assert rows[4.0]["disparity_px"] == pytest.approx(K_MM / 4000, abs=0.01)
    steps = mjpeg_loss.measured_steps([0, 12, 12, 6, 30], K_MM)
    assert steps["points"] == 4 and steps["distinct_disparities"] == 3 and steps["disparity_range"] == [6, 30]
    one_m = steps["by_distance"]["1-2"]
    assert one_m["points"] == 2 and one_m["step_mm"] == pytest.approx(K_MM / 12 - K_MM / 13, abs=0.05)
    assert one_m["theory_mm"] == pytest.approx((K_MM / 12) ** 2 / K_MM, abs=0.05)
    assert steps["by_distance"]["<0.5"]["points"] == 1 and steps["by_distance"][">=4"]["step_mm"] is None
    assert mjpeg_loss.measured_steps([], K_MM)["disparity_range"] is None


def test_temporal_median_and_noise():
    frames = [np.full((4, 4), v, np.float64) for v in (1000.0, 1010.0, 0.0, 990.0)]
    frames[0][0, 0] = frames[1][0, 0] = frames[3][0, 0] = 0  # one valid frame of 4 < 50 %
    median = mjpeg_loss.temporal_median(frames)
    assert median[1, 1] == 1000.0 and median[0, 0] == 0.0
    noise = mjpeg_loss.temporal_noise(frames, [[1.0, 1.0], [0.0, 0.0]], (4, 4))
    assert noise["points"] == 1 and noise["std_mm"]["median"] == pytest.approx(np.std([1000, 1010, 990]), abs=0.1)


def test_conversion_check_finds_the_camera_constant():
    disparity = integer_disparity(scene_depth())
    depth = np.where(disparity > 0, np.round(1.02 * K_MM / np.maximum(disparity, 1)), 0).astype(np.uint16)
    out = mjpeg_loss.conversion_check([{"seq": 4, "ts": 0.4, "image": disparity}],
                                      [{"seq": 4, "ts": 0.4, "image": depth}], K_MM, 0.1)
    assert out["frames"] == 1 and out["pixels"] == int((disparity > 0).sum())
    assert out["ratio"] == pytest.approx(1.02, abs=0.002)


def test_subpixel_loss_is_the_integer_quantisation():
    z = scene_depth()
    a = [{"seq": s, "ts": s / 10, "image": integer_disparity(z)} for s in range(0, 8, 2)]
    a_depth = [{"seq": s, "ts": s / 10, "image": np.where(r["image"] > 0, np.round(K_MM / np.maximum(r["image"], 1)), 0)
                .astype(np.uint16)} for s, r in zip(range(0, 8, 2), a)]
    b = [{"seq": s, "ts": 5 + s / 10, "image": np.round(z).astype(np.uint16)} for s in range(0, 8, 2)]
    uv = np.array([[20.0, 30.0], [70.0, 30.0], [110.0, 40.0], [120.0, 50.0], [40.0, 2.0]])
    b_features = [{"seq": s, "ts": 5 + s / 10, "uv": uv} for s in range(8)]
    out = mjpeg_loss.subpixel_loss(a, a_depth, b, b_features, K_MM, (128, 64), PARAMS, 0.1)
    assert out["frames"] == {"a": 4, "a_camera_depth": 4, "b": 4, "b_with_features": 4}
    centre = out["median_maps"]["centre"]
    assert centre["points"] == 20 and centre["both_valid"] == 16
    # Rounding the disparity moves the depth by at most half a step: z^2 / (2 fB) (+ rounding to mm)
    true = z[mjpeg_loss.centre_pixels(uv, (128, 64), (128, 64))[::-1]]
    half_step = (true[true > 0] ** 2 / K_MM / 2).max()
    assert 0 < centre["error_mm"]["max"] <= half_step + 1
    far, near = centre["by_distance"]["2-4"]["error_mm"]["max"], centre["by_distance"]["0.5-1"]["error_mm"]["max"]
    assert far > near
    np.testing.assert_allclose(out["median_maps_camera_depth"]["centre"]["error_mm"]["max"],
                               centre["error_mm"]["max"], atol=1)
    assert out["per_frame"]["points"]["both_valid"] > 0 and out["noise"]["b"]["std_mm"]["max"] == 0.0
    missing = mjpeg_loss.subpixel_loss([], [], b, b_features, K_MM, (128, 64), PARAMS, 0.1)
    assert "error" in missing


# --- the mode on a fake camera --------------------------------------------------------------------------

W, H, FPS = 128, 64, 10
FEATURE_UV = np.array([(x, y) for x in (12.0, 40.0, 70.0, 96.0, 118.0) for y in (4.0, 24.0, 44.0, 58.0)])


class Scene:
    """What the fake camera sees: frame n is taken at start + n / fps (the fake clock)."""

    def __init__(self, clock, k_mm, max_encoders=None, encoder_sequence_offset=0):
        self.clock, self.k = clock, k_mm
        self.z = scene_depth(W, H)
        self.start = None
        self.max_encoders, self.encoder_sequence_offset = max_encoders, encoder_sequence_offset
        self.trackers = []

    def current(self):
        return -1 if self.start is None else int((self.clock() - self.start) * FPS + 1e-9)

    def ts(self, seq):
        return 50.0 + seq / FPS

    def disparity(self):
        return integer_disparity(self.z, self.k)

    def depth(self, subpixel):
        if subpixel:
            return np.round(self.z).astype(np.uint16)
        d = self.disparity()
        return np.where(d > 0, np.round(self.k / np.maximum(d, 1)), 0).astype(np.uint16)


class Message(FakeFrame):
    def __init__(self, type_name, image, sequence, ts):
        super().__init__(type_name, image, sequence=sequence)
        self.ts = ts

    def getTimestampDevice(self):
        return timedelta(seconds=self.ts)


class Queue:
    def __init__(self, scene, make, size):
        self.scene, self.make, self.size, self.next = scene, make, size, 0

    def tryGetAll(self):
        out = []
        while self.next <= self.scene.current():
            msg = self.make(self.next)
            self.next += 1
            if msg is not None:
                out.append(msg)
        return out[-self.size:]


class Output:
    def __init__(self, owner, make):
        self.owner, self.make = owner, make

    def link(self, target):
        target.source = self

    def createOutputQueue(self, maxSize, blocking):
        assert blocking is False
        return Queue(self.owner.scene, self.make, maxSize)


class LossStereo(FakeStereoDepth):
    def __init__(self):
        super().__init__()
        self.disparity = Output(self, lambda seq: Message("RAW16" if self.wide() else "RAW8",
                                                          self.scene.disparity(), seq, self.scene.ts(seq)))
        self.depth = Output(self, lambda seq: Message(
            "RAW16", self.scene.depth(self.initialConfig.algorithmControl.enableSubpixel), seq, self.scene.ts(seq)))
        self.rectifiedLeft = Output(self, lambda seq: None)

    def getName(self):
        return "StereoDepth"


class LossEncoder:
    def __init__(self):
        self.input = FakeInput(self)
        self.out = Output(self, self.produce)

    def setDefaultProfilePreset(self, fps, profile):
        self.fps, self.profile = fps, profile

    def setQuality(self, quality):
        self.quality = quality

    def produce(self, seq):
        index = [n for n in self.pipeline.nodes if isinstance(n, LossEncoder)].index(self)
        if self.scene.max_encoders is not None and index >= self.scene.max_encoders:
            return None
        frame = self.input.source.make(seq)
        if frame.type != "RAW8":
            return None
        data = jpeg(frame.image, self.quality)
        return SimpleNamespace(getData=lambda: data, getSequenceNum=lambda: seq + self.scene.encoder_sequence_offset,
                               getTimestampDevice=lambda: timedelta(seconds=self.scene.ts(seq)))


class FakeTracker:
    def __init__(self):
        self.inputImage = FakeInput(self)
        self.initialConfig = mock.MagicMock()
        self.outputFeatures = Output(self, self.produce)

    def setHardwareResources(self, shaves, memory):
        self.resources = (shaves, memory)

    def produce(self, seq):
        if not self.scene.trackers or self.scene.trackers[-1] is not self:
            self.scene.trackers.append(self)
        features = [SimpleNamespace(id=i, position=SimpleNamespace(x=u, y=v)) for i, (u, v) in enumerate(FEATURE_UV)]
        return SimpleNamespace(trackedFeatures=features, getSequenceNum=lambda: seq,
                               getTimestampDevice=lambda: timedelta(seconds=self.scene.ts(seq)))


class LossPipeline:
    def __init__(self, device, scene):
        self.device, self.scene, self.nodes = device, scene, []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.device.close()

    def create(self, cls):
        node = cls()
        node.device, node.scene, node.pipeline = self.device, self.scene, self
        self.nodes.append(node)
        return node

    def start(self):
        self.scene.start = self.scene.clock()


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


CALIBRATION = StereoCalibration(raw=Intrinsics(100.0, 100.0, 64.0, 32.0), distortion=[0.0] * 14, baseline_m=0.075,
                                baseline_spec_m=0.075)


@pytest.fixture
def pnp_vio(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", mock.MagicMock())
    sys.modules.pop("pnp_vio", None)
    module = importlib.import_module("pnp_vio")
    module.dai.node.FeatureTracker = FakeTracker
    monkeypatch.setattr(module, "load_calibration", lambda device, args: CALIBRATION)
    yield module
    sys.modules.pop("pnp_vio", None)


def camera(**scene_options):
    clock = FakeClock()
    rectified, _ = rectified_for_settings(CALIBRATION, SAVED, (W, H))
    scene = Scene(clock, mjpeg_loss.depth_times_disparity_mm(rectified.fx, CALIBRATION.baseline_m), **scene_options)
    dai = fake_dai()
    dai.node.StereoDepth = type("StereoDepth", (LossStereo,), {"PresetMode": dai.node.StereoDepth.PresetMode})
    dai.node.VideoEncoder = LossEncoder
    dai.node.Camera = FakeCamera
    dai.Pipeline = lambda device: LossPipeline(device, scene)
    return clock, scene, dai


def run_probe(pnp_vio, dai, clock, *extra, open_device=FakeDevice):
    return stereo_probe.main(["--mode", "mjpeg_loss", "--width", str(W), "--height", str(H), "--fps", str(FPS),
                              "--seconds", "3", *extra], dai=dai, open_device=open_device, clock=clock,
                             sleep=clock.sleep, wall_clock=lambda: 1e9, vio=pnp_vio)


def test_all_encoders_at_once(pnp_vio, tmp_path, capsys):
    clock, scene, dai = camera()
    output = tmp_path / "probe" / "loss.json"
    assert run_probe(pnp_vio, dai, clock, "--output", str(output)) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed == json.loads(output.read_text())
    doc = printed
    assert doc["mode"] == "mjpeg_loss" and doc["complete"] is True and doc["qualities"] == [70, 85, 95, 100]
    assert [r["name"] for r in doc["runs"]] == ["A", "B"] and all(r["error"] is None for r in doc["runs"])
    a, b = doc["runs"]
    assert a["applied"]["enableSubpixel"] is False and a["applied"]["filteringOrder"][-1] == "MEDIAN"
    assert a["applied"]["speckleFilter"] is True and a["disparity_format"]["raw8"] is True
    assert b["settings"] == SAVED and b["applied"]["enableSubpixel"] is True
    assert a["captured"]["disparity"] == 30 and a["captured"]["jpeg_q70"] == 30
    assert a["frames"]["disparity_types"] == ["RAW8"]
    assert a["pairing"]["q95"]["method"] == "sequence" and a["pairing"]["features"]["paired"] == 30
    # The FeatureTracker is the VIO's: pnp_vio.create_feature_tracker with the saved feature settings
    tracker = scene.trackers[0]
    assert tracker.resources == (1, 2)
    tracker.initialConfig.setNumTargetFeatures.assert_called_with(300)
    tracker.initialConfig.setFeatureMaintainer.assert_called_with(True)
    assert doc["calibration"]["fx_source"] == "rectified" and doc["calibration"]["baseline_m"] == 0.075
    assert doc["calibration"]["depth_times_disparity_mm"] == pytest.approx(scene.k)

    assert doc["encoders"]["parallel"] == [70, 85, 95, 100] and doc["encoders"]["sequential"] == []
    assert "all 4 encoders ran at once" in doc["encoders"]["note"]
    assert set(doc["mjpeg"]) == {"70", "85", "95", "100"}
    q100, q70 = doc["mjpeg"]["100"], doc["mjpeg"]["70"]
    assert q100["run"] == "A" and q100["frames_compared"] == 30 and q100["encoders_in_run"] == 4
    assert q70["rate"]["bytes_mean"] < q100["rate"]["bytes_mean"]
    assert q100["gray_error_features"]["levels_pct"]["0"] > 90
    assert q100["decode"]["zero"]["points"]["both_valid"] > 0
    assert doc["raw_rates_mbps"]["disparity_raw8"] == pytest.approx(W * H * 8 * FPS / 1e6, abs=0.01)

    assert doc["conversion_check"]["ratio"] == pytest.approx(1.0, abs=0.002)
    sub = doc["subpixel"]
    assert sub["reference_run"] == "A" and sub["subpixel"] is True and "camera at rest" in sub["method"]
    assert sub["frames"]["a"] == sub["frames"]["b"] == 15 and sub["median_maps"]["points"]["both_valid"] > 0
    assert sub["median_maps"]["centre"]["error_mm"]["max"] > 0
    quant = doc["quantisation"]
    assert quant["subpixel_bits"] == 3 and len(quant["theory"]) == 6
    assert quant["measured_integer"]["points"] > 0
    table = doc["summary_table"]
    assert "MJPEG loss of the RAW8 disparity" in table and "\n100 " in table and "subpixel loss" in table
    assert "quantisation step" in table and "camera depth x disparity" in table


def test_encoders_that_do_not_run_at_once_are_measured_alone(pnp_vio, capsys):
    clock, scene, dai = camera(max_encoders=2)
    assert run_probe(pnp_vio, dai, clock) == 0
    doc = json.loads(capsys.readouterr().out)
    assert [r["name"] for r in doc["runs"]] == ["A", "B", "A-q95", "A-q100"]
    assert doc["encoders"]["parallel"] == [70, 85] and doc["encoders"]["sequential"] == [95, 100]
    assert doc["encoders"]["jpeg_share_run_a"]["q95"] == 0.0 and "in sequence" in doc["encoders"]["note"]
    assert doc["mjpeg"]["95"]["run"] == "A-q95" and doc["mjpeg"]["95"]["alone"] is True
    assert doc["mjpeg"]["70"]["run"] == "A" and doc["mjpeg"]["70"]["alone"] is False
    assert doc["mjpeg"]["100"]["frames_compared"] == 30 and doc["subpixel"]["reference_run"] == "A"
    assert "A-q95" in doc["summary_table"]


def test_an_encoder_with_its_own_sequence_numbers_is_paired_by_timestamp(pnp_vio, capsys):
    clock, scene, dai = camera(encoder_sequence_offset=-7)
    assert run_probe(pnp_vio, dai, clock, "--qualities", "90") == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc["runs"][0]["pairing"]["q90"]["method"] == "timestamp"
    assert doc["mjpeg"]["90"]["frames_compared"] == 30 and doc["encoders"]["parallel"] == [90]


def test_a_failed_run_a_is_replaced_by_one_run_per_quality(pnp_vio, capsys):
    clock, scene, dai = camera()
    opened = []

    def open_device():
        opened.append(1)
        if len(opened) == 1:
            raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")
        return FakeDevice()

    assert run_probe(pnp_vio, dai, clock, "--qualities", "70,95", open_device=open_device) == 0
    doc = json.loads(capsys.readouterr().out)
    assert [r["name"] for r in doc["runs"]] == ["A", "B", "A-q70", "A-q95"]
    assert doc["runs"][0]["error"] == "RuntimeError: X_LINK_DEVICE_NOT_FOUND" and "traceback" in doc["runs"][0]
    assert doc["encoders"]["parallel"] == [] and doc["encoders"]["sequential"] == [70, 95]
    assert "X_LINK_DEVICE_NOT_FOUND" in doc["encoders"]["note"]
    assert doc["subpixel"]["reference_run"] == "A-q70" and doc["subpixel"]["median_maps"]["points"]["both_valid"] > 0
    assert doc["calibration"] is not None and doc["complete"] is True


def test_the_budget_skips_the_runs_alone(pnp_vio, capsys):
    clock, scene, dai = camera(max_encoders=1)
    assert run_probe(pnp_vio, dai, clock, "--qualities", "70,95", "--budget", "12") == 0
    doc = json.loads(capsys.readouterr().out)
    assert [r["name"] for r in doc["runs"]] == ["A", "B"]
    assert doc["skipped"] == [{"name": "A-q95", "reason": doc["skipped"][0]["reason"]}]
    assert "time budget" in doc["skipped"][0]["reason"] and set(doc["mjpeg"]) == {"70"}


def test_bad_mjpeg_loss_arguments_exit_2(capsys):
    assert stereo_probe.main(["--mode", "mjpeg_loss", "--qualities", "0"], dai=fake_dai(), open_device=FakeDevice) == 2
    assert stereo_probe.main(["--mode", "mjpeg_loss", "--seconds", "0"], dai=fake_dai(), open_device=FakeDevice) == 2
    assert capsys.readouterr().out == ""


def test_the_probe_defaults_are_the_vios(pnp_vio):
    vio, probe = pnp_vio.parse_args([]), stereo_probe.parse_args(["--mode", "mjpeg_loss"])
    for key in ("num_features", "max_features", "corner_detector", "min_depth", "max_depth"):
        assert getattr(probe, key) == getattr(vio, key), key
    assert stereo_probe.CORNER_DETECTOR_CHOICES == tuple(sorted(pnp_vio.CORNER_DETECTORS))
    assert probe.seconds == stereo_probe.LOSS_SECONDS and probe.budget is None
    cases = stereo_probe.parse_args([])
    assert cases.seconds == stereo_probe.SAMPLE_SECONDS and cases.budget == stereo_probe.TIME_BUDGET_S


def test_budget_and_timeout():
    assert stereo_probe.mjpeg_loss_budget(10, 4) == 6 * (stereo_probe.WARMUP_S + 10 + stereo_probe.RUN_OVERHEAD_S)
    assert stereo_probe.mjpeg_loss_timeout(10, 4) > stereo_probe.mjpeg_loss_budget(10, 4) + stereo_probe.OPEN_TIMEOUT_S
    assert stereo_probe.reference_every(15, 10) == 8 and stereo_probe.reference_every(1, 3) == 1


# --- POST /api/stereo_probe?mode=mjpeg_loss -------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    main.save_config({**main.DEFAULT_CONFIG, "fps": 12, "num_features": 250, "max_features": 280,
                      "stereo_median": "5x5"})
    actions = []
    monkeypatch.setattr(main, "stop_vio", lambda: actions.append("stop"))
    monkeypatch.setattr(main, "start_vio", lambda: actions.append("start"))
    return main, fastapi_testclient.TestClient(main.app), actions


def test_mjpeg_loss_command_matches_the_vio_settings(client):
    main, _, _ = client
    config = main.load_config()
    cmd = main.stereo_probe_command(config, main.CONFIG_FILE.parent / "probe" / "x.json", None,
                                    stereo_probe.MJPEG_LOSS, 7.5, [95, 70])
    assert cmd[:3] == ["python", "-u", "/app/stereo_probe.py"] and "--cases" not in cmd
    args = stereo_probe.parse_args(cmd[3:])
    assert (args.mode, args.seconds, args.qualities, args.fps) == ("mjpeg_loss", 7.5, "95,70", 12)
    assert (args.num_features, args.max_features) == (250, 280)
    assert resolve_stereo_settings(args) == resolve_stereo_settings(config)


def test_a_mjpeg_loss_probe_runs_and_restarts_the_vio(client, monkeypatch, tmp_path):
    main, http, actions = client
    seen = {}

    def run(cmd, timeout):
        actions.append("probe")
        seen.update(cmd=cmd, timeout=timeout)
        document = {"probe": "stereo", "mode": "mjpeg_loss", "complete": True}
        with open(cmd[cmd.index("--output") + 1], "w") as f:
            json.dump(document, f)
        return 0, "", "table"

    monkeypatch.setattr(main, "run_stereo_probe_process", run)
    response = http.post("/api/stereo_probe?mode=mjpeg_loss&seconds=5&qualities=95,70")
    assert response.status_code == 200 and actions == ["stop", "probe", "start"]
    body = response.json()
    assert body["mode"] == "mjpeg_loss" and body["seconds"] == 5.0 and body["qualities"] == [95, 70]
    assert body["cases"] is None and body["probe"]["mode"] == "mjpeg_loss" and body["error"] is None
    assert seen["timeout"] == stereo_probe.mjpeg_loss_timeout(5.0, 2)
    assert seen["cmd"][seen["cmd"].index("--qualities") + 1] == "95,70"
    saved = list((tmp_path / "probe").glob("mjpeg_loss-*.json"))
    assert len(saved) == 1 and body["saved_to"] == str(saved[0]) and json.loads(saved[0].read_text()) == body
    # Defaults: 10 s, the four qualities
    http.post("/api/stereo_probe?mode=mjpeg_loss")
    assert seen["cmd"][seen["cmd"].index("--seconds") + 1] == "10.0"
    assert seen["cmd"][seen["cmd"].index("--qualities") + 1] == "70,85,95,100"


def test_a_timed_out_mjpeg_loss_probe_still_restarts_the_vio(client, monkeypatch):
    main, http, actions = client
    monkeypatch.setattr(main, "run_stereo_probe_process",
                        lambda cmd, timeout: (actions.append("probe"), (None, "", "stuck in B"))[1])
    response = http.post("/api/stereo_probe?mode=mjpeg_loss")
    assert response.status_code == 504 and actions == ["stop", "probe", "start"]
    assert "killed" in response.json()["error"] and response.json()["vio_restarted"] is True


@pytest.mark.parametrize("query", ["mode=other", "mode=mjpeg_loss&seconds=2", "mode=mjpeg_loss&seconds=31",
                                   "mode=mjpeg_loss&seconds=nan", "mode=mjpeg_loss&qualities=0",
                                   "mode=mjpeg_loss&qualities=1,2,3,4,5,6,7", "mode=mjpeg_loss&cases=A",
                                   "seconds=5", "qualities=70", "mode=cases&qualities=70"])
def test_bad_requests_are_refused_without_stopping_the_vio(client, monkeypatch, query):
    main, http, actions = client
    monkeypatch.setattr(main, "run_stereo_probe_process", lambda cmd, timeout: actions.append("probe"))
    assert http.post(f"/api/stereo_probe?{query}").status_code == 400
    assert actions == []


def test_mjpeg_loss_is_refused_in_mock_mode_and_while_a_probe_runs(client, monkeypatch):
    main, http, actions = client
    assert main.stereo_probe_lock.acquire(blocking=False)
    try:
        assert http.post("/api/stereo_probe?mode=mjpeg_loss").status_code == 409
    finally:
        main.stereo_probe_lock.release()
    monkeypatch.setattr(main, "MOCK", True)
    assert http.post("/api/stereo_probe?mode=mjpeg_loss").status_code == 403
    assert actions == []
