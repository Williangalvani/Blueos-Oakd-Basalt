import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pnp_odometry import Intrinsics  # noqa: E402
from rectified_intrinsics import (  # noqa: E402
    StereoCalibration,
    choose_intrinsics,
    focal_check,
    focal_from_depth_and_disparity,
    frame_matches,
    intrinsics_override,
    measure_rectified_focal,
    rectified_for_settings,
    rectified_intrinsics,
    same_intrinsics,
)
from stereo_config import STEREO_SETTINGS, parse_depth_scale, parse_intrinsics_override  # noqa: E402

STEREO_DEFAULTS = {setting.key: setting.default for setting in STEREO_SETTINGS}

DATA = Path(__file__).parent / "data"
CALIB = json.loads((DATA / "calib_oakd_w_poe.json").read_text())
K_CAM_B = np.array(CALIB["cam_b"]["K"])
D_CAM_B = CALIB["cam_b"]["distortion"]
SIZE = (640, 400)
RAW = Intrinsics.from_matrix(K_CAM_B)
# getOptimalNewCameraMatrix(K, D[:8], (640, 400), 0) for this camera (OpenCV 4.14 and 5.0 agree)
EXPECTED_ALPHA0 = (163.1, 237.1, 329.4, 199.0)
SNAP4 = DATA / "snap4"


def calibration():
    return StereoCalibration(raw=RAW, distortion=D_CAM_B, sockets={"cam_b": RAW},
                             baseline_m=CALIB["baseline_cm"] / 100)


def test_rectified_intrinsics_matches_the_device_for_the_dump():
    K = rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, alpha=0.0)
    for value, expected in zip((K.fx, K.fy, K.cx, K.cy), EXPECTED_ALPHA0):
        assert value == pytest.approx(expected, rel=0.005)
    assert K.fy / K.fx == pytest.approx(1.45, abs=0.01)  # measured 1.40-1.42 from RGB matches


def test_only_the_first_eight_coefficients_are_used():
    eight = rectified_intrinsics(K_CAM_B, D_CAM_B[:8], SIZE, 0.0)
    assert rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, 0.0) == eight
    # With all 14 (the thin-prism and tilt terms) OpenCV would give cx 328.7, cy 198.0
    all14, _ = cv2.getOptimalNewCameraMatrix(K_CAM_B, np.array(D_CAM_B), SIZE, 0.0)
    assert abs(all14[0, 2] - eight.cx) > 0.5


def test_rectified_intrinsics_follows_alpha_and_resolution():
    wide = rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, alpha=1.0)
    tight = rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, alpha=0.0)
    assert wide.fx < tight.fx and wide.fy < tight.fy  # alpha 1 keeps every source pixel: smaller focal
    full = rectified_intrinsics(np.diag([2.0, 2.0, 1.0]) @ K_CAM_B, D_CAM_B, (1280, 800), alpha=0.0)
    for a, b in zip((full.fx, full.fy, full.cx, full.cy), (tight.fx, tight.fy, tight.cx, tight.cy)):
        assert a == pytest.approx(2 * b, rel=0.005)


def test_rectified_intrinsics_unknown_without_alpha_or_distortion_correction():
    assert rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, alpha=None) is None
    assert rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, alpha=0.0, distortion_correction=False) is None
    settings = dict(STEREO_DEFAULTS)
    K, problem = rectified_for_settings(calibration(), settings, SIZE)
    assert problem is None and K.fx == pytest.approx(163.1, rel=0.005)
    assert rectified_for_settings(calibration(), {**settings, "stereo_alpha": None}, SIZE) == (
        None, "alpha scaling is not set (device default)")
    K, problem = rectified_for_settings(calibration(), {**settings, "stereo_distortion_correction": False}, SIZE)
    assert K is None and "homography" in problem
    no_distortion = StereoCalibration(raw=RAW)
    assert rectified_for_settings(no_distortion, settings, SIZE)[0] is None


@pytest.mark.parametrize("text, expected", [
    ("163.1,237.1,329.4,199.0", (163.1, 237.1, 329.4, 199.0)),
    (" 163.1, 237.1 ,329.4 199 ", (163.1, 237.1, 329.4, 199.0)),
    ("", None),
    (None, None),
])
def test_parse_intrinsics_override(text, expected):
    assert parse_intrinsics_override(text) == expected


@pytest.mark.parametrize("text", ["163,237,329", "1,2,3,4,5", "163,-237,329,199", "a,b,c,d", "nan,1,1,1", "inf,1,1,1"])
def test_parse_intrinsics_override_rejects(text):
    with pytest.raises(ValueError):
        parse_intrinsics_override(text)


def test_intrinsics_override_builds_intrinsics():
    assert intrinsics_override("1,2,3,4") == Intrinsics(1.0, 2.0, 3.0, 4.0)
    assert intrinsics_override("  ") is None


@pytest.mark.parametrize("value, ok", [(1.0, True), ("1.33", True), (0.2, True), (5.0, True), (0.19, False),
                                       (5.1, False), (0, False), (-1, False), (float("nan"), False)])
def test_parse_depth_scale(value, ok):
    if ok:
        assert parse_depth_scale(value) == float(value)
    else:
        with pytest.raises(ValueError):
            parse_depth_scale(value)


def test_frame_matrix_equal_to_raw_is_detected():
    rectified = rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, 0.0)
    # The ImgTransformation matrix comes back as float32
    frame = Intrinsics(*np.float32([RAW.fx, RAW.fy, RAW.cx, RAW.cy]).astype(float))
    assert same_intrinsics(frame, RAW)
    assert frame_matches(frame, RAW, rectified) == "raw"
    assert frame_matches(rectified, RAW, rectified) == "rectified"
    assert frame_matches(Intrinsics(RAW.fx * 1.01, RAW.fy, RAW.cx, RAW.cy), RAW, rectified) == "neither"
    assert frame_matches(None, RAW, rectified) is None
    assert not same_intrinsics(RAW, None)


RECT = Intrinsics(163.1, 237.1, 329.4, 199.0)
OTHER = Intrinsics(170.0, 240.0, 320.0, 200.0)
CAM_C = Intrinsics(284.8, 284.8, 321.8, 197.3)
OVERRIDE = Intrinsics(150.0, 220.0, 330.0, 200.0)


@pytest.mark.parametrize("requested, rectified, frame, override, source, intrinsics, warns", [
    ("rectified", RECT, RAW, None, "rectified", RECT, []),
    ("rectified", RECT, None, None, "rectified", RECT, []),  # the frame is not even needed
    ("rectified", None, OTHER, None, "frame (fallback)", OTHER, ["unverified"]),
    ("rectified", None, None, None, "cam_b (fallback)", RAW, ["unverified", "no depth frame", "raw CAM_B"]),
    ("frame", RECT, RAW, None, "rectified (frame matrix is raw)", RECT, ["unrectified"]),
    ("frame", RECT, OTHER, None, "frame", OTHER, []),
    ("frame", RECT, None, None, "rectified (fallback)", RECT, ["no depth frame"]),
    ("frame", None, RAW, None, "frame", RAW, []),  # alpha unset: the raw frame matrix may be right
    ("frame", None, None, None, "cam_b (fallback)", RAW, ["no depth frame", "raw CAM_B"]),
    ("cam_b", RECT, RAW, None, "cam_b", RAW, []),
    ("cam_c", RECT, RAW, None, "cam_c", CAM_C, []),
    ("rectified", RECT, RAW, OVERRIDE, "override", OVERRIDE, []),
    ("frame", None, None, OVERRIDE, "override", OVERRIDE, []),
])
def test_choose_intrinsics(requested, rectified, frame, override, source, intrinsics, warns):
    choice = choose_intrinsics(requested, RAW, rectified, frame, "no depth frame camera matrix (boom)", override,
                               {"cam_b": RAW, "cam_c": CAM_C}, "alpha scaling is not set")
    assert choice.source == source
    assert choice.intrinsics == intrinsics
    assert len(choice.warnings) == len(warns)
    for warning, fragment in zip(choice.warnings, warns):
        assert fragment in warning
    assert choice.frame == frame


def test_choose_intrinsics_reports_what_the_frame_matched():
    assert choose_intrinsics("rectified", RAW, RECT, RAW).frame_matches == "raw"
    assert choose_intrinsics("rectified", RAW, RECT, RECT).frame_matches == "rectified"


def test_focal_from_depth_and_disparity():
    # 12 px of disparity at 1.048 m with a 7.496 cm baseline: f = 167.8 px (a snap4 match)
    assert focal_from_depth_and_disparity(1048, 12.0, 0.07496) == pytest.approx(167.77, abs=0.01)
    np.testing.assert_allclose(focal_from_depth_and_disparity([1000, 2000], [12.0, 6.0], 0.075), [160.0, 160.0])


def textured_pair(f=163.1, baseline_m=0.075, z_m=1.2, seed=0):
    """Fronto-parallel textured wall at z_m: right image = left shifted by the disparity f * B / Z."""
    rng = np.random.default_rng(seed)
    texture = cv2.GaussianBlur(rng.uniform(0, 255, (400, 700)).astype(np.float32), (0, 0), 2.0)
    texture = cv2.normalize(texture, None, 0, 255, cv2.NORM_MINMAX)
    disparity = f * baseline_m / z_m
    left = texture[:, 30:670]
    shift = np.float32([[1, 0, -disparity], [0, 1, 0]])
    right = cv2.warpAffine(texture, shift, (700, 400), flags=cv2.INTER_LINEAR)[:, 30:670]
    depth = np.full((400, 640), int(round(z_m * 1000)), dtype=np.uint16)
    depth[:, :60] = 0  # rectification border without depth
    return left.astype(np.uint8), right.astype(np.uint8), depth


def test_measure_rectified_focal_on_synthetic_pair():
    left, right, depth = textured_pair(f=163.1)
    measurement = measure_rectified_focal(left, right, depth, 0.075)
    assert measurement["n_matches"] >= 30
    assert measurement["f_from_depth"] == pytest.approx(163.1, rel=0.02)
    assert focal_check(163.1, measurement)["ok"]
    raw = focal_check(285.1, measurement, fx_raw=285.1)
    assert not raw["ok"] and raw["ratio"] == pytest.approx(0.572, abs=0.02)
    assert "--intrinsics-from rectified" in raw["message"]
    # Depth decimated by 2 is looked up on its own grid
    decimated = measure_rectified_focal(left, right, depth[::2, ::2], 0.075)
    assert decimated["f_from_depth"] == pytest.approx(163.1, rel=0.03)


def test_focal_check_needs_enough_matches():
    result = focal_check(163.1, {"f_from_depth": 160.0, "n_matches": 3})
    assert not result["ok"] and result["ratio"] is None and "only 3" in result["message"]
    blank = np.zeros((400, 640), np.uint8)
    measurement = measure_rectified_focal(blank, blank, np.zeros((400, 640), np.uint16), 0.075)
    assert measurement["n_matches"] == 0 and measurement["f_from_depth"] is None
    assert not focal_check(163.1, measurement)["ok"]


def snap4():
    paths = [SNAP4 / name for name in ("rect_left.png", "rect_right.png", "depth_raw.png")]
    if not all(path.exists() for path in paths):
        pytest.skip("snap4 snapshots not available")
    return [cv2.imread(str(path), cv2.IMREAD_UNCHANGED) for path in paths]


def test_focal_check_on_device_snapshots():
    """OAK-D-W-PoE snapshots: the device depth implies f ~ 167 px, the alpha-0 K (163), not the raw 285."""
    left, right, depth = snap4()
    measurement = measure_rectified_focal(left, right, depth, CALIB["baseline_cm"] / 100)
    assert measurement["n_matches"] >= 20
    assert 155 < measurement["f_from_depth"] < 180
    rectified = rectified_intrinsics(K_CAM_B, D_CAM_B, SIZE, 0.0)
    assert focal_check(rectified.fx, measurement)["ok"]
    assert not focal_check(RAW.fx, measurement, fx_raw=RAW.fx)["ok"]


def test_check_rectified_focal_tool(capsys):
    snap4()
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
    import check_rectified_focal
    paths = [str(SNAP4 / name) for name in ("rect_left.png", "rect_right.png", "depth_raw.png")]
    assert check_rectified_focal.main([*paths, str(CALIB["baseline_cm"]), "--fx", "163.1"]) == 0
    output = json.loads(capsys.readouterr().out)
    assert output["ok"] and 155 < output["f_from_depth"] < 180
    assert check_rectified_focal.main([*paths, str(CALIB["baseline_cm"]), "--fx", "285.1"]) == 1
