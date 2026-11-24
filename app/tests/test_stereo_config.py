import importlib
import os
import sys
from unittest import mock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import stereo_config  # noqa: E402
import vio_log  # noqa: E402
from stereo_config import (STEREO_SETTINGS, apply_stereo_config, effective_stereo_config,  # noqa: E402
                           resolve_stereo_settings, stereo_command_args, stereo_warnings)

STEREO_DEFAULTS = {setting.key: setting.default for setting in STEREO_SETTINGS}

EXPECTED_DEFAULTS = {
    "stereo_preset": "ROBOTICS",
    "stereo_lr_check": True,
    "stereo_lr_check_threshold": 5,
    "stereo_confidence_threshold": 50,
    "stereo_brightness_min": 1,  # auto: edge fill color is 0
    "stereo_brightness_max": 255,
    "stereo_speckle_filter": True,
    "stereo_speckle_range": 50,
    "stereo_depth_min_mm": 300,
    "stereo_depth_max_mm": 4000,
    "stereo_spatial_filter": False,
    "stereo_temporal_filter": False,
    "stereo_decimation": 1,
    "stereo_subpixel": True,
    "stereo_subpixel_bits": 3,
    "stereo_median": "7x7",
    "stereo_median_last": False,
    "stereo_distortion_correction": True,
    "stereo_alpha": 0.0,
    "stereo_edge_fill_color": 0,
    "stereo_p2": None,
}

CUSTOM = {
    "stereo_preset": "ACCURACY",
    "stereo_lr_check": False,
    "stereo_lr_check_threshold": 8,
    "stereo_confidence_threshold": 30,
    "stereo_brightness_min": 4,
    "stereo_brightness_max": 250,
    "stereo_speckle_filter": False,
    "stereo_speckle_range": 80,
    "stereo_depth_min_mm": 200,
    "stereo_depth_max_mm": 6000,
    "stereo_spatial_filter": True,
    "stereo_temporal_filter": True,
    "stereo_decimation": 2,
    "stereo_subpixel": True,
    "stereo_subpixel_bits": 4,
    "stereo_median": "5x5",
    "stereo_distortion_correction": False,
    "stereo_alpha": 0.25,
    "stereo_edge_fill_color": -1,
    "stereo_p2": 120,
}


@pytest.fixture
def pnp_vio(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", mock.MagicMock())
    sys.modules.pop("pnp_vio", None)
    module = importlib.import_module("pnp_vio")
    yield module
    sys.modules.pop("pnp_vio", None)


@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "start_vio", lambda: None)
    monkeypatch.delenv(vio_log.LOG_ONLY_ENV, raising=False)
    return main, fastapi_testclient.TestClient(main.app)


def test_defaults_are_the_sparse_vio_tuning():
    assert resolve_stereo_settings(dict(STEREO_DEFAULTS)) == EXPECTED_DEFAULTS


def test_brightness_min_auto_follows_edge_fill_color():
    assert resolve_stereo_settings({"stereo_edge_fill_color": 0})["stereo_brightness_min"] == 1
    assert resolve_stereo_settings({"stereo_edge_fill_color": -1})["stereo_brightness_min"] == 0
    assert resolve_stereo_settings({"stereo_edge_fill_color": -1, "stereo_brightness_min": 3})["stereo_brightness_min"] == 3


def test_pnp_vio_defaults_match_config_defaults(pnp_vio):
    assert resolve_stereo_settings(pnp_vio.parse_args([])) == EXPECTED_DEFAULTS


@pytest.mark.parametrize("values", [EXPECTED_DEFAULTS, CUSTOM, {**EXPECTED_DEFAULTS, "stereo_alpha": None}])
def test_config_survives_command_line_round_trip(pnp_vio, values):
    args = pnp_vio.parse_args(stereo_command_args(values))
    assert resolve_stereo_settings(args) == resolve_stereo_settings(values)


def test_unset_optionals_on_the_command_line():
    args = stereo_command_args({**STEREO_DEFAULTS, "stereo_alpha": None})
    # Unset with no default: omitted; unset with a default (alpha 0): explicit "none"
    assert "--stereo-p2" not in args and "--stereo-brightness-min" not in args
    assert args[args.index("--stereo-alpha") + 1] == "none"
    assert "--stereo-lr-check" in args and "--no-stereo-spatial-filter" in args
    assert args[args.index("--stereo-confidence-threshold") + 1] == "50"


def test_main_command_passes_stereo_recovery_and_snapshot_args(client, pnp_vio):
    main, _ = client
    config = {**main.DEFAULT_CONFIG, **CUSTOM, "tracking_recovery_frames": 7, "snapshots": False,
              "max_reference_age": 0.5}
    cmd = main.build_vio_command(config)
    assert cmd[:3] == ["python", "-u", "/app/pnp_vio.py"]
    args = pnp_vio.parse_args(cmd[3:])
    assert resolve_stereo_settings(args) == resolve_stereo_settings(CUSTOM)
    assert args.recovery_frames == 7 and args.snapshots is False and args.max_reference_age == 0.5
    assert pnp_vio.parse_args(main.build_vio_command(main.DEFAULT_CONFIG)[3:]
                              ).max_reference_age == 1.0


def test_config_api_validates_and_persists_stereo_settings(client):
    main, http = client
    body = {**main.DEFAULT_CONFIG, **CUSTOM}
    assert http.post("/api/config", json=body).status_code == 200
    saved = main.load_config()
    for key, value in CUSTOM.items():
        assert saved[key] == value
    assert http.post("/api/config", json={**body, "stereo_p2": None, "stereo_alpha": None}).status_code == 200
    assert main.load_config()["stereo_p2"] is None
    assert http.post("/api/config", json={**body, "stereo_median": "9x9"}).status_code == 422
    assert http.post("/api/config", json={**body, "stereo_decimation": 5}).status_code == 422


def test_old_config_files_get_stereo_defaults(client):
    main, _ = client
    main.save_config({"fps": 10})
    config = main.load_config()
    assert config["stereo_confidence_threshold"] == 50 and config["tracking_recovery_frames"] == 20
    assert config["max_reference_age"] == 1.0
    assert "--stereo-preset" in main.build_vio_command(config)


def test_web_form_renders_stereo_settings(client):
    main, http = client
    main.save_config({**main.DEFAULT_CONFIG, **CUSTOM, "snapshots": True})
    page = http.get("/").text
    assert '<option value="ACCURACY" selected>' in page
    assert '<option value="5x5" selected>' in page
    assert 'name="stereo_lr_check">' in page  # unchecked
    assert 'name="stereo_spatial_filter" checked>' in page
    assert 'name="stereo_p2" value="120"' in page
    assert 'name="snapshots" checked' in page
    assert "{stereo_fields}" not in page and "'api/snapshot?stream='" in page


def test_apply_sets_preset_first_then_overrides():
    stereo, dai = mock.MagicMock(), mock.MagicMock()
    apply_stereo_config(stereo, EXPECTED_DEFAULTS, dai)

    calls = [c[0] for c in stereo.mock_calls]
    assert calls[0] == "setDefaultProfilePreset"
    stereo.setDefaultProfilePreset.assert_called_once_with(dai.node.StereoDepth.PresetMode.ROBOTICS)
    config = stereo.initialConfig
    config.setLeftRightCheckThreshold.assert_called_once_with(5)
    config.setConfidenceThreshold.assert_called_once_with(50)
    config.setMedianFilter.assert_called_once_with(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck.assert_called_once_with(True)
    stereo.setSubpixel.assert_called_once_with(True)
    stereo.setSubpixelFractionalBits.assert_called_once_with(3)
    stereo.setExtendedDisparity.assert_called_once_with(False)
    stereo.enableDistortionCorrection.assert_called_once_with(True)
    stereo.setAlphaScaling.assert_called_once_with(0.0)
    stereo.setRectifyEdgeFillColor.assert_called_once_with(0)
    stereo.setDepthAlign.assert_called_once_with(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

    post = config.postProcessing
    assert (post.brightnessFilter.minBrightness, post.brightnessFilter.maxBrightness) == (1, 255)
    assert (post.speckleFilter.enable, post.speckleFilter.speckleRange) == (True, 50)
    assert (post.thresholdFilter.minRange, post.thresholdFilter.maxRange) == (300, 4000)
    assert post.spatialFilter.enable is False and post.temporalFilter.enable is False
    # The ROBOTICS preset decimates by 2; the override keeps depth on the feature grid
    assert post.decimationFilter.decimationFactor == 1
    # P2 left to the preset (never assigned, so still an auto-created mock attribute)
    assert isinstance(config.costAggregation.horizontalPenaltyCostP2, mock.MagicMock)


def test_apply_optional_and_constrained_settings():
    stereo, dai = mock.MagicMock(), mock.MagicMock()
    apply_stereo_config(stereo, {**CUSTOM, "stereo_preset": "NONE", "stereo_alpha": None}, dai)
    stereo.setDefaultProfilePreset.assert_not_called()
    stereo.setAlphaScaling.assert_not_called()
    assert stereo.initialConfig.costAggregation.horizontalPenaltyCostP2 == 120
    # Median needs 3 subpixel bits
    stereo.initialConfig.setMedianFilter.assert_called_once_with(dai.MedianFilter.MEDIAN_OFF)


def test_warnings():
    assert stereo_warnings(EXPECTED_DEFAULTS) == []
    warnings = stereo_warnings(CUSTOM, intrinsics_from="cam_b")
    assert any("median" in w for w in warnings)
    assert any("decimation 2" in w for w in warnings)
    assert any("alpha" in w for w in warnings)


def test_effective_config_reads_back_from_node():
    stereo = mock.MagicMock()
    config = stereo.initialConfig
    config.getConfidenceThreshold.return_value = 50
    config.getLeftRightCheck.return_value = True
    config.postProcessing.decimationFilter.decimationFactor = 1
    config.costAggregation.horizontalPenaltyCostP2 = 95
    config.getMedianFilter.side_effect = RuntimeError("boom")
    effective = effective_stereo_config(stereo, EXPECTED_DEFAULTS)
    assert effective["confidence_threshold"] == 50 and effective["lr_check"] is True
    assert effective["decimation"] == 1 and effective["p2"] == 95
    assert effective["median"].startswith("unavailable")
    assert effective["alpha"] == 0.0 and effective["depth_align"] == "RECTIFIED_LEFT"


def test_every_setting_has_a_flag_and_label():
    flags = [setting.flag for setting in stereo_config.STEREO_SETTINGS]
    assert len(set(flags)) == len(flags)
    assert all(flag.startswith("--stereo-") for flag in flags)


def test_main_command_passes_intrinsics_and_depth_scale(client, pnp_vio):
    main, _ = client
    default = pnp_vio.parse_args(main.build_vio_command({**main.DEFAULT_CONFIG})[3:])
    assert default.intrinsics_from == "rectified" and default.intrinsics_override is None and default.depth_scale == 1.0
    cmd = main.build_vio_command({**main.DEFAULT_CONFIG, "intrinsics_from": "frame",
                                  "intrinsics_override": "163.1,237.1,329.4,199.0", "depth_scale": 1.33})
    args = pnp_vio.parse_args(cmd[3:])
    assert args.intrinsics_from == "frame" and args.depth_scale == 1.33
    assert args.intrinsics_override == (163.1, 237.1, 329.4, 199.0)


def test_config_api_validates_intrinsics_and_depth_scale(client):
    main, http = client
    body = {**main.DEFAULT_CONFIG}
    assert http.post("/api/config", json={**body, "intrinsics_override": " 160,230,330,199 ",
                                          "depth_scale": 1.2}).status_code == 200
    saved = main.load_config()
    assert saved["intrinsics_override"] == "160,230,330,199" and saved["depth_scale"] == 1.2
    assert saved["intrinsics_from"] == "rectified"
    assert http.post("/api/config", json={**body, "intrinsics_override": ""}).status_code == 200
    assert main.load_config()["intrinsics_override"] is None
    for bad in ({"depth_scale": 0.1}, {"depth_scale": 6}, {"intrinsics_override": "1,2,3"},
                {"intrinsics_override": "1,2,-3,4"}, {"intrinsics_from": "sideways"}):
        assert http.post("/api/config", json={**body, **bad}).status_code == 422, bad


def test_web_form_renders_intrinsics_and_depth_scale(client):
    main, http = client
    main.save_config({**main.DEFAULT_CONFIG})
    page = http.get("/").text
    assert 'name="intrinsics_from" data-value="rectified"' in page
    assert 'name="intrinsics_override" value=""' in page and 'name="depth_scale" value="1.0"' in page
    main.save_config({**main.DEFAULT_CONFIG, "intrinsics_override": "1,2,3,4", "depth_scale": 0.8})
    page = http.get("/").text
    assert 'name="intrinsics_override" value="1,2,3,4"' in page and 'name="depth_scale" value="0.8"' in page


def test_status_reports_the_camera_matrix(client, monkeypatch, tmp_path):
    main, http = client
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    assert http.get("/api/status").json()["camera"] is None
    import snapshots
    snapshots.write_camera_status({"source": "rectified", "intrinsics": [163.1, 237.1, 329.4, 199.0]},
                                  tmp_path / "snapshots")
    assert http.get("/api/status").json()["camera"]["source"] == "rectified"


@pytest.mark.parametrize("value, ok", [("1", True), ("1.333", True), ("2", True), ("0.999", False), ("2.5", False),
                                       ("inf", False), ("nan", False), ("x", False)])
def test_parse_refractive_index(value, ok):
    if ok:
        assert stereo_config.parse_refractive_index(value) == float(value)
    else:
        with pytest.raises(ValueError):
            stereo_config.parse_refractive_index(value)


@pytest.mark.parametrize("value, clamped", [(1.34, 1.34), (0.5, 1.0), (3, 2.0), (1, 1.0), ("1.2", 1.2),
                                            (None, 1.333), ("water", 1.333), (float("nan"), 1.333)])
def test_clamp_refractive_index(value, clamped):
    assert stereo_config.clamp_refractive_index(value) == clamped


def test_main_command_passes_the_refractive_index_clamped(client, pnp_vio):
    main, _ = client
    assert main.DEFAULT_CONFIG["refractive_index"] == 1.333
    pnp = {**main.DEFAULT_CONFIG}
    assert pnp_vio.parse_args(main.build_vio_command(pnp)[3:]).refractive_index == 1.333
    assert pnp_vio.parse_args(main.build_vio_command({**pnp, "refractive_index": 1.0})[3:]).refractive_index == 1.0
    # A hand-edited config.json out of range must not crash-loop the VIO process on its arguments
    assert pnp_vio.parse_args(main.build_vio_command({**pnp, "refractive_index": 7})[3:]).refractive_index == 2.0
    config = {key: value for key, value in pnp.items() if key != "refractive_index"}
    assert pnp_vio.parse_args(main.build_vio_command(config)[3:]).refractive_index == 1.333


def test_config_api_validates_the_refractive_index_and_the_form_shows_it(client):
    main, http = client
    body = {**main.DEFAULT_CONFIG}
    assert http.post("/api/config", json={**body, "refractive_index": 1.0}).status_code == 200
    assert main.load_config()["refractive_index"] == 1.0
    assert 'name="refractive_index" value="1.0"' in http.get("/").text
    for bad in (0.9, 2.1, "salty"):
        assert http.post("/api/config", json={**body, "refractive_index": bad}).status_code == 422, bad
    main.save_config({**main.DEFAULT_CONFIG})
    page = http.get("/").text
    assert 'name="refractive_index" value="1.333"' in page and "PnP Refractive Index (water)" in page
    assert 'id="camera-status"' in page and "effective_intrinsics" in page


@pytest.mark.parametrize("value, expected", [
    ("0.1655,-0.0069,0.18", (0.1655, -0.0069, 0.18)), (" 1 2 3 ", (1.0, 2.0, 3.0)), ([0, 0, -5], (0.0, 0.0, -5.0)),
])
def test_parse_lever_arm(value, expected):
    assert stereo_config.parse_lever_arm(value) == expected


@pytest.mark.parametrize("value", ["1,2", "1,2,3,4", "a,b,c", "0,0,5.1", [0, 0, float("nan")], None, 3.0])
def test_parse_lever_arm_rejects(value):
    with pytest.raises(ValueError):
        stereo_config.parse_lever_arm(value)


def test_main_command_passes_the_lever_arm(client, pnp_vio):
    main, _ = client
    pnp = {**main.DEFAULT_CONFIG}
    assert pnp["lever_arm_enabled"] is False and pnp["lever_arm_m"] == [0.1655, -0.0069, 0.18]
    args = pnp_vio.parse_args(main.build_vio_command(pnp)[3:])
    assert args.lever_arm is False and args.lever_arm_m == (0.1655, -0.0069, 0.18)
    # A negative x must not be taken for an option
    args = pnp_vio.parse_args(main.build_vio_command({**pnp, "lever_arm_enabled": True,
                                                      "lever_arm_m": [-0.2, 0.01, 0.3]})[3:])
    assert args.lever_arm is True and args.lever_arm_m == (-0.2, 0.01, 0.3)
    # Unusable hand-edited values fall back to the default
    args = pnp_vio.parse_args(main.build_vio_command({**pnp, "lever_arm_m": "bogus"})[3:])
    assert args.lever_arm_m == (0.1655, -0.0069, 0.18)


def test_config_api_and_form_for_the_lever_arm(client):
    main, http = client
    body = {**main.DEFAULT_CONFIG}
    # The form posts the text field as it is
    assert http.post("/api/config", json={**body, "lever_arm_enabled": True,
                                          "lever_arm_m": " 0.2, -0.01, 0.15 "}).status_code == 200
    saved = main.load_config()
    assert saved["lever_arm_enabled"] is True and saved["lever_arm_m"] == [0.2, -0.01, 0.15]
    page = http.get("/").text
    assert 'name="lever_arm_enabled" checked' in page and 'name="lever_arm_m" value="0.2,-0.01,0.15"' in page
    assert "VISO_POS_X/Y/Z" in page and "Does NOT change what is sent" in page
    for bad in ("1,2", "0,0,9", [1, 2, "x"], None):
        assert http.post("/api/config", json={**body, "lever_arm_m": bad}).status_code == 422, bad
    main.save_config({**main.DEFAULT_CONFIG})
    page = http.get("/").text
    assert 'name="lever_arm_enabled" >' in page and 'name="lever_arm_m" value="0.1655,-0.0069,0.18"' in page


def test_status_reports_the_refractive_index_and_effective_matrix(client, monkeypatch, tmp_path):
    main, http = client
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    import snapshots
    snapshots.write_camera_status({"source": "override", "intrinsics": [160.0, 230.0, 330.0, 199.0],
                                   "effective_intrinsics": [213.28, 306.59, 330.0, 199.0], "refractive_index": 1.333,
                                   "depth_scale": 1.0, "total_depth_scale": 1.333}, tmp_path / "snapshots")
    camera = http.get("/api/status").json()["camera"]
    assert camera["refractive_index"] == 1.333 and camera["effective_intrinsics"][0] == 213.28
    assert camera["intrinsics"][0] == 160.0 and camera["total_depth_scale"] == 1.333
