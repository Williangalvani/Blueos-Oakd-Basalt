import importlib
import os
import sys
from unittest import mock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import stereo_config  # noqa: E402
import vio_log  # noqa: E402
from stereo_config import (STEREO_DEFAULTS, apply_stereo_config, effective_stereo_config,  # noqa: E402
                           resolve_stereo_settings, stereo_command_args, stereo_warnings)

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
    config = {**main.DEFAULT_CONFIG, "algorithm": "pnp", **CUSTOM, "tracking_recovery_frames": 7, "snapshots": False}
    cmd = main.build_vio_command(config)
    assert cmd[:3] == ["python", "-u", "/app/pnp_vio.py"]
    args = pnp_vio.parse_args(cmd[3:])
    assert resolve_stereo_settings(args) == resolve_stereo_settings(CUSTOM)
    assert args.recovery_frames == 7 and args.snapshots is False

    basalt = main.build_vio_command({**config, "algorithm": "basalt"})
    assert not any(token.startswith("--stereo") for token in basalt)


def test_config_api_validates_and_persists_stereo_settings(client):
    main, http = client
    body = {**main.DEFAULT_CONFIG, "algorithm": "pnp", **CUSTOM}
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
    main.save_config({"algorithm": "pnp", "fps": 10})
    config = main.load_config()
    assert config["stereo_confidence_threshold"] == 50 and config["tracking_recovery_frames"] == 20
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
    assert "{stereo_fields}" not in page and "/api/snapshot?stream=" in page


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
