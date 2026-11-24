import os
import sys
import urllib.request

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import autopilot_params  # noqa: E402
from autopilot_params import SUGGESTED_OPTICAL_HOLD_PARAMS, AutopilotParams  # noqa: E402
from stereo_config import DEFAULT_LEVER_ARM_M  # noqa: E402

SUGGESTED = {param.name: param for param in SUGGESTED_OPTICAL_HOLD_PARAMS}


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError("no autopilot access in these tests")
    monkeypatch.setattr(urllib.request, "urlopen", refuse)


def test_suggested_optical_hold_params():
    assert len(SUGGESTED) == len(SUGGESTED_OPTICAL_HOLD_PARAMS)
    assert {name: param.value for name, param in SUGGESTED.items()} == {
        "AHRS_EKF_TYPE": 3, "EK3_ENABLE": 1,
        "VISO_TYPE": 1, "VISO_POS_X": 0.1655, "VISO_POS_Y": -0.0069, "VISO_POS_Z": 0.180, "VISO_ORIENT": 0,
        "VISO_DELAY_MS": None, "EK3_SRC1_VELXY": 6, "EK3_SRC1_POSXY": 0, "EK3_SRC1_POSZ": 1, "EK3_SRC1_VELZ": 0,
        "EK3_SRC1_YAW": 1,
    }
    assert [name for name, param in SUGGESTED.items() if param.needs_reboot] == ["AHRS_EKF_TYPE", "EK3_ENABLE",
                                                                                  "VISO_TYPE"]
    assert all(param.rationale for param in SUGGESTED_OPTICAL_HOLD_PARAMS)
    # The same lever arm as the extension's diagnostics
    assert (SUGGESTED["VISO_POS_X"].value, SUGGESTED["VISO_POS_Y"].value, SUGGESTED["VISO_POS_Z"].value) == \
        DEFAULT_LEVER_ARM_M
    assert "None" in SUGGESTED["EK3_SRC1_POSXY"].rationale and "GPS" in SUGGESTED["EK3_SRC1_POSXY"].rationale
    assert "ExternalNav" in SUGGESTED["EK3_SRC1_POSXY"].rationale
    assert "unverified" in SUGGESTED["VISO_DELAY_MS"].rationale
    # Every recommended parameter can be written from the page, within ArduPilot's values
    assert set(autopilot_params.PARAM_LIMITS) == set(SUGGESTED)
    for param in SUGGESTED_OPTICAL_HOLD_PARAMS:
        if param.value is not None:
            assert autopilot_params.validate_param_values({param.name: param.value}) == [], param.name


def test_nothing_configures_the_autopilot_automatically():
    """The old start-up auto-configuration (EK3_SRC1_POSXY=6 at every VIO start) is gone for good."""
    assert not hasattr(autopilot_params, "VISUAL_ODOMETRY_PARAMS")
    assert not hasattr(AutopilotParams, "configure_visual_odometry")
    app = os.path.join(os.path.dirname(__file__), "..")
    for name in os.listdir(app):
        if name.endswith(".py"):
            with open(os.path.join(app, name)) as f:
                assert "configure_visual_odometry" not in f.read(), name


def test_recommended_values_match_through_float32_rounding():
    # As PARAM_VALUE returns them: float32
    for param in SUGGESTED_OPTICAL_HOLD_PARAMS:
        if param.value is not None:
            assert autopilot_params._same_value(float(np.float32(param.value)), param.value), param.name
    assert float(np.float32(0.1655)) != 0.1655
    assert not autopilot_params._same_value(0.0, 0.18) and not autopilot_params._same_value(6, 0)


@pytest.mark.parametrize("message, expected", [
    (None, None),
    ({"message": {"type": "SYSTEM_TIME", "time_boot_ms": 60_000},
      "status": {"time": {"last_update": "2026-11-24T12:00:00+00:00"}}}, 1795521540.0),
    ({"message": {"type": "SYSTEM_TIME", "time_boot_ms": 60_000},  # 10 s old: the autopilot may be gone
      "status": {"time": {"last_update": "2026-11-24T11:59:50+00:00"}}}, None),
    ({"message": {"type": "SYSTEM_TIME"}, "status": {"time": {"last_update": "2026-11-24T12:00:00+00:00"}}}, None),
])
def test_boot_time_from_system_time(monkeypatch, message, expected):
    autopilot = AutopilotParams("192.0.2.1")
    monkeypatch.setattr(autopilot, "_vehicle_message", lambda name: message if name == "SYSTEM_TIME" else None)
    now = 1795521600.0  # 2026-11-24T12:00:00Z
    assert autopilot.boot_time(now=now) == expected
