import os
import sys
import urllib.request

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import autopilot_params  # noqa: E402
from autopilot_params import SUGGESTED_OPTICAL_HOLD_PARAMS, AutopilotParams, diff_params  # noqa: E402
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


def test_diff_of_an_empty_vehicle_lists_every_value_as_missing():
    diff = diff_params({})
    assert [entry["name"] for entry in diff] == [p.name for p in SUGGESTED_OPTICAL_HOLD_PARAMS if p.value is not None]
    assert all(entry["status"] == "missing" and entry["current"] is None for entry in diff)
    assert diff[0] == {"name": "AHRS_EKF_TYPE", "current": None, "suggested": 3, "needs_reboot": True,
                       "rationale": SUGGESTED["AHRS_EKF_TYPE"].rationale, "status": "missing"}


def test_diff_ignores_float32_rounding_and_keep_current_params():
    # As PARAM_VALUE returns them: float32
    current = {p.name: float(np.float32(p.value)) for p in SUGGESTED_OPTICAL_HOLD_PARAMS if p.value is not None}
    assert current["VISO_POS_X"] != 0.1655
    current["VISO_DELAY_MS"] = 87
    assert diff_params(current) == []


def test_diff_reports_changes_in_order_and_leaves_the_input_alone():
    current = {p.name: p.value for p in SUGGESTED_OPTICAL_HOLD_PARAMS if p.value is not None}
    current.update(EK3_SRC1_POSXY=6, VISO_POS_Z=0.0, VISO_TYPE=0, EXTRA_PARAM=5)
    del current["EK3_SRC1_YAW"]
    snapshot = dict(current)
    diff = diff_params(current)
    assert current == snapshot
    assert [(e["name"], e["current"], e["suggested"], e["status"]) for e in diff] == [
        ("VISO_TYPE", 0, 1, "change"), ("VISO_POS_Z", 0.0, 0.18, "change"),
        ("EK3_SRC1_POSXY", 6, 0, "change"), ("EK3_SRC1_YAW", None, 1, "missing")]


def test_diff_treats_unreadable_values_as_a_change():
    (entry,) = [e for e in diff_params({"VISO_ORIENT": "forward"}) if e["name"] == "VISO_ORIENT"]
    assert entry["status"] == "change" and entry["current"] == "forward"


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
