import os
import sys
import urllib.request

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from autopilot_params import (SUGGESTED_OPTICAL_HOLD_PARAMS, VISUAL_ODOMETRY_PARAMS, AutopilotParams,  # noqa: E402
                              diff_params)
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
        "VISO_TYPE": 1, "VISO_POS_X": 0.1655, "VISO_POS_Y": -0.0069, "VISO_POS_Z": 0.180, "VISO_ORIENT": 0,
        "VISO_DELAY_MS": None, "EK3_SRC1_VELXY": 6, "EK3_SRC1_POSXY": 0, "EK3_SRC1_POSZ": 1, "EK3_SRC1_YAW": 1,
    }
    assert [name for name, param in SUGGESTED.items() if param.needs_reboot] == ["VISO_TYPE"]
    assert all(param.rationale for param in SUGGESTED_OPTICAL_HOLD_PARAMS)
    # The same lever arm as the extension's diagnostics
    assert (SUGGESTED["VISO_POS_X"].value, SUGGESTED["VISO_POS_Y"].value, SUGGESTED["VISO_POS_Z"].value) == \
        DEFAULT_LEVER_ARM_M
    assert "None" in SUGGESTED["EK3_SRC1_POSXY"].rationale and "GPS" in SUGGESTED["EK3_SRC1_POSXY"].rationale
    assert "unverified" in SUGGESTED["VISO_DELAY_MS"].rationale


def test_diff_of_an_empty_vehicle_lists_every_value_as_missing():
    diff = diff_params({})
    assert [entry["name"] for entry in diff] == [p.name for p in SUGGESTED_OPTICAL_HOLD_PARAMS if p.value is not None]
    assert all(entry["status"] == "missing" and entry["current"] is None for entry in diff)
    assert diff[0] == {"name": "VISO_TYPE", "current": None, "suggested": 1, "needs_reboot": True,
                       "rationale": SUGGESTED["VISO_TYPE"].rationale, "status": "missing"}


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


def test_auto_configure_is_unchanged_and_ignores_the_suggestions():
    assert VISUAL_ODOMETRY_PARAMS == [
        ("AHRS_EKF_TYPE", 3, True), ("EK3_ENABLE", 1, True), ("VISO_TYPE", 1, True),
        ("EK3_SRC1_POSXY", 6, False), ("EK3_SRC1_VELXY", 6, False), ("EK3_SRC1_POSZ", 1, False)]

    class FakeAutopilot(AutopilotParams):
        """No I/O: every parameter reads 0 and every write succeeds."""
        def __init__(self):
            super().__init__("192.0.2.1")
            self.written = []

        def is_armed(self):
            return False

        def read(self, name):
            return 0.0, 9

        def write(self, name, value, param_type):
            self.written.append((name, value))
            return value, param_type

    autopilot = FakeAutopilot()
    autopilot.configure_visual_odometry()
    assert autopilot.written == [(name, value) for name, value, _ in VISUAL_ODOMETRY_PARAMS]
