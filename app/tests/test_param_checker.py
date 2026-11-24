"""
param_checker.ParamChecker against the fake mavlink2rest (tools/fake_mavlink2rest.py): read-only background
checks, the /api/status summary, backoff, serialisation with writes, pending reboots; and the banner's
formatting (static/param_check.js) under node, from a real summary.
"""
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import unittest.mock
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import autopilot_params  # noqa: E402
import param_checker  # noqa: E402
from fake_mavlink2rest import FakeVehicle  # noqa: E402

# What the dev vehicle had on 2026-11-24 (CURSOR_TASK23.md)
DEV_VEHICLE = {
    "AHRS_EKF_TYPE": 3, "EK3_ENABLE": 1, "VISO_TYPE": 1, "VISO_POS_X": 0, "VISO_POS_Y": 0, "VISO_POS_Z": 0,
    "VISO_ORIENT": 0, "VISO_DELAY_MS": 10, "EK3_SRC1_POSXY": 6, "EK3_SRC1_VELXY": 6, "EK3_SRC1_POSZ": 1,
    "EK3_SRC1_VELZ": 0, "EK3_SRC1_YAW": 1,
}
DEV_MISMATCHES = ["VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z", "EK3_SRC1_POSXY"]
PARAM_JS = os.path.join(os.path.dirname(__file__), "..", "static", "param_check.js")
NODE = shutil.which("node")
RUNNER = r"""
const check = require(process.argv[1]);
const input = JSON.parse(require('fs').readFileSync(0, 'utf8'));
process.stdout.write(JSON.stringify(input.calls.map(([name, ...args]) => check[name](...args))));
"""


@pytest.fixture
def vehicle(monkeypatch):
    vehicle = FakeVehicle(params=DEV_VEHICLE, heartbeat_hz=20.0)
    monkeypatch.setattr(urllib.request, "urlopen", vehicle.urlopen)
    return vehicle


def make_checker(**kwargs):
    heartbeats = []
    checker = param_checker.ParamChecker(lambda: autopilot_params.AutopilotParams("fake-vehicle", timeout=0.2),
                                         attempts=lambda: 1, on_heartbeat=heartbeats.append, **kwargs)
    checker.heartbeats = heartbeats
    return checker


def test_before_the_first_check_nothing_is_known():
    summary = make_checker().summary()
    assert summary["checked_at"] is None and summary["reachable"] is None and summary["ok"] is False
    assert summary["mismatches"] == [] and summary["error"] == "not checked yet"


def test_a_check_reads_only_and_reports_the_dev_vehicle_mismatches(vehicle):
    checker = make_checker()
    assert checker.check_now() is True
    assert vehicle.param_sets() == []
    assert {m["type"] for m in vehicle.posted} == {"PARAM_REQUEST_READ"}
    summary = checker.summary()
    assert set(summary) == {"checked_at", "age_s", "reachable", "armed", "ok", "mismatches", "unreadable",
                            "pending_reboot", "error", "checking", "interval_s", "mavlink2rest"}
    assert summary["reachable"] is True and summary["armed"] is False and summary["ok"] is False
    assert [m["name"] for m in summary["mismatches"]] == DEV_MISMATCHES
    posxy = summary["mismatches"][-1]
    assert posxy == {"name": "EK3_SRC1_POSXY", "current": 6.0, "recommended": 0, "needs_reboot": False}
    assert summary["mismatches"][0]["recommended"] == pytest.approx(0.1655)
    assert summary["unreadable"] == [] and summary["pending_reboot"] == [] and summary["error"] is None
    assert summary["mavlink2rest"] == "http://fake-vehicle/mavlink2rest"
    json.dumps(summary)
    assert checker.heartbeats[-1]["armed"] is False  # the page header learns the armed state


def test_keep_current_entries_never_count(vehicle):
    vehicle.params["VISO_DELAY_MS"] = 87.0
    del vehicle.params["EK3_SRC1_POSXY"]
    checker = make_checker()
    checker.check_now()
    summary = checker.summary()
    assert "VISO_DELAY_MS" not in [m["name"] for m in summary["mismatches"]]
    assert summary["unreadable"] == ["EK3_SRC1_POSXY"]
    for name, value in DEV_VEHICLE.items():
        recommended = autopilot_params.SUGGESTED_BY_NAME[name].value
        vehicle.params[name] = float(value if recommended is None else recommended)
    vehicle.params["VISO_DELAY_MS"] = 87.0
    checker.check_now()
    assert checker.summary()["ok"] is True and checker.summary()["mismatches"] == []
    del vehicle.params["VISO_DELAY_MS"]
    checker.check_now()
    assert checker.summary()["ok"] is True and checker.summary()["unreadable"] == []


@pytest.mark.parametrize("setup, message", [
    (lambda v: setattr(v, "reachable", False), "not reachable"),
    (lambda v: setattr(v, "heartbeat", False), "no autopilot HEARTBEAT"),
])
def test_unreachable_is_reported_and_backs_off(vehicle, setup, message):
    setup(vehicle)
    checker = make_checker(interval_s=30.0, backoff_max_s=300.0)
    delays = []
    for _ in range(6):
        checker.check_now()
        delays.append(checker.next_delay())
    summary = checker.summary()
    assert summary["reachable"] is False and summary["ok"] is False and message in summary["error"]
    assert summary["mismatches"] == [] and vehicle.param_sets() == []
    assert not [m for m in vehicle.posted if m["type"] == "PARAM_REQUEST_READ"]  # nothing asked without HEARTBEAT
    assert delays == [60.0, 120.0, 240.0, 300.0, 300.0, 300.0]
    vehicle.reachable, vehicle.heartbeat = True, True
    checker.check_now()
    assert checker.next_delay() == 30.0 and checker.summary()["reachable"] is True


def test_a_vehicle_that_never_answers_param_reads_backs_off_too(vehicle):
    vehicle.params = {}
    checker = make_checker()
    checker.check_now()
    summary = checker.summary()
    assert summary["reachable"] is True and len(summary["unreadable"]) == 12 and summary["ok"] is False
    assert checker.next_delay() == 60.0


def test_the_background_check_skips_while_a_write_or_read_runs(vehicle):
    checker = make_checker()
    with checker._io:  # what write() and read() hold
        assert checker.check_now() is False
    assert vehicle.requests == [] and checker.summary()["checked_at"] is None


def test_the_background_thread_checks_and_stops(vehicle):
    checker = make_checker(interval_s=0.05, backoff_max_s=0.05)
    checker.start(first_delay=0.0)
    deadline = time.monotonic() + 10.0
    while checker.summary()["checked_at"] is None and time.monotonic() < deadline:
        time.sleep(0.02)
    checker.stop()
    assert checker.summary()["reachable"] is True and checker._thread is None
    assert vehicle.param_sets() == []


def test_write_refuses_when_armed_and_writes_nothing(vehicle):
    vehicle.armed = True
    checker = make_checker()
    with pytest.raises(autopilot_params.WriteRefused, match="ARMED"):
        checker.write({"EK3_SRC1_POSXY": 0}, heartbeat_wait=0.5)
    assert vehicle.param_sets() == [] and checker.heartbeats[-1]["armed"] is True


def test_write_reads_back_rechecks_and_keeps_the_reboot_note_until_the_autopilot_rebooted(vehicle):
    vehicle.params["VISO_TYPE"] = 0.0
    checker = make_checker()
    checker.check_now()
    assert [m["name"] for m in checker.summary()["mismatches"]] == ["VISO_TYPE", *DEV_MISMATCHES]
    changes = {m["name"]: m["recommended"] for m in checker.summary()["mismatches"]}
    result = checker.write(changes, heartbeat_wait=0.5)
    assert [c["name"] for c in result["changed"]] == ["VISO_TYPE", *DEV_MISMATCHES] and result["failed"] == []
    assert all(c["readback"] == pytest.approx(c["new"]) for c in result["changed"])
    assert result["reboot_required"] == ["VISO_TYPE"]
    # Checked again right after the write: all as recommended, but VISO_TYPE only applies after a reboot
    summary = checker.summary()
    assert summary["mismatches"] == [] and summary["pending_reboot"] == ["VISO_TYPE"] and summary["ok"] is False
    assert summary["checking"] is False
    checker.check_now()
    assert checker.summary()["pending_reboot"] == ["VISO_TYPE"]
    vehicle.reboot()
    checker.check_now()
    assert checker.summary()["pending_reboot"] == [] and checker.summary()["ok"] is True
    assert len(vehicle.param_sets()) == 5  # only the write's PARAM_SETs, none from any check


def test_reads_wait_for_a_write_and_the_background_check_skips_it(vehicle):
    checker = make_checker()
    order = []
    original = autopilot_params.write_suggested_params

    def slow_write(*args, **kwargs):
        order.append("write start")
        time.sleep(0.2)
        order.append("write end")
        return original(*args, **kwargs)

    with unittest.mock.patch.object(autopilot_params, "write_suggested_params", slow_write):
        writer = threading.Thread(target=checker.write, args=({"EK3_SRC1_POSXY": 0},), kwargs={"heartbeat_wait": 0.5})
        writer.start()
        time.sleep(0.05)
        assert checker.summary()["checking"] is True
        assert checker.check_now() is False  # the background check skips its turn
        checker.read()  # the panel's read waits for the write
        order.append("read")
        writer.join()
    assert order == ["write start", "write end", "read"]


# --- the banner (static/param_check.js) under node ---------------------------------------------------

def run_js(*calls):
    if NODE is None:
        pytest.skip("node is not installed")
    result = subprocess.run([NODE, "-e", RUNNER, os.path.abspath(PARAM_JS)], input=json.dumps({"calls": calls}),
                            capture_output=True, text=True, timeout=30, check=True)
    return json.loads(result.stdout)


def checked_summary(vehicle, **changes):
    for name, value in changes.items():
        setattr(vehicle, name, value)
    checker = make_checker()
    checker.check_now()
    return checker.summary()


def test_banner_warns_with_the_table_and_offers_configure(vehicle):
    summary = checked_summary(vehicle)
    view, changes, widget = run_js(["paramCheckView", summary], ["paramConfigureChanges", summary],
                                   ["paramWidgetText", summary])
    assert view["state"] == "warn" and view["title"] == "Autopilot parameters differ from the recommended setup"
    assert view["rows"] == [["VISO_POS_X", "0", "0.1655", False], ["VISO_POS_Y", "0", "-0.0069", False],
                            ["VISO_POS_Z", "0", "0.18", False], ["EK3_SRC1_POSXY", "6", "0", False]]
    assert view["configure"] is True and view["configureEnabled"] is True
    assert [c["name"] for c in changes] == DEV_MISMATCHES and changes[-1] == {"name": "EK3_SRC1_POSXY", "old": 6,
                                                                              "new": 0, "reboot": False}
    assert widget == "autopilot params differ from the recommended setup"


def test_banner_disables_configure_while_armed(vehicle):
    (view,) = run_js(["paramCheckView", checked_summary(vehicle, armed=True)])
    assert view["state"] == "warn" and view["configure"] is True and view["configureEnabled"] is False
    assert "ARMED" in view["configureTitle"]


def test_banner_unreadable_without_mavlink2rest_or_heartbeat(vehicle):
    for change in ({"reachable": False}, {"heartbeat": False}):
        (view,) = run_js(["paramCheckView", checked_summary(vehicle, **change)])
        assert view["state"] == "unreadable" and view["title"] == "Autopilot parameters not checked"
        assert view["configure"] is False and view["rows"] == []
        vehicle.reachable, vehicle.heartbeat = True, True


def test_banner_marks_reboots_and_explains_hidden_viso_params(vehicle):
    vehicle.params["VISO_TYPE"] = 0.0
    for name in ("VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z", "VISO_ORIENT", "VISO_DELAY_MS"):
        del vehicle.params[name]  # ArduPilot hides VISO_* until VISO_TYPE is set and the autopilot rebooted
    (view,) = run_js(["paramCheckView", checked_summary(vehicle)])
    assert view["rows"][0] == ["VISO_TYPE", "0", "1", True]
    assert "Not readable: VISO_POS_X, VISO_POS_Y, VISO_POS_Z, VISO_ORIENT." in view["notes"][0]
    assert "rebooted" in view["notes"][0]


def test_banner_ok_pending_and_checking(vehicle):
    for name, value in DEV_VEHICLE.items():
        recommended = autopilot_params.SUGGESTED_BY_NAME[name].value
        vehicle.params[name] = float(value if recommended is None else recommended)
    ok = checked_summary(vehicle)
    pending = {**ok, "ok": False, "pending_reboot": ["VISO_TYPE"]}
    views = run_js(["paramCheckView", ok], ["paramCheckView", pending], ["paramCheckView", make_checker().summary()],
                   ["paramWidgetText", ok], ["paramWidgetText", pending])
    assert views[0]["state"] == "ok" and views[0]["title"] == "Autopilot parameters match the recommended setup"
    assert views[0]["detail"].startswith("checked ") and views[0]["configure"] is False
    assert views[1]["state"] == "pending"
    assert views[1]["notes"] == ["Reboot the autopilot for VISO_TYPE to take effect (not done automatically)."]
    assert views[2]["state"] == "checking"
    assert views[3:] == ["", "autopilot reboot pending"]
