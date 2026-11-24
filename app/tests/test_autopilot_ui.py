import json
import os
import sys
import time
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import autopilot_params  # noqa: E402
import vio_log  # noqa: E402
from fake_mavlink2rest import FakeVehicle  # noqa: E402

GOOD_WRITE = {"VISO_TYPE": 1, "VISO_POS_X": 0.1655, "EK3_SRC1_POSZ": 1}


@pytest.fixture
def vehicle(monkeypatch):
    """The only 'network': urlopen answers for http://fake-vehicle/mavlink2rest/... and nothing else."""
    vehicle = FakeVehicle(heartbeat_hz=20.0)
    monkeypatch.setattr(urllib.request, "urlopen", vehicle.urlopen)
    return vehicle


@pytest.fixture
def client(monkeypatch, tmp_path, vehicle):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    monkeypatch.setattr(main, "AUTOPILOT_TIMEOUT", 0.2)
    monkeypatch.setattr(main, "AUTOPILOT_ATTEMPTS", 1)
    monkeypatch.setattr(main, "HEARTBEAT_WAIT", 0.5)
    monkeypatch.setattr(main, "param_checker", main.make_param_checker())
    (tmp_path / "config.json").write_text(json.dumps({"vehicle_ip": "fake-vehicle"}))
    return main, fastapi_testclient.TestClient(main.app)


def by_name(result):
    return {param["name"]: param for param in result["params"]}


# --- POST /api/autopilot/params/read: read only ---------------------------------------------------

def test_get_reads_and_diffs_without_writing(client, vehicle):
    main, http = client
    response = http.post("/api/autopilot/params/read")
    assert response.status_code == 200
    result = response.json()
    assert result["armed"] is False and result["heartbeat"]["armed"] is False
    assert result["mavlink2rest"] == "http://fake-vehicle/mavlink2rest"
    params = by_name(result)
    assert list(params) == [p.name for p in autopilot_params.SUGGESTED_OPTICAL_HOLD_PARAMS]
    viso_type = params["VISO_TYPE"]
    assert (viso_type["current"], viso_type["suggested"], viso_type["status"], viso_type["needs_reboot"]) == \
        (0.0, 1, "differs", True)
    assert params["VISO_POS_X"]["status"] == "differs" and params["VISO_POS_X"]["suggested"] == pytest.approx(0.1655)
    assert params["EK3_SRC1_POSZ"]["status"] == "ok" and params["EK3_SRC1_YAW"]["status"] == "ok"
    # Keep-current entries suggest what the vehicle has
    assert params["VISO_DELAY_MS"]["keep_current"] is True and params["VISO_DELAY_MS"]["status"] == "ok"
    assert params["VISO_DELAY_MS"]["suggested"] == 10.0
    assert params["VISO_TYPE"]["rationale"] and params["VISO_TYPE"]["allowed"] == [0, 1, 2, 3]
    assert params["VISO_POS_X"]["min"] == -5.0 and params["VISO_POS_X"]["integer"] is False
    # Read never writes
    assert vehicle.param_sets() == []
    assert {m["type"] for m in vehicle.posted} == {"PARAM_REQUEST_READ"}
    assert vehicle.params["VISO_TYPE"] == 0.0
    # The page header learns the armed state
    assert http.get("/api/status").json()["vehicle"]["armed"] is False


def test_get_reports_unreadable_params(client, vehicle):
    main, http = client
    del vehicle.params["VISO_ORIENT"]
    params = by_name(http.post("/api/autopilot/params/read").json())
    assert params["VISO_ORIENT"]["status"] == "unreadable" and params["VISO_ORIENT"]["current"] is None
    assert params["VISO_ORIENT"]["error"] == "no PARAM_VALUE reply"
    assert params["VISO_TYPE"]["status"] == "differs"
    assert vehicle.param_sets() == []


def test_get_without_mavlink2rest(client, vehicle):
    main, http = client
    vehicle.reachable = False
    result = http.post("/api/autopilot/params/read").json()
    assert result["heartbeat"]["reachable"] is False and result["armed"] is None
    assert all(p["status"] == "unreadable" and "not reachable" in p["error"] for p in result["params"])
    assert vehicle.posted == []


def test_get_without_an_autopilot_heartbeat(client, vehicle):
    main, http = client
    vehicle.heartbeat = False
    result = http.post("/api/autopilot/params/read").json()
    assert result["heartbeat"]["reachable"] is True and result["armed"] is None
    assert all(p["status"] == "unreadable" for p in result["params"])
    assert vehicle.posted == []


def test_get_shows_armed(client, vehicle):
    main, http = client
    vehicle.armed = True
    result = http.post("/api/autopilot/params/read").json()
    assert result["armed"] is True and vehicle.param_sets() == []


# --- POST: guarded writes -------------------------------------------------------------------------

@pytest.mark.parametrize("body", [
    {"params": GOOD_WRITE},
    {"confirm": False, "params": GOOD_WRITE},
    {"confirm": "true", "params": GOOD_WRITE},
    [GOOD_WRITE],
])
def test_post_without_confirm_is_400(client, vehicle, body):
    main, http = client
    response = http.post("/api/autopilot/params", json=body)
    assert response.status_code == 400 and "confirm" in response.json()["detail"]
    assert vehicle.posted == []


@pytest.mark.parametrize("params, message", [
    ({"EK2_ENABLE": 0}, "not one of the suggested parameters"),
    ({"AHRS_EKF_TYPE": 1}, "not one of 0, 2, 3, 10, 11"),
    ({"SYSID_THISMAV": 2}, "not one of the suggested parameters"),
    ({"VISO_POS_X": 9.0}, "outside -5..5"),
    ({"VISO_TYPE": 7}, "outside 0..3"),
    ({"VISO_TYPE": 1.5}, "whole number"),
    ({"VISO_ORIENT": 3}, "not one of 0, 2, 4, 6, 24, 25"),
    ({"VISO_DELAY_MS": "10"}, "finite number"),
    ({"VISO_TYPE": True}, "finite number"),
    ({}, "non-empty"),
])
def test_post_rejects_non_whitelisted_and_out_of_range(client, vehicle, params, message):
    main, http = client
    # One bad entry rejects the whole request, valid ones included
    body = {"confirm": True, "params": {"EK3_SRC1_POSZ": 1, **params} if params else {}}
    response = http.post("/api/autopilot/params", json=body)
    assert response.status_code == 400 and message in response.json()["detail"]
    assert vehicle.posted == []


def test_post_refuses_when_armed(client, vehicle):
    main, http = client
    vehicle.armed = True
    response = http.post("/api/autopilot/params", json={"confirm": True, "params": GOOD_WRITE})
    assert response.status_code == 409
    assert "ARMED" in response.json()["detail"] and response.json()["heartbeat"]["armed"] is True
    assert vehicle.posted == [] and vehicle.params["VISO_TYPE"] == 0.0
    assert http.get("/api/status").json()["vehicle"]["armed"] is True


@pytest.mark.parametrize("setup, message", [
    (lambda v: setattr(v, "heartbeat", False), "unknown"),
    (lambda v: setattr(v, "reachable", False), "not reachable"),
    (lambda v: setattr(v, "heartbeat_hz", 1e-6), "no new HEARTBEAT"),  # mavlink2rest keeps a stale message
])
def test_post_refuses_when_the_armed_state_is_unknown(client, vehicle, setup, message):
    main, http = client
    setup(vehicle)
    response = http.post("/api/autopilot/params", json={"confirm": True, "params": GOOD_WRITE})
    assert response.status_code == 409 and message in response.json()["detail"]
    assert response.json()["heartbeat"]["armed"] is None
    assert vehicle.param_sets() == []


def test_post_writes_when_disarmed_and_confirmed(client, vehicle):
    main, http = client
    response = http.post("/api/autopilot/params", json={"confirm": True, "params": GOOD_WRITE})
    assert response.status_code == 200
    result = response.json()
    changed = {entry["name"]: entry for entry in result["changed"]}
    assert set(changed) == {"VISO_TYPE", "VISO_POS_X"}
    assert changed["VISO_TYPE"] == {"name": "VISO_TYPE", "old": 0.0, "new": 1.0, "readback": 1.0}
    assert changed["VISO_POS_X"]["old"] == 0.0 and changed["VISO_POS_X"]["readback"] == pytest.approx(0.1655)
    assert result["unchanged"] == [{"name": "EK3_SRC1_POSZ", "value": 1.0}]
    assert result["failed"] == [] and result["reboot_required"] == ["VISO_TYPE"]
    assert result["heartbeat"]["armed"] is False
    sets = vehicle.param_sets()
    assert [vehicle._param_name(m["param_id"]) for m in sets] == ["VISO_TYPE", "VISO_POS_X"]
    assert sets[1]["param_type"] == {"type": "MAV_PARAM_TYPE_REAL32"}
    assert vehicle.params["VISO_TYPE"] == 1.0
    # A fresh read now agrees
    params = by_name(http.post("/api/autopilot/params/read").json())
    assert params["VISO_TYPE"]["status"] == "ok" and params["VISO_POS_X"]["status"] == "ok"


def test_post_reports_a_readback_mismatch(client, vehicle):
    main, http = client
    vehicle.ignore_param_set = {"VISO_POS_Y"}
    result = http.post("/api/autopilot/params", json={"confirm": True, "params": {"VISO_POS_Y": -0.0069}}).json()
    assert result["changed"] == [] and result["failed"][0]["name"] == "VISO_POS_Y"
    assert result["failed"][0]["error"] == "read back does not match" and result["failed"][0]["readback"] == 0.0


def test_post_stops_writing_when_the_vehicle_arms(client, vehicle):
    main, http = client
    post = vehicle.post

    def arm_after_first_set(payload):
        post(payload)
        if payload["message"]["type"] == "PARAM_SET":
            vehicle.armed = True

    vehicle.post = arm_after_first_set
    result = http.post("/api/autopilot/params",
                       json={"confirm": True, "params": {"VISO_TYPE": 1, "VISO_POS_X": 0.2, "VISO_POS_Z": 0.1}}).json()
    assert [entry["name"] for entry in result["changed"]] == ["VISO_TYPE"]
    assert [entry["name"] for entry in result["failed"]] == ["VISO_POS_X", "VISO_POS_Z"]
    assert all("armed" in entry["error"] for entry in result["failed"])
    assert len(vehicle.param_sets()) == 1 and vehicle.params["VISO_POS_X"] == 0.0


def test_nothing_is_written_at_startup_or_by_reading(client, vehicle):
    main, http = client
    http.get("/")
    http.get("/api/status")
    http.get("/api/history")
    http.post("/api/autopilot/params/read")
    assert vehicle.param_sets() == []


def test_a_get_never_reaches_the_autopilot(client, vehicle):
    """BlueOS's helper GETs every path in /openapi.json: reading and writing are POST only."""
    main, http = client
    response = http.get("/api/autopilot/params")
    assert response.status_code == 405 and response.headers["allow"] == "POST"
    assert http.get("/api/autopilot/params/read").status_code == 405
    assert vehicle.requests == [] and vehicle.param_sets() == []


# --- the parameter check in /api/status (param_checker) -------------------------------------------

def test_status_has_the_parameter_check_and_its_get_never_reaches_the_autopilot(client, vehicle):
    main, http = client
    check = http.get("/api/status").json()["autopilot_params"]
    assert check["checked_at"] is None and check["reachable"] is None and check["ok"] is False
    assert check["error"] == "not checked yet" and check["mismatches"] == []
    assert vehicle.requests == []
    http.post("/api/autopilot/params/read")  # the on-demand check
    check = http.get("/api/status").json()["autopilot_params"]
    assert check["reachable"] is True and check["armed"] is False and check["ok"] is False
    assert [m["name"] for m in check["mismatches"]] == ["VISO_TYPE", "VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z",
                                                        "EK3_SRC1_VELXY", "EK3_SRC1_POSXY"]
    assert check["mismatches"][0] == {"name": "VISO_TYPE", "current": 0.0, "recommended": 1, "needs_reboot": True}
    assert check["mavlink2rest"] == "http://fake-vehicle/mavlink2rest"
    requests = len(vehicle.requests)
    http.get("/api/status")
    # The Cockpit widget's status line (sent on its event streams) carries the compact check
    widget = main.widget_status_now()["autopilot_params"]
    assert widget["ok"] is False and widget["reachable"] is True and len(widget["mismatches"]) == 6
    assert set(widget["mismatches"][0]) == {"name"}
    assert len(vehicle.requests) == requests and vehicle.param_sets() == []


def test_server_startup_runs_the_read_only_check_and_configure_fixes_it(client, vehicle, monkeypatch):
    main, http = client
    monkeypatch.setattr(main, "start_vio", lambda: None)
    monkeypatch.setattr(main.supervisor, "shutdown", lambda: None)
    start = main.param_checker.start
    monkeypatch.setattr(main.param_checker, "start", lambda: start(first_delay=0.0))
    with type(http)(main.app) as live:
        deadline = time.monotonic() + 10.0
        while live.get("/api/status").json()["autopilot_params"]["checked_at"] is None:
            assert time.monotonic() < deadline
            time.sleep(0.02)
        check = live.get("/api/status").json()["autopilot_params"]
        assert len(check["mismatches"]) == 6 and vehicle.param_sets() == []
        # Configure: exactly the mismatches, through the guarded POST; checked again right after
        changes = {m["name"]: m["recommended"] for m in check["mismatches"]}
        result = live.post("/api/autopilot/params", json={"confirm": True, "params": changes}).json()
        assert [c["name"] for c in result["changed"]] == list(changes) and result["reboot_required"] == ["VISO_TYPE"]
        check = live.get("/api/status").json()["autopilot_params"]
        assert check["mismatches"] == [] and check["pending_reboot"] == ["VISO_TYPE"] and check["ok"] is False
        vehicle.reboot()
        live.post("/api/autopilot/params/read")
        check = live.get("/api/status").json()["autopilot_params"]
        assert check["pending_reboot"] == [] and check["ok"] is True
    assert main.param_checker._thread is None  # stopped at shutdown
    assert len(vehicle.param_sets()) == 6


def test_live_armed_state_needs_a_heartbeat_counter(vehicle):
    autopilot = autopilot_params.AutopilotParams("fake-vehicle", timeout=0.2)
    entry = vehicle.heartbeat_entry()
    del entry["status"]["time"]["counter"]
    vehicle.heartbeat_entry = lambda: entry
    state = autopilot.live_armed_state(wait=0.2)
    assert state["armed"] is None and "counter" in state["detail"]
