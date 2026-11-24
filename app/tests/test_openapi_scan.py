"""
BlueOS's helper (core/services/helper/main.py, detect_service) GETs every path listed in each port's
/openapi.json every few seconds, with http.client (timeout=1 s per socket operation) and response.read() of
every 200 body to its end. A stream listed there hangs that scan, and with it /helper/v1.0/web_services
for the whole of BlueOS; a listed GET that starts work runs it every few seconds. These tests run that
scan against the real app, served by uvicorn, while it believes the PnP VIO is running.
"""
import http.client
import json
import os
import socket
import sys
import threading
import time
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import vio_log  # noqa: E402
import vio_telemetry  # noqa: E402
from fake_mavlink2rest import FakeVehicle  # noqa: E402

# Every GET the schema may list: answers at once with a bounded body and starts nothing on the camera, the
# VIO process or the autopilot. Anything else must be include_in_schema=False, or a POST if it runs something.
SAFE_GETS = {
    "/": "the web page",
    "/widget/depth": "the Cockpit widget page (the streams are opened by its script, not by this GET)",
    "/widget/pose": "the 3D pose Cockpit widget page (its script opens /api/stream/pose, not this GET)",
    "/register_service": "BlueOS service metadata",
    "/cockpit_extras.json": "Cockpit extras manifest",
    "/api/config": "the saved configuration",
    "/api/config/schema": "the configuration schema",
    "/api/status": "status; renews the status lease (the VIO writes status.json about once a second meanwhile)",
    "/api/logs": "the list of data logs",
    "/api/logs/{name}": "one data log by name (the scan's literal {name} is a 404)",
}
# The only files those GETs may create or change (relative to the test's tmp_path)
ALLOWED_WRITES = {"telemetry/status.lease"}
HELPER_TIMEOUT_S = 1.0
HELPER_HEADERS = {"User-Agent": "python", "Accept-Encoding": "gzip, deflate", "Accept": "*/*"}


def helper_get(port, path, budget_s=HELPER_TIMEOUT_S):
    """
    GET as BlueOS's Helper.simple_http_request does it (http.client, timeout 1 s, read a 200 body to the end),
    except that it gives up after budget_s in total instead of hanging. (status, body or None, seconds);
    body is None for a 200 that did not end in time, status is "timeout" if the server went silent for 1 s.
    """
    start = time.monotonic()
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=HELPER_TIMEOUT_S)
    try:
        conn.request("GET", path, headers=HELPER_HEADERS)
        response = conn.getresponse()
        if response.status != http.client.OK:
            return response.status, b"", time.monotonic() - start
        body = b""
        while True:
            chunk = response.read1(65536)
            if not chunk:
                return response.status, body, time.monotonic() - start
            body += chunk
            if time.monotonic() - start > budget_s:
                return response.status, None, time.monotonic() - start
    except socket.timeout:
        return "timeout", None, time.monotonic() - start
    finally:
        conn.close()


def files_under(root):
    return {os.path.relpath(os.path.join(directory, name), root): os.stat(os.path.join(directory, name)).st_mtime_ns
            for directory, _, names in os.walk(root) for name in names}


@pytest.fixture
def served(monkeypatch, tmp_path):
    """The app on a local port, as BlueOS sees it, with the PnP VIO 'running' and every side channel recorded."""
    pytest.importorskip("fastapi")
    uvicorn = pytest.importorskip("uvicorn")
    import main
    from test_web_ui import live_telemetry

    vehicle = FakeVehicle(heartbeat_hz=20.0)
    monkeypatch.setattr(urllib.request, "urlopen", vehicle.urlopen)  # every mavlink2rest request lands here
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    (tmp_path / "config.json").write_text(json.dumps({**main.DEFAULT_CONFIG,
                                                      "vehicle_ip": "fake-vehicle"}))
    (tmp_path / "logs").mkdir()
    (tmp_path / "logs" / "vio_20261123_120000.jsonl").write_text('{"type": "event"}\n')
    monkeypatch.setattr(main.supervisor, "status", lambda: {"running": True, "pid": os.getpid(), "restart_count": 0,
                                                            "last_exit_code": None, "restart_in": None})
    actions = []
    monkeypatch.setattr(main, "start_vio", lambda: actions.append("start_vio"))
    monkeypatch.setattr(main, "stop_vio", lambda: actions.append("stop_vio"))
    touch = main.dvl_poller.touch
    monkeypatch.setattr(main.dvl_poller, "touch", lambda *a, **k: (actions.append("dvl_poller"), touch(*a, **k))[1])
    live_telemetry(seconds=2.0).write_status()

    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    server = uvicorn.Server(uvicorn.Config(main.app, lifespan="off", log_level="warning"))
    thread = threading.Thread(target=server.run, kwargs={"sockets": [sock]}, daemon=True)
    thread.start()
    deadline = time.monotonic() + 10
    while not server.started and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.started
    try:
        yield main, sock.getsockname()[1], vehicle, actions
    finally:
        server.should_exit = server.force_exit = True
        thread.join(10)
        sock.close()


def schema_gets(main):
    from fastapi.testclient import TestClient
    paths = TestClient(main.app).get("/openapi.json").json()["paths"]
    return {path for path, operations in paths.items() if "get" in operations}


def test_the_schema_lists_only_known_safe_gets():
    pytest.importorskip("fastapi")
    import main
    gets = schema_gets(main)
    assert not gets - set(SAFE_GETS), f"GET routes in /openapi.json that BlueOS would scan: {gets - set(SAFE_GETS)}"
    assert gets == set(SAFE_GETS), f"SAFE_GETS lists routes no longer in the schema: {set(SAFE_GETS) - gets}"


def test_no_stream_or_side_effect_route_is_in_the_schema():
    pytest.importorskip("fastapi")
    import main
    from fastapi.routing import APIRoute
    from starlette.routing import WebSocketRoute
    assert not [route for route in main.app.routes if isinstance(route, WebSocketRoute)]
    hidden = {route.path for route in main.app.routes
              if isinstance(route, APIRoute) and "GET" in route.methods and not route.include_in_schema}
    assert {"/api/stream/depth", "/api/stream/features", "/api/snapshot", "/api/history", "/api/features"} <= hidden
    posts = {route.path for route in main.app.routes if isinstance(route, APIRoute) and "POST" in route.methods}
    gets = {route.path for route in main.app.routes if isinstance(route, APIRoute) and "GET" in route.methods}
    for path in ("/api/depth_probe", "/api/focal_check", "/api/autopilot/params", "/api/autopilot/params/read"):
        assert path in posts and path not in gets


def test_the_blueos_helper_scan_finishes_at_once_and_starts_nothing(served, tmp_path):
    main, port, vehicle, actions = served
    before = files_under(tmp_path)
    start = time.monotonic()
    # detect_service: the page, the metadata, the docs, the schema, then every path in it
    for path in ("/", "/register_service", "/docs"):
        status, body, _ = helper_get(port, path)
        assert status == 200 and body, path
    status, body, _ = helper_get(port, "/openapi.json")
    assert status == 200
    paths = json.loads(body)["paths"]
    results = {}
    for path in paths:
        results[path] = helper_get(port, path)
    total = time.monotonic() - start

    for path, (status, body, seconds) in results.items():
        assert path in SAFE_GETS or "get" not in paths[path], path
        assert status != "timeout" and body is not None, f"{path}: the scan would hang ({status})"
        assert seconds < 0.5, f"{path} took {seconds:.2f} s"
        # The helper GETs POST-only paths too: 405 at once
        expected = 405 if "get" not in paths[path] else 404 if path == "/api/logs/{name}" else 200
        assert status == expected, (path, status)
    assert total < 3.0
    # Nothing was started: no MAVLink (mavlink2rest), no VIO restart, no DVL polling, no camera requests
    assert vehicle.requests == [] and vehicle.param_sets() == []
    assert actions == []
    after = files_under(tmp_path)
    changed = {path for path, mtime in after.items() if before.get(path) != mtime}
    assert changed <= ALLOWED_WRITES, changed - ALLOWED_WRITES
    assert not (tmp_path / "snapshots" / "requests").exists()
    assert not (tmp_path / "snapshots" / "depth_probe").exists()


def test_the_scan_check_catches_a_stream(served):
    """What /api/stream/features did to the scan while it was in the schema: the body never ends."""
    main, port, vehicle, actions = served
    status, body, seconds = helper_get(port, "/api/stream/features")
    assert body is None and seconds < 2.0


def test_the_old_side_effect_gets_are_gone(served, tmp_path):
    main, port, vehicle, actions = served
    for path in ("/api/depth_probe", "/api/focal_check", "/api/autopilot/params"):
        status, _, seconds = helper_get(port, path)
        assert status == 405 and seconds < 0.5, path
    assert vehicle.requests == [] and actions == []
    assert not (tmp_path / "snapshots" / "requests").exists()
    assert not (tmp_path / "snapshots" / "depth_probe").exists()
