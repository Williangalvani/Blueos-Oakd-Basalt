"""
Mock mode (mock_mode.py, mock_vio.py): the real server started as on a PC, `python -m uvicorn main:app` with
OAKD_MOCK=1, its VIO process the fake one, and everything checked through HTTP.
"""
import http.client
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import pytest

APP = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(APP))
sys.path.insert(0, str(APP / "tools"))

import depth_stream  # noqa: E402
import dvl_source  # noqa: E402
import feature_stream  # noqa: E402
import mock_mode  # noqa: E402

pytest.importorskip("fastapi")
STARTUP_TIMEOUT_S = 30.0


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def request(port, method, path, body=None, timeout=15.0):
    """(status, headers, body bytes)"""
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=timeout)
    try:
        headers = {"Content-Type": "application/json"} if body is not None else {}
        conn.request(method, path, body=None if body is None else json.dumps(body), headers=headers)
        response = conn.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally:
        conn.close()


def get_json(port, path, method="GET", body=None):
    status, _, data = request(port, method, path, body)
    return status, json.loads(data)


def wait_running(port, process=None, timeout=STARTUP_TIMEOUT_S):
    """The /api/status of a mock server once its fake VIO has published at least a second of frames."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        assert process is None or process.poll() is None, f"server exited with {process.returncode}"
        try:
            status, last = get_json(port, "/api/status")
            telemetry = last.get("telemetry") or {}
            if status == 200 and telemetry.get("phase") == "running" and telemetry.get("frames_total", 0) >= 10 \
                    and not telemetry.get("stale"):
                return last
        except (OSError, ValueError):
            pass
        time.sleep(0.2)
    raise AssertionError(f"mock server not running after {timeout} s: {last}")


def start_server(tmp_path, command, mock_env=True):
    env = {**os.environ, mock_mode.DIR_ENV: str(tmp_path)}
    env.pop("OAKD_LOG_ONLY", None)
    env.pop(mock_mode.ENV, None)
    if mock_env:
        env[mock_mode.ENV] = "1"
    log = open(tmp_path / "server.log", "wb")
    return subprocess.Popen(command, cwd=APP, env=env, stdout=log, stderr=subprocess.STDOUT)


def stop_server(process):
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:  # a zombie has exited
        return Path(f"/proc/{pid}/stat").read_text().split(") ")[1][0] != "Z"
    except OSError:
        return False


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    """`python -m uvicorn main:app --port N` from app/ with OAKD_MOCK=1, its data in a temporary OAKD_MOCK_DIR."""
    root = tmp_path_factory.mktemp("mock")
    port = free_port()
    process = start_server(root, [sys.executable, "-m", "uvicorn", "main:app", "--host", "127.0.0.1",
                                  "--port", str(port)])
    try:
        first = wait_running(port, process)
        yield port, root, first
    finally:
        stop_server(process)


# --- the pieces, in-process -------------------------------------------------------------------------

def test_mock_mode_imports_without_depthai(tmp_path):
    code = ("import sys; sys.modules['depthai'] = None\n"
            "import main, mock_vio\n"
            "assert main.MOCK and 'depthai' not in [m for m in sys.modules if sys.modules[m] is not None]\n"
            "print(main.CONFIG_FILE)")
    env = {**os.environ, mock_mode.ENV: "1", mock_mode.DIR_ENV: str(tmp_path)}
    out = subprocess.run([sys.executable, "-c", code], cwd=APP, env=env, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().splitlines()[-1] == str(tmp_path / "config.json")
    assert json.loads((tmp_path / "config.json").read_text())["algorithm"] == "pnp"


def test_enabled_and_data_dir():
    assert mock_mode.enabled({"OAKD_MOCK": "1"}) and mock_mode.enabled({"OAKD_MOCK": " True "})
    assert not mock_mode.enabled({}) and not mock_mode.enabled({"OAKD_MOCK": "0"})
    assert mock_mode.data_dir({"OAKD_MOCK_DIR": "/x/y"}) == Path("/x/y")
    assert mock_mode.data_dir({}).name == f"oakd-vio-mock-{os.getuid()}"


@pytest.mark.parametrize("algorithm", ["pnp", "basalt"])
def test_the_mock_vio_takes_the_real_command_line(algorithm, tmp_path):
    import main
    import mock_vio
    config = {**main.DEFAULT_CONFIG, "algorithm": algorithm, "fps": 12, "resolution_width": 480,
              "resolution_height": 300, "stereo_depth_min_mm": 400, "stereo_depth_max_mm": 3500, "log_only": True,
              "lever_arm_enabled": True, "intrinsics_override": "300,300,240,150", "camera_angle": -30}
    command = mock_mode.vio_command(main.build_vio_command(config), tmp_path / "snapshots", tmp_path / "logs",
                                    python="python3")
    assert command[:3] == ["python3", "-u", str(mock_mode.MOCK_VIO)]
    assert "/app/pnp_vio.py" not in command and "/app/basalt_configurable.py" not in command
    args = mock_vio.parse_args(command[3:])
    assert args.algorithm == algorithm and args.fps == 12 and (args.width, args.height) == (480, 300)
    assert args.log_only and args.snapshot_dir == tmp_path / "snapshots" and args.log_dir == tmp_path / "logs"
    if algorithm == "pnp":
        assert (args.stereo_depth_min_mm, args.stereo_depth_max_mm) == (400, 3500)
        assert args.telemetry_dir == main.telemetry_dir()


def test_mock_autopilot_reads_and_never_writes():
    import autopilot_params
    autopilot = mock_mode.MockAutopilot()
    result = autopilot_params.read_suggested_params(autopilot)
    assert result["armed"] is False and result["heartbeat"]["reachable"]
    assert {p["name"] for p in result["params"]} == set(mock_mode.PARAM_VALUES)
    assert all(p["status"] in ("ok", "differs") for p in result["params"])
    with pytest.raises(PermissionError):
        autopilot.write("VISO_TYPE", 1, "MAV_PARAM_TYPE_REAL32")
    written = autopilot_params.write_suggested_params(autopilot, {"VISO_POS_X": 0.2}, heartbeat_wait=0)
    assert written["changed"] == [] and "PARAM_SET failed" in written["failed"][0]["error"]
    assert autopilot.values["VISO_POS_X"] == mock_mode.PARAM_VALUES["VISO_POS_X"]


def test_the_write_route_refuses_in_mock_mode_before_anything_else(monkeypatch):
    from fastapi.testclient import TestClient
    import main
    monkeypatch.setattr(main, "MOCK", True)
    monkeypatch.setattr(main, "make_autopilot", lambda: pytest.fail("no autopilot may be contacted"))
    client = TestClient(main.app)
    for body in ({"confirm": True, "params": {"VISO_POS_X": 0.1}}, {"params": {}}, {}):
        r = client.post("/api/autopilot/params", json=body)
        assert r.status_code == 403 and r.json()["detail"] == mock_mode.WRITE_REFUSED


def test_block_network_refuses_every_url(monkeypatch):
    monkeypatch.setattr(urllib.request, "urlopen", urllib.request.urlopen)  # restored after the test
    mock_mode.block_network()
    for url in ("http://127.0.0.1/mavlink2rest/mavlink/vehicles", urllib.request.Request("http://192.168.2.2/")):
        with pytest.raises(urllib.error.URLError, match="mock mode"):
            urllib.request.urlopen(url, timeout=1)


def test_the_mock_dvl_feeds_the_real_poller():
    now = [1795000000.0]
    poller = dvl_source.DvlPoller("http://vehicle/mavlink2rest", "auto",
                                  fetch=lambda url, timeout=None: mock_mode.dvl_fetch(url, clock=lambda: now[0]),
                                  clock=lambda: now[0])
    for _ in range(6):
        poller.poll_once()
        now[0] += 0.5
    snapshot = poller.snapshot(now=now[0])
    assert snapshot["source"] == {"system_id": mock_mode.DVL_SYSTEM, "component_id": mock_mode.DVL_COMPONENT}
    assert snapshot["available"] and len(snapshot["t"]) >= 4
    expected = mock_mode.motion(snapshot["t"][-1])[0]
    assert abs(snapshot["vx"][-1] - 1.03 * expected) < 0.06


def test_the_fake_pnp_tracks_then_fails_in_a_turbid_spell(tmp_path):
    import mock_vio
    import vio_log
    import vio_telemetry
    now = [1795000000.0]
    args = mock_vio.parse_args(["--fps", "10", "--recovery-frames", "20", "--snapshot-dir", str(tmp_path)])
    vio = mock_vio.FakeVio(args, np.random.default_rng(2), clock=lambda: now[0])
    vio.telemetry = vio_telemetry.Telemetry(tmp_path / "telemetry", fps=10, clock=lambda: now[0], log=print)
    vio.data_log = vio_log.JsonlLog(tmp_path / "log.jsonl")

    def run(seconds):
        frames = []
        for _ in range(int(seconds * 10)):
            now[0] += 0.1
            frames.append(vio.step(now[0], 0.1))
        return frames

    calm = run(mock_vio.FAILURE_PERIOD_S - mock_vio.TURBID_S - 1.0)
    assert calm[0]["reason"] == "no_reference"
    ok = [f for f in calm if f["ok"]]
    assert len(ok) > 0.9 * len(calm)
    assert all(f["position_delta"] is not None and f["inliers"] >= args.min_inliers for f in ok)
    assert {f["reason"] for f in calm if not f["ok"]} <= {"no_reference", "gyro_mismatch"}
    assert any(f["inlier_index"] is not None and len(f["inlier_index"]) < f["matches"] for f in ok)
    assert any(not f["valid"].all() for f in calm)

    turbid = run(1.0 + mock_vio.TURBID_S)[10:]
    assert vio.turbidity(now[0] - 0.05) > 0.5
    reasons = {f["reason"] for f in turbid if not f["ok"]}
    assert len([f for f in turbid if not f["ok"]]) > 0.8 * len(turbid)
    assert reasons & {"too_few_matches", "too_few_inliers"}
    assert vio.telemetry.recoveries >= 1 and any(f["reference_reset"] for f in turbid)
    assert all(f["ok"] for f in run(2.0)[-5:])  # clear water: a fresh reference within max_reference_age


def test_the_mock_scene_pan_folds_back_instead_of_sticking_at_the_edge():
    import mock_vio
    assert [mock_vio.fold(v, 10) for v in (-10, -3, 0, 7, 10)] == [-10, -3, 0, 7, 10]
    assert mock_vio.fold(12, 10) == 8 and mock_vio.fold(30, 10) == -10 and mock_vio.fold(-12, 10) == -8
    values = [mock_vio.fold(v / 10, 10) for v in range(-1000, 1000)]
    assert max(map(abs, np.diff(values))) <= 0.1 + 1e-9 and min(values) >= -10 and max(values) <= 10


def mock_depth_vio(tmp_path, *argv):
    import mock_vio
    import vio_telemetry
    now = [1795000000.0]
    args = mock_vio.parse_args(["--fps", "10", "--snapshot-dir", str(tmp_path), "--telemetry-dir",
                                str(tmp_path / "telemetry"), *argv])
    vio = mock_vio.FakeVio(args, np.random.default_rng(2), clock=lambda: now[0])
    vio.telemetry = vio_telemetry.Telemetry(tmp_path / "telemetry", fps=10, clock=lambda: now[0], log=print)
    return vio, vio.make_depth_stream()


def test_the_mock_host_jpeg_variant_encodes_on_the_pi(tmp_path):
    import cv2
    vio, stream = mock_depth_vio(tmp_path, "--depth-stream-encoding", "host_jpeg")
    assert stream.on_host and stream.variant == "host_jpeg" and stream.meta()["encoded_on"] == "pi"
    stream.host_encoder.threaded = False
    depth_stream.touch_lease(tmp_path / "telemetry")
    depth = vio.scene.depth_image(1795000000.0, vio.offset)
    stream.tick(depth, 1)
    assert stream.enabled and stream.frames == 1
    jpeg = (tmp_path / "telemetry" / depth_stream.FRAME_FILE).read_bytes()
    gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert gray.shape == (400, 640)
    reference = depth_stream.gray_reference(depth, *vio.depth_range).astype(int)
    assert np.abs(gray.astype(int) - reference).mean() < 6
    stream.close()


@pytest.mark.parametrize("encoding, variant", [("auto", "manip"), ("nv12", "nv12"), ("yuv400p", "yuv400p")])
def test_the_mock_reports_the_camera_variant_and_its_debug_log(tmp_path, encoding, variant):
    vio, stream = mock_depth_vio(tmp_path, "--depth-stream-encoding", encoding, "--depth-stream-debug")
    meta = stream.meta()
    assert (meta["variant"], meta["requested_variant"], meta["encoded_on"]) == (variant, encoding, "camera")
    depth_stream.touch_lease(tmp_path / "telemetry")
    stream.tick()
    assert stream.enabled and vio.log_device.level == "DEBUG"
    assert stream.on_frame(vio.encode_depth(1795000000.0), 1)
    lines = [line["text"] for line in stream.meta()["device_log"]["stream"]]
    assert lines[0].startswith("log level DEBUG") and "VideoEncoder took" in lines[-1] and "(mock)" in lines[-1]
    stream.close()
    assert vio.log_device.level == "WARN"


def test_the_synthetic_stereo_is_consistent_with_its_depth():
    import mock_vio
    import rectified_intrinsics
    scene = mock_vio.Scene(640, 400, (300, 4000), np.random.default_rng(1))
    t, offset = 1795000123.4, (600.0, 380.0)
    left, right, depth = (scene.frame(s, t, offset) for s in ("rect_left", "rect_right", "depth"))
    assert left.dtype == right.dtype == np.uint8 and depth.dtype == np.uint16 and depth.shape == (400, 640)
    valid = depth > 0
    assert 0.7 < valid.mean() < 0.99 and not valid[:, :scene.edge_band].any()
    measured = rectified_intrinsics.measure_rectified_focal(left, right, depth, mock_vio.BASELINE_M)
    assert measured["n_matches"] > 100
    assert abs(measured["f_from_depth"] / scene.fx - 1.0) < 0.03


# --- the server, over HTTP -----------------------------------------------------------------------------

def test_status_shows_mock_and_a_running_vio(server):
    port, root, _ = server
    status, s = get_json(port, "/api/status")
    assert status == 200 and s["mock"] is True and s["running"] and s["algorithm"] == "pnp"
    t = s["telemetry"]
    assert t["phase"] == "running" and not t["stale"] and t["pid"] == s["pid"]
    recent = t["recent"]
    assert recent["fps"] > 3 and recent["ok_percent"] > 50 and recent["inliers_mean"] > 15
    assert 0 < recent["proc_ms_mean"] <= recent["proc_ms_max"]
    assert t["last_frame"]["valid_depth"] > 0 and t["sent_total"] > 0 and t["output"]["mode"] == "sending"
    assert s["camera_connected"] is True and 40 < s["chip_temperature_c"] < 55
    assert s["cpu_percent"] is None or 0 <= s["cpu_percent"] <= 100
    assert s["vehicle"]["armed"] is False and "mock" in s["vehicle"]["detail"]
    assert s["camera"]["intrinsics"] and s["camera"]["mock"]
    depth = s["depth_stream"]
    assert depth["enabled"] and depth["mock"] and not depth["stale"]
    assert depth["frames_total"] == 0 and not depth["streaming"]  # nothing encoded before a client asks
    _, history = get_json(port, "/api/history?seconds=30")
    assert history["vio_running"]


def test_the_depth_stream_sends_three_jpeg_parts(server):
    import cv2
    port, root, _ = server
    status, headers, body = request(port, "GET", "/api/stream/depth?frames=3")
    assert status == 200 and headers["content-type"].startswith("multipart/x-mixed-replace")
    parts = [part for part in body.split(b"--depthframe\r\n") if part]
    assert len(parts) == 3
    for part in parts:
        head, jpeg = part.split(b"\r\n\r\n", 1)
        assert b"Content-Type: image/jpeg" in head
        jpeg = jpeg[:-2] if jpeg.endswith(b"\r\n") else jpeg
        assert jpeg[:2] == b"\xff\xd8"
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_UNCHANGED)
        assert image.shape == (400, 640) and image.max() > 100 and (image < 8).mean() > 0.01
    deadline = time.monotonic() + 3.0
    while True:  # the metadata (with the centre depth) is rewritten once a second while streaming
        _, s = get_json(port, "/api/status")
        depth = s["depth_stream"]
        if (depth["center"] and depth["frames_total"] >= 3) or time.monotonic() > deadline:
            break
        time.sleep(0.2)
    assert depth["frames_total"] >= 3 and 300 < depth["center"]["median_mm"] < 4000


def test_the_pose_stream_follows_the_mock_survey(server):
    port, root, _ = server
    status, headers, body = request(port, "GET", "/api/stream/pose?events=12")
    assert status == 200 and headers["content-type"].startswith("text/event-stream")
    events = [block.split("\n") for block in body.decode().split("\n\n") if block.startswith("event: ")]
    kinds = [lines[0][len("event: "):] for lines in events]
    poses = [json.loads(lines[1][len("data: "):]) for lines in events if lines[0] == "event: pose"]
    assert "status" in kinds and len(poses) >= 5
    assert all(p["frame"] == "start-FRD" and len(p["q"]) == 4 for p in poses)
    assert len({p["epoch"] for p in poses}) == 1 and poses[-1]["frames"] > poses[0]["frames"]
    moved = math.dist([poses[0][k] for k in "xyz"], [poses[-1][k] for k in "xyz"])
    assert moved > 0.01  # the survey moves at 0.15 - 0.35 m/s
    status, pose = get_json(port, "/api/pose")
    assert status == 200 and pose["epoch"] == poses[0]["epoch"]
    _, s = get_json(port, "/api/status")
    assert s["depth_stream"]["variant"] == "manip" and s["depth_stream"]["configured_encoding"] == "auto"


def test_the_feature_stream_yields_events(server):
    port, root, _ = server
    wanted = {feature_stream.INLIER, feature_stream.OUTLIER, feature_stream.NO_DEPTH}
    states = set()
    for _ in range(4):  # a random frame can lack a state: ask again
        status, headers, body = request(port, "GET", "/api/stream/features?events=8")
        assert status == 200 and headers["content-type"].startswith("text/event-stream")
        events = [block.split("\n") for block in body.decode().split("\n\n") if block.startswith("event: ")]
        kinds = [lines[0][len("event: "):] for lines in events]
        assert len(events) == 8 and "features" in kinds and "status" in kinds
        for lines in events:
            data = json.loads(lines[1][len("data: "):])
            if lines[0] == "event: status":
                assert data["mock"] is True and data["problem"] is None
                continue
            assert data["frame_size"] == [640, 400] and data["states"] == list(feature_stream.STATES)
            points = np.array(data["points"])
            assert len(points) == 300 and sum(data["counts"].values()) == 300
            assert (points[:, 0] >= 0).all() and (points[:, 0] < 640).all()
            assert (points[:, 1] >= 0).all() and (points[:, 1] < 400).all()
            states |= set(points[:, 2].astype(int).tolist())
        if wanted <= states:
            break
    assert wanted <= states


def test_parameters_read_from_the_mock_and_writes_are_refused(server):
    port, root, _ = server
    status, read = get_json(port, "/api/autopilot/params/read", method="POST")
    assert status == 200 and read["armed"] is False and read["mavlink2rest"] == mock_mode.AUTOPILOT_URL
    before = {p["name"]: p["current"] for p in read["params"]}
    assert before == mock_mode.PARAM_VALUES
    status, refused = get_json(port, "/api/autopilot/params", method="POST",
                               body={"confirm": True, "params": {"VISO_POS_X": 0.1655, "EK3_SRC1_POSXY": 0}})
    assert status == 403 and refused["detail"] == mock_mode.WRITE_REFUSED
    _, again = get_json(port, "/api/autopilot/params/read", method="POST")
    assert {p["name"]: p["current"] for p in again["params"]} == before


def test_snapshots_probe_and_focal_check_give_synthetic_results(server):
    import cv2
    port, root, _ = server
    status, headers, png = request(port, "GET", "/api/snapshot?stream=rect_left&overlay=1")
    assert status == 200 and headers["x-overlay"] == "features"
    assert sum(int(headers[f"x-overlay-{kind}"]) for kind in ("inliers", "outliers", "unmatched")) == 300
    assert cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED).shape == (400, 640, 3)
    status, _, png = request(port, "GET", "/api/snapshot?stream=depth&raw=1")
    raw = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    assert status == 200 and raw.dtype == np.uint16 and 300 < np.median(raw[raw > 0]) < 4000
    status, error = get_json(port, "/api/snapshot?stream=rgb")
    assert status == 404 and "mock mode" in error["detail"]
    status, probe = get_json(port, "/api/depth_probe?frames=3", method="POST")
    assert status == 200 and probe["frames"] == 3 and 300 < probe["median_mm"] < 4000
    status, focal = get_json(port, "/api/focal_check", method="POST")
    assert status == 200 and focal["ok"] and focal["n_matches"] >= 10, focal


def test_the_blueos_helper_scan_still_finishes_at_once_in_mock_mode(server):
    """test_openapi_scan's walk of /openapi.json, against the mock server with its fake VIO running."""
    from test_openapi_scan import SAFE_GETS, helper_get
    port, root, _ = server
    _, s = get_json(port, "/api/status")
    pid = s["pid"]
    snapshot_files = {p.name: p.stat().st_mtime_ns for p in (root / "snapshots").glob("*.json")}
    probe_results = sorted((root / "snapshots" / "depth_probe" / "results").glob("*.json"))
    start = time.monotonic()
    for path in ("/", "/register_service", "/docs", "/widget/depth", "/widget/pose", "/cockpit_extras.json"):
        status, body, _ = helper_get(port, path)
        assert status == 200 and body, path
    status, body, _ = helper_get(port, "/openapi.json")
    paths = json.loads(body)["paths"]
    assert {path for path, operations in paths.items() if "get" in operations} == set(SAFE_GETS)
    for path, operations in paths.items():
        status, body, seconds = helper_get(port, path)
        assert status != "timeout" and body is not None, f"{path}: the scan would hang ({status})"
        assert seconds < 0.5, f"{path} took {seconds:.2f} s"
        expected = 405 if "get" not in operations else 404 if path == "/api/logs/{name}" else 200
        assert status == expected, (path, status)
    assert time.monotonic() - start < 3.0
    _, s = get_json(port, "/api/status")
    assert s["pid"] == pid and s["restart_count"] == 0  # no VIO restart
    assert {p.name: p.stat().st_mtime_ns for p in (root / "snapshots").glob("*.json")} == snapshot_files
    assert sorted((root / "snapshots" / "depth_probe" / "results").glob("*.json")) == probe_results
    _, read = get_json(port, "/api/autopilot/params/read", method="POST")
    assert {p["name"]: p["current"] for p in read["params"]} == mock_mode.PARAM_VALUES


def test_stop_and_start_through_the_api(server):
    port, root, _ = server
    _, s = get_json(port, "/api/status")
    old = s["pid"]
    assert get_json(port, "/api/stop", method="POST")[0] == 200
    _, s = get_json(port, "/api/status")
    assert not s["running"] and not pid_alive(old)
    assert get_json(port, "/api/start", method="POST")[0] == 200
    s = wait_running(port)
    assert s["running"] and s["pid"] != old and s["mock"]


def test_main_py_mock_flag_and_shutdown_stops_the_fake_vio(tmp_path):
    """`python main.py --mock --port N`; on SIGTERM the server stops its fake VIO process too."""
    port = free_port()
    process = start_server(tmp_path, [sys.executable, "main.py", "--mock", "--host", "127.0.0.1", "--port",
                                      str(port)], mock_env=False)
    try:
        s = wait_running(port, process)
        assert s["mock"] and (tmp_path / "logs").is_dir()
        vio_pid = s["pid"]
        assert pid_alive(vio_pid)
    finally:
        stop_server(process)
    deadline = time.monotonic() + 10
    while pid_alive(vio_pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert not pid_alive(vio_pid)
    logs = sorted((tmp_path / "logs").glob("vio-*.jsonl"))
    assert logs and json.loads(logs[-1].read_text().splitlines()[-1])["event"] == "exit"
