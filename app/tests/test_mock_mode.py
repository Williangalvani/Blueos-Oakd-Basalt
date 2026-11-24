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


MOCK_MISMATCHES = ["VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z", "EK3_SRC1_POSXY"]


def test_mock_autopilot_is_the_dev_vehicle_and_writes_in_memory():
    import autopilot_params
    autopilot = mock_mode.MockAutopilot()
    result = autopilot_params.read_suggested_params(autopilot)
    assert result["armed"] is False and result["heartbeat"]["reachable"]
    assert [p["name"] for p in result["params"]] == [p.name for p in autopilot_params.SUGGESTED_OPTICAL_HOLD_PARAMS]
    assert {p["name"] for p in result["params"]} == set(mock_mode.PARAM_VALUES)
    assert [p["name"] for p in result["params"] if p["status"] == "differs"] == MOCK_MISMATCHES
    written = autopilot_params.write_suggested_params(autopilot, {"VISO_POS_X": 0.1655, "VISO_TYPE": 1},
                                                      heartbeat_wait=0)
    assert [c["name"] for c in written["changed"]] == ["VISO_POS_X"] and written["failed"] == []
    assert written["changed"][0]["readback"] == pytest.approx(0.1655) and written["unchanged"][0]["name"] == "VISO_TYPE"
    assert autopilot.values["VISO_POS_X"] == pytest.approx(0.1655)
    assert mock_mode.MockAutopilot().values["VISO_POS_X"] == 0.0  # each instance starts from PARAM_VALUES


def test_armed_mock_refuses_writes(monkeypatch):
    import autopilot_params
    assert mock_mode.mock_armed({mock_mode.ARMED_ENV: "1"}) and not mock_mode.mock_armed({})
    autopilot = mock_mode.MockAutopilot(armed=True)
    assert autopilot.heartbeat_status()["armed"] is True and "ARMED" in autopilot.heartbeat_status()["detail"]
    with pytest.raises(autopilot_params.WriteRefused, match="ARMED"):
        autopilot_params.write_suggested_params(autopilot, {"VISO_POS_X": 0.1655}, heartbeat_wait=0)
    assert autopilot.values["VISO_POS_X"] == 0.0


def test_the_write_route_in_mock_mode_writes_the_mock_or_refuses_when_armed(monkeypatch):
    from fastapi.testclient import TestClient
    import main
    monkeypatch.setattr(main, "MOCK", True)
    monkeypatch.setattr(main, "mock_autopilot", mock_mode.MockAutopilot(armed=True))
    monkeypatch.setattr(main, "param_checker", main.make_param_checker())
    client = TestClient(main.app)
    r = client.post("/api/autopilot/params", json={"confirm": True, "params": {"EK3_SRC1_POSXY": 0}})
    assert r.status_code == 409 and "ARMED" in r.json()["detail"]
    assert main.mock_autopilot.values["EK3_SRC1_POSXY"] == 6.0
    assert client.get("/api/status").json()["vehicle"]["armed"] is True
    main.mock_autopilot.armed = False
    r = client.post("/api/autopilot/params", json={"confirm": True, "params": {"EK3_SRC1_POSXY": 0}})
    assert r.status_code == 200 and r.json()["mock"] is True
    assert r.json()["changed"] == [{"name": "EK3_SRC1_POSXY", "old": 6.0, "new": 0.0, "readback": 0.0}]
    checked = client.get("/api/status").json()["autopilot_params"]
    assert [m["name"] for m in checked["mismatches"]] == ["VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z"]
    assert client.post("/api/autopilot/params", json={"params": {"EK3_SRC1_POSXY": 6}}).status_code == 400


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


@pytest.mark.parametrize("argv", [[], ["--no-stereo-subpixel"]], ids=["saved", "subpixel-off"])
def test_the_mock_disparity_request_with_a_raw16_disparity_encodes_on_the_pi_and_says_why(tmp_path, argv):
    vio, stream = mock_depth_vio(tmp_path, "--depth-stream-encoding", "disparity", *argv)
    meta = stream.meta()
    assert (meta["variant"], meta["requested_variant"], meta["encoded_on"]) == ("host_jpeg", "disparity", "pi")
    assert meta["fallback"]["at_start"] and meta["fallback"]["reason"].startswith("disparity is RAW16")
    assert meta["source"] == "stereo.depth" and meta["disparity_format"]["type"] == "RAW16"
    stream.close()


def test_the_mock_auto_streams_the_rectified_left_image_on_the_camera(tmp_path):
    import cv2
    vio, stream = mock_depth_vio(tmp_path)  # the saved settings: auto, subpixel and speckle on
    meta = stream.meta()
    assert (meta["variant"], meta["requested_variant"], meta["encoded_on"]) == ("mono_features", "auto", "camera")
    assert meta["encoding"] == "gray8-mono-rectified-left" and meta["source"] == "stereo.rectifiedLeft"
    assert meta["fallback"] is None and meta["variant_reason"] is None
    t = 1795000000.0
    gray = cv2.imdecode(np.frombuffer(vio.encode_depth(t), np.uint8), cv2.IMREAD_GRAYSCALE)
    # The image the mock features are on (rect_left), not depth
    assert gray.shape == (400, 640)
    assert np.abs(gray.astype(int) - vio.scene.frame("rect_left", t, vio.offset).astype(int)).mean() < 4
    stream.close()


@pytest.mark.parametrize("encoding, variant, argv", [
    ("auto", "mono_features", []), ("mono_features", "mono_features", []),
    ("disparity", "disparity", ["--no-stereo-subpixel", "--no-stereo-speckle-filter"]),
    ("disparity", "disparity", ["--no-stereo-subpixel", "--stereo-median-last"]),
    ("manip", "manip", []), ("nv12", "nv12", []), ("yuv400p", "yuv400p", [])])
def test_the_mock_reports_the_camera_variant_and_its_debug_log(tmp_path, encoding, variant, argv):
    vio, stream = mock_depth_vio(tmp_path, "--depth-stream-encoding", encoding, "--depth-stream-debug", *argv)
    meta = stream.meta()
    assert (meta["variant"], meta["requested_variant"], meta["encoded_on"]) == (variant, encoding, "camera")
    assert meta["encoding"] == depth_stream.encoding_for(variant) and meta["fallback"] is None
    depth_stream.touch_lease(tmp_path / "telemetry")
    stream.tick()
    assert stream.enabled and vio.log_device.level == "DEBUG"
    assert stream.on_frame(vio.encode_depth(1795000000.0), 1)
    lines = [line["text"] for line in stream.meta()["device_log"]["stream"]]
    assert lines[0].startswith("log level DEBUG") and "VideoEncoder took" in lines[-1] and "(mock)" in lines[-1]
    stream.close()
    assert vio.log_device.level == "WARN"


def test_the_mock_link_emulates_plausible_counters_and_streams(tmp_path):
    import mock_vio
    now = [1795000000.0]
    args = mock_vio.parse_args(["--fps", "15", "--snapshot-dir", str(tmp_path), "--depth-transport", "full_frame"])
    vio = mock_vio.FakeVio(args, np.random.default_rng(2), clock=lambda: now[0])
    vio.start_link()
    vio.link.sample()
    for frame in range(12 * 15):
        now[0] += 1.0 / 15
        vio.count_link_frame(300)
        if frame % 15 == 14:
            vio.link.sample()
    status = vio.link.status()
    assert status["available"] and status["mock"] is True and "mock" in status["source"]
    assert (status["usb_speed"], status["protocol"]) == ("SUPER", "USB")
    streams = {s["name"]: s for s in status["streams"]}
    depth = 640 * 400 * 2 * 15 * 8 / 1e6
    assert streams["depth"]["mbps"] == pytest.approx(depth, rel=0.01)
    assert streams["imu"]["msgs_per_s"] == pytest.approx(mock_vio.IMU_RATE_HZ, rel=0.02)
    assert streams["features"]["msgs_per_s"] == pytest.approx(15, rel=0.01)
    assert status["down_mbps_10s"] > status["streams_sum_mbps"] > depth
    assert 0 < status["unaccounted_mbps"] < 0.05 * status["streams_sum_mbps"]
    assert 0 < status["up_mbps_10s"] < 0.1
    assert status["share_of_100mbps"] == pytest.approx(status["down_mbps_10s"] / 100)


@pytest.mark.parametrize("rois", ["host", "camera"])
def test_the_mock_points_transport_pairs_every_frame_and_counts_its_streams(tmp_path, rois):
    import depth_points
    import link_stats
    import mock_vio
    now = [1795000000.0]
    args = mock_vio.parse_args(["--fps", "15", "--snapshot-dir", str(tmp_path), "--depth-points-rois", rois])
    assert args.depth_transport == "points"
    vio = mock_vio.FakeVio(args, np.random.default_rng(2), clock=lambda: now[0])
    vio.start_link()
    vio.start_points()
    vio.link.sample()
    n = len(vio.features.uv)
    for sequence in range(1, 12 * 15 + 1):
        now[0] += 1.0 / 15
        frame = {"uv": vio.features.uv}
        points = vio.points_frame(now[0], frame, sequence)
        assert points is not None and points.sequence == sequence and frame["depth_problem"] is None
        # The SLC's median at each feature: the synthetic depth at the window's sampled pixels
        assert 0.9 < frame["valid"].mean() <= 1.0
        vio.count_link_frame(n)
        if sequence % 15 == 0:
            vio.link.sample()
    status = vio.depth_transport_status()
    assert status["transport"] == "points" and status["rois"] == rois and status["paired_percent"] == 100.0
    assert status["missing"] == 0 and status["device"]["size"] == [640, 400] and status["mock"] is True
    assert status["full_depth"] == {"on": False, "every": 0, "reasons": [], "frames": 0, "bytes": 0, "since": None}
    link = vio.link.status()
    streams = {s["name"]: s for s in link["streams"]}
    assert "depth" not in streams and "sync_group" not in streams
    points_mbps = link_stats.spatial_data_bytes(n + depth_points.EXTRA_ROIS) * 15 * 8 / 1e6
    assert streams["depth_points"]["mbps"] == pytest.approx(points_mbps, rel=0.01)
    assert streams["depth_points"]["direction"] == "down"
    if rois == "host":
        up = link_stats.points_request_bytes(n + depth_points.EXTRA_ROIS) * 15 * 8 / 1e6
        assert streams["depth_points_rois"]["mbps"] == pytest.approx(up, rel=0.01)
        assert streams["depth_points_rois"]["direction"] == "up"
        assert link["up_mbps_10s"] > link["streams_up_sum_mbps"] >= up
    else:
        assert "depth_points_rois" not in streams
    # Far below the ~41 Mbps of RAW16 depth at 640x400 (here 300 features at 15 fps: ~3.3 Mbps of points, ~2.3
    # of features)
    assert link["down_mbps_10s"] < 7.0


def test_the_mock_gate_sends_full_depth_frames_only_while_needed(tmp_path):
    import mock_vio
    now = [1795000000.0]
    vio = mock_vio.FakeVio(mock_vio.parse_args(["--fps", "10", "--snapshot-dir", str(tmp_path)]),
                           np.random.default_rng(3), clock=lambda: now[0])
    vio.start_link()
    vio.start_points()
    assert vio.full_depth(now[0], 1, {"depth_probe": 0, "host_jpeg": 0}) is None
    frames = [vio.full_depth(now[0], s, {"depth_probe": 0, "host_jpeg": 2}) for s in range(2, 8)]
    assert [f is not None for f in frames] == [True, False, True, False, True, False]
    assert frames[0].shape == (400, 640) and frames[0].dtype == np.uint16
    assert vio.full_depth(now[0], 9, {"depth_probe": 1, "host_jpeg": 2}) is not None
    assert vio.full_depth(now[0], 10, {}) is None
    gate = vio.gate.status()
    assert gate["frames"] == 4 and gate["bytes"] == 4 * vio.link_stats.frame_bytes(640 * 400 * 2) and not gate["on"]
    assert vio.link.streams.totals()["depth_on_demand"][0] == 4
    assert vio.link.streams.totals()["control"][0] == 4  # off -> 2 -> 1 -> off: sent on each change


def test_the_mock_slc_matches_the_camera_rules():
    import mock_vio
    depth = np.zeros((10, 10), np.uint16)
    depth[0:5, 0:5] = np.arange(25, dtype=np.uint16).reshape(5, 5) * 100 + 1000
    at = lambda u, v: depth[np.clip(v, 0, 9), np.clip(u, 0, 9)]  # noqa: E731
    windows = [[0, 0, 5, 5], [0, 0, 5, 5], [5, 5, 1, 1], [0, 0, 1, 1], [9, 9, 1, 1]]
    rows = mock_vio.slc_rows(at, windows, (1000, 3400))
    # AVERAGE, every pixel: (0,0) 1000 and (4,4) 3400 are excluded (thresholds are exclusive), the other 23 in;
    # only the average is filled, as the firmware does
    inside = [d for d in depth[0:5, 0:5].ravel() if 1000 < d < 3400]
    assert rows[0, 6] == len(inside) == 23 and rows[0, 5] == pytest.approx(np.mean(inside)) and rows[0, 4] == 0
    assert rows[0, :4].tolist() == [0, 0, 5, 5]
    # The last EXTRA_ROIS use the centre thresholds (everything but 0)
    assert rows[2, 6] == 0 and rows[3, 5] == 1000 and rows[3, 6] == 1 and rows[4, 6] == 0


def test_the_mock_counts_the_camera_stream_jpegs(tmp_path):
    vio, stream = mock_depth_vio(tmp_path)
    jpeg = vio.encode_depth(1795000000.0)
    assert vio.link.streams.totals()["depth_stream"] == (1, len(jpeg) + vio.link_stats.ENCODED_FRAME_OVERHEAD)
    stream.close()


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
    link = t["link"]
    assert link["mock"] is True and "mock" in link["source"] and link["available"] and link["usb_speed"] == "SUPER"
    deadline = time.monotonic() + 3.0
    while not link["streams"] and time.monotonic() < deadline:  # rates need two samples, 1 s apart
        time.sleep(0.2)
        link = get_json(port, "/api/status")[1]["telemetry"]["link"]
    names = {stream["name"] for stream in link["streams"]}
    # points (the default): no depth frames, the depth at the features and the ROI configs up
    assert {"depth_points", "depth_points_rois", "features", "imu"} <= names and "depth" not in names
    assert link["down_mbps"] > link["streams_sum_mbps"] * 0.5 > 0
    transport = s["depth_transport"]
    assert transport["configured"] == "points" and transport["configured_rois"] == "host"
    deadline = time.monotonic() + 3.0
    while not (transport["live"] and transport["camera_load"]) and time.monotonic() < deadline:
        time.sleep(0.2)
        transport = get_json(port, "/api/status")[1]["depth_transport"]
    assert transport["live"]["transport"] == "points" and transport["live"]["paired_percent"] > 90
    assert transport["live"]["full_depth"]["on"] is False
    assert 0 < transport["camera_load"]["leon_css_cpu_percent"] < 100
    assert s["viewers"]["video_idle"] is True and s["viewers"]["total"] == 0
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
        # auto = mono_features: the synthetic camera image, textured everywhere (no black no-depth band)
        assert image.shape == (400, 640) and image.std() > 20 and (image[:, :8] > 8).mean() > 0.5
    deadline = time.monotonic() + 3.0
    while True:  # the metadata (with the centre depth) is rewritten once a second while streaming
        _, s = get_json(port, "/api/status")
        depth = s["depth_stream"]
        # The centre patch can be all invalid for a moment (the mock's turbid water): wait for a valid one
        valid = depth["center"] and depth["center"]["median_mm"] is not None
        if (valid and depth["frames_total"] >= 3) or time.monotonic() > deadline:
            break
        time.sleep(0.2)
    assert depth["frames_total"] >= 3 and valid, depth["center"]
    assert 300 < depth["center"]["median_mm"] < 4000


def test_the_pose_stream_follows_the_mock_survey(server):
    port, root, _ = server
    status, headers, body = request(port, "GET", "/api/stream/pose?events=12")
    assert status == 200 and headers["content-type"].startswith("text/event-stream")
    events = [block.split("\n") for block in body.decode().split("\n\n") if block.startswith("event: ")]
    kinds = [lines[0][len("event: "):] for lines in events]
    poses = [json.loads(lines[1][len("data: "):]) for lines in events if lines[0] == "event: pose"]
    assert "status" in kinds and len(poses) >= 5
    # The mock autopilot's ATTITUDE (no gap in the first 112 s): NED with its heading, in rad and deg
    assert all(p["frame"] == "NED" and len(p["q"]) == 4 for p in poses)
    assert all(p["attitude_source"] == "autopilot" and p["attitude_text"] == "ArduPilot ATTITUDE" for p in poses)
    assert all(p["attitude_age_s"] < 3.0 and p["units"]["roll, pitch, yaw"] == "rad" for p in poses)
    assert all(abs(p["yaw_deg"] - math.degrees(p["yaw"])) < 0.01 for p in poses)
    assert len({p["epoch"] for p in poses}) == 1 and poses[-1]["frames"] > poses[0]["frames"]
    moved = math.dist([poses[0][k] for k in "xyz"], [poses[-1][k] for k in "xyz"])
    assert moved > 0.01  # the survey moves at 0.15 - 0.35 m/s
    status, pose = get_json(port, "/api/pose")
    assert status == 200 and pose["epoch"] == poses[0]["epoch"]
    _, s = get_json(port, "/api/status")
    assert s["pose"]["configured_source"] == "autopilot" and s["pose"]["attitude_source"] == "autopilot"
    assert s["pose"]["autopilot"]["fresh"] and "mock autopilot ATTITUDE (gaps)" in s["pose"]["autopilot"]["description"]
    # auto: the rectified-left camera image, encoded on the (mock) camera
    assert s["depth_stream"]["variant"] == "mono_features" and s["depth_stream"]["configured_encoding"] == "auto"
    assert s["depth_stream"]["encoded_on"] == "camera" and s["depth_stream"]["fallback"] is None
    assert s["depth_stream"]["source"] == "stereo.rectifiedLeft"


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


def test_the_parameter_check_shows_the_mock_mismatches_and_configure_fixes_them(server):
    port, root, _ = server
    # The background check runs on its own from startup (FIRST_CHECK_DELAY_S)
    deadline = time.monotonic() + 10.0
    while True:
        _, s = get_json(port, "/api/status")
        checked = s["autopilot_params"]
        if checked["checked_at"] is not None or time.monotonic() > deadline:
            break
        time.sleep(0.2)
    assert checked["reachable"] is True and checked["armed"] is False and checked["ok"] is False
    assert [m["name"] for m in checked["mismatches"]] == MOCK_MISMATCHES
    assert checked["unreadable"] == [] and checked["pending_reboot"] == [] and checked["error"] is None
    status, read = get_json(port, "/api/autopilot/params/read", method="POST")
    assert status == 200 and read["armed"] is False and read["mavlink2rest"] == mock_mode.AUTOPILOT_URL
    before = {p["name"]: p["current"] for p in read["params"]}
    assert before == mock_mode.PARAM_VALUES
    # Configure: exactly the mismatches, at their recommended values
    configure = {m["name"]: m["recommended"] for m in checked["mismatches"]}
    status, written = get_json(port, "/api/autopilot/params", method="POST", body={"confirm": True, "params": configure})
    assert status == 200 and [c["name"] for c in written["changed"]] == MOCK_MISMATCHES and written["failed"] == []
    assert written["reboot_required"] == []
    _, s = get_json(port, "/api/status")
    assert s["autopilot_params"]["ok"] is True and s["autopilot_params"]["mismatches"] == []
    # Put the dev vehicle's values back for the other tests
    status, _ = get_json(port, "/api/autopilot/params", method="POST",
                         body={"confirm": True, "params": {name: before[name] for name in configure}})
    _, again = get_json(port, "/api/autopilot/params/read", method="POST")
    assert status == 200 and {p["name"]: p["current"] for p in again["params"]} == before


def test_snapshots_probe_and_focal_check_give_synthetic_results(server):
    import cv2
    port, root, _ = server
    status, headers, png = request(port, "GET", "/api/snapshot?stream=rect_left&overlay=1")
    assert status == 200 and headers["x-overlay"] == "features"
    assert sum(int(headers[f"x-overlay-{kind}"]) for kind in ("inliers", "outliers", "unmatched")) == 300
    assert cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED).shape == (400, 640, 3)
    # Without an overlay: the (mock) camera's JPEG as it came over the link
    status, headers, jpeg = request(port, "GET", "/api/snapshot?stream=rect_left")
    assert status == 200 and headers["content-type"] == "image/jpeg" and headers["x-snapshot-encoded-on"] == "camera"
    assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_UNCHANGED).shape == (400, 640)
    _, s = get_json(port, "/api/status")
    shots = {item["name"]: item for item in s["image_streams"]["streams"]}
    assert shots["snapshot_rect_left"]["encoded_on"] == "camera" and shots["snapshot_rect_left"]["last_bytes"] > 0
    assert shots["snapshot_depth"]["encoded_on"] == "pi" and s["image_streams"]["fallbacks"] == []
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
