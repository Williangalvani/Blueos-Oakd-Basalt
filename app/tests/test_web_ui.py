import json
import os
import re
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import config_schema  # noqa: E402
import depth_stream  # noqa: E402
import snapshots  # noqa: E402
import vio_log  # noqa: E402
import vio_telemetry  # noqa: E402
from test_snapshots import RECT, FakeDevice, decode, vio_tick  # noqa: E402

SVG_NAMESPACE = "http://www.w3.org/2000/svg"
STOPPED = {"running": False, "pid": None, "restart_count": 0, "last_exit_code": None, "restart_in": None}


@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    process = dict(STOPPED)
    started = []
    monkeypatch.setattr(main.supervisor, "status", lambda: dict(process))
    monkeypatch.setattr(main, "start_vio", lambda: started.append(True))
    http = fastapi_testclient.TestClient(main.app)
    http.process, http.started = process, started
    return main, http


def running(http, pid=None):
    http.process.update(running=True, pid=os.getpid() if pid is None else pid)


def live_telemetry(fps=10, seconds=0.0, now=None, **kwargs):
    """A Telemetry publishing into the (test) telemetry dir, with frames up to now."""
    now = time.time() if now is None else now
    telemetry = vio_telemetry.Telemetry(vio_telemetry.DEFAULT_DIR, fps=fps, log=lambda m: None, **kwargs)
    n = int(seconds * fps)
    for i in range(n + 1):
        t = now - (n - i) / fps
        ok = i % 5 != 1
        telemetry.record_frame(t, ok, "ok" if ok else "too_few_matches", 150, 120, 80, 40 if ok else 4, 30.0,
                               dt=1 / fps, position_delta=(0.02, -0.01, 0.0) if ok else None,
                               yaw_delta=0.001 if ok else None, sent=ok)
    telemetry.set_phase("running")
    return telemetry


# --- page ---------------------------------------------------------------------------------------

def external_urls(text):
    return {url for url in re.findall(r"""(?:https?:)?//[A-Za-z0-9.-]+\.[A-Za-z]{2,}[^\s"'<>)]*""", text)
            if url != SVG_NAMESPACE}


def test_page_is_one_self_contained_offline_page(client):
    main, http = client
    response = http.get("/")
    assert response.status_code == 200 and response.headers["content-type"].startswith("text/html")
    page = response.text
    assert "<style>" in page and "<script>" in page
    assert "<!--FIELDS" not in page and "<!--UI_" not in page
    assert not re.search(r"<script[^>]+src=|<link[^>]+href=|@import|url\(", page)
    assert external_urls(page) == set()
    for panel in ("panel-status", "panel-views", "panel-trajectory", "panel-velocity", "panel-vxvy",
                  "panel-inliers", "panel-settings", "panel-autopilot", "plot-dial", "plot-vbar", "ap-dialog",
                  "panel-depth-stream", "depth-stream-img", "depth-stream-canvas", "depth-legend", "ds-cost",
                  "ds-errors", "depth-stream-title", "depth-stream-overlay", "depth-stream-features",
                  "depth-legend-box", "feature-legend-box", "ds-source", "ds-features", "ds-n-inlier",
                  "ds-n-no_depth"):
        assert f'id="{panel}"' in page
    # The shared features overlay is inlined before ui.js, which uses it
    assert page.index("const FEATURE_STATES = ") < page.index("function drawStreamFeatures")
    assert "integrated, drifts" in page
    # Every configuration field is in the form
    for field in config_schema.CONFIG_FIELDS:
        assert f'name="{field.key}"' in page, field.key


@pytest.mark.parametrize("asset, kind", [("ui.css", "text/css"), ("ui.js", "javascript"), ("ui.html", "text/html"),
                                         ("features_overlay.js", "javascript")])
def test_static_assets_are_served_without_external_urls(client, asset, kind):
    main, http = client
    response = http.get(f"/static/{asset}")
    assert response.status_code == 200 and kind in response.headers["content-type"]
    assert external_urls(response.text) == set()


def test_page_polls_politely(client):
    main, http = client
    js = http.get("/static/ui.js").text
    assert "visibilityState" in js and "If-None-Match" in js
    assert "WebSocket" not in js
    # The only event stream: the tracked features, opened only while the live stream runs (startDepthStream)
    assert js.count("new EventSource(") == 1 and "new EventSource(FEATURES_URL)" in js
    guard = "if (depthStream.events || !$('depth-stream-features').checked || !depthStream.on || !visible()) return;"
    assert guard in js
    assert "setInterval(tick, 1000)" in js


# --- configuration --------------------------------------------------------------------------------

def test_config_schema_describes_every_field(client):
    main, http = client
    schema = http.get("/api/config/schema").json()
    fields = {field["name"]: field for field in schema["fields"]}
    assert set(fields) == set(main.DEFAULT_CONFIG)
    for field in fields.values():
        assert {"name", "type", "min", "max", "default", "help", "label", "group"} <= set(field)
        assert field["default"] == main.DEFAULT_CONFIG[field["name"]]
    assert (fields["min_inliers"]["type"], fields["min_inliers"]["min"], fields["min_inliers"]["max"]) == ("int", 6, 500)
    assert fields["fps"]["min"] == 1 and fields["fps"]["max"] == 60
    assert fields["dvl_source"]["choices"] == ["auto", "off"]
    assert fields["stereo_preset"]["type"] == "choice"
    assert {group["name"] for group in schema["groups"]} == {field["group"] for field in fields.values()}


def test_config_rejects_invalid_values_with_clear_errors(client, tmp_path):
    main, http = client
    response = http.post("/api/config", json={"fps": 0, "refractive_index": "abc", "vehicle_ip": "bad host",
                                              "min_inliers": 2.5, "dvl_source": "maybe"})
    assert response.status_code == 422
    errors = {error["field"]: error["msg"] for error in response.json()["detail"]}
    assert set(errors) == {"fps", "refractive_index", "vehicle_ip", "min_inliers", "dvl_source"}
    assert errors["fps"] == "FPS (fps): must be between 1 and 60 (got 0)"
    assert "must be a number" in errors["refractive_index"]
    assert "whole number" in errors["min_inliers"]
    assert "FPS (fps)" in response.json()["message"]
    assert not (tmp_path / "config.json").exists() and not http.started


def test_config_cross_field_and_range_checks(client):
    main, http = client
    response = http.post("/api/config", json={"stereo_depth_min_mm": 3000, "stereo_depth_max_mm": 2000})
    assert response.status_code == 422
    assert [error["field"] for error in response.json()["detail"]] == ["stereo_depth_max_mm"]
    response = http.post("/api/config", json={"stereo_confidence_threshold": 300})
    assert response.status_code == 422 and "between 0 and 255" in response.json()["detail"][0]["msg"]
    assert http.post("/api/config", json=[1, 2]).status_code == 422


def test_valid_config_is_saved_and_restarts_vio(client, tmp_path):
    main, http = client
    response = http.post("/api/config", json={**main.DEFAULT_CONFIG, "algorithm": "pnp", "min_inliers": 25,
                                              "dvl_source": "off", "lever_arm_m": "0.1, 0, 0.2"})
    assert response.status_code == 200 and http.started == [True]
    saved = json.loads((tmp_path / "config.json").read_text())
    assert saved["min_inliers"] == 25 and saved["dvl_source"] == "off" and saved["lever_arm_m"] == [0.1, 0.0, 0.2]
    assert http.get("/api/config").json()["min_inliers"] == 25
    command = main.build_vio_command(saved)
    assert command[command.index("--min-inliers") + 1] == "25"
    assert command[command.index("--telemetry-dir") + 1] == str(vio_telemetry.DEFAULT_DIR)


def test_log_only_toggle_persists_and_restarts(client, tmp_path):
    main, http = client
    response = http.post("/api/log_only", json={"log_only": True})
    assert response.status_code == 200 and response.json()["log_only"] is True
    assert json.loads((tmp_path / "config.json").read_text())["log_only"] is True and http.started == [True]
    assert http.post("/api/log_only", json={"log_only": "yes"}).status_code == 400
    assert http.post("/api/log_only", json={}).status_code == 400
    assert http.post("/api/log_only", json={"log_only": False}).json()["log_only"] is False


# --- /api/status ----------------------------------------------------------------------------------

def test_status_without_vio(client):
    main, http = client
    status = http.get("/api/status").json()
    assert status["running"] is False and status["telemetry"] is None
    assert status["camera_connected"] is None and status["chip_temperature_c"] is None
    assert status["algorithm"] == "basalt" and status["uptime_s"] is None
    assert "cpu_percent" in status and status["vehicle"]["armed"] is None
    # The page's request asks the VIO process to publish
    assert (vio_telemetry.DEFAULT_DIR / vio_telemetry.STATUS_LEASE).exists()


def test_status_with_live_telemetry(client):
    main, http = client
    running(http)
    telemetry = live_telemetry(seconds=12.0, temperature=lambda: 51.25, output_error=lambda: None)
    telemetry.sample_temperature()
    telemetry.write_status()
    status = http.get("/api/status").json()
    t = status["telemetry"]
    assert t["stale"] is False and t["phase"] == "running"
    assert status["camera_connected"] is True and status["chip_temperature_c"] == 51.25
    assert t["recent"]["fps"] == pytest.approx(10.0, rel=0.1)
    assert t["recent"]["ok_percent"] == pytest.approx(80.0, abs=2.0)
    assert t["last_frame"]["inliers"] == 40 and t["last_frame"]["features"] == 150
    assert t["failures"]["by_reason"][0]["reason"] == "too_few_matches"
    assert t["output"]["mode"] == "sending" and t["output"]["rate_hz"] == pytest.approx(8.0, rel=0.15)
    assert status["uptime_s"] is not None and status["uptime_s"] >= 0


def test_status_marks_telemetry_of_a_stopped_or_other_process_stale(client):
    main, http = client
    live_telemetry(seconds=1.0).write_status()
    status = http.get("/api/status").json()
    assert status["telemetry"]["stale"] is True and status["camera_connected"] is None
    running(http, pid=os.getpid() + 1)
    assert http.get("/api/status").json()["telemetry"]["stale"] is True


def test_status_reports_camera_lost(client):
    main, http = client
    running(http)
    telemetry = live_telemetry(seconds=1.0, now=time.time() - 10)
    telemetry.write_status(time.time())
    assert http.get("/api/status").json()["camera_connected"] is False


# --- /api/history ---------------------------------------------------------------------------------

def test_history_empty(client):
    main, http = client
    history = http.get("/api/history").json()
    assert history["points"] == 0 and history["vio_running"] is False and history["available_seconds"] == 0
    assert all(values == [] for values in history["columns"].values())
    assert set(history["columns"]) == {"t", "vx", "vy", "vz", "x", "y", "yaw", "inliers", "ok"}
    assert history["dvl"]["available"] is False and history["dvl"]["hint"].startswith("DVL not found")
    assert history["min_inliers"] == 15
    assert (vio_telemetry.DEFAULT_DIR / vio_telemetry.HISTORY_LEASE).exists()


def test_history_shape_downsampling_and_tail(client):
    main, http = client
    running(http)
    live_telemetry(fps=20, seconds=120.0).write_history()
    history = http.get("/api/history?seconds=60&max_points=100&tail=2").json()
    columns = history["columns"]
    # ~1200 frames in the last 60 s at 20 fps: every 12th (13th if one more frame fell inside the window)
    assert history["points"] == len(columns["t"]) <= 100 and history["stride"] in (12, 13)
    assert all(len(values) == history["points"] for values in columns.values())
    assert -60.5 <= columns["t"][0] and -0.5 <= columns["t"][-1] <= 0.0
    assert columns["t"] == sorted(columns["t"])
    assert history["available_seconds"] == pytest.approx(120.0, abs=1.0)
    # Rejected frames have no velocity
    for ok, vx in zip(columns["ok"], columns["vx"]):
        assert (vx is None) == (ok == 0)
        if vx is not None:
            assert vx == pytest.approx(0.02 * 20)  # 2 cm per frame at 20 fps
    assert len(history["tail"]["t"]) == pytest.approx(40, abs=2)
    assert min(history["tail"]["t"]) >= -2.05
    full = http.get("/api/history?seconds=600").json()
    assert full["points"] <= main.MAX_HISTORY_POINTS


@pytest.mark.parametrize("query", ["seconds=601", "seconds=0", "max_points=601", "max_points=5", "tail=11"])
def test_history_caps(client, query):
    main, http = client
    assert http.get(f"/api/history?{query}").status_code == 422


def test_history_etag(client):
    main, http = client
    telemetry = live_telemetry(seconds=5.0)
    telemetry.write_history()
    first = http.get("/api/history?seconds=30")
    etag = first.headers["etag"]
    again = http.get("/api/history?seconds=30", headers={"If-None-Match": etag})
    assert again.status_code == 304 and again.content == b""
    assert http.get("/api/history?seconds=60", headers={"If-None-Match": etag}).status_code == 200
    time.sleep(0.01)
    telemetry.record_frame(time.time(), True, "ok", 1, 1, 1, 30, 1.0)
    telemetry.write_history()
    path = vio_telemetry.DEFAULT_DIR / vio_telemetry.HISTORY_FILE
    os.utime(path, (time.time() + 1, time.time() + 1))
    assert http.get("/api/history?seconds=30", headers={"If-None-Match": etag}).status_code == 200


# --- live depth stream ----------------------------------------------------------------------------

def save_config(tmp_path, **values):
    import main
    (tmp_path / "config.json").write_text(json.dumps({**main.DEFAULT_CONFIG, **values}))


def test_depth_stream_needs_the_pnp_vio_running(client, tmp_path):
    main, http = client
    response = http.get("/api/stream/depth")
    assert response.status_code == 503 and "PnP" in response.json()["detail"]
    save_config(tmp_path, algorithm="pnp")
    assert "stopped" in http.get("/api/stream/depth").json()["detail"]
    running(http)
    save_config(tmp_path, algorithm="pnp", depth_stream=False)
    response = http.get("/api/stream/depth")
    assert response.status_code == 503 and "off in the settings" in response.json()["detail"]
    assert not (vio_telemetry.DEFAULT_DIR / depth_stream.LEASE_FILE).exists()


def jpeg(value):
    import cv2
    ok, data = cv2.imencode(".jpg", np.full((40, 64), value, np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 70])
    assert ok
    return data.tobytes()


@pytest.fixture
def streaming_vio(tmp_path):
    """A VIO stand-in: the real DepthStreamService, with a fake camera that encodes frames while enabled."""
    directory = vio_telemetry.DEFAULT_DIR
    controls = []
    service = depth_stream.DepthStreamService(directory, controls.append, {"fps_target": 20.0, "quality": 70,
                                                                           "depth_range_mm": [300, 4000]},
                                              poll_interval=0.01, meta_interval=0.02, log=lambda m: None)
    stop = threading.Event()
    depth = np.full((400, 640), 1234, np.uint16)

    def loop():
        n = 0
        while not stop.is_set():
            service.tick(depth)
            if service.enabled:
                service.on_frame(jpeg(10 * (n % 20)), sequence=n)
                n += 1
            time.sleep(0.05)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield service, controls
    stop.set()
    thread.join()


def multipart_frames(body, boundary=depth_stream.BOUNDARY):
    frames = []
    for part in body.split(f"--{boundary}\r\n".encode())[1:]:
        headers, _, rest = part.partition(b"\r\n\r\n")
        length = int(re.search(rb"Content-Length: (\d+)", headers).group(1))
        assert b"Content-Type: image/jpeg" in headers and rest[length:] == b"\r\n"
        frames.append(rest[:length])
    return frames


def test_depth_stream_forwards_the_camera_jpegs(client, tmp_path, streaming_vio):
    import cv2
    main, http = client
    service, controls = streaming_vio
    save_config(tmp_path, algorithm="pnp")
    running(http)
    assert not service.enabled
    response = http.get("/api/stream/depth?frames=3")
    assert response.status_code == 200
    assert response.headers["content-type"] == "multipart/x-mixed-replace; boundary=depthframe"
    assert response.headers["cache-control"] == "no-store"
    frames = multipart_frames(response.content)
    assert len(frames) == 3 and len(set(frames)) == 3
    # The bytes the camera encoded, unchanged
    for frame in frames:
        image = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_UNCHANGED)
        assert image.shape[:2] == (40, 64)
    assert True in controls

    deadline = time.time() + 3
    while http.get("/api/status").json()["depth_stream"].get("frames_total", 0) < 3 and time.time() < deadline:
        time.sleep(0.02)
    status = http.get("/api/status").json()["depth_stream"]
    assert status["enabled"] is True and status["streaming"] is True and status["stale"] is False
    assert status["fps_target"] == 20.0 and status["depth_range_mm"] == [300, 4000]
    assert status["center"]["median_mm"] == 1234.0 and status["center"]["pixel_mm"] == 1234
    assert status["frames_total"] >= 3 and status["kbps"] > 0

    # Nobody watching: the lease runs out and the camera is told to stop
    lease = vio_telemetry.DEFAULT_DIR / depth_stream.LEASE_FILE
    old = time.time() - depth_stream.LEASE_S - 1
    os.utime(lease, (old, old))
    deadline = time.time() + 3
    while service.enabled and time.time() < deadline:
        time.sleep(0.02)
    assert not service.enabled and controls[-1] is False


def test_status_depth_stream_without_vio(client, tmp_path):
    main, http = client
    status = http.get("/api/status").json()["depth_stream"]
    assert status == {"enabled": False, "streaming": False, "stale": True, "age_s": None, "errors": 0,
                      "last_error": None, "encoder_stalled": False, "variant": None, "encoded_on": None,
                      "fallback": None, "device_log": None, "configured_encoding": "auto",
                      "debug_configured": False}
    save_config(tmp_path, algorithm="pnp")
    assert http.get("/api/status").json()["depth_stream"]["enabled"] is True


def test_status_shows_the_camera_scripts_errors(client, tmp_path, streaming_vio):
    main, http = client
    service, _ = streaming_vio
    save_config(tmp_path, algorithm="pnp")
    running(http)
    status = http.get("/api/status").json()["depth_stream"]
    assert status["errors"] == 0 and status["last_error"] is None
    text = "convert: TypeError: __init__(): incompatible constructor arguments"
    service.on_device_status(f"error\t12\t0\t{text}".encode())
    status = http.get("/api/status").json()["depth_stream"]
    assert status["errors"] == 12 and status["last_error"]["text"] == text
    assert status["last_error"]["fatal"] is False and abs(status["last_error"]["unix_time"] - time.time()) < 5
    service.on_device_status(b"stats\t12\t150\t41.5 60.2 640 400")
    status = http.get("/api/status").json()["depth_stream"]
    assert status["device_stats"]["convert_ms_mean"] == 41.5 and status["last_error"]["text"] == text


def test_depth_stream_settings_reach_the_vio_command(client):
    main, http = client
    pnp = {**main.DEFAULT_CONFIG, "algorithm": "pnp"}
    command = main.build_vio_command(pnp)
    assert "--depth-stream" in command
    assert command[command.index("--depth-stream-fps") + 1] == "5.0"
    assert command[command.index("--depth-stream-quality") + 1] == "70"
    command = main.build_vio_command({**pnp, "depth_stream": False, "depth_stream_fps": 2.5,
                                      "depth_stream_quality": 50})
    assert "--no-depth-stream" in command and command[command.index("--depth-stream-fps") + 1] == "2.5"
    # A hand-edited config.json can't make the VIO exit on its arguments
    command = main.build_vio_command({**pnp, "depth_stream_fps": 1000, "depth_stream_quality": "bad"})
    assert command[command.index("--depth-stream-fps") + 1] == "15.0"
    assert command[command.index("--depth-stream-quality") + 1] == "70"
    response = http.post("/api/config", json={**pnp, "depth_stream_fps": 0.1})
    assert response.status_code == 422 and response.json()["detail"][0]["field"] == "depth_stream_fps"


def test_depth_stream_encoding_and_debug_reach_the_vio_command(client, tmp_path):
    import depth_stream
    main, http = client
    assert config_schema.DEPTH_STREAM_ENCODINGS == depth_stream.VARIANTS
    pnp = {**main.DEFAULT_CONFIG, "algorithm": "pnp"}
    assert (pnp["depth_stream_encoding"], pnp["depth_stream_debug"]) == ("auto", False)
    command = main.build_vio_command(pnp)
    assert command[command.index("--depth-stream-encoding") + 1] == "auto" and "--no-depth-stream-debug" in command
    for variant in depth_stream.VARIANTS:
        command = main.build_vio_command({**pnp, "depth_stream_encoding": variant, "depth_stream_debug": True})
        assert command[command.index("--depth-stream-encoding") + 1] == variant and "--depth-stream-debug" in command
    # A hand-edited config.json falls back to the default
    command = main.build_vio_command({**pnp, "depth_stream_encoding": "h264"})
    assert command[command.index("--depth-stream-encoding") + 1] == "auto"
    response = http.post("/api/config", json={**pnp, "depth_stream_encoding": "h264"})
    assert response.status_code == 422 and response.json()["detail"][0]["field"] == "depth_stream_encoding"
    response = http.post("/api/config", json={**pnp, "depth_stream_encoding": "host_jpeg", "depth_stream_debug": True})
    assert response.status_code == 200
    status = http.get("/api/status").json()["depth_stream"]
    assert status["configured_encoding"] == "host_jpeg" and status["debug_configured"] is True
    fields = {f["name"]: f for f in http.get("/api/config/schema").json()["fields"]}
    assert fields["depth_stream_encoding"]["choices"] == list(depth_stream.VARIANTS)
    assert fields["depth_stream_debug"]["type"] == "bool"


# --- snapshot feature overlay ---------------------------------------------------------------------

OVERLAY_POINTS = dict(ids=[1, 2, 3, 4], uv=[[100, 100], [200, 150], [300, 200], [400, 250]], matched_index=[0, 1, 2],
                      inlier_index=[0, 2], previous_ids=[1, 2, 3], previous_uv=[[90, 95], [190, 150], [300, 190]])


def test_build_overlay_classifies_features():
    overlay = snapshots.build_overlay(**OVERLAY_POINTS, sequence=7, image_size=(640, 400), ok=True, reason="ok")
    assert [p[2] for p in overlay["points"]] == ["inlier", "outlier", "inlier", "unmatched"]
    assert overlay["points"][0] == [100.0, 100.0, "inlier", 90.0, 95.0]
    assert overlay["points"][3][3:] == [None, None]
    assert (overlay["inliers"], overlay["outliers"], overlay["unmatched"]) == (2, 1, 1)
    json.dumps(overlay)


def drawn_near(image, u, v, status, radius=6):
    """Some pixel around (u, v) close to the status colour (circles are anti-aliased)."""
    patch = image[v - radius:v + radius + 1, u - radius:u + radius + 1].astype(int)
    color = np.array(snapshots.OVERLAY_COLORS[status])
    return bool((np.abs(patch - color).max(axis=2) <= 30).any())


def test_draw_feature_overlay_colors(tmp_path):
    overlay = snapshots.build_overlay(**OVERLAY_POINTS, image_size=(640, 400))
    png = snapshots.encode_png(np.zeros((400, 640), np.uint8))
    image = decode(snapshots.draw_feature_overlay(png, overlay))
    assert image.shape == (400, 640, 3)
    assert drawn_near(image, 100, 100, "inlier") and not drawn_near(image, 100, 100, "outlier")
    assert drawn_near(image, 200, 150, "outlier") and not drawn_near(image, 200, 150, "inlier")
    assert drawn_near(image, 400, 250, "unmatched")
    assert not image[20:60, 20:60].any()
    # Half-size image: points scale with it
    small = decode(snapshots.draw_feature_overlay(snapshots.encode_png(np.zeros((200, 320), np.uint8)), overlay))
    assert drawn_near(small, 50, 50, "inlier") and not drawn_near(small, 100, 100, "inlier", radius=3)


@pytest.fixture
def fake_vio(tmp_path):
    """A VIO stand-in serving snapshots; rect_left carries the overlay of its last processed frame."""
    snapshot_dir = tmp_path / "snapshots"
    device = FakeDevice()
    writer = snapshots.SnapshotWriter(snapshot_dir, log=lambda m: None)
    builds = []

    def overlay():
        builds.append(True)
        return lambda: snapshots.build_overlay(**OVERLAY_POINTS, sequence=41, image_size=(640, 400))

    service = snapshots.SnapshotService(("rect_left", "rect_right", "depth"), device.send_request, writer,
                                        snapshot_dir, min_interval=0.0, poll_interval=0.01, log=lambda m: None,
                                        overlay=overlay)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            vio_tick(service, device)
            time.sleep(0.01)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield builds
    stop.set()
    thread.join()
    writer.close()


def test_api_snapshot_overlay(client, fake_vio):
    main, http = client
    plain = http.get("/api/snapshot?stream=rect_left")
    assert plain.status_code == 200 and "x-overlay" not in plain.headers
    assert np.array_equal(decode(plain.content), RECT)
    response = http.get("/api/snapshot?stream=rect_left&overlay=1")
    assert response.status_code == 200
    assert response.headers["x-overlay"] == "features"
    assert (response.headers["x-overlay-inliers"], response.headers["x-overlay-outliers"],
            response.headers["x-overlay-unmatched"], response.headers["x-overlay-frame-sequence"]) == ("2", "1", "1", "41")
    image = decode(response.content)
    assert image.shape == (400, 640, 3) and drawn_near(image, 100, 100, "inlier")
    # Built only for rect_left snapshots that were asked for
    http.get("/api/snapshot?stream=depth")
    assert len(fake_vio) == 2
    assert http.get("/api/snapshot?stream=depth&overlay=1").status_code == 400


def test_api_snapshot_overlay_unavailable(client, tmp_path):
    main, http = client
    snapshot_dir = tmp_path / "snapshots"
    device = FakeDevice()
    writer = snapshots.SnapshotWriter(snapshot_dir, log=lambda m: None)
    service = snapshots.SnapshotService(("rect_left",), device.send_request, writer, snapshot_dir, min_interval=0.0,
                                        poll_interval=0.0, log=lambda m: None, overlay=lambda: None)
    result = {}
    thread = threading.Thread(target=lambda: result.update(r=http.get("/api/snapshot?stream=rect_left&overlay=1")))
    thread.start()
    deadline = time.time() + 3
    while thread.is_alive() and time.time() < deadline:
        vio_tick(service, device)
        time.sleep(0.01)
    thread.join()
    writer.close()
    assert result["r"].status_code == 200 and result["r"].headers["x-overlay"] == "unavailable"
