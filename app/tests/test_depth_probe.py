import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_probe  # noqa: E402
import vio_log  # noqa: E402
from depth_probe import DepthProbeService, ProbeRequest, compute_result, make_request, roi_bounds  # noqa: E402
from pnp_odometry import Intrinsics  # noqa: E402

SIZE = (640, 400)
K = Intrinsics(fx=163.1, fy=237.1, cx=329.4, cy=199.0)

# A wall at 1.5 m with holes: zeros, a patch nearer than 300 mm and one beyond 4000 mm (both invalid)
PLANE = np.full((400, 640), 1500, dtype=np.uint16)
PLANE[195:205, 310:320] = 0
PLANE[180:185, 300:340] = 200
PLANE[215:220, 300:340] = 4500


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def make_service(tmp_path, clock=None, wall=None, **kwargs):
    return DepthProbeService(tmp_path, (300, 4000), kwargs.pop("depth_scale", 1.0), SIZE,
                             clock=clock or FakeClock(), wall_clock=wall or time.time, run=lambda fn: fn(),
                             log=lambda m: None, **kwargs)


def read_result(tmp_path, request_id):
    return depth_probe.wait_for_probe(request_id, 0, tmp_path)


# --- request validation -------------------------------------------------------------------------

def test_make_request_defaults_to_the_image_centre():
    request, clamped = make_request(image_size=SIZE)
    assert request == ProbeRequest(320.0, 200.0, 41, 41, 10) and clamped == []


def test_make_request_clamps_sizes_and_frames():
    request, clamped = make_request(10, 20, 500, 201, 80, SIZE)
    assert (request.w, request.h, request.frames) == (200, 200, 50)
    assert clamped == ["w 500 -> 200", "h 201 -> 200", "frames 80 -> 50"]


@pytest.mark.parametrize("args", [(640, 200), (-1, 200), (320, 400), (float("nan"), 1), (320, 200, 0),
                                  (320, 200, 41, -3), (320, 200, 41, 41, 0)])
def test_make_request_rejects(args):
    with pytest.raises(ValueError):
        make_request(*args, image_size=SIZE)


def test_roi_bounds_are_centred_and_clipped():
    assert roi_bounds(ProbeRequest(320, 200, 41, 41, 1), (400, 640), SIZE) == (300, 180, 341, 221)
    assert roi_bounds(ProbeRequest(321, 200, 4, 4, 1), (400, 640), SIZE) == (319, 198, 323, 202)
    assert roi_bounds(ProbeRequest(5, 395, 41, 41, 1), (400, 640), SIZE) == (0, 375, 26, 400)
    # Depth decimated by 2: the same ROI on the half-size grid
    assert roi_bounds(ProbeRequest(320, 200, 40, 40, 1), (200, 320), SIZE) == (150, 90, 170, 110)


# --- statistics ---------------------------------------------------------------------------------

def test_roi_stats_on_a_plane_with_invalid_pixels():
    request = ProbeRequest(320, 200, 41, 41, 3)
    x0, y0, x1, y1 = roi_bounds(request, PLANE.shape, SIZE)
    rois = [PLANE[y0:y1, x0:x1]] * 3
    result = compute_result(request, rois, (300, 4000), 1.0, K, "rectified", [1.0, 1.1, 1.2], [7, 8, 9])
    invalid = 10 * 10 + 5 * 40 * 2
    assert result["n_px"] == 3 * 41 * 41 and result["n_valid_px"] == 3 * (41 * 41 - invalid)
    assert result["valid_fraction"] == pytest.approx(1 - invalid / 41 ** 2)
    for key in ("median_mm", "p10_mm", "p90_mm", "mean_mm", "median_of_frame_medians_mm"):
        assert result[key] == pytest.approx(1500.0)
    assert result["raw_m"] == result["scaled_m"] == pytest.approx(1.5)
    assert result["frame_valid_fractions"] == [pytest.approx(result["valid_fraction"])] * 3
    assert result["device_timestamps"] == [1.0, 1.1, 1.2] and result["sequences"] == [7, 8, 9]
    assert result["valid_range_mm"] == [300, 4000] and "Z" in result["note"]
    assert result["summary"] == f"median 1.500 m (scaled 1.500 m), valid {100 * result['valid_fraction']:.0f}%"


def test_frame_medians_are_robust_to_one_bad_frame():
    request = ProbeRequest(320, 200, 10, 10, 5)
    rois = [np.full((10, 10), v, np.uint16) for v in (1000, 1010, 990, 3900, 1005)]
    rois.append(np.zeros((10, 10), np.uint16))  # no valid pixels: no median, still counted as a frame
    result = compute_result(request, rois)
    assert result["median_of_frame_medians_mm"] == pytest.approx(1005.0)
    assert result["frame_medians_mm"][-1] is None and result["frame_valid_fractions"][-1] == 0.0
    assert result["mean_mm"] == pytest.approx(np.mean([1000, 1010, 990, 3900, 1005]))


def test_no_valid_depth():
    result = compute_result(ProbeRequest(320, 200, 5, 5, 1), [np.zeros((5, 5), np.uint16)], intrinsics=K)
    assert result["raw_m"] is None and result["median_mm"] is None and result["range_m"]["raw"] is None
    assert result["valid_fraction"] == 0.0 and result["summary"].startswith("no valid depth")


def test_range_and_point_use_fx_and_fy_separately():
    u, v, z = 500.0, 50.0, 2.0
    request = ProbeRequest(u, v, 3, 3, 1)
    result = compute_result(request, [np.full((3, 3), 2000, np.uint16)], (300, 4000), 1.0, K, "rectified")
    X, Y = (u - K.cx) / K.fx * z, (v - K.cy) / K.fy * z
    np.testing.assert_allclose(result["point_m"]["raw"], [X, Y, z])
    assert result["range_m"]["raw"] == pytest.approx(z * math.sqrt(1 + ((u - K.cx) / K.fx) ** 2 + ((v - K.cy) / K.fy) ** 2))
    # A single focal length (fx for both) would be clearly different here
    assert abs(result["range_m"]["raw"] - z * math.hypot(1, math.hypot((u - K.cx) / K.fx, (v - K.cy) / K.fx))) > 0.1
    # At the principal point range == Z
    centre = compute_result(ProbeRequest(K.cx, K.cy, 3, 3, 1), [np.full((3, 3), 2000, np.uint16)], intrinsics=K)
    assert centre["range_m"]["raw"] == pytest.approx(2.0)


def test_depth_scale_only_changes_the_scaled_outputs():
    request = ProbeRequest(500, 50, 3, 3, 1)
    rois = [np.full((3, 3), 2000, np.uint16)]
    plain = compute_result(request, rois, (300, 4000), 1.0, K)
    scaled = compute_result(request, rois, (300, 4000), 1.25, K)
    for key in ("median_mm", "p10_mm", "p90_mm", "mean_mm", "median_of_frame_medians_mm", "raw_m", "valid_fraction"):
        assert scaled[key] == plain[key]
    assert scaled["range_m"]["raw"] == plain["range_m"]["raw"]
    assert scaled["point_m"]["raw"] == plain["point_m"]["raw"]
    assert scaled["scaled_m"] == pytest.approx(1.25 * plain["scaled_m"])
    assert scaled["range_m"]["scaled"] == pytest.approx(1.25 * plain["range_m"]["raw"])
    np.testing.assert_allclose(scaled["point_m"]["scaled"], 1.25 * np.array(plain["point_m"]["raw"]))
    assert scaled["depth_scale"] == 1.25 and "scaled 2.500 m" in scaled["summary"]


def test_refractive_index_scales_z_and_fx_fy_but_not_the_device_numbers():
    request = ProbeRequest(500, 50, 3, 3, 1)
    rois = [np.full((3, 3), 2000, np.uint16)]
    air = compute_result(request, rois, (300, 4000), 1.0, K)
    water = compute_result(request, rois, (300, 4000), 1.1, K, refractive_index=1.333)
    for key in ("median_mm", "median_of_frame_medians_mm", "raw_m", "valid_fraction"):
        assert water[key] == air[key]
    assert water["point_m"]["raw"] == air["point_m"]["raw"] and water["range_m"]["raw"] == air["range_m"]["raw"]
    assert water["refractive_index"] == 1.333 and water["depth_scale"] == 1.1
    assert water["total_scale"] == pytest.approx(1.333 * 1.1)
    assert water["scaled_m"] == pytest.approx(2.0 * 1.333 * 1.1)
    # Z x n x scale with fx, fy x n: X and Y change by the depth scale only, Z by both
    X, Y, Z = air["point_m"]["raw"]
    np.testing.assert_allclose(water["point_m"]["scaled"], [1.1 * X, 1.1 * Y, 1.333 * 1.1 * Z])
    assert ", n 1.333)" in water["summary"] and ", n " not in air["summary"]


# --- request / response mechanics ---------------------------------------------------------------

def test_probe_is_served_over_the_next_frames(tmp_path):
    clock = FakeClock()
    service = make_service(tmp_path, clock, depth_scale=1.1)
    service.set_intrinsics(K, "rectified")
    request, _ = make_request(320, 200, 41, 41, 4, SIZE)
    request_id = depth_probe.request_probe(request, tmp_path)
    assert service.on_depth(PLANE) is False  # nothing pending yet: no work at all
    service.tick()
    assert service.active is not None
    for n in range(4):
        assert service.on_depth(PLANE, device_timestamp=10.0 + n, sequence=n)
    assert service.active is None and service.served == 1
    result = read_result(tmp_path, request_id)
    assert result["id"] == request_id and result["frames"] == 4
    assert result["median_mm"] == pytest.approx(1500.0) and result["scaled_m"] == pytest.approx(1.65)
    assert result["intrinsics"] == {"fx": 163.1, "fy": 237.1, "cx": 329.4, "cy": 199.0}
    assert result["intrinsics_source"] == "rectified"
    assert result["roi_depth_px"] == {"x0": 300, "y0": 180, "x1": 341, "y1": 221}
    assert result["device_timestamps"] == [10.0, 11.0, 12.0, 13.0]
    # Collected: the result file is gone, the request directory is empty
    assert not list((tmp_path / "results").iterdir()) and not list((tmp_path / "requests").iterdir())


def test_requests_are_served_one_at_a_time_oldest_first(tmp_path):
    clock = FakeClock()
    service = make_service(tmp_path, clock)
    first = depth_probe.request_probe(make_request(100, 100, 5, 5, 2, SIZE)[0], tmp_path)
    second = depth_probe.request_probe(make_request(200, 100, 5, 5, 1, SIZE)[0], tmp_path)
    os.utime(tmp_path / "requests" / f"{first}.json", (1.0, 1.0))
    os.utime(tmp_path / "requests" / f"{second}.json", (2.0, 2.0))
    service.tick()
    assert service.active["id"] == first
    clock.now = 1.0
    service.tick()  # busy: the second request waits
    assert service.active["id"] == first
    service.on_depth(PLANE)
    service.on_depth(PLANE)
    service.tick()
    assert service.active["id"] == second
    service.on_depth(PLANE)
    assert read_result(tmp_path, first)["roi"]["x"] == 100
    assert read_result(tmp_path, second)["roi"]["x"] == 200


def test_tick_polls_at_most_every_poll_interval(tmp_path, monkeypatch):
    clock = FakeClock()
    service = make_service(tmp_path, clock)
    scans = []
    real_scandir = os.scandir
    monkeypatch.setattr(depth_probe.os, "scandir", lambda path: scans.append(path) or real_scandir(path))
    (tmp_path / "requests").mkdir()
    for i in range(10):
        clock.now = i * 0.05
        service.tick()
    assert len(scans) == 3


def test_stale_and_garbage_requests_are_dropped(tmp_path):
    service = make_service(tmp_path, wall=lambda: 1000.0)
    old = depth_probe.request_probe(make_request(image_size=SIZE)[0], tmp_path, clock=lambda: 900.0)
    (tmp_path / "requests" / ("0" * 32 + ".json")).write_text("not json")
    (tmp_path / "requests" / "passwd").write_text("{}")
    service.tick()
    assert service.active is None
    assert not (tmp_path / "requests" / f"{old}.json").exists()
    assert not (tmp_path / "results").exists()
    assert (tmp_path / "requests" / "passwd").exists()  # not a request name: never touched


def test_invalid_request_is_answered_with_an_error(tmp_path):
    service = make_service(tmp_path)
    request_id = depth_probe.request_probe(ProbeRequest(900, 200, 41, 41, 10), tmp_path)
    service.tick()
    assert service.active is None
    with pytest.raises(depth_probe.ProbeError, match="inside the 640x400"):
        read_result(tmp_path, request_id)


def test_wait_times_out_and_withdraws_an_unclaimed_request(tmp_path):
    request_id = depth_probe.request_probe(make_request(image_size=SIZE)[0], tmp_path)
    times = iter([0.0, 1.0, 2.5])
    with pytest.raises(depth_probe.ProbeTimeout, match="did not pick up"):
        depth_probe.wait_for_probe(request_id, 2.0, tmp_path, clock=lambda: next(times), sleep=lambda s: None)
    assert not (tmp_path / "requests" / f"{request_id}.json").exists()


def test_wait_times_out_when_the_probe_was_taken_but_never_finished(tmp_path):
    service = make_service(tmp_path)
    request_id = depth_probe.request_probe(make_request(image_size=SIZE)[0], tmp_path)
    service.tick()  # claimed, but no depth frames follow
    with pytest.raises(depth_probe.ProbeTimeout, match="did not finish"):
        depth_probe.wait_for_probe(request_id, 0, tmp_path)


def test_failing_statistics_answer_with_an_error(tmp_path, monkeypatch):
    service = make_service(tmp_path)
    request_id = depth_probe.request_probe(make_request(320, 200, 5, 5, 1, SIZE)[0], tmp_path)
    monkeypatch.setattr(depth_probe, "compute_result", lambda *a, **k: 1 / 0)
    service.tick()
    service.on_depth(PLANE)
    with pytest.raises(depth_probe.ProbeError, match="probe failed"):
        read_result(tmp_path, request_id)


def test_decimated_depth_uses_the_scaled_roi(tmp_path):
    service = make_service(tmp_path)
    request_id = depth_probe.request_probe(make_request(320, 200, 40, 40, 1, SIZE)[0], tmp_path)
    service.tick()
    service.on_depth(PLANE[::2, ::2])
    result = read_result(tmp_path, request_id)
    assert result["roi_depth_px"] == {"x0": 150, "y0": 90, "x1": 170, "y1": 110}
    assert result["depth_shape"] == [320, 200] and result["n_px"] == 400


def test_per_frame_cost_is_negligible(tmp_path):
    """Micro-benchmark: the per-frame work of a 200 x 200 probe, and of an idle service, stays well under 2 ms."""
    service = DepthProbeService(tmp_path, run=lambda fn: None, log=lambda m: None)
    depth = np.random.default_rng(0).integers(300, 4000, (400, 640)).astype(np.uint16)
    depth_probe.request_probe(make_request(320, 200, 200, 200, 50, SIZE)[0], tmp_path)
    service.tick()
    start = time.perf_counter()
    for _ in range(50):
        service.on_depth(depth, 1.0, 1)
    active_ms = (time.perf_counter() - start) * 1000 / 50
    start = time.perf_counter()
    for _ in range(1000):
        service.tick()
        service.on_depth(depth)
    idle_ms = (time.perf_counter() - start) * 1000 / 1000
    assert service.served == 1
    assert active_ms < 2.0 and idle_ms < 0.5, (active_ms, idle_ms)


# --- overlay ------------------------------------------------------------------------------------

def test_overlay_draws_the_roi_on_a_gray_image():
    gray = np.full((400, 640), 100, np.uint8)
    png = cv2.imencode(".png", gray)[1].tobytes()
    out = cv2.imdecode(np.frombuffer(depth_probe.draw_probe_overlay(png, ProbeRequest(320, 200, 41, 41, 1), SIZE),
                                     np.uint8), cv2.IMREAD_UNCHANGED)
    assert out.shape == (400, 640, 3)
    magenta = (out[:, :, 0] == 255) & (out[:, :, 1] == 0) & (out[:, :, 2] == 255)
    assert magenta[180, 300:341].all() and magenta[220, 300:341].all()  # top and bottom of the box
    assert magenta[200, 320] and magenta[165, 320] and magenta[200, 285]  # crosshair centre and arms
    assert not magenta[10, 10]


def test_overlay_rejects_16_bit():
    png = cv2.imencode(".png", np.zeros((400, 640), np.uint16))[1].tobytes()
    with pytest.raises(ValueError):
        depth_probe.draw_probe_overlay(png, ProbeRequest(320, 200, 41, 41, 1), SIZE)


# --- web API ------------------------------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    monkeypatch.setattr(main, "start_vio", lambda: None)
    monkeypatch.setattr(main.supervisor, "status", lambda: {"running": True, "pid": 1, "restart_count": 0,
                                                            "last_exit_code": None, "restart_in": None})
    main.save_config({**main.DEFAULT_CONFIG, "algorithm": "pnp", "fps": 15,
                      "resolution_width": 640, "resolution_height": 400})
    return main, fastapi_testclient.TestClient(main.app)


@pytest.fixture
def fake_vio(tmp_path):
    """The VIO loop's probe handling, on a thread fed with PLANE frames."""
    service = DepthProbeService(tmp_path / "snapshots" / "depth_probe", (300, 4000), 1.2, SIZE, poll_interval=0.01,
                                log=lambda m: None)
    service.set_intrinsics(K, "rectified")
    stop = threading.Event()

    def loop():
        sequence = 0
        while not stop.is_set():
            service.tick()
            service.on_depth(PLANE, device_timestamp=sequence / 15, sequence=sequence)
            sequence += 1
            time.sleep(0.005)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield service
    stop.set()
    thread.join()


def test_api_depth_probe(client, fake_vio):
    main, http = client
    response = http.get("/api/depth_probe?x=320&y=200&w=41&h=41&frames=5")
    assert response.status_code == 200, response.text
    assert response.headers["cache-control"] == "no-store"
    data = response.json()
    assert data["frames"] == 5 and data["roi"] == {"x": 320.0, "y": 200.0, "w": 41, "h": 41}
    assert data["median_mm"] == pytest.approx(1500.0) and data["raw_m"] == pytest.approx(1.5)
    assert data["scaled_m"] == pytest.approx(1.8) and data["depth_scale"] == 1.2
    assert data["valid_fraction"] == pytest.approx(1 - 500 / 41 ** 2) and data["intrinsics_source"] == "rectified"
    assert data["summary"].startswith("median 1.500 m (scaled 1.800 m)")
    # Defaults: centre, 41 x 41, 10 frames
    default = http.get("/api/depth_probe").json()
    assert default["roi"] == {"x": 320.0, "y": 200.0, "w": 41, "h": 41} and default["frames"] == 10
    clamped = http.get("/api/depth_probe?w=500&frames=60").json()
    assert clamped["roi"]["w"] == 200 and clamped["frames"] == 50 and clamped["clamped"] == ["w 500 -> 200", "frames 60 -> 50"]


def test_api_depth_probe_validation(client):
    main, http = client
    assert http.get("/api/depth_probe?x=700").status_code == 400
    assert "inside the 640x400" in http.get("/api/depth_probe?y=-1").json()["detail"]
    assert http.get("/api/depth_probe?w=0").status_code == 400
    assert http.get("/api/depth_probe?w=abc").status_code == 422


def test_api_depth_probe_needs_pnp_running(client, monkeypatch):
    main, http = client
    monkeypatch.setattr(main.supervisor, "status", lambda: {"running": False})
    response = http.get("/api/depth_probe")
    assert response.status_code == 503 and "stopped" in response.json()["detail"]
    main.save_config({**main.DEFAULT_CONFIG, "algorithm": "basalt"})
    response = http.get("/api/depth_probe")
    assert response.status_code == 503 and "PnP algorithm" in response.json()["detail"]


def test_api_depth_probe_times_out_without_vio(client, monkeypatch, tmp_path):
    main, http = client
    monkeypatch.setattr(main, "PROBE_EXTRA_TIMEOUT", 0.1)
    response = http.get("/api/depth_probe?frames=1")
    assert response.status_code == 504 and "did not pick up" in response.json()["detail"]
    assert not list((tmp_path / "snapshots" / "depth_probe" / "requests").iterdir())


def test_index_page_has_the_depth_probe(client):
    main, http = client
    page = http.get("/").text
    assert "'api/depth_probe?" in page and "crosshair" in page and "'api/focal_check'" in page
    assert 'id="probe-frames"' in page
