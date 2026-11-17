import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import snapshots  # noqa: E402
import vio_log  # noqa: E402

DEPTH = np.zeros((400, 640), dtype=np.uint16)
DEPTH[100:300, 200:400] = 1500
DEPTH[0:50, :] = 3900
RECT = (np.arange(400 * 640) % 251).astype(np.uint8).reshape(400, 640)
RGB = np.dstack([RECT, RECT // 2, 255 - RECT])

FRAMES = {"depth": DEPTH, "rect_left": RECT, "rect_right": RECT[:, ::-1].copy(), "rgb": RGB}


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


class SyncWriter:
    """SnapshotWriter stand-in that writes on the caller's thread."""

    def __init__(self, snapshot_dir):
        self.snapshot_dir = snapshot_dir

    def submit(self, stream, load, meta):
        snapshots.write_snapshot(self.snapshot_dir, stream, load(), meta)
        return True

    def submit_error(self, stream, requested_at, error):
        snapshots.write_error(self.snapshot_dir, stream, requested_at, error)
        return True


class FakeDevice:
    """Plays the device side: a request for a stream yields one frame of it on the snapshot queue."""

    def __init__(self):
        self.requests = []
        self.queue = []

    def send_request(self, stream):
        self.requests.append(stream)
        self.queue.append(FRAMES[stream])

    def try_get(self):
        return self.queue.pop(0) if self.queue else None


def make_service(tmp_path, clock, device, available=snapshots.STREAMS, unavailable=None, writer=None):
    return snapshots.SnapshotService(available, device.send_request, writer or SyncWriter(tmp_path), tmp_path,
                                     unavailable=unavailable, clock=clock, log=lambda m: None)


def vio_tick(service, device):
    """One VIO frame worth of snapshot handling, as in pnp_vio.run()."""
    service.tick()
    frame = device.try_get()
    if frame is not None:
        service.on_frame(lambda stream, f=frame: f, device_timestamp=12.5, sequence=42)


def decode(data, flags=cv2.IMREAD_UNCHANGED):
    return cv2.imdecode(np.frombuffer(data, np.uint8), flags)


@pytest.mark.parametrize("stream", snapshots.STREAMS)
def test_request_is_served_with_a_fresh_png(tmp_path, stream):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    requested_at = snapshots.request_snapshot(stream, tmp_path)
    vio_tick(service, device)
    assert device.requests == [stream]

    data, meta = snapshots.wait_for_snapshot(stream, requested_at, snapshot_dir=tmp_path, timeout=0)
    image = decode(data)
    if stream == "depth":
        # Colorized: invalid pixels black, valid ones not
        assert image.shape == (400, 640, 3)
        assert not image[DEPTH == 0].any()
        assert image[DEPTH > 0].any(axis=1).all()
        raw, _ = snapshots.wait_for_snapshot(stream, requested_at, raw=True, snapshot_dir=tmp_path, timeout=0)
        raw_image = decode(raw)
        assert raw_image.dtype == np.uint16 and np.array_equal(raw_image, DEPTH)
    else:
        assert np.array_equal(image, FRAMES[stream])
    assert meta["requested_at"] == requested_at
    assert meta["device_timestamp"] == 12.5 and meta["sequence"] == 42
    assert (meta["width"], meta["height"]) == (640, 400)
    assert not (tmp_path / "requests" / stream).exists()


def test_stale_snapshot_is_not_returned(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    first = snapshots.request_snapshot("depth", tmp_path, clock=lambda: 100.0)
    vio_tick(service, device)
    snapshots.wait_for_snapshot("depth", first, snapshot_dir=tmp_path, timeout=0)

    second = snapshots.request_snapshot("depth", tmp_path, clock=lambda: 101.0)
    times = iter([0.0, 1.0, 2.0, 3.5])
    with pytest.raises(snapshots.SnapshotTimeout):
        snapshots.wait_for_snapshot("depth", second, snapshot_dir=tmp_path, timeout=3.0,
                                    clock=lambda: next(times), sleep=lambda s: None)


def test_device_requests_are_throttled_and_one_at_a_time(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    snapshots.request_snapshot("depth", tmp_path, clock=lambda: 2.0)
    snapshots.request_snapshot("rect_left", tmp_path, clock=lambda: 1.0)
    os.utime(tmp_path / "requests" / "depth", (2.0, 2.0))
    os.utime(tmp_path / "requests" / "rect_left", (1.0, 1.0))
    service.tick()
    assert device.requests == ["rect_left"]  # oldest first, one in flight
    clock.now = 0.5
    service.tick()
    assert device.requests == ["rect_left"]  # still pending
    vio_tick(service, device)
    clock.now = 0.9
    service.tick()
    assert device.requests == ["rect_left"]  # answered, but within min_interval
    clock.now = 1.2
    service.tick()
    assert device.requests == ["rect_left", "depth"]


def test_repeated_requests_for_a_stream_coalesce(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    for t in (1.0, 2.0, 3.0):
        snapshots.request_snapshot("rgb", tmp_path, clock=lambda t=t: t)
    vio_tick(service, device)
    clock.now = 5.0
    vio_tick(service, device)
    assert device.requests == ["rgb"]
    # One reply serves every request made before it was claimed
    for t in (1.0, 2.0, 3.0):
        snapshots.wait_for_snapshot("rgb", t, snapshot_dir=tmp_path, timeout=0)


def test_request_written_after_claim_is_served_next(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    snapshots.request_snapshot("depth", tmp_path, clock=lambda: 1.0)
    service.tick()  # claimed and sent
    snapshots.request_snapshot("depth", tmp_path, clock=lambda: 2.0)  # arrives while in flight
    frame = device.try_get()
    service.on_frame(lambda stream: frame)
    assert snapshots.read_metadata("depth", tmp_path)["requested_at"] == 1.0
    clock.now = 2.0
    vio_tick(service, device)
    assert device.requests == ["depth", "depth"]
    assert snapshots.read_metadata("depth", tmp_path)["requested_at"] == 2.0


def test_unavailable_stream_answers_with_error(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device, available=("rect_left", "rect_right", "depth"),
                           unavailable={"rgb": "this device has no CAM_A (RGB) camera"})
    requested_at = snapshots.request_snapshot("rgb", tmp_path)
    vio_tick(service, device)
    assert device.requests == []
    with pytest.raises(snapshots.SnapshotUnavailable, match="no CAM_A"):
        snapshots.wait_for_snapshot("rgb", requested_at, snapshot_dir=tmp_path, timeout=0)


def test_device_reply_timeout_reports_error_and_unblocks(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    requested_at = snapshots.request_snapshot("depth", tmp_path)
    service.tick()
    device.queue.clear()  # the device never answers
    clock.now = 2.5
    service.tick()
    with pytest.raises(snapshots.SnapshotUnavailable, match="did not return"):
        snapshots.wait_for_snapshot("depth", requested_at, snapshot_dir=tmp_path, timeout=0)
    assert service.on_frame(lambda stream: DEPTH) is False  # late reply ignored
    snapshots.request_snapshot("rect_left", tmp_path)
    clock.now = 3.0
    service.tick()
    assert device.requests == ["depth", "rect_left"]


def test_no_request_dir_and_garbage_files_are_harmless(tmp_path):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    service.tick()
    (tmp_path / "requests").mkdir()
    (tmp_path / "requests" / "depth").write_text("not json")
    (tmp_path / "requests" / "passwd").write_text("{}")
    clock.now = 1.0
    service.tick()
    assert device.requests == []


def test_tick_polls_the_filesystem_at_most_every_poll_interval(tmp_path, monkeypatch):
    clock, device = FakeClock(), FakeDevice()
    service = make_service(tmp_path, clock, device)
    scans = []
    real_scandir = os.scandir
    monkeypatch.setattr(snapshots.os, "scandir", lambda path: scans.append(path) or real_scandir(path))
    (tmp_path / "requests").mkdir()
    for i in range(10):
        clock.now = i * 0.05
        service.tick()
    assert len(scans) == 3  # t = 0, 0.2, 0.4


def test_writer_never_blocks_the_caller(tmp_path):
    writer = snapshots.SnapshotWriter(tmp_path, log=lambda m: None)
    release = threading.Event()

    def slow_load():
        release.wait(2.0)
        return RECT

    start = time.perf_counter()
    results = [writer.submit("rect_left", slow_load, {"requested_at": 1.0}) for _ in range(10)]
    elapsed = time.perf_counter() - start
    assert elapsed < 0.05
    assert results.count(True) <= 5 and results[-1] is False  # bounded queue: extra snapshots dropped
    release.set()
    writer.close()
    assert (tmp_path / "rect_left.png").exists()
    assert json.loads((tmp_path / "rect_left.json").read_text())["requested_at"] == 1.0


def test_writer_survives_a_failing_frame(tmp_path):
    messages = []
    writer = snapshots.SnapshotWriter(tmp_path, log=messages.append)

    def broken():
        raise RuntimeError("frame conversion failed")

    writer.submit("rgb", broken, {"requested_at": 1.0})
    writer.submit("rect_left", lambda: RECT, {"requested_at": 2.0})
    writer.close()
    assert any("frame conversion failed" in m for m in messages)
    assert (tmp_path / "rect_left.png").exists()


def test_colorize_depth_near_red_far_blue_invalid_black():
    depth = np.array([[0, 300, 4000, 9000]], dtype=np.uint16)
    color = snapshots.colorize_depth(depth, 300, 4000)
    blue, _, red = color[0].astype(int).T
    assert not color[0, 0].any()
    assert red[1] > blue[1] and blue[2] > red[2]
    assert np.array_equal(color[0, 2], color[0, 3])  # clipped at max


def test_unknown_stream_rejected(tmp_path):
    with pytest.raises(ValueError):
        snapshots.request_snapshot("../config", tmp_path)


# --- web API -------------------------------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    return main, fastapi_testclient.TestClient(main.app)


@pytest.fixture
def fake_vio(tmp_path):
    """A VIO process stand-in: polls requests and serves them like pnp_vio.run() does."""
    snapshot_dir = tmp_path / "snapshots"
    device = FakeDevice()
    writer = snapshots.SnapshotWriter(snapshot_dir, log=lambda m: None)
    service = snapshots.SnapshotService(("rect_left", "rect_right", "depth"), device.send_request, writer,
                                        snapshot_dir, unavailable={"rgb": "this device has no CAM_A (RGB) camera"},
                                        min_interval=0.0, poll_interval=0.01, log=lambda m: None)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            vio_tick(service, device)
            time.sleep(0.01)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield device
    stop.set()
    thread.join()
    writer.close()


def test_api_snapshot_returns_png(client, fake_vio):
    main, http = client
    response = http.get("/api/snapshot?stream=rect_left")
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    assert response.headers["cache-control"] == "no-store"
    assert response.headers["x-snapshot-sequence"] == "42"
    assert np.array_equal(decode(response.content), RECT)

    depth = http.get("/api/snapshot?stream=depth")
    assert decode(depth.content).shape == (400, 640, 3)
    raw = http.get("/api/snapshot?stream=depth&raw=1")
    assert np.array_equal(decode(raw.content), DEPTH)


def test_api_snapshot_errors(client, fake_vio, monkeypatch):
    main, http = client
    assert http.get("/api/snapshot?stream=rgb").status_code == 404
    assert "CAM_A" in http.get("/api/snapshot?stream=rgb").json()["detail"]
    assert http.get("/api/snapshot?stream=rect_left&raw=1").status_code == 400
    assert http.get("/api/snapshot?stream=passwd").status_code == 422
    assert http.get("/api/snapshot").status_code == 422


def test_api_snapshot_times_out_without_vio(client, monkeypatch):
    main, http = client
    monkeypatch.setattr(main, "SNAPSHOT_TIMEOUT", 0.2)
    response = http.get("/api/snapshot?stream=depth")
    assert response.status_code == 504
    assert "PnP" in response.json()["detail"]
