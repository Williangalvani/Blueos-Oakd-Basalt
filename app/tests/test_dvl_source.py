import json
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import dvl_source  # noqa: E402
import vio_log  # noqa: E402
from fake_mavlink2rest import DVL, FakeVehicle  # noqa: E402

BASE = "http://fake-vehicle/mavlink2rest"


class Clock:
    def __init__(self, now=1_800_000_000.0):
        self.now = now

    def __call__(self):
        return self.now


def fetcher(vehicle, log=None):
    def fetch(url, timeout=None):
        if log is not None:
            log.append(url)
        with vehicle.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read())
    return fetch


def make(velocity=(0.25, -0.1, 0.05), mode="auto", **kwargs):
    clock = Clock()
    vehicle = FakeVehicle(dvl_velocity=None if velocity is None else (lambda t: velocity), clock=clock, **kwargs)
    urls = []
    poller = dvl_source.DvlPoller(BASE, mode, fetch=fetcher(vehicle, urls), clock=clock)
    return poller, vehicle, clock, urls


def run(poller, clock, seconds, step=0.5):
    for _ in range(int(seconds / step)):
        clock.now += step
        poller.poll_once()


def test_velocity_from_message():
    assert dvl_source.velocity_from_message({"time_delta_usec": 200000, "position_delta": [0.1, -0.02, 0.0]}) == \
        pytest.approx((0.5, -0.1, 0.0))
    assert dvl_source.velocity_from_message({"time_delta_usec": 0, "position_delta": [1, 1, 1]}) is None
    assert dvl_source.velocity_from_message({"position_delta": [1, 1, 1]}) is None
    assert dvl_source.velocity_from_message({"time_delta_usec": 1, "position_delta": [1, "x", 1]}) is None


def test_find_sources_excludes_our_own_message():
    vehicle = FakeVehicle(dvl_velocity=lambda t: (0.1, 0, 0))
    sources = dvl_source.find_sources(vehicle.tree())
    assert [(s[0], s[1]) for s in sources] == [DVL]
    assert dvl_source.find_sources({"vehicles": vehicle.tree()})[0][:2] == DVL
    assert dvl_source.find_sources(None) == [] and dvl_source.find_sources({"1": "junk"}) == []


def test_poller_discovers_the_dvl_and_samples_new_messages():
    poller, vehicle, clock, urls = make()
    poller.poll_once()
    assert poller.source == DVL
    snapshot = poller.snapshot()
    assert snapshot["available"] is False  # the first read may be old: no sample yet
    run(poller, clock, 3.0)
    snapshot = poller.snapshot()
    assert snapshot["available"] is True and snapshot["hint"] is None
    assert snapshot["source"] == {"system_id": 1, "component_id": 197} and snapshot["message"] == "VISION_POSITION_DELTA"
    assert len(snapshot["vx"]) >= 5
    assert snapshot["vx"][-1] == pytest.approx(0.25) and snapshot["vy"][-1] == pytest.approx(-0.1)
    assert snapshot["vz"][-1] == pytest.approx(0.05)
    # After discovery only the one message is polled
    assert sum(url.endswith("/mavlink/vehicles") for url in urls) == 1
    assert all(url.endswith("/vehicles/1/components/197/messages/VISION_POSITION_DELTA") for url in urls[1:])
    assert poller.snapshot(since=clock.now - 1.0)["t"] == [t for t in snapshot["t"] if t >= clock.now - 1.0]


def test_no_dvl_gives_a_hint_and_rediscovers_slowly():
    poller, vehicle, clock, urls = make(velocity=None)
    run(poller, clock, 30.0)
    snapshot = poller.snapshot()
    assert snapshot["available"] is False and snapshot["source"] is None and snapshot["vx"] == []
    assert snapshot["hint"] == "DVL not found (no VISION_POSITION_DELTA from another MAVLink component)"
    assert len(urls) == 3  # one tree read per DISCOVERY_INTERVAL_S


def test_unreachable_mavlink2rest_never_raises():
    poller, vehicle, clock, urls = make(reachable=False)
    run(poller, clock, 5.0)
    snapshot = poller.snapshot()
    assert snapshot["available"] is False and "not reachable" in snapshot["hint"]


def test_off_mode_never_polls():
    poller, vehicle, clock, urls = make(mode="off")
    run(poller, clock, 30.0)
    poller.touch()  # no thread either
    assert urls == [] and poller._thread is None
    assert poller.snapshot()["hint"].startswith("DVL overlay off")


def test_lost_source_is_dropped_and_searched_again():
    poller, vehicle, clock, urls = make()
    run(poller, clock, 2.0)
    assert poller.source == DVL
    vehicle.dvl_velocity = None
    run(poller, clock, 0.5 * dvl_source.MAX_FAILURES)
    assert poller.source is None
    clock.now += dvl_source.STALE_S
    assert poller.snapshot()["hint"].startswith("DVL not found")


def test_stale_dvl_is_reported():
    poller, vehicle, clock, urls = make()
    run(poller, clock, 2.0)
    frozen = vehicle.message(*DVL, "VISION_POSITION_DELTA")
    poller.fetch = lambda url, timeout=None: frozen  # mavlink2rest keeps answering the last message
    poller.poll_once()
    clock.now += dvl_source.STALE_S + 1
    poller.poll_once()
    assert poller.snapshot()["hint"] == "DVL 1:197 not updating"


def test_reconfiguring_resets_the_overlay():
    poller, vehicle, clock, urls = make()
    run(poller, clock, 2.0)
    version = poller.version
    poller.configure("http://other/mavlink2rest", "auto")
    assert poller.source is None and poller.snapshot()["vx"] == [] and poller.version > version


# --- /api/history overlay -------------------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.setattr(main, "SNAPSHOT_DIR", tmp_path / "snapshots")
    (tmp_path / "config.json").write_text(json.dumps({"vehicle_ip": "fake-vehicle", "dvl_source": "auto"}))
    return main, fastapi_testclient.TestClient(main.app)


def use_poller(monkeypatch, main, vehicle, clock):
    poller = dvl_source.DvlPoller(fetch=fetcher(vehicle), clock=clock)
    monkeypatch.setattr(poller, "touch", lambda start_thread=True: dvl_source.DvlPoller.touch(poller, False))
    monkeypatch.setattr(main, "dvl_poller", poller)
    return poller


def test_history_carries_the_dvl_overlay(client, monkeypatch):
    main, http = client
    clock = Clock(time.time() - 10.0)  # samples land in the last 10 s of the server's clock
    vehicle = FakeVehicle(dvl_velocity=lambda t: (0.3, 0.1, 0.0), clock=clock)
    poller = use_poller(monkeypatch, main, vehicle, clock)
    first = http.get("/api/history?seconds=30").json()
    assert poller.base_url == BASE and first["dvl"]["available"] is False
    run(poller, clock, 4.0)
    history = http.get("/api/history?seconds=30").json()
    dvl = history["dvl"]
    assert dvl["available"] is True and dvl["source"] == {"system_id": 1, "component_id": 197}
    assert len(dvl["t"]) == len(dvl["vx"]) >= 5 and all(-30 <= t <= 0 for t in dvl["t"])
    assert dvl["vx"][-1] == pytest.approx(0.3)


def test_history_without_a_dvl(client, monkeypatch):
    main, http = client
    clock = Clock()
    vehicle = FakeVehicle(dvl_velocity=None, clock=clock)
    poller = use_poller(monkeypatch, main, vehicle, clock)
    http.get("/api/history")
    run(poller, clock, 2.0)
    dvl = http.get("/api/history").json()["dvl"]
    assert dvl["available"] is False and dvl["hint"].startswith("DVL not found") and dvl["vx"] == []


def test_history_with_the_overlay_off(client, monkeypatch, tmp_path):
    main, http = client
    (tmp_path / "config.json").write_text(json.dumps({"vehicle_ip": "fake-vehicle", "dvl_source": "off"}))
    vehicle = FakeVehicle(dvl_velocity=lambda t: (0.3, 0.1, 0.0))
    poller = use_poller(monkeypatch, main, vehicle, Clock())
    dvl = http.get("/api/history").json()["dvl"]
    run(poller, Clock(), 2.0)
    assert dvl["mode"] == "off" and dvl["available"] is False and vehicle.requests == []
