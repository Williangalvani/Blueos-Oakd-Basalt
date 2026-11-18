import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import vio_telemetry  # noqa: E402


@pytest.fixture(autouse=True)
def isolated_telemetry(monkeypatch, tmp_path):
    """Keep every test's telemetry (status.json, history.npy, leases) out of /dev/shm."""
    monkeypatch.setattr(vio_telemetry, "DEFAULT_DIR", tmp_path / "telemetry")
    try:
        import main
    except ImportError:  # no FastAPI: the web tests skip themselves
        main = None
    if main is not None:
        _isolate_main(monkeypatch, main)
    yield


def _isolate_main(monkeypatch, main):
    import dvl_source

    def no_network(url, timeout=None):
        raise OSError(f"tests do not contact {url}")

    # A fresh poller per test that never starts a thread or reaches the configured vehicle
    poller = dvl_source.DvlPoller(fetch=no_network)
    monkeypatch.setattr(poller, "touch", lambda start_thread=True: dvl_source.DvlPoller.touch(poller, False))
    monkeypatch.setattr(main, "dvl_poller", poller)
    monkeypatch.setattr(main, "history_reader", vio_telemetry.HistoryReader())
    with main.vehicle_state_lock:
        monkeypatch.setitem(main.vehicle_state, "armed", None)
        monkeypatch.setitem(main.vehicle_state, "checked_at", None)
