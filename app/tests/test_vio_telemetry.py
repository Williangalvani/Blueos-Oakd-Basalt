import json
import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import vio_telemetry  # noqa: E402
from vio_telemetry import COLUMN, FailureWindow, RingBuffer, Telemetry, TrajectoryIntegrator  # noqa: E402


class Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def make(tmp_path, fps=10, **kwargs):
    clock = Clock()
    return Telemetry(tmp_path / "telemetry", fps=fps, clock=clock, log=lambda m: None, **kwargs), clock


def frame(telemetry, t, ok=True, reason="ok", inliers=40, delta=(0.01, 0.0, 0.0), dyaw=0.0, sent=False, **kwargs):
    telemetry.record_frame(t, ok, reason, 120, 100, 60, inliers if ok else 3, 25.0, dt=0.1,
                           position_delta=delta if ok else None, yaw_delta=dyaw if ok else None, sent=sent, **kwargs)


def test_ring_buffer_keeps_newest_rows_in_order():
    ring = RingBuffer(4, 2)
    for i in range(6):
        ring.append((i, -i))
    assert ring.count == 4
    assert ring.ordered()[:, 0].tolist() == [2, 3, 4, 5]
    assert ring.ordered(last=2)[:, 0].tolist() == [4, 5]
    assert ring.ordered(last=10)[:, 0].tolist() == [2, 3, 4, 5]
    assert RingBuffer(3, 2).ordered().shape == (0, 2)


def test_history_memory_is_bounded():
    # Documented bound: 10 min at <= 30 fps (+10 %), 12 float64 columns, < 2 MB
    assert vio_telemetry.history_capacity(10) == 6600
    assert vio_telemetry.history_capacity(1000) == vio_telemetry.history_capacity(30) == 19800
    assert RingBuffer(vio_telemetry.history_capacity(1000), len(vio_telemetry.HISTORY_COLUMNS)).nbytes < 2e6


def test_failure_window_counts_reasons_in_the_last_five_minutes():
    window = FailureWindow(window_s=300)
    for i in range(10):
        window.add(float(i), None if i % 2 else "too_few_matches")
    window.add(10.0, "too_few_inliers")
    summary = window.summary()
    assert summary["frames"] == 11 and summary["failed"] == 6
    assert summary["by_reason"][0] == {"reason": "too_few_matches", "count": 5,
                                       "percent_of_frames": pytest.approx(500 / 11),
                                       "percent_of_failures": pytest.approx(500 / 6)}
    window.add(305.5, None)  # everything at t <= 5.5 drops out
    summary = window.summary()
    assert summary["frames"] == 6 and {r["reason"]: r["count"] for r in summary["by_reason"]} == {
        "too_few_matches": 2, "too_few_inliers": 1}


def test_failure_window_is_bounded_by_frame_count():
    window = FailureWindow(window_s=300, max_frames=5)
    for i in range(20):
        window.add(i * 0.01, "x")
    assert window.summary()["frames"] == 5 and window.summary()["failed"] == 5


def test_reason_details_collapse_to_one_key():
    assert vio_telemetry.normalize_reason("pnp_error: cv2 said something") == "pnp_error"
    assert vio_telemetry.normalize_reason(None) == "unknown"


def test_trajectory_rotates_deltas_by_the_integrated_yaw():
    path = TrajectoryIntegrator()
    path.step(1.0, 0.0, math.pi / 2)  # forward 1 m, then turn right 90 degrees
    x, y, yaw = path.step(1.0, 0.0, 0.0)  # "forward" is now +y
    assert (x, y, yaw) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_record_frame_builds_history_and_status(tmp_path):
    telemetry, clock = make(tmp_path, min_inliers=15, mode="log-only")
    for i in range(30):
        frame(telemetry, 1000.0 + i * 0.1, sent=True)
    frame(telemetry, 1003.0, ok=False, reason="too_few_matches")
    frame(telemetry, 1003.1, ok=False, reason="pnp_error: bad")
    frame(telemetry, 1003.2, reference_reset=True, gap_frames=2)
    telemetry.record_recovery()
    clock.now = 1003.25
    status = telemetry.status()
    assert status["frames_total"] == 33 and status["ok_total"] == 31 and status["sent_total"] == 30
    assert status["reference_resets"] == 1 and status["gap_frames_total"] == 2 and status["recoveries"] == 1
    assert status["last_frame"]["inliers"] == 40 and status["last_frame"]["ok"] is True
    assert status["recent"]["fps"] == pytest.approx(10.0, rel=0.1)
    assert status["recent"]["ok_percent"] == pytest.approx(100 * 31 / 33)
    assert status["failures"]["failed"] == 2
    assert sorted(r["reason"] for r in status["failures"]["by_reason"]) == ["pnp_error", "too_few_matches"]
    assert status["output"]["mode"] == "log-only" and status["output"]["messages_sent"] == 30
    assert status["output"]["rate_hz"] == pytest.approx(30 / 5)
    assert status["min_inliers"] == 15
    rows = telemetry.history.ordered()
    assert rows.shape == (33, len(vio_telemetry.HISTORY_COLUMNS))
    assert rows[0, COLUMN["vx"]] == pytest.approx(0.1)
    assert np.isnan(rows[30, COLUMN["vx"]]) and rows[30, COLUMN["ok"]] == 0
    assert rows[-1, COLUMN["x"]] == pytest.approx(0.31)
    json.dumps(status)  # publishable


def test_publishes_only_while_leased(tmp_path):
    telemetry, clock = make(tmp_path)
    directory = tmp_path / "telemetry"
    frame(telemetry, clock.now)
    telemetry.publish_due()
    assert not (directory / vio_telemetry.STATUS_FILE).exists()
    assert not (directory / vio_telemetry.HISTORY_FILE).exists()

    vio_telemetry.touch_lease(vio_telemetry.STATUS_LEASE, directory)
    clock.now = time.time()
    telemetry.publish_due()
    assert vio_telemetry.read_status(directory)["frames_total"] == 1
    assert not (directory / vio_telemetry.HISTORY_FILE).exists()

    vio_telemetry.touch_lease(vio_telemetry.HISTORY_LEASE, directory)
    telemetry.publish_due(clock.now + 1.0)
    rows, mtime = vio_telemetry.HistoryReader().read(directory)
    assert rows.shape == (1, len(vio_telemetry.HISTORY_COLUMNS)) and mtime is not None

    # Leases expire: nothing more is written
    os.utime(directory / vio_telemetry.STATUS_LEASE, (0, 0))
    (directory / vio_telemetry.STATUS_FILE).unlink()
    telemetry.publish_due(clock.now + 2.0)
    assert not (directory / vio_telemetry.STATUS_FILE).exists()


def test_a_new_run_removes_the_previous_runs_files(tmp_path):
    directory = tmp_path / "telemetry"
    directory.mkdir()
    (directory / vio_telemetry.STATUS_FILE).write_text("{}")
    (directory / vio_telemetry.HISTORY_FILE).write_bytes(b"old")
    make(tmp_path)
    assert not (directory / vio_telemetry.STATUS_FILE).exists()
    assert not (directory / vio_telemetry.HISTORY_FILE).exists()


def test_unusable_directory_is_logged_not_raised(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("")
    messages = []
    telemetry = Telemetry(blocker / "telemetry", log=messages.append)
    telemetry.record_frame(1.0, True, "ok", 1, 1, 1, 1, 1.0)
    assert messages and "unusable" in messages[0]


def test_temperature_is_sampled_at_most_every_five_seconds(tmp_path):
    calls = []

    class Reading:
        average = 47.5

    def temperature():
        calls.append(1)
        return Reading()

    telemetry, clock = make(tmp_path, temperature=temperature)
    telemetry.publish_due(100.0)
    telemetry.publish_due(103.0)
    assert len(calls) == 1 and telemetry.chip_temperature == 47.5
    telemetry.publish_due(105.0)
    assert len(calls) == 2


def test_temperature_errors_give_none(tmp_path):
    def broken():
        raise RuntimeError("device gone")

    telemetry, clock = make(tmp_path, temperature=broken)
    telemetry.publish_due(100.0)
    assert telemetry.chip_temperature is None and telemetry.status()["chip_temperature_c"] is None


def test_link_is_sampled_about_every_second_and_published(tmp_path):
    import link_stats
    from types import SimpleNamespace

    link_clock = Clock(50.0)
    counters = {"read": 0}

    def profiling():
        counters["read"] += 1_250_000  # 10 Mbit per sample
        return SimpleNamespace(numBytesRead=counters["read"], numBytesWritten=0)

    link = link_stats.LinkMonitor(clock=link_clock)
    link.set_device(profiling, usb_speed="SUPER", protocol="USB")
    telemetry, clock = make(tmp_path, link=link)
    for now in (100.0, 100.4, 100.8, 101.0, 101.5, 102.0):
        link_clock.now = 50.0 + (now - 100.0)
        telemetry.publish_due(now)
    assert len(link.samples) == 3  # at 100, 101 and 102: about once a second
    status = telemetry.status()["link"]
    assert status["available"] is True and status["usb_speed"] == "SUPER"
    assert status["down_mbps"] == pytest.approx(10.0) and status["down_mbps_10s"] == pytest.approx(10.0)
    json.dumps(telemetry.status())


def test_without_a_link_monitor_the_status_has_none(tmp_path):
    telemetry, clock = make(tmp_path)
    telemetry.publish_due(100.0)
    assert telemetry.status()["link"] is None


def test_a_failing_link_sample_does_not_stop_the_publisher(tmp_path):
    import link_stats

    class Broken(link_stats.LinkMonitor):
        def sample(self):
            raise RuntimeError("boom")

    telemetry, clock = make(tmp_path, link=Broken(), temperature=lambda: 40.0)
    telemetry.publish_due(100.0)
    assert telemetry.chip_temperature == 40.0


def test_output_error_comes_from_the_sender(tmp_path):
    telemetry, clock = make(tmp_path, output_error=lambda: "12:00:01 connection refused")
    assert telemetry.status()["output"]["last_error"] == "12:00:01 connection refused"


def test_downsample_keeps_the_newest_row_and_caps_points():
    rows = np.zeros((1000, len(vio_telemetry.HISTORY_COLUMNS)))
    rows[:, COLUMN["t"]] = np.arange(1000) * 0.1  # 0 .. 99.9 s
    picked, stride = vio_telemetry.downsample(rows, 60.0, 99.9, 100)
    assert len(picked) <= 100 and stride == 7
    assert picked[-1, COLUMN["t"]] == pytest.approx(99.9)
    assert picked[0, COLUMN["t"]] >= 39.9 - 1e-9
    empty, stride = vio_telemetry.downsample(None, 60.0, 0.0, 100)
    assert empty.shape == (0, len(vio_telemetry.HISTORY_COLUMNS)) and stride == 1


def test_history_reader_rejects_bad_files(tmp_path):
    directory = tmp_path / "telemetry"
    directory.mkdir()
    np.save(directory / vio_telemetry.HISTORY_FILE, np.zeros((3, 2)))
    assert vio_telemetry.HistoryReader().read(directory) == (None, None)
    (directory / vio_telemetry.HISTORY_FILE).write_bytes(b"garbage")
    assert vio_telemetry.HistoryReader().read(directory) == (None, None)
    assert vio_telemetry.HistoryReader().read(tmp_path / "missing") == (None, None)


def test_vio_loop_cost_is_tiny(tmp_path):
    """The VIO loop's share (record_frame) plus the publisher's step (publish_due) stay far below 0.2 ms
    a frame, also while the page is open (both leases fresh, status and history written at their rates)."""
    telemetry, clock = make(tmp_path, fps=30)
    directory = tmp_path / "telemetry"
    frames = 3000
    for i in range(frames):  # fill the ring like a long run
        frame(telemetry, i / 30.0)

    def run(start):
        t0 = time.perf_counter()
        for i in range(frames):
            now = start + i / 30.0
            clock.now = now
            frame(telemetry, now, ok=i % 7 != 0, reason="too_few_matches", sent=True)
            telemetry.publish_due(now)
        return (time.perf_counter() - t0) / frames * 1e3

    idle_ms = run(1000.0)
    vio_telemetry.touch_lease(vio_telemetry.STATUS_LEASE, directory)
    vio_telemetry.touch_lease(vio_telemetry.HISTORY_LEASE, directory)
    watched_ms = run(time.time())
    assert (directory / vio_telemetry.HISTORY_FILE).exists()
    print(f"record_frame + publish_due: {idle_ms:.4f} ms/frame idle, {watched_ms:.4f} ms/frame with the page open")
    assert idle_ms < 0.2
    assert watched_ms < 0.2
