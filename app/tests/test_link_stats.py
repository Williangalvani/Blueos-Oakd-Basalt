import json
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import link_stats  # noqa: E402
from link_stats import LinkMonitor, StreamCounter  # noqa: E402


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeProfiling:
    """device.getProfilingData(): cumulative bytes read (camera -> host) and written (host -> camera)."""

    def __init__(self):
        self.read = self.written = 0
        self.error = None

    def __call__(self):
        if self.error:
            raise RuntimeError(self.error)
        return SimpleNamespace(numBytesRead=self.read, numBytesWritten=self.written)


def mbit(megabits):
    """Bytes in that many megabits (1 Mbit = 1e6 bit)."""
    return int(megabits * 1e6 / 8)


def run_seconds(monitor, clock, profiling, rates_down, rates_up=None, streams=None, per_second=None):
    """One sample a second; rates in Mbps for the second before each sample."""
    for i, down in enumerate(rates_down):
        clock.t += 1.0
        profiling.read += mbit(down)
        profiling.written += mbit(rates_up[i] if rates_up else 0.0)
        if streams is not None and per_second is not None:
            for name, (messages, nbytes) in per_second.items():
                streams.add(name, nbytes, messages)
        monitor.sample()


def test_rates_mean_and_peak_from_the_cumulative_counters():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling, usb_speed="SUPER", protocol="USB")
    monitor.sample()  # first sample: no rate yet
    status = monitor.status()
    assert status["available"] is True and status["down_mbps"] is None and status["samples"] == 1

    run_seconds(monitor, clock, profiling, [10, 20, 30, 80, 40], rates_up=[0.1] * 5)
    status = monitor.status()
    assert status["source"] == "xlink profiling"
    assert status["usb_speed"] == "SUPER" and status["protocol"] == "USB"
    assert status["down_mbps"] == pytest.approx(40, rel=1e-6)
    assert status["up_mbps"] == pytest.approx(0.1, rel=1e-5)
    assert status["down_mbps_10s"] == pytest.approx((10 + 20 + 30 + 80 + 40) / 5, rel=1e-6)
    assert status["window_span_s"] == pytest.approx(5.0)
    assert status["peak_down_mbps_60s"] == pytest.approx(80, rel=1e-6)
    assert status["peak_up_mbps_60s"] == pytest.approx(0.1, rel=1e-5)
    assert status["totals_bytes"] == {"down": profiling.read, "up": profiling.written}
    assert status["share_of_100mbps"] == pytest.approx(0.36, rel=1e-6)
    assert status["peak_share_of_100mbps"] == pytest.approx(0.8, rel=1e-6)
    json.dumps(status)


def test_the_mean_covers_the_last_10_s_and_the_peak_the_last_60_s():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    monitor.sample()
    run_seconds(monitor, clock, profiling, [90] + [20] * 30 + [50] * 10)
    status = monitor.status()
    assert status["down_mbps_10s"] == pytest.approx(50, rel=1e-6)
    assert status["peak_down_mbps_60s"] == pytest.approx(90, rel=1e-6)
    # 60 s later the 90 Mbps second has left the peak window
    run_seconds(monitor, clock, profiling, [30] * 60)
    status = monitor.status()
    assert status["peak_down_mbps_60s"] == pytest.approx(30, rel=1e-6)
    assert status["down_mbps_10s"] == pytest.approx(30, rel=1e-6)
    # Memory bound: about one peak window of samples
    assert len(monitor.samples) <= link_stats.PEAK_WINDOW_S / link_stats.SAMPLE_INTERVAL_S + 2


def test_uneven_sample_spacing_gives_rates_per_second():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    monitor.sample()
    clock.t += 2.0  # the publisher was late: 2 s of traffic in one interval
    profiling.read += mbit(60)
    monitor.sample()
    assert monitor.status()["down_mbps"] == pytest.approx(30, rel=1e-6)


def test_without_profiling_the_status_says_so_and_has_no_numbers():
    monitor = LinkMonitor(clock=FakeClock())
    monitor.sample()
    status = monitor.status()
    assert status["available"] is False and "no camera" in status["reason"]
    for key in ("down_mbps", "up_mbps", "down_mbps_10s", "peak_down_mbps_60s", "share_of_100mbps",
                "totals_bytes", "unaccounted_mbps"):
        assert status[key] is None, key
    assert "n/a" in monitor.summary()


def test_a_failing_get_profiling_data_is_reported_not_raised():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    profiling.error = "Couldn't retrieve profiling data"
    monitor.sample()
    clock.t += 1.0
    monitor.sample()
    status = monitor.status()
    assert status["available"] is False and status["down_mbps"] is None
    assert "Couldn't retrieve profiling data" in status["reason"]
    # It recovers once the call works again
    profiling.error = None
    monitor.sample()
    clock.t += 1.0
    profiling.read += mbit(8)
    monitor.sample()
    assert monitor.status()["available"] is True and monitor.status()["down_mbps"] == pytest.approx(8, rel=1e-6)


def test_counters_that_never_move_are_flagged():
    """A build where the counters stay at zero must not read as an idle link."""
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    for _ in range(4):
        monitor.sample()
        clock.t += 1.0
    status = monitor.status()
    assert status["available"] is False and "stay at 0" in status["reason"]
    assert status["down_mbps"] is None


def test_non_numeric_counters_are_unavailable():
    clock = FakeClock()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(lambda: SimpleNamespace(numBytesRead="x", numBytesWritten=None))
    monitor.sample()
    assert monitor.status()["available"] is False


def test_a_new_connection_starts_over():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    monitor.sample()
    run_seconds(monitor, clock, profiling, [50, 50])
    monitor.clear_device("camera reconnecting")
    status = monitor.status()
    assert status["available"] is False and status["reason"] == "camera reconnecting"
    assert status["down_mbps"] is None and status["peak_down_mbps_60s"] is None

    fresh = FakeProfiling()  # the reopened link counts from 0
    monitor.set_device(fresh, usb_speed="HIGH")
    monitor.sample()
    run_seconds(monitor, clock, fresh, [5])
    status = monitor.status()
    assert status["down_mbps"] == pytest.approx(5, rel=1e-6) and status["peak_down_mbps_60s"] == pytest.approx(5)
    assert status["usb_speed"] == "HIGH"


def test_a_counter_that_goes_backwards_resets_the_window():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling)
    monitor.sample()
    run_seconds(monitor, clock, profiling, [50, 50])
    profiling.read = 0
    monitor.sample()
    assert monitor.status()["down_mbps"] is None
    run_seconds(monitor, clock, profiling, [7])
    assert monitor.status()["down_mbps"] == pytest.approx(7, rel=1e-6)


def test_per_stream_rates_over_the_window_and_the_remainder():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    streams = monitor.streams
    monitor.set_device(profiling)
    monitor.sample()
    per_second = {"depth": (15, 15 * 512_500), "imu": (200, 200 * 196), "features": (15, 15 * 19_248)}
    sent = sum(nbytes for _, nbytes in per_second.values()) * 8 / 1e6
    run_seconds(monitor, clock, profiling, [sent * 1.05] * 12, streams=streams, per_second=per_second)
    status = monitor.status()
    by_name = {s["name"]: s for s in status["streams"]}
    assert list(by_name) == ["depth", "features", "imu"]  # link_stats.STREAMS order
    depth = by_name["depth"]
    assert depth["mbps"] == pytest.approx(15 * 512_500 * 8 / 1e6, rel=1e-6)
    assert depth["msgs_per_s"] == pytest.approx(15) and depth["bytes_per_msg"] == pytest.approx(512_500)
    assert by_name["imu"]["msgs_per_s"] == pytest.approx(200) and by_name["imu"]["bytes_per_msg"] == pytest.approx(196)
    assert depth["description"]
    assert status["streams_sum_mbps"] == pytest.approx(sent, rel=1e-6)
    assert status["unaccounted_mbps"] == pytest.approx(sent * 0.05, rel=1e-4)
    assert status["streams_window_s"] == pytest.approx(10.0)


def test_streams_are_reported_without_profiling_too():
    clock = FakeClock()
    monitor = LinkMonitor(clock=clock)
    monitor.sample()
    for _ in range(3):
        clock.t += 1.0
        monitor.streams.add("depth", 512_500)
        monitor.sample()
    status = monitor.status()
    assert status["available"] is False and status["unaccounted_mbps"] is None
    (depth,) = status["streams"]
    assert depth["mbps"] == pytest.approx(512_500 * 8 / 1e6, rel=1e-6)
    assert status["streams_sum_mbps"] == pytest.approx(depth["mbps"])


def test_an_idle_stream_stays_listed_with_zero_rate():
    clock = FakeClock()
    monitor = LinkMonitor(clock=clock)
    monitor.streams.add("snapshot", 256_500)
    monitor.sample()
    clock.t += 1.0
    monitor.sample()
    (snapshot,) = monitor.status()["streams"]
    assert snapshot["mbps"] == 0.0 and snapshot["msgs_per_s"] == 0.0 and snapshot["bytes_per_msg"] is None


def test_unknown_stream_names_sort_after_the_known_ones():
    clock = FakeClock()
    monitor = LinkMonitor(clock=clock)
    monitor.sample()
    clock.t += 1.0
    monitor.streams.add("zzz_custom", 10)
    monitor.streams.add("imu", 196)
    monitor.sample()
    assert [s["name"] for s in monitor.status()["streams"]] == ["imu", "zzz_custom"]


def test_stream_counter_counts_messages_and_bytes():
    counter = StreamCounter()
    counter.add("imu", 196)
    counter.add("imu", 352)
    counter.add("depth", 512_500, messages=1)
    assert counter.totals() == {"imu": (2, 548), "depth": (1, 512_500)}


def test_size_estimates():
    assert link_stats.frame_bytes(512_000) == 512_000 + link_stats.IMG_FRAME_OVERHEAD
    assert link_stats.encoded_bytes(30_000) == 30_000 + link_stats.ENCODED_FRAME_OVERHEAD
    assert link_stats.features_bytes(300) == link_stats.TRACKED_FEATURES_OVERHEAD + 300 * link_stats.TRACKED_FEATURE_BYTES
    assert link_stats.imu_bytes(2) == link_stats.IMU_DATA_OVERHEAD + 2 * link_stats.IMU_PACKET_BYTES
    assert link_stats.buffer_bytes(20) == 20 + link_stats.BUFFER_OVERHEAD
    assert link_stats.log_bytes("hello") == 5 + link_stats.LOG_MESSAGE_OVERHEAD
    assert link_stats.data_bytes(SimpleNamespace(getData=lambda: b"abcd")) == 4
    assert all(name in link_stats.STREAMS for name in ("depth", "features", "sync_group", "imu", "snapshot",
                                                        "depth_stream", "depth_stream_status", "device_log"))


def test_feature_sizes_follow_the_libnop_integer_widths():
    # libnop: < 128 in the prefix byte, then 1 + 1 / 2 / 4 / 8 bytes
    assert link_stats.uint_bytes([0, 127, 128, 255, 256, 65535, 65536, 2 ** 32 - 1, 2 ** 32]).tolist() == \
        [1, 1, 2, 2, 3, 3, 5, 5, 9]
    ids = np.array([5, 200, 1000, 70000])
    fixed = link_stats.TRACKED_FEATURE_FIXED_BYTES + link_stats.FEATURE_AGE_BYTES
    assert link_stats.features_bytes(ids) == link_stats.TRACKED_FEATURES_OVERHEAD + 4 * fixed + 1 + 2 + 3 + 5
    # A count: ids of 3 bytes (what a tracker running for a while has), the 64 B per feature of test15
    assert link_stats.features_bytes(255) == link_stats.TRACKED_FEATURES_OVERHEAD + 255 * 64
    assert link_stats.features_bytes(np.zeros(0, np.int64)) == link_stats.TRACKED_FEATURES_OVERHEAD
    # Every message pays the 24 B trailer: datatype, metadata size, the 16 B end-of-packet marker
    assert link_stats.TRAILER_BYTES == 24 and link_stats.buffer_bytes(0) == link_stats.BUFFER_OVERHEAD
    assert link_stats.points_request_bytes(303) == link_stats.BUFFER_OVERHEAD + link_stats.spatial_config_bytes(303)


def test_summary_for_the_periodic_log_line():
    clock, profiling = FakeClock(), FakeProfiling()
    monitor = LinkMonitor(clock=clock)
    monitor.set_device(profiling, usb_speed="SUPER", protocol="USB")
    monitor.sample()
    run_seconds(monitor, clock, profiling, [60, 64], rates_up=[0.2, 0.2])
    text = monitor.summary()
    assert text.startswith("link ") and "62.0" in text and "0.2" in text and "peak 64.0" in text
    assert "62% of 100 Mbps" in text and "USB SUPER" in text


def test_device_link_info_reads_usb_speed_and_protocol():
    device = SimpleNamespace(getDeviceInfo=lambda: SimpleNamespace(protocol=SimpleNamespace(name="X_LINK_USB_VSC")),
                             getPlatformAsString=lambda: "RVC2")
    assert link_stats.device_link_info(device, "UsbSpeed.SUPER") == {
        "usb_speed": "SUPER", "protocol": "USB", "protocol_raw": "X_LINK_USB_VSC", "platform": "RVC2"}
    poe = SimpleNamespace(getDeviceInfo=lambda: SimpleNamespace(protocol=SimpleNamespace(name="X_LINK_TCP_IP")))
    info = link_stats.device_link_info(poe, "unavailable (RuntimeError)")
    assert info["protocol"] == "TCP/IP (PoE)" and info["usb_speed"] is None and info["platform"] is None

    def broken():
        raise RuntimeError("no info")
    info = link_stats.device_link_info(SimpleNamespace(getDeviceInfo=broken), None)
    assert info["protocol"] is None and info["usb_speed"] is None


def test_upstream_streams_are_marked_and_summed_apart():
    import link_stats
    clock = FakeClock()
    monitor = LinkMonitor(clock=clock)
    monitor.sample()
    for _ in range(3):
        clock.t += 1.0
        monitor.streams.add("depth_points", 10 * link_stats.spatial_data_bytes(303), 10)
        monitor.streams.add("depth_points_rois", 10 * link_stats.spatial_config_bytes(303), 10)
        monitor.streams.add("control", link_stats.BUFFER_OVERHEAD)
        monitor.sample()
    status = monitor.status()
    by_name = {s["name"]: s for s in status["streams"]}
    assert by_name["depth_points"]["direction"] == "down"
    assert by_name["depth_points_rois"]["direction"] == "up" and by_name["control"]["direction"] == "up"
    assert status["streams_sum_mbps"] == pytest.approx(by_name["depth_points"]["mbps"])
    assert status["streams_up_sum_mbps"] == pytest.approx(by_name["depth_points_rois"]["mbps"]
                                                          + by_name["control"]["mbps"])
    assert link_stats.UP_STREAMS == {"depth_points_rois", "control"}
    # 303 ROIs (300 features + 3): ~27 kB down and ~14 kB up per frame
    assert link_stats.spatial_data_bytes(303) == 64 + 303 * 90
    assert link_stats.spatial_config_bytes(303) == 56 + 303 * 45
