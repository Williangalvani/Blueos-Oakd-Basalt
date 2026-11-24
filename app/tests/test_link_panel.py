"""
static/link_panel.js under node: the camera link rows of the status panel and the Cockpit widget's line,
from a real link_stats.LinkMonitor status (so the server's field names and the browser's agree). Skipped
without node.
"""
import json
import os
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import link_stats  # noqa: E402

PANEL_JS = os.path.join(os.path.dirname(__file__), "..", "static", "link_panel.js")
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

RUNNER = r"""
const panel = require(process.argv[1]);
const input = JSON.parse(require('fs').readFileSync(0, 'utf8'));
process.stdout.write(JSON.stringify(input.calls.map(([name, ...args]) => panel[name](...args))));
"""


def run_panel(*calls):
    result = subprocess.run([NODE, "-e", RUNNER, os.path.abspath(PANEL_JS)], input=json.dumps({"calls": calls}),
                            capture_output=True, text=True, timeout=30, check=True)
    return json.loads(result.stdout)


def live_status(mock=False):
    clock = SimpleNamespace(t=0.0)
    counters = SimpleNamespace(read=0, written=0)
    monitor = link_stats.LinkMonitor(clock=lambda: clock.t, mock=mock)
    monitor.set_device(lambda: SimpleNamespace(numBytesRead=counters.read, numBytesWritten=counters.written),
                       usb_speed="SUPER", protocol="USB", protocol_raw="X_LINK_USB_VSC", platform="RVC2")
    monitor.sample()
    for second in range(12):
        clock.t += 1.0
        monitor.streams.add("depth", 15 * 512_500, 15)
        monitor.streams.add("imu", 200 * 196, 200)
        monitor.streams.add("features", 15 * 19_248, 15)
        counters.read += int((15 * 512_500 + 200 * 196 + 15 * 19_248) * 1.02) + (2_000_000 if second == 5 else 0)
        counters.written += 1_000
        monitor.sample()
    return monitor.status()


def test_summary_rows_of_a_measured_link():
    status = live_status()
    (rows,) = run_panel(["linkSummaryRows", status])
    text = dict(rows)
    assert text["down (camera → Pi)"] == (f"{status['down_mbps']:.1f} Mbps now · {status['down_mbps_10s']:.1f} "
                                          f"Mbps mean (10 s)")
    assert text["up (Pi → camera)"].startswith("0.01 Mbps")
    assert text["peak down (60 s)"] == f"{status['peak_down_mbps_60s']:.1f} Mbps"
    assert text["share of 100 Mbps"] == (f"{100 * status['share_of_100mbps']:.0f}% mean · "
                                         f"{100 * status['peak_share_of_100mbps']:.0f}% peak")
    assert text["link"] == "USB SUPER (5 Gbps) · RVC2"
    assert "unaccounted" in text["streams (estimate)"]


def test_stream_rows_with_bars_as_a_share_of_the_measured_total():
    status = live_status()
    (rows,) = run_panel(["linkStreamRows", status])
    assert [r["name"] for r in rows] == ["depth", "features", "imu"]
    depth, features, imu = rows
    assert depth["mbps"] == "61.50" and depth["rate"] == "15.0/s" and depth["size"] == "512.5 kB"
    assert imu["mbps"] == "0.31" and imu["rate"] == "200/s" and imu["size"] == "196 B"
    assert depth["fraction"] == pytest.approx(61.5 / status["down_mbps_10s"], rel=1e-6)
    assert depth["description"].startswith("stereo.depth")


def test_unavailable_profiling_says_why_without_numbers():
    monitor = link_stats.LinkMonitor()
    status = monitor.status()
    (rows, streams, widget) = run_panel(["linkSummaryRows", status], ["linkStreamRows", status],
                                        ["linkWidgetText", status])
    assert rows == [["camera link", "n/a: no camera open"]]
    assert streams == [] and widget == ""
    (rows,) = run_panel(["linkSummaryRows", None])
    assert rows[0][1].startswith("n/a")


def test_mock_is_labelled():
    (rows, widget) = run_panel(["linkSummaryRows", live_status(mock=True)], ["linkWidgetText", live_status(True)])
    assert dict(rows)["source"] == "xlink profiling (mock)"
    assert widget == "link 67.0 Mbps (peak 81.4) · 67% of 100 Mbps (mock)"


def test_widget_line():
    (text,) = run_panel(["linkWidgetText", live_status()])
    assert text == "link 67.0 Mbps (peak 81.4) · 67% of 100 Mbps"


def test_the_stream_list_without_a_measured_total_is_relative_to_the_sum():
    clock = SimpleNamespace(t=0.0)
    monitor = link_stats.LinkMonitor(clock=lambda: clock.t)
    monitor.sample()
    clock.t += 1.0
    monitor.streams.add("depth", 750_000)
    monitor.streams.add("imu", 250_000)
    monitor.sample()
    (rows,) = run_panel(["linkStreamRows", monitor.status()])
    assert [r["fraction"] for r in rows] == [0.75, 0.25]


def test_up_streams_are_marked_and_measured_against_the_up_total():
    clock = SimpleNamespace(t=0.0)
    counters = SimpleNamespace(read=0, written=0)
    monitor = link_stats.LinkMonitor(clock=lambda: clock.t)
    monitor.set_device(lambda: SimpleNamespace(numBytesRead=counters.read, numBytesWritten=counters.written))
    monitor.sample()
    for _ in range(3):
        clock.t += 1.0
        monitor.streams.add("depth_points", 10 * link_stats.spatial_data_bytes(303), 10)
        monitor.streams.add("depth_points_rois", 10 * link_stats.spatial_config_bytes(303), 10)
        counters.read += 10 * link_stats.spatial_data_bytes(303) * 2
        counters.written += 10 * link_stats.spatial_config_bytes(303) * 2
        monitor.sample()
    status = monitor.status()
    (rows, summary) = run_panel(["linkStreamRows", status], ["linkSummaryRows", status])
    by_name = {r["name"]: r for r in rows}
    assert by_name["↑ depth_points_rois"]["direction"] == "up" and by_name["depth_points"]["direction"] == "down"
    # Each bar is a share of its own direction's measured total (half of it: the counters doubled the bytes)
    assert by_name["↑ depth_points_rois"]["fraction"] == pytest.approx(0.5, rel=0.02)
    assert by_name["depth_points"]["fraction"] == pytest.approx(0.5, rel=0.02)
    assert dict(summary)["streams up (estimate)"].endswith("Mbps")


POINTS_LIVE = {
    "transport": "points", "rois": "host", "frames": 1000, "paired": 990, "paired_percent": 99.0, "missing": 10,
    "missing_by_reason": {"timeout": 7, "roi": 3}, "last_problem": {"reason": "roi", "text": "1 ROIs differ"},
    "points_received": 995, "orphan_points": 5, "points_gaps": 2, "points_gap_frames": 4, "feature_gaps": 1,
    "feature_gap_frames": 2, "waiting": True, "wait_ms_mean": 3.25, "wait_ms_max": 40.0, "wait_limit_ms": 300.0,
    "valid_percent": 87.5, "device": {"errors": 0, "misses": 3, "fatal": False,
                                      "stats": {"slc_ms_mean": 4.0, "slc_ms_max": 6.0}},
    "full_depth": {"on": True, "every": 2, "reasons": ["host_jpeg"], "frames": 12, "bytes": 1, "since": 1.0},
}
LOAD = {"leon_css_cpu_percent": 41.0, "leon_mss_cpu_percent": 30.0, "ddr": {"used_percent": 52.0},
        "cmx": {"used_percent": 76.0}, "leon_css_heap": {"used_percent": 56.0}, "leon_mss_heap": None}
VIEWERS = {"total": 2, "video": 1, "video_idle": False, "by_client": {"page": 1, "widget": 1},
           "text": "1 watching video, 1 other"}


def test_transport_rows_points():
    transport = {"configured": "points", "configured_rois": "host", "live": POINTS_LIVE, "camera_load": LOAD}
    (rows,) = run_panel(["transportRows", transport, VIEWERS])
    text = dict(rows)
    assert text["setting"] == "points (ROIs listed on the Pi)"
    assert text["frames with depth"] == "99.00% (990 of 1000)"
    assert text["missing depth"] == "10 (timeout 7, roi 3)" and text["last problem"] == "roi: 1 ROIs differ"
    assert text["frame drops"] == "features 2 (1 gaps) · points 4 (2 gaps) · unpaired points 5"
    assert text["wait for points"] == "3.3 ms mean, 40.0 max (limit 300 ms)"
    assert text["camera script"] == "0 errors · 3 misses · SLC 4.0 ms mean"
    assert text["full depth frames"] == "on, every 2 frame(s), for host_jpeg · 12 sent"
    assert text["camera CPU (Leon)"] == "CSS 41% · MSS 30%"
    assert text["camera memory"] == "DDR 52% · CMX 76% · heap CSS 56% / MSS –"
    assert text["viewers"] == "1 watching video, 1 other (page 1, widget 1)"


def test_transport_rows_full_frame_and_not_running():
    full = {"configured": "full_frame", "configured_rois": "host", "camera_load": None,
            "live": {"transport": "full_frame", "frames": 50, "feature_gaps": 0, "feature_gap_frames": 0}}
    stopped = {"configured": "points", "configured_rois": "camera", "live": None, "camera_load": None}
    idle = {**VIEWERS, "total": 0, "video": 0, "video_idle": True, "by_client": {}, "text": "video idle"}
    (rows, stopped_rows) = run_panel(["transportRows", full, idle], ["transportRows", stopped, None])
    text = dict(rows)
    assert text["setting"] == "full_frame" and text["running"].startswith("full_frame: every depth frame")
    assert text["viewers"] == "video idle" and text["camera CPU"] == "–"
    stopped_text = dict(stopped_rows)
    assert stopped_text["setting"] == "points (ROIs listed on the camera)"
    assert stopped_text["running"] == "n/a (VIO not running)" and stopped_text["viewers"] == "–"


def test_transport_widget_parts():
    (good, bad, full, none) = run_panel(
        ["transportWidgetParts", {"transport": "points", "paired_percent": 99.4}, VIEWERS],
        ["transportWidgetParts", {"transport": "points", "paired_percent": 50.0}, None],
        ["transportWidgetParts", {"transport": "full_frame"}, {"text": "video idle"}],
        ["transportWidgetParts", None, None])
    assert good == [["depth points 99%", "muted"], ["1 watching video, 1 other", "muted"]]
    assert bad == [["depth points 50%", "bad"]]
    assert full == [["depth: full frames", "muted"], ["video idle", "muted"]]
    assert none == []


def test_image_stream_rows_flag_raw_fallbacks():
    images = {"streams": [
        {"name": "live_stream", "label": "Live stream", "codec": "mjpeg", "quality": 70, "encoded_on": "camera",
         "viewed": True, "mbps": 0.912},
        {"name": "live_stream", "label": "Live stream", "codec": "mjpeg", "quality": 70, "encoded_on": "pi",
         "viewed": False, "mbps": None,
         "fallback": "fallback: raw depth over the link, encoded on the Pi: blank frames from mono_features"},
        {"name": "snapshot_rect_left", "label": "Snapshot rect_left", "codec": "mjpeg", "quality": 95,
         "encoded_on": "camera", "per_request": True, "last_bytes": 61440},
        {"name": "snapshot_depth", "label": "Snapshot depth", "codec": "png", "quality": None, "encoded_on": "pi",
         "per_request": True, "measurement": True, "reason": "16-bit millimetres"},
        {"name": "snapshot_rect_right", "label": "Snapshot rect_right", "codec": "png", "encoded_on": "pi",
         "per_request": True, "fallback": "fallback: raw frames over the link, encoded on the Pi: no JPEG"},
        {"name": "snapshot_rgb", "label": "Snapshot rgb", "per_request": True}]}
    (rows, empty) = run_panel(["imageStreamRows", images], ["imageStreamRows", None])
    camera, pi, left, depth, right, rgb = (value for _, value in rows)
    assert camera == "MJPEG q70 · encoded on the camera · viewed · 0.91 Mbps"
    assert pi.startswith("MJPEG q70 · encoded on the Pi · not viewed: nothing sent · fallback: raw depth over the link")
    assert left.startswith("MJPEG q95 · encoded on the camera · one frame per request, last ")
    assert depth == "PNG · encoded on the Pi · one frame per request · 16-bit measurement data, raw"
    assert right.endswith("fallback: raw frames over the link, encoded on the Pi: no JPEG")
    assert rgb == "– · n/a (VIO not running) · one frame per request"
    assert empty == [["image streams", "–"]]
