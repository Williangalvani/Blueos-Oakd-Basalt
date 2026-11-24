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
