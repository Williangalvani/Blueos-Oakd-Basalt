"""
depth_transport "points" (depth_points.py): the windows, the host's ROI config, the camera Scripts run against
strict fakes of the camera's Script bindings and of the SpatialLocationCalculator (SLC), the host's pairing
and checks, the on-demand full depth gate, and the host's use of the points (pnp_odometry.depth_from_points).

The SLC types and computation are slc_fakes.py's. FakeSLC adds the node's behaviour: one config per depth frame
(inputConfig.setWaitForMessage(True)) and, by default, the worst case: it neither copies the config's sequence
number to its output nor keeps the depth frame's timestamp, so only the Script's re-tagging makes them right.
The host's configs reach the Script through slc_fakes.over_link: with sequence number 0, as on the hardware.
"""
import math
import os
import sys
from datetime import timedelta
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_points  # noqa: E402
from pnp_odometry import PnPParams, depth_from_points, lookup_depth  # noqa: E402
from slc_fakes import (FAKE_DAI, Algorithm, Config, ConfigData, HostBuffer, Point2f, Rect, Size2f,  # noqa: E402
                       Thresholds, answer, over_link, slc_compute)

IMAGE = (640, 400)


class Done(BaseException):
    """Ends a fake Script run; not an Exception, so the Script's own error handling does not catch it."""


# --- the camera's Script bindings (lpb), strict ------------------------------------------------------------

class Buffer:
    API = frozenset({"getData", "setData", "getSequenceNum", "setSequenceNum", "getTimestamp", "setTimestamp"})

    def __init__(self, *args, **kwargs):
        if kwargs or len(args) != 1 or not isinstance(args[0], int) or isinstance(args[0], bool):
            raise TypeError("__init__(): incompatible constructor arguments ... Buffer(arg0: int)")
        self.size, self.data = args[0], None

    def setData(self, data):
        if len(data) > self.size:
            raise RuntimeError("Buffer size too small")
        self.data = bytes(data)


SCRIPT_GLOBALS = {"Buffer": Buffer, "Rect": Rect, "Point2f": Point2f, "Size2f": Size2f,
                  "SpatialLocationCalculatorConfig": Config, "SpatialLocationCalculatorConfigData": ConfigData,
                  "SpatialLocationCalculatorConfigThresholds": Thresholds,
                  "SpatialLocationCalculatorAlgorithm": Algorithm}


# --- messages ----------------------------------------------------------------------------------------------

class DepthFrame:
    def __init__(self, depth, sequence, t=None):
        self.depth, self.sequence = depth, sequence
        self.timestamp = timedelta(seconds=100.0 + sequence / 10.0 if t is None else t)

    def getSequenceNum(self):
        return self.sequence

    def getWidth(self):
        return self.depth.shape[1]

    def getHeight(self):
        return self.depth.shape[0]

    def getTimestamp(self):
        return self.timestamp


class Features:
    """TrackedFeatures as the Script sees it: trackedFeatures[i].position.x/.y on the image grid."""

    def __init__(self, uv, sequence):
        self.trackedFeatures = [SimpleNamespace(position=Point2f(u, v)) for u, v in uv]
        self.sequence = sequence

    def getSequenceNum(self):
        return self.sequence


class FakeInput:
    def __init__(self, items=()):
        self.items = list(items)

    def get(self):
        if not self.items:
            raise Done()
        return self.items.pop(0)

    def tryGet(self):
        return self.items.pop(0) if self.items else None


class Timeline:
    """
    Messages arriving in order, (input name, message). A blocking get() on an empty input lets time run: the next
    arrivals go into their inputs until that one has a message (Done when none are left). An input with a
    max_size drops its oldest message when a new one comes to a full queue, like a non-blocking device input.
    """

    def __init__(self, events):
        self.events, self.inputs = list(events), {}

    def input(self, name, max_size=None):
        self.inputs[name] = TimedInput(self, max_size)
        return self.inputs[name]

    def advance(self):
        if not self.events:
            raise Done()
        name, message = self.events.pop(0)
        self.inputs[name].push(message)


class TimedInput(FakeInput):
    def __init__(self, timeline, max_size):
        super().__init__()
        self.timeline, self.max_size, self.dropped = timeline, max_size, []

    def push(self, message):
        self.items.append(message)
        if self.max_size is not None and len(self.items) > self.max_size:
            self.dropped.append(self.items.pop(0))

    def get(self):
        while not self.items:
            self.timeline.advance()
        return self.items.pop(0)


class FakeOutput:
    def __init__(self, on_send=None):
        self.sent, self.on_send = [], on_send

    def send(self, message):
        self.sent.append(message)
        if self.on_send:
            self.on_send(message)


class FakeSLC:
    """Blocking, lockstep: each depth frame takes the config sent before it (waitForMessage); output untagged."""

    def __init__(self, node, copy_sequence=False, out_of_order=False):
        self.node, self.copy_sequence, self.out_of_order = node, copy_sequence, out_of_order
        self.config, self.configs = None, []

    def on_config(self, config):
        assert self.config is None, "a second config before its depth frame"
        self.config = config
        self.configs.append(config)

    def on_depth(self, frame):
        assert self.config is not None, "a depth frame without a config"
        spatial = slc_compute(self.config, frame.depth)
        if self.copy_sequence:
            spatial.setSequenceNum(self.config.getSequenceNum())
        self.config = None
        passthrough = DepthFrame(frame.depth, frame.sequence + 1, frame.timestamp.total_seconds()) \
            if self.out_of_order else frame
        self.node.inputs["spatial"].items.append(spatial)
        self.node.inputs["passthrough"].items.append(passthrough)


def script_node(node, inputs, outputs):
    """A fake Script node; depthai (Pipeline.cpp) rejects an input and an output with the same name at start."""
    assert not set(inputs) & set(outputs), f"'Script.io[{sorted(set(inputs) & set(outputs))}]' redefined"
    node.inputs, node.outputs = inputs, outputs
    return node


def run_device_script(roi_source, depth_frames=(), requests=(), thresholds=(199, 15001), clock=None, events=None,
                      **slc_options):
    """
    depth_frames, requests: all there from the start (an unbounded depth queue). events: (input, message) in
    arrival order instead (Timeline), into pnp_vio's queue sizes: the depth input keeps the newest DEPTH_BUFFER.
    """
    node = SimpleNamespace(warnings=[])
    node.warn = node.warnings.append
    slc = FakeSLC(node, **slc_options)
    request_input = "features" if roi_source == depth_points.ROIS_CAMERA else "rois"
    if events is None:
        depth, requested = FakeInput(depth_frames), FakeInput(requests)
    else:
        timeline = Timeline(events)
        depth = timeline.input("depth", depth_points.DEPTH_BUFFER)
        requested = timeline.input(request_input, depth_points.ROIS_QUEUE if request_input == "rois" else None)
    script_node(node, {"depth": depth, "spatial": FakeInput(), "passthrough": FakeInput(), request_input: requested},
                {"config": FakeOutput(slc.on_config), depth_points.SLC_DEPTH_OUT: FakeOutput(slc.on_depth),
                 "points": FakeOutput(), "status": FakeOutput()})
    node.slc = slc
    code = depth_points.device_script(roi_source, IMAGE, thresholds)
    if clock is not None:
        code = code.replace("    clock = time.monotonic", "    clock = fake_clock")
    with pytest.raises(Done):
        exec(compile(code, "depth_points_script", "exec"), {"node": node, "fake_clock": clock, **SCRIPT_GLOBALS})
    return node


def statuses(node):
    return [depth_points.parse_status(message.data) for message in node.outputs["status"].sent]


def scene_depth(size=(320, 200), seed=0):
    rng = np.random.default_rng(seed)
    depth = rng.integers(500, 3000, (size[1], size[0])).astype(np.uint16)
    depth[rng.random(depth.shape) < 0.15] = 0
    return depth


def feature_uv(n=40, seed=1):
    rng = np.random.default_rng(seed)
    uv = rng.uniform((0, 0), IMAGE, (n, 2))
    # Edges, corners and positions just outside the image (the tracker can report them)
    return np.vstack([uv, [[0, 0], [639.9, 399.9], [-0.4, 200], [640.3, 10], [1.0, 398.6], [320, 200]]])


def expected_rows(depth, uv, thresholds=(199, 15001)):
    """What the SLC gives for this frame's windows (the reference for the Script's output)."""
    config = depth_points.make_config(FAKE_DAI, depth_points.all_windows(uv, IMAGE, depth.shape[::-1]),
                                      thresholds, 0)
    return depth_points.decode(slc_compute(config, depth))


# --- windows, thresholds, host config ---------------------------------------------------------------------

def test_windows_are_the_5x5_around_the_feature_on_the_depth_grid_clipped_at_the_edges():
    uv = np.array([[320.0, 200.0], [0.0, 0.0], [639.9, 399.9], [3.0, 1.0], [-5.0, 450.0]])
    windows = depth_points.feature_windows(uv, IMAGE, (320, 200))
    assert windows.tolist() == [[158, 98, 5, 5], [0, 0, 3, 3], [317, 197, 3, 3], [0, 0, 5, 4], [0, 197, 3, 3]]
    full = depth_points.feature_windows(uv, IMAGE, IMAGE)
    assert full[0].tolist() == [318, 198, 5, 5]
    extra = depth_points.extra_windows((320, 200))
    assert extra.tolist() == [[158, 98, 5, 5], [160, 100, 1, 1], [319, 199, 1, 1]]


def test_every_window_stays_inside_the_depth_frame():
    rng = np.random.default_rng(5)
    uv = rng.uniform((-20, -20), (660, 420), (2000, 2))
    for size in ((640, 400), (320, 200), (213, 133), (1, 1)):
        w = depth_points.all_windows(uv, IMAGE, size)
        assert (w[:, :2] >= 0).all() and (w[:, 2:] >= 1).all()
        assert (w[:, 0] + w[:, 2] <= size[0]).all() and (w[:, 1] + w[:, 3] <= size[1]).all()


def test_thresholds_take_the_inclusive_pnp_range_with_the_slc_exclusive_bounds():
    lower, upper = depth_points.thresholds_mm(0.2, 15.0)
    assert (lower, upper) == (199, 15001)
    depth = np.array([[199, 200, 15000, 15001, 0]], np.uint16)
    config = depth_points.make_config(FAKE_DAI, [[0, 0, 5, 1], [0, 0, 1, 1], [0, 0, 1, 1], [0, 0, 1, 1]],
                                      (lower, upper), 0)
    for roi in config.rois:
        roi.calculationAlgorithm = Algorithm.AVERAGE  # every pixel, to see each threshold
    rows = depth_points.decode(slc_compute(config, depth))
    assert rows[0, 6] == 2 and rows[0, 5] == (200 + 15000) / 2


def test_host_config_is_pixel_rects_average_and_its_tag_carries_the_frame():
    windows = depth_points.all_windows(np.array([[0.0, 0.0], [320, 200]]), IMAGE, (320, 200))
    config = depth_points.make_config(FAKE_DAI, windows, (199, 15001), 42)
    assert config.getSequenceNum() == 42 and len(config.rois) == 2 + depth_points.EXTRA_ROIS
    # A 1x1 rect at (0, 0) would be "the whole frame" if normalized: every rect says it is not
    assert not any(roi.roi.normalized for roi in config.rois)
    assert [[r.roi.x, r.roi.y, r.roi.width, r.roi.height] for r in config.rois] == windows.tolist()
    # AVERAGE: what the firmware computes anyway on a frame that comes through the Script
    assert depth_points.ALGORITHM == depth_points.AVERAGE
    assert all(r.calculationAlgorithm == Algorithm.AVERAGE for r in config.rois)
    assert [(r.depthThresholds.lowerThreshold, r.depthThresholds.upperThreshold) for r in config.rois] == \
        [(199, 15001)] * 2 + [depth_points.CENTER_THRESHOLDS] * 3
    tag = depth_points.make_tag(FAKE_DAI, 42)
    assert isinstance(tag, HostBuffer) and not hasattr(tag, "getConfigData")
    # On the camera the config's number is gone (test16b), the tag's is not
    assert over_link(config).getSequenceNum() == 0 and over_link(tag).getSequenceNum() == 42


# --- the camera Script, rois "host" -------------------------------------------------------------------------

def host_requests(depth_size, frames):
    """(sequence, uv) -> what the camera Script receives for the Pi's tag + config of each frame, in order."""
    messages = []
    for s, uv in frames:
        config = depth_points.make_config(FAKE_DAI, depth_points.all_windows(uv, IMAGE, depth_size), (199, 15001), s)
        messages += [over_link(depth_points.make_tag(FAKE_DAI, s)), over_link(config)]
    return messages


def test_host_rois_each_request_gets_the_depth_of_its_own_frame_retagged():
    depths = {s: scene_depth(seed=s) for s in range(1, 6)}
    uvs = {s: feature_uv(seed=s) for s in range(1, 6)}
    node = run_device_script("host", [DepthFrame(depths[s], s) for s in range(1, 6)],
                             host_requests((320, 200), [(s, uvs[s]) for s in range(1, 6)]))
    points = node.outputs["points"].sent
    # The fake SLC tags nothing: the sequence numbers and timestamps are the Script's, from the passthrough
    assert [p.getSequenceNum() for p in points] == [1, 2, 3, 4, 5]
    assert [p.getTimestamp() for p in points] == [DepthFrame(depths[s], s).timestamp for s in range(1, 6)]
    for s, p in zip(range(1, 6), points):
        np.testing.assert_array_equal(depth_points.decode(p), expected_rows(depths[s], uvs[s]))
    # Every config read 0 on the camera, as on test16b: the pairing went by the tags alone
    assert [c.getSequenceNum() for c in node.slc.configs] == [0] * 5
    report = statuses(node)
    assert report[0]["kind"] == "size" and report[0]["size"] == [320, 200]
    (tagging,) = [r for r in report if r["kind"] == "tagging"]
    assert (tagging["tag"], tagging["header"]) == (1, 0)
    assert not [r for r in report if r["kind"] in ("error", "fatal", "miss")] and not node.warnings


def test_host_rois_older_depth_frames_are_dropped_and_a_lost_frame_is_a_miss():
    depth = scene_depth()
    frames = [DepthFrame(depth, s) for s in (1, 2, 3, 5, 6, 7)]  # 4 lost on the camera
    requests = host_requests((320, 200), [(s, feature_uv(seed=s)) for s in (3, 4, 5, 7)])
    node = run_device_script("host", frames, requests)
    assert [p.getSequenceNum() for p in node.outputs["points"].sent] == [3, 5, 7]
    misses = [r for r in statuses(node) if r["kind"] == "miss"]
    assert len(misses) == 1 and misses[0]["misses"] == 1 and "request 4" in misses[0]["text"]
    # One config per depth frame, in lockstep
    assert len(node.slc.configs) == 3


def test_host_rois_a_slc_answer_for_another_frame_is_reported():
    depth = scene_depth()
    node = run_device_script("host", [DepthFrame(depth, 1)], host_requests((320, 200), [(1, feature_uv())]),
                             out_of_order=True)
    errors = [r for r in statuses(node) if r["kind"] == "error"]
    assert errors and "order" in errors[0]["text"]
    # Tagged with the frame the SLC says it used, so the Pi's pairing drops it instead of using it for frame 1
    assert node.outputs["points"].sent[0].getSequenceNum() == 2


def test_host_rois_progress_and_stats_reports():
    t = [0.0]

    def clock():
        t[0] += 0.004
        return t[0]

    depth = scene_depth(seed=3)
    n = depth_points.STATS_EVERY
    node = run_device_script("host", [DepthFrame(depth, s) for s in range(1, n + 1)],
                             host_requests((320, 200), [(s, feature_uv(5)) for s in range(1, n + 1)]), clock=clock)
    stats = [r for r in statuses(node) if r["kind"] == "stats"]
    assert len(stats) == 1 and stats[0]["sent"] == n and stats[0]["size"] == [320, 200]
    assert stats[0]["slc_ms_mean"] > 0 and stats[0]["build_ms_mean"] >= 0


def stepping_clock(step=2.0):
    """A clock that moves past REPORT_INTERVAL at each read: every report goes out."""
    t = [0.0]

    def clock():
        t[0] += step
        return t[0]
    return clock


def test_host_rois_a_lost_tag_or_config_is_reported_and_costs_only_its_frame():
    depth = scene_depth()
    uv = {s: feature_uv(seed=s) for s in range(1, 6)}
    tag1, config1, tag2, config2, tag3, config3, tag4, config4, tag5, config5 = host_requests(
        (320, 200), [(s, uv[s]) for s in range(1, 6)])
    # 2: its tag lost; 3: its config lost; 4 and 5 arrive whole
    requests = [tag1, config1, config2, tag3, tag4, config4, tag5, config5]
    node = run_device_script("host", [DepthFrame(depth, s) for s in range(1, 6)], requests, clock=stepping_clock())
    points = node.outputs["points"].sent
    assert [p.getSequenceNum() for p in points] == [1, 4, 5]
    for s, p in zip((1, 4, 5), points):
        assert depth_points.resolve(depth_points.decode(p), uv[s], IMAGE, (320, 200), s)[1] is None
    errors = [r["text"] for r in statuses(node) if r["kind"] == "error"]
    assert len(errors) == 2 and errors[0].startswith("pairing:") and "without a tag" in errors[0]
    assert "no config came after the tag of frame 3" in errors[1]


def test_host_rois_requests_behind_their_frames_find_them_in_the_depth_buffer():
    depths = {s: scene_depth(seed=s) for s in range(1, 12)}
    uv = {s: feature_uv(seed=s) for s in range(1, 12)}

    def request(s):
        return [("rois", m) for m in host_requests((320, 200), [(s, uv[s])])]

    buffer = depth_points.DEPTH_BUFFER
    first = list(range(1, buffer + 1))
    # The Pi fetches `buffer` features messages at once and requests them all: their depth frames are already
    # queued on the camera. Then buffer + 2 frames come before the next request (the Pi stalled): the oldest two
    # are dropped from the queue, so the request for one of them misses, and the next one still pairs
    late = list(range(buffer + 1, 2 * buffer + 3))
    events = [("depth", DepthFrame(depths[s], s)) for s in first] + [e for s in first for e in request(s)]
    events += [("depth", DepthFrame(depths[s], s)) for s in late]
    events += request(late[0]) + request(late[-2])
    node = run_device_script("host", events=events)
    points = node.outputs["points"].sent
    assert [p.getSequenceNum() for p in points] == first + [late[-2]]
    for p in points:
        s = p.getSequenceNum()
        np.testing.assert_array_equal(depth_points.decode(p), expected_rows(depths[s], uv[s]))
    misses = [r for r in statuses(node) if r["kind"] == "miss"]
    assert len(misses) == 1 and f"request {late[0]}," in misses[0]["text"]
    assert [f.getSequenceNum() for f in node.inputs["depth"].dropped] == late[:2]


# --- the camera Script, rois "camera" -----------------------------------------------------------------------

def test_camera_rois_build_the_same_windows_as_the_host():
    depths = {s: scene_depth(seed=s) for s in (1, 2, 3)}
    uvs = {s: feature_uv(seed=10 + s) for s in (1, 2, 3)}
    node = run_device_script("camera", [DepthFrame(depths[s], s) for s in (1, 2, 3)],
                             [Features(uvs[s], s) for s in (1, 2, 3)])
    points = node.outputs["points"].sent
    assert [p.getSequenceNum() for p in points] == [1, 2, 3]
    for s, config, p in zip((1, 2, 3), node.slc.configs, points):
        windows = [[r.roi.x, r.roi.y, r.roi.width, r.roi.height] for r in config.rois]
        assert windows == depth_points.all_windows(uvs[s], IMAGE, (320, 200)).tolist()
        assert not any(r.roi.normalized for r in config.rois) and config.getSequenceNum() == s
        np.testing.assert_array_equal(depth_points.decode(p), expected_rows(depths[s], uvs[s]))
        # The Pi needs nothing but the message: the size marker gives the depth frame size
        result, problem = depth_points.resolve(depth_points.decode(p), uvs[s], IMAGE, None, s)
        assert problem is None and result.depth_size == (320, 200)


def test_camera_rois_fall_back_to_the_other_rect_constructors():
    class FourArgRect(Rect):
        def __init__(self, *args):
            if len(args) == 5:
                raise TypeError("no 5-argument Rect")
            super().__init__(*args)

    depth = scene_depth()
    uv = feature_uv(8)
    node = SimpleNamespace(warnings=[])
    node.warn = node.warnings.append
    slc = FakeSLC(node)
    script_node(node, {"depth": FakeInput([DepthFrame(depth, 1)]), "features": FakeInput([Features(uv, 1)]),
                       "spatial": FakeInput(), "passthrough": FakeInput()},
                {"config": FakeOutput(slc.on_config), depth_points.SLC_DEPTH_OUT: FakeOutput(slc.on_depth),
                 "points": FakeOutput(), "status": FakeOutput()})
    with pytest.raises(Done):
        exec(compile(depth_points.device_script("camera", IMAGE, (199, 15001)), "s", "exec"),
             {"node": node, **SCRIPT_GLOBALS, "Rect": FourArgRect})
    # Rect(x, y, w, h): only a 1x1 rect at the origin would read as normalized, and no window is one
    (points,) = node.outputs["points"].sent
    assert not any(r.roi.normalized for r in slc.configs[0].rois)
    assert depth_points.resolve(depth_points.decode(points), uv, IMAGE, None, 1)[1] is None


# --- the Pi: resolve and the receiver -------------------------------------------------------------------------

def test_resolve_checks_count_rois_and_scales_a_normalized_echo():
    depth = scene_depth()
    uv = feature_uv(10)
    rows = expected_rows(depth, uv)
    points, problem = depth_points.resolve(rows, uv, IMAGE, (320, 200), 7)
    assert problem is None and points.sequence == 7 and len(points.depth_mm) == len(uv)
    assert points.center["source"] == "depth_points" and points.center["x"] == 160
    assert depth_points.SAMPLES_PER_WINDOW == 25 and (points.samples == 25).all() and points.other_algorithm == 0
    np.testing.assert_array_equal(points.depth_mm, rows[:len(uv), depth_points.DEPTH_COLUMNS[depth_points.AVERAGE]])
    assert points.center["algorithm"] == "average"
    # A firmware that computed MEDIAN instead (only depthMedian filled): taken, with MEDIAN's 9 samples, counted
    median_rows = rows.copy()
    median_rows[:, 4], median_rows[:, 5] = rows[:, 5] + 1.0, 0.0
    median_rows[rows[:, 5] <= 0, 4] = 0.0
    other, problem = depth_points.resolve(median_rows, uv, IMAGE, (320, 200), 7)
    filled = rows[:len(uv), 5] > 0
    assert problem is None and other.other_algorithm == int(filled.sum())
    np.testing.assert_array_equal(other.depth_mm[filled], rows[:len(uv), 5][filled] + 1.0)
    assert (other.samples[filled] == 9).all() and (other.samples[~filled] == 25).all()
    _, problem = depth_points.resolve(rows[:-1], uv, IMAGE, (320, 200))
    assert problem[0] == "count"
    moved = uv + [10.0, 0.0]
    _, problem = depth_points.resolve(rows, moved, IMAGE, (320, 200))
    assert problem[0] == "roi" and "differ" in problem[1]
    normalized = rows.copy()
    normalized[:, [0, 2]] /= 320.0
    normalized[:, [1, 3]] /= 200.0
    assert depth_points.resolve(normalized, uv, IMAGE, (320, 200))[1] is None
    # rois "camera" with a normalized size marker: the size is unknown
    assert depth_points.resolve(normalized, uv, IMAGE, None)[1][0] == "size"


class HostQueue:
    """The Pi's points queue: get(timeout) advances the fake clock by the timeout when empty."""

    def __init__(self, clock):
        self.items, self.clock = [], clock

    def tryGetAll(self):
        items, self.items = self.items, []
        return items

    def get(self, timeout):
        if self.items:
            return self.items.pop(0)
        self.clock.t += timeout.total_seconds()
        return None


def make_receiver(roi_source="host", wait_s=0.3, fps=10):
    clock = SimpleNamespace(t=0.0)
    queue = HostQueue(clock)
    sent = []
    logs = []
    receiver = depth_points.DepthPointsReceiver(
        queue, IMAGE, roi_source, wait_s, send=sent.append,
        build_config=lambda windows, limits, sequence: depth_points.make_config(FAKE_DAI, windows, limits, sequence),
        build_tag=lambda sequence: depth_points.make_tag(FAKE_DAI, sequence), thresholds=(199, 15001),
        frame_period_s=1.0 / fps, clock=lambda: clock.t, wall_clock=lambda: 1000.0, log=logs.append)
    return receiver, queue, sent, clock, logs


def test_receiver_pairs_by_sequence_and_validates_each_frame():
    receiver, queue, sent, clock, logs = make_receiver()
    uv = feature_uv(20)
    assert receiver.request(1, uv) is False  # the depth size is not known yet
    assert receiver.take(1, uv, 100.1, requested=False) == (None, "not_requested")
    receiver.on_status(b"size\t0\t0\t0\t320 200")
    depth = scene_depth()
    for sequence in (2, 3):
        assert receiver.request(sequence, uv)
        # The tag goes first, on the same queue, then the config
        tag, config = sent[-2:]
        assert isinstance(tag, HostBuffer) and tag.getSequenceNum() == sequence and isinstance(config, Config)
        queue.items.append(answer(config, depth, 100.0 + sequence / 10))
        points, problem = receiver.take(sequence, uv, 100.0 + sequence / 10)
        assert problem is None and points.sequence == sequence
        np.testing.assert_array_equal(points.depth_mm, expected_rows(depth, uv)[:len(uv), 5])
    status = receiver.status()
    assert status["frames"] == 3 and status["paired"] == 2 and status["missing_by_reason"] == {"not_requested": 1}
    assert status["depth_size"] == [320, 200] and status["configs_sent"] == 2 and status["feature_gaps"] == 0
    assert len(sent) == 4


def test_receiver_takes_requests_made_ahead_in_order():
    receiver, queue, sent, clock, logs = make_receiver()
    receiver.on_status(b"size\t0\t0\t0\t320 200")
    depth = scene_depth()
    uv = {s: feature_uv(seed=s) for s in range(1, 1 + depth_points.REQUESTS_AHEAD)}
    for s in uv:
        assert receiver.request(s, uv[s])
    for s in uv:
        queue.items.append(answer(sent[2 * s - 1], depth, 100.0 + s / 10))
    for s in uv:
        points, problem = receiver.take(s, uv[s], 100.0 + s / 10)
        assert problem is None and points.sequence == s
    assert receiver.status()["paired"] == depth_points.REQUESTS_AHEAD


@pytest.mark.parametrize("fps", [10, 15, 20, 30])
def test_receiver_wait_and_timestamp_limits_follow_the_frame_period(fps):
    wait = depth_points.wait_s(fps)
    assert wait == pytest.approx(max(depth_points.WAIT_MIN_S, depth_points.WAIT_PERIODS / fps))
    assert depth_points.wait_s(fps, depth_points.ROIS_CAMERA) == pytest.approx(wait + depth_points.CAMERA_BUILD_S)
    # The camera keeps a frame long enough for a request that comes as late as the Pi waits
    assert depth_points.DEPTH_BUFFER >= math.ceil(wait * fps) + 1
    receiver, queue, sent, clock, logs = make_receiver(wait_s=wait, fps=fps)
    receiver.on_status(b"size\t0\t0\t0\t320 200")
    depth = scene_depth()
    uv = feature_uv(20)
    period = 1.0 / fps
    receiver.request(1, uv)
    queue.items.append(answer(sent[-1], depth, 100.0 + 0.4 * period))
    assert receiver.take(1, uv, 100.0)[1] is None
    receiver.request(2, uv)
    queue.items.append(answer(sent[-1], depth, 100.0 + period + 0.6 * period))
    assert receiver.take(2, uv, 100.0 + period)[1] == "timestamp"
    receiver.request(3, uv)
    before = clock.t
    assert receiver.take(3, uv, 100.0 + 2 * period) == (None, "timeout")
    assert clock.t - before == pytest.approx(wait, abs=1e-6)
    assert receiver.status()["wait_limit_ms"] == round(wait * 1000.0, 1)


def test_wait_limits_scale_with_the_rate():
    assert depth_points.wait_s(10) == pytest.approx(0.25)
    assert depth_points.wait_s(30) == pytest.approx(2.5 / 30)
    assert depth_points.wait_s(100) == depth_points.WAIT_MIN_S
    assert depth_points.wait_s(10, depth_points.ROIS_CAMERA) == pytest.approx(0.4)


def test_receiver_never_uses_another_frames_depth():
    receiver, queue, sent, clock, logs = make_receiver()
    receiver.on_status(b"size\t0\t0\t0\t320 200")
    depth = scene_depth()
    uv = feature_uv(20)
    # Points for frame 5 arrive while the Pi asks for frame 4: frame 4's are gone
    receiver.request(4, uv)
    receiver.request(5, uv)
    queue.items.append(answer(sent[-1], depth, 100.5))
    assert receiver.take(4, uv, 100.4) == (None, "dropped")
    points, problem = receiver.take(5, uv, 100.5)
    assert problem is None
    # The ROIs of frame 6 do not match frame 6's features (e.g. a config built for other features)
    receiver.request(6, uv + 30.0)
    queue.items.append(answer(sent[-1], depth, 100.6))
    assert receiver.take(6, uv, 100.6)[1] == "roi"
    # A depth frame far from the features' time
    receiver.request(7, uv)
    queue.items.append(answer(sent[-1], depth, 100.0))
    assert receiver.take(7, uv, 100.7)[1] == "timestamp"
    # Nothing at all: a timeout, and after DEAD_AFTER of them the Pi stops waiting
    for sequence in range(8, 8 + depth_points.DEAD_AFTER):
        receiver.request(sequence, uv)
        assert receiver.take(sequence, uv, 100.0 + sequence / 10) == (None, "timeout")
    before = clock.t
    assert receiver.take(20, uv, 102.0) == (None, "timeout") and clock.t == before
    status = receiver.status()
    assert status["waiting"] is False and status["missing_by_reason"]["timeout"] == depth_points.DEAD_AFTER + 1
    assert status["last_problem"]["reason"] == "timeout" and status["feature_gaps"] == 1
    assert status["missing_by_reason"]["dropped"] == 1 and status["missing_by_reason"]["roi"] == 1
    # Points coming back revive it
    receiver.request(21, uv)
    queue.items.append(answer(sent[-1], depth, 102.1))
    assert receiver.take(21, uv, 102.1)[1] is None and receiver.status()["waiting"] is True


def test_receiver_status_shape_and_device_reports():
    receiver, queue, sent, clock, logs = make_receiver("camera")
    receiver.on_status(b"stats\t0\t100\t2\t80.5 120.0 4.1 9.9 320 200")
    receiver.on_status(b"error\t3\t100\t2\tbuild: ValueError: x")
    receiver.on_status(b"garbage")
    status = receiver.status()
    assert set(status) == {
        "transport", "rois", "frames", "paired", "paired_percent", "missing", "missing_by_reason", "last_problem",
        "points_received", "orphan_points", "points_gaps", "points_gap_frames", "feature_gaps",
        "feature_gap_frames", "waiting", "wait_ms_mean", "wait_ms_max", "wait_limit_ms", "valid_percent",
        "algorithm", "other_algorithm_rois", "pixel_count_missing_frames", "timestamp_diff_ms_max",
        "configs_sent", "config_errors", "depth_size", "window_px", "samples_per_window", "depth_buffer_frames",
        "device"}
    assert status["transport"] == "points" and status["rois"] == "camera" and status["window_px"] == 5
    assert status["algorithm"] == "average" and status["samples_per_window"] == 25
    assert status["depth_buffer_frames"] == depth_points.DEPTH_BUFFER and status["device"]["config_sequence"] is None
    receiver.on_status(b"tagging\t3\t100\t2\t7 0")
    assert receiver.status()["device"]["config_sequence"] == {"tag": 7, "config": 0, "survives": False}
    assert any("lost on the link" in line for line in logs)
    device = status["device"]
    assert device["errors"] == 3 and device["misses"] == 2 and device["stats"]["build_ms_mean"] == 80.5
    assert device["last_error"]["text"] == "build: ValueError: x" and not device["fatal"]
    assert any("unreadable" in line for line in logs)
    # rois "camera": the Pi sends nothing up
    assert receiver.request(1, feature_uv(3)) is True and sent == []


def test_points_matcher_counts_gaps_and_orphans():
    clock = SimpleNamespace(t=0.0)
    queue = HostQueue(clock)
    matcher = depth_points.PointsMatcher(queue, 0.3, clock=lambda: clock.t)
    depth = scene_depth()
    config = depth_points.make_config(FAKE_DAI, depth_points.all_windows(feature_uv(3), IMAGE, (320, 200)),
                                      (199, 15001), 0)
    for sequence in (1, 2, 5, 6):
        config.setSequenceNum(sequence)
        queue.items.append(answer(config, depth, sequence))
    entry, reason = matcher.take(2)
    assert reason is None and entry.msg.getSequenceNum() == 2
    assert matcher.orphans == 1  # frame 1's points: its features never came
    assert matcher.gaps == 1 and matcher.gap_frames == 2


# --- the on-demand full depth gate ------------------------------------------------------------------------

def test_full_depth_gate_sends_the_finest_need_on_change_and_periodically():
    t = [0.0]
    sent, logs = [], []
    gate = depth_points.FullDepthGate(sent.append, clock=lambda: t[0], log=logs.append, resend_interval=5.0)
    assert gate.update({"depth_probe": 0, "host_jpeg": 0}) == 0 and sent == [0]
    assert gate.update({"depth_probe": 0, "host_jpeg": 2}) == 2 and sent == [0, 2]
    assert gate.update({"depth_probe": 1, "host_jpeg": 2}) == 1 and sent == [0, 2, 1]
    assert gate.update({"depth_probe": 1, "host_jpeg": 2}) == 1 and sent == [0, 2, 1]
    t[0] = 6.0
    gate.update({"depth_probe": 1, "host_jpeg": 2})
    assert sent == [0, 2, 1, 1]
    gate.on_frame(1000)
    status = gate.status()
    assert status["on"] and status["every"] == 1 and status["reasons"] == ["depth_probe", "host_jpeg"]
    assert status["frames"] == 1 and status["bytes"] == 1000
    assert gate.update({}) == 0 and not gate.status()["on"] and "off" in logs[-1]


def run_gate(depth_frames, controls):
    node = script_node(SimpleNamespace(), {"depth": FakeInput(depth_frames), "control": FakeInput(controls)},
                       {depth_points.GATE_OUT: FakeOutput()})
    with pytest.raises(Done):
        exec(compile(depth_points.gate_script(), "gate", "exec"), {"node": node})
    return [frame.getSequenceNum() for frame in node.outputs[depth_points.GATE_OUT].sent], node


def test_gate_script_forwards_every_nth_frame_only_while_on():
    control = lambda every: SimpleNamespace(getSequenceNum=lambda: every)  # noqa: E731
    depth = scene_depth((4, 4))
    # Off: it waits on the control and takes no depth frame
    forwarded, node = run_gate([DepthFrame(depth, s) for s in range(1, 5)], [])
    assert forwarded == [] and len(node.inputs["depth"].items) == 4
    # The first frame after switching on goes at once, then every n-th
    forwarded, _ = run_gate([DepthFrame(depth, s) for s in range(1, 8)], [control(2)])
    assert forwarded == [1, 3, 5, 7]
    forwarded, _ = run_gate([DepthFrame(depth, s) for s in range(1, 4)], [control(1)])
    assert forwarded == [1, 2, 3]


# --- the host's use of the points ---------------------------------------------------------------------------

def test_depth_from_points_applies_the_lookup_rules():
    params = PnPParams(min_depth_m=0.5, max_depth_m=5.0, depth_correction=1.1, refractive_index=1.333,
                       min_valid_depth_fraction=0.3)
    z = depth_from_points([1000, 400, 6000, 2000, 0, 2000], [9, 9, 9, 2, 0, 3], [9] * 6, params)
    assert z[0] == pytest.approx(1.0 * 1.1 * 1.333)
    # Out of range, too few valid samples (2 < ceil(0.3 * 9) = 3), no depth: NaN
    assert np.isnan(z[1:5]).all() and z[5] == pytest.approx(2.0 * 1.1 * 1.333)
    # No pixel counts from the camera: the fraction rule is skipped
    assert not np.isnan(depth_from_points([2000], [0], [9], params, pixel_count_reported=False)[0])


def test_points_and_frame_lookup_agree_on_smooth_depth():
    """On a smooth surface the camera's 5x5 average and the Pi's 5x5 median give the same depth."""
    yy, xx = np.mgrid[0:400, 0:640]
    depth = (1500 + xx * 0.5 + yy * 0.25).astype(np.uint16)
    uv = np.random.default_rng(4).uniform((10, 10), (630, 390), (50, 2))
    params = PnPParams(refractive_index=1.333)
    host = lookup_depth(depth, uv, params)
    config = depth_points.make_config(FAKE_DAI, depth_points.all_windows(uv, IMAGE, IMAGE), (199, 15001), 1)
    points, problem = depth_points.resolve(depth_points.decode(slc_compute(config, depth)), uv, IMAGE, IMAGE, 1)
    assert problem is None
    camera = depth_from_points(points.depth_mm, points.pixels, points.samples, params)
    np.testing.assert_allclose(camera, host, atol=0.002)


def test_status_parsing():
    assert depth_points.parse_status(b"size\t0\t0\t0\t320 200")["size"] == [320, 200]
    assert depth_points.parse_status(b"miss\t0\t5\t1\trequest 4, next depth frame 5")["misses"] == 1
    assert depth_points.parse_status(b"stats\t0\t100\t0\t1 2 3 4 5 6")["slc_ms_max"] == 4.0
    tagging = depth_points.parse_status(b"tagging\t0\t0\t0\t12 0")
    assert (tagging["tag"], tagging["header"]) == (12, 0)
    assert depth_points.parse_status(b"tagging\t0\t0\t0\t12") is None
    assert depth_points.parse_status(b"nonsense") is None
