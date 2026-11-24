"""
Depth at the tracked features only (depth_transport "points"): the camera computes it, the full depth frame
stays on the camera.

Pipeline (pnp_vio.build_depth_points):

    FeatureTracker.outputFeatures --------------------------------------------> host 'features' queue
          |  (depth_points_rois "camera" only)                          host 'rois' queue (rois "host":
          v                                                             tag + config per frame)
    Script 'features' / 'rois' --+                                                v
    StereoDepth.depth -> Script 'depth' --> SLC.inputConfig + SLC.inputDepth (blocking, lockstep)
                         Script 'spatial' <- SLC.out, Script 'passthrough' <- SLC.passthroughDepth
                         Script 'points' ---------------------------------------> host 'points' queue
                         Script 'status' ---------------------------------------> host 'points_status'

One SpatialLocationCalculator (SLC) ROI per feature: the (2 * WINDOW_RADIUS + 1)^2 window around the feature
on the depth grid (clamped to the frame), ALGORITHM, depth thresholds = the PnP depth range (exclusive bounds, so
min_depth_m..max_depth_m are taken), then EXTRA_ROIS: the centre patch and the centre pixel (the centre depth
readout of /api/status) and a 1x1 marker at the bottom-right pixel (the depth frame size, so the host can check
every message on its own). ALGORITHM is AVERAGE: the firmware computes MEDIAN / MODE only on a depth frame
straight from StereoDepth ("Depth frame must be originating from StereoDepth node! Defaulting to 'AVERAGE/MEAN'"),
and ours reaches the SLC through the pairing Script (test16b: every ROI came back as an average). AVERAGE uses
every pixel (step 1): SAMPLES_PER_WINDOW = 25 for a 5x5 window.

depth_points_rois (ROI_SOURCES): "host" (default) - the Pi builds the ROI config from the features it receives
anyway (make_config) and sends it up right after a tag (make_tag: a plain Buffer carrying the features' sequence
number); "camera" - the Script builds it from the TrackedFeatures (slow: 111 ms of Leon CPU per frame of ~255
features on test16b, more than a frame at 10 fps).

The tag exists because the camera reads 0 as the sequence number of every SpatialLocationCalculatorConfig the
host sends (test16b), while a plain Buffer's sequence number survives the link (the gates' controls since
test15). Both go on the same queue, tag first, so they arrive in order; the Script pairs a config with the tag
right before it and reports a tag without a config or a config without a tag.

Sync: the Script pairs each request (the tagged config, or the features) with the depth frame of the same
sequence number from the last DEPTH_BUFFER frames it holds, sends both to the SLC and waits for its output and
passthrough depth before the next request, then tags the output with the passthrough frame's sequence number and
timestamp. The host pairs points and features by sequence number (PointsMatcher) and checks the echoed ROIs
against the windows of that features message (resolve): anything else is missing depth for that frame (counted,
reason recorded), never stale depth. That check, not the tag, is what makes the depth right: the output's
sequence number is the depth frame the SLC computed on, its ROIs are the windows it computed at, and the host
takes it only when both are this frame's. A mispaired request can cost a frame, never give it wrong depth.

Rate: the host's wait (wait_s) and the Script's depth buffer are frame periods (WAIT_PERIODS), so nothing here
is tied to 10 fps. The host may request several frames before it takes the first one (PointsSource in
pnp_vio requests every features message as it fetches it); the SLC still has one config and one frame at a
time.

Everything here is depthai-free (the host config builder takes the dai module), so it runs in the tests.
"""

import math
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import timedelta
from typing import Optional

import numpy as np

POINTS = "points"
FULL_FRAME = "full_frame"
TRANSPORTS = (POINTS, FULL_FRAME)
DEFAULT_TRANSPORT = POINTS
ROIS_HOST = "host"
ROIS_CAMERA = "camera"
ROI_SOURCES = (ROIS_HOST, ROIS_CAMERA)
DEFAULT_ROI_SOURCE = ROIS_HOST
# A Script's inputs and outputs are one key space in depthai (group "io"): none may share a name, and both
# Scripts take stereo.depth on their input 'depth'
SLC_DEPTH_OUT = "slc_depth"  # device_script: the depth frame to the SLC
GATE_OUT = "full_depth"  # gate_script: the forwarded depth frame

# pnp_odometry.PnPParams.depth_window: the same 5x5 window as the host lookup
WINDOW_RADIUS = 2
# SpatialLocationCalculatorConfigData stepSize AUTO (SpatialLocationCalculatorConfig.hpp): 1 for AVERAGE, 2 for
# MEDIAN; stepSize is not settable from Python
AVERAGE, MEDIAN = "AVERAGE", "MEDIAN"
STEPS = {AVERAGE: 1, MEDIAN: 2}
ALGORITHM = AVERAGE
CENTER_PATCH = 5
# centre patch, centre pixel, size marker
EXTRA_ROIS = 3
# Thresholds of the centre ROIs: every non-zero depth counts (the readout shows the raw device depth)
CENTER_THRESHOLDS = (0, 65535)
# Echoed ROI vs expected window, px (the firmware stores floats)
ROI_TOLERANCE_PX = 0.5
REPORT_INTERVAL_S = 1.0
REPORT_MAX_CHARS = 400
STATS_EVERY = 100
# The host waits this many frame periods for a frame's points (at least WAIT_MIN_S; rois "camera" adds
# CAMERA_BUILD_S, its Script's build time)
WAIT_PERIODS = 2.5
WAIT_MIN_S = 0.05
CAMERA_BUILD_S = 0.15
# The Script holds the newest DEPTH_BUFFER depth frames (its non-blocking input queue): a request that comes up
# to WAIT_PERIODS after its frame still finds it; one coming later is past the host's wait anyway
DEPTH_BUFFER = int(math.ceil(WAIT_PERIODS)) + 1
# StereoDepth pool frames: the buffer, the frame the Script keeps for the next request, the SLC's input and
# passthrough, and the one being handed over
POOL_FRAMES = DEPTH_BUFFER + 4
# Requests the host may have in flight (PointsSource fetching ahead) and the 'rois' queues for them (tag + config)
REQUESTS_AHEAD = 4
ROIS_QUEUE = 2 * REQUESTS_AHEAD
# Points paired by sequence number are kept this many frames for a late features message
KEEP_POINTS = 16
# After this many timeouts in a row the host stops waiting (only takes what is there) until points come back
DEAD_AFTER = 5
# Points older than this many frames behind the newest are orphans: their features never came
GAP_TOLERANCE = 1


def samples_per_window(radius=WINDOW_RADIUS, step=STEPS[ALGORITHM]):
    return int(math.ceil((2 * radius + 1) / step)) ** 2


SAMPLES_PER_WINDOW = samples_per_window()


def wait_s(fps, roi_source=ROIS_HOST):
    """How long the host waits for a frame's points: WAIT_PERIODS frame periods (at least WAIT_MIN_S)."""
    wait = max(WAIT_MIN_S, WAIT_PERIODS / max(float(fps), 1.0))
    return wait + (CAMERA_BUILD_S if roi_source == ROIS_CAMERA else 0.0)


def thresholds_mm(min_depth_m, max_depth_m):
    """SLC depth thresholds (exclusive: lower <= value or value >= upper are not taken) for the inclusive range."""
    lower = max(int(round(min_depth_m * 1000.0)) - 1, 0)
    upper = min(int(round(max_depth_m * 1000.0)) + 1, 65535)
    return lower, upper


def feature_windows(uv, image_size, depth_size, radius=WINDOW_RADIUS):
    """
    (N, 4) int64 x, y, width, height of each feature's window on the depth grid: the pixel at
    floor(u * depth_w / image_w + 0.5) (the camera Script's int(x + 0.5)), clamped into the frame, +-radius,
    clipped at the frame edges (at least radius + 1 px wide).
    """
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    width, height = image_size
    depth_w, depth_h = depth_size
    x = np.clip(np.floor(uv[:, 0] * (depth_w / float(width)) + 0.5), 0, depth_w - 1).astype(np.int64)
    y = np.clip(np.floor(uv[:, 1] * (depth_h / float(height)) + 0.5), 0, depth_h - 1).astype(np.int64)
    x0, y0 = np.maximum(x - radius, 0), np.maximum(y - radius, 0)
    x1, y1 = np.minimum(x + radius + 1, depth_w), np.minimum(y + radius + 1, depth_h)
    return np.stack([x0, y0, x1 - x0, y1 - y0], axis=1)


def extra_windows(depth_size, patch=CENTER_PATCH):
    """(EXTRA_ROIS, 4): the centre patch, the centre pixel, the size marker (bottom-right pixel)."""
    depth_w, depth_h = depth_size
    cx, cy, r = depth_w // 2, depth_h // 2, patch // 2
    x0, y0 = max(cx - r, 0), max(cy - r, 0)
    x1, y1 = min(cx + r + 1, depth_w), min(cy + r + 1, depth_h)
    return np.array([[x0, y0, x1 - x0, y1 - y0], [cx, cy, 1, 1], [depth_w - 1, depth_h - 1, 1, 1]], dtype=np.int64)


def all_windows(uv, image_size, depth_size, radius=WINDOW_RADIUS):
    return np.concatenate([feature_windows(uv, image_size, depth_size, radius), extra_windows(depth_size)])


# --- host: the ROI config (depth_points_rois "host") ------------------------------------------------------

def make_tag(dai, sequence):
    """The plain Buffer sent right before a frame's config: its sequence number is the features' (it survives)."""
    tag = dai.Buffer()
    tag.setSequenceNum(int(sequence))
    return tag


def make_config(dai, windows, thresholds, sequence, center_thresholds=CENTER_THRESHOLDS):
    """
    A dai.SpatialLocationCalculatorConfig with one ALGORITHM ROI per window (the last EXTRA_ROIS with
    center_thresholds). Rect(x, y, w, h, False): explicitly not normalized (a 1x1 rect at the origin would
    otherwise mean the whole frame, Rect.hpp isNormalized). It also carries the features' sequence number, which
    the camera does not get (it reads 0, test16b; make_tag carries it): the Script reports what it read.
    """
    config = dai.SpatialLocationCalculatorConfig()
    config.setSequenceNum(int(sequence))
    algorithm = getattr(dai.SpatialLocationCalculatorAlgorithm, ALGORITHM)
    rois = []
    for lower, upper in (thresholds, center_thresholds):
        roi = dai.SpatialLocationCalculatorConfigData()
        roi.depthThresholds.lowerThreshold = int(lower)
        roi.depthThresholds.upperThreshold = int(upper)
        roi.calculationAlgorithm = algorithm
        rois.append(roi)
    feature_roi, center_roi = rois
    rect, add = dai.Rect, config.addROI
    windows = np.asarray(windows).tolist()
    split = len(windows) - EXTRA_ROIS
    for x, y, w, h in windows[:split]:
        feature_roi.roi = rect(x, y, w, h, False)
        add(feature_roi)
    for x, y, w, h in windows[split:]:
        center_roi.roi = rect(x, y, w, h, False)
        add(center_roi)
    return config


# --- camera Script -----------------------------------------------------------------------------------------

def device_script(roi_source, image_size, thresholds, radius=WINDOW_RADIUS, stats_every=STATS_EVERY,
                  report_interval=REPORT_INTERVAL_S, center_thresholds=CENTER_THRESHOLDS, patch=CENTER_PATCH):
    """
    Script node source. Inputs: 'depth' (stereo.depth, non-blocking, DEPTH_BUFFER frames), 'rois' (the host's
    tag + config per frame, rois "host") or 'features' (FeatureTracker.outputFeatures, rois "camera"), 'spatial'
    (SLC.out) and 'passthrough' (SLC.passthroughDepth); outputs 'config', SLC_DEPTH_OUT (to the SLC), 'points' (the
    SLC's output, re-tagged) and 'status' (parse_status). Only the camera's Script bindings: Buffer(size), the SLC
    config types by their firmware names, Rect probed (Rect(x, y, w, h, False), Rect(x, y, w, h), Rect(Point2f,
    Size2f)).

    rois "host": a message without getConfigData is a tag (its sequence number is the frame's); a config takes
    the tag right before it. A second tag before a config, or a config with no tag, is reported ('error',
    "pairing") and skipped. The own sequence number of the first paired config with a non-zero tag is reported
    once ('tagging': the tag's and the config's), so the host can tell whether the config's number crossed the
    link.

    Per request: depth frames older than its sequence number are dropped; a newer one means its frame is gone
    (a miss: reported at most once per report_interval, the frame kept for the next request). It reports
    'size' when the depth frame size changes (first frame included, before any request with rois "host"),
    'sent' progress at most every report_interval, 'stats' (mean/max build and SLC ms) every stats_every, and
    errors like depth_stream.device_script.
    """
    width, height = image_size
    lower, upper = thresholds
    center_lower, center_upper = center_thresholds
    camera = roi_source == ROIS_CAMERA
    lines = [
        f"CAMERA_ROIS = {camera!r}",
        f"IMAGE_W = {float(width)!r}",
        f"IMAGE_H = {float(height)!r}",
        f"R = {int(radius)}",
        f"PATCH_R = {int(patch) // 2}",
        f"LOWER = {int(lower)}",
        f"UPPER = {int(upper)}",
        f"CENTER_LOWER = {int(center_lower)}",
        f"CENTER_UPPER = {int(center_upper)}",
        f"STATS_EVERY = {int(stats_every)}",
        f"REPORT_INTERVAL = {float(report_interval)!r}",
        f"REPORT_MAX = {int(REPORT_MAX_CHARS)}",
        "",
        "try:",
        "    import time",
        "    clock = time.monotonic",
        "except Exception:",
        "    clock = None",
        "",
        "errors = 0",
        "sent = 0",
        "misses = 0",
        "reported = None",
        "progressed = None",
        "missed_at = None",
        "width = 0",
        "height = 0",
        "pending = None",
        "busy_build = 0.0",
        "busy_slc = 0.0",
        "worst_build = 0.0",
        "worst_slc = 0.0",
        "",
        "def report(kind, text):",
        "    try:",
        "        data = (kind + '\\t' + str(errors) + '\\t' + str(sent) + '\\t' + str(misses) + '\\t'",
        "                + text)[:REPORT_MAX].encode()",
        "        message = Buffer(len(data))",
        "        message.setData(data)",
        "        node.outputs['status'].send(message)",
        "    except Exception as e:",
        "        node.warn('depth points: status not sent: ' + str(e))",
        "",
        "def failed(where, e, kind='error'):",
        "    global errors, reported",
        "    errors += 1",
        "    text = where + ': ' + type(e).__name__ + ': ' + str(e)",
        "    if errors == 1 or errors % 100 == 0 or kind != 'error':",
        "        node.warn('depth points: ' + text + ' (' + str(errors) + ' errors)')",
        "    now = clock() if clock else None",
        "    if (kind != 'error' or reported is None or (now is None and errors % 100 == 0)",
        "            or (now is not None and now - reported >= REPORT_INTERVAL)):",
        "        reported = now if now is not None else 0.0",
        "        report(kind, text)",
        "",
        "def note_size(frame):",
        "    global width, height",
        "    w = frame.getWidth()",
        "    h = frame.getHeight()",
        "    if w != width or h != height:",
        "        width = w",
        "        height = h",
        "        report('size', str(w) + ' ' + str(h))",
        "",
        "def depth_for(sequence):",
        "    global pending",
        "    frame = pending",
        "    pending = None",
        "    while True:",
        "        if frame is None:",
        "            frame = node.inputs['depth'].get()",
        "        number = frame.getSequenceNum()",
        "        if number == sequence:",
        "            return frame",
        "        if number > sequence:",
        "            pending = frame",
        "            return None",
        "        frame = None",
        "",
        "def missed(sequence):",
        "    global misses, missed_at",
        "    misses += 1",
        "    now = clock() if clock else None",
        "    if (missed_at is None or (now is None and misses % 100 == 0)",
        "            or (now is not None and now - missed_at >= REPORT_INTERVAL)):",
        "        missed_at = now if now is not None else 0.0",
        "        report('miss', 'request ' + str(sequence) + ', next depth frame ' + str(pending.getSequenceNum()))",
        "",
    ]
    if not camera:
        lines += [
            "tag = None",
            "lost_tags = 0",
            "lost_configs = 0",
            "checked_header = False",
            "",
            "def next_request():",
            "    global tag, lost_tags, lost_configs, checked_header",
            "    while True:",
            "        message = node.inputs['rois'].get()",
            "        if not hasattr(message, 'getConfigData'):",
            "            if tag is not None:",
            "                lost_configs += 1",
            "                failed('pairing', ValueError('no config came after the tag of frame ' + str(tag) + ' ('",
            "                                             + str(lost_configs) + ' times)'))",
            "            tag = message.getSequenceNum()",
            "            continue",
            "        if tag is None:",
            "            lost_tags += 1",
            "            failed('pairing', ValueError('a config came without a tag (' + str(lost_tags) + ' times)'))",
            "            continue",
            "        sequence = tag",
            "        tag = None",
            "        if not checked_header and sequence != 0:",
            "            checked_header = True",
            "            report('tagging', str(sequence) + ' ' + str(message.getSequenceNum()))",
            "        return message, sequence",
            "",
        ]
    if camera:
        lines += [
            "make_rect = None",
            "",
            "def probe_rect():",
            "    global make_rect",
            "    for make in (lambda x, y, w, h: Rect(x, y, w, h, False), lambda x, y, w, h: Rect(x, y, w, h),",
            "                 lambda x, y, w, h: Rect(Point2f(x, y), Size2f(w, h))):",
            "        try:",
            "            make(1, 1, 3, 3)",
            "        except Exception:",
            "            continue",
            "        make_rect = make",
            "        return",
            "    raise RuntimeError('no Rect constructor')",
            "",
            "def roi_data(lower, upper):",
            "    thresholds = SpatialLocationCalculatorConfigThresholds()",
            "    thresholds.lowerThreshold = lower",
            "    thresholds.upperThreshold = upper",
            "    data = SpatialLocationCalculatorConfigData()",
            "    data.depthThresholds = thresholds",
            f"    data.calculationAlgorithm = SpatialLocationCalculatorAlgorithm.{ALGORITHM}",
            "    return data",
            "",
            "def build(features):",
            "    config = SpatialLocationCalculatorConfig()",
            "    add = config.addROI",
            "    rect = make_rect",
            "    roi = feature_roi",
            "    sx = width / IMAGE_W",
            "    sy = height / IMAGE_H",
            "    right = width - 1",
            "    bottom = height - 1",
            "    for f in features.trackedFeatures:",
            "        p = f.position",
            "        x = int(p.x * sx + 0.5)",
            "        y = int(p.y * sy + 0.5)",
            "        x = 0 if x < 0 else (right if x > right else x)",
            "        y = 0 if y < 0 else (bottom if y > bottom else y)",
            "        x0 = x - R if x > R else 0",
            "        y0 = y - R if y > R else 0",
            "        x1 = x + R + 1 if x + R + 1 < width else width",
            "        y1 = y + R + 1 if y + R + 1 < height else height",
            "        roi.roi = rect(x0, y0, x1 - x0, y1 - y0)",
            "        add(roi)",
            "    cx = width // 2",
            "    cy = height // 2",
            "    x0 = cx - PATCH_R if cx > PATCH_R else 0",
            "    y0 = cy - PATCH_R if cy > PATCH_R else 0",
            "    x1 = cx + PATCH_R + 1 if cx + PATCH_R + 1 < width else width",
            "    y1 = cy + PATCH_R + 1 if cy + PATCH_R + 1 < height else height",
            "    for x, y, w, h in ((x0, y0, x1 - x0, y1 - y0), (cx, cy, 1, 1), (right, bottom, 1, 1)):",
            "        center_roi.roi = rect(x, y, w, h)",
            "        add(center_roi)",
            "    config.setSequenceNum(features.getSequenceNum())",
            "    return config",
            "",
        ]
    lines += [
        "try:",
        *([
            "    probe_rect()",
            "    feature_roi = roi_data(LOWER, UPPER)",
            "    center_roi = roi_data(CENTER_LOWER, CENTER_UPPER)",
        ] if camera else [
            "    pending = node.inputs['depth'].get()",
            "    note_size(pending)",
        ]),
        "    while True:",
        *([
            "        request = node.inputs['features'].get()",
            "        sequence = request.getSequenceNum()",
        ] if camera else [
            "        request, sequence = next_request()",
        ]),
        "        frame = depth_for(sequence)",
        "        if frame is None:",
        "            request = None",
        "            missed(sequence)",
        "            continue",
        "        note_size(frame)",
        "        start = clock() if clock else 0.0",
        *([
            "        try:",
            "            config = build(request)",
            "        except Exception as e:",
            "            failed('build', e)",
            "            request = None",
            "            frame = None",
            "            continue",
        ] if camera else [
            "        config = request",
        ]),
        "        request = None",
        "        built = clock() if clock else 0.0",
        "        node.outputs['config'].send(config)",
        f"        node.outputs[{SLC_DEPTH_OUT!r}].send(frame)",
        "        config = None",
        "        frame = None",
        "        spatial = node.inputs['spatial'].get()",
        "        passthrough = node.inputs['passthrough'].get()",
        "        number = passthrough.getSequenceNum()",
        "        if number != sequence:",
        "            failed('order', ValueError('the SLC computed depth frame ' + str(number) + ' for request '",
        "                                       + str(sequence)))",
        "        try:",
        "            spatial.setSequenceNum(number)",
        "            spatial.setTimestamp(passthrough.getTimestamp())",
        "        except Exception as e:",
        "            failed('tag', e)",
        "        passthrough = None",
        "        node.outputs['points'].send(spatial)",
        "        spatial = None",
        "        sent += 1",
        "        if clock:",
        "            now = clock()",
        "            busy_build += built - start",
        "            busy_slc += now - built",
        "            worst_build = max(worst_build, built - start)",
        "            worst_slc = max(worst_slc, now - built)",
        "            if progressed is None or now - progressed >= REPORT_INTERVAL:",
        "                progressed = now",
        "                report('sent', '')",
        "            if sent % STATS_EVERY == 0:",
        "                text = (str(round(1000.0 * busy_build / STATS_EVERY, 2)) + ' '",
        "                        + str(round(1000.0 * worst_build, 2)) + ' '",
        "                        + str(round(1000.0 * busy_slc / STATS_EVERY, 2)) + ' '",
        "                        + str(round(1000.0 * worst_slc, 2)) + ' ' + str(width) + ' ' + str(height))",
        "                report('stats', text)",
        "                busy_build = 0.0",
        "                busy_slc = 0.0",
        "                worst_build = 0.0",
        "                worst_slc = 0.0",
        "        elif sent == 1 or sent % 10 == 0:",
        "            report('sent', '')",
        "except Exception as e:",
        "    failed('stopped', e, 'fatal')",
        "    raise",
    ]
    return "\n".join(lines)


def gate_script():
    """
    Script node source of the on-demand full depth gate (points mode): input 'depth' (stereo.depth,
    non-blocking size 1) and 'control' (Buffer, sequence number n: forward every n-th frame, 0: none);
    output GATE_OUT. While off it blocks on 'control', so it costs nothing per frame.
    """
    return "\n".join([
        "every = 0",
        "count = 0",
        "while True:",
        "    if every <= 0:",
        "        every = node.inputs['control'].get().getSequenceNum()",
        "        count = every - 1",
        "        continue",
        "    control = node.inputs['control'].tryGet()",
        "    while control is not None:",
        "        if control.getSequenceNum() != every:",
        "            every = control.getSequenceNum()",
        "            count = every - 1",
        "        control = node.inputs['control'].tryGet()",
        "    if every <= 0:",
        "        continue",
        "    frame = node.inputs['depth'].get()",
        "    count += 1",
        "    if count >= every:",
        "        count = 0",
        f"        node.outputs[{GATE_OUT!r}].send(frame)",
        "    frame = None",
    ])


def parse_status(data):
    """
    One 'status' message of device_script: kind, error count, points sent, misses, text (tab-separated).
    kind: 'size' (text: width height), 'sent', 'miss' (text: which request), 'stats' (text: build ms mean,
    max, SLC ms mean, max, width, height), 'tagging' (text: the tag's sequence number, the config's own), 'error',
    'fatal'. None if malformed.
    """
    try:
        kind, errors, sent, misses, text = bytes(data).decode("utf-8", "replace").split("\t", 4)
        status = {"kind": kind, "errors": int(errors), "sent": int(sent), "misses": int(misses), "text": text}
        if kind == "size":
            width, height = text.split()
            status["size"] = [int(width), int(height)]
        elif kind == "tagging":
            tag, header = text.split()
            status.update(tag=int(tag), header=int(header))
        elif kind == "stats":
            build_mean, build_max, slc_mean, slc_max, width, height = text.split()
            status.update(build_ms_mean=float(build_mean), build_ms_max=float(build_max),
                          slc_ms_mean=float(slc_mean), slc_ms_max=float(slc_max), size=[int(width), int(height)])
    except (TypeError, ValueError):
        return None
    return status


# --- host: decoding and checking the points -----------------------------------------------------------------

def decode(msg):
    """
    (N, 7) float64 rows of a SpatialLocationCalculatorData: echoed roi x, y, width, height, depthMedian,
    depthAverage, depthAveragePixelCount. The firmware fills only the computed algorithm's depth (the other is 0).
    """
    rows = []
    for location in msg.getSpatialLocations():
        roi = location.config.roi
        rows.append((roi.x, roi.y, roi.width, roi.height, location.depthMedian, location.depthAverage,
                     location.depthAveragePixelCount))
    return np.array(rows, dtype=np.float64).reshape(-1, 7)


@dataclass
class DepthPoints:
    """The camera's depth at one frame's features, checked against them."""
    sequence: int
    depth_mm: np.ndarray  # (N,) per feature: the SLC's ALGORITHM depth (0 = no valid sample)
    pixels: np.ndarray  # (N,) the SLC's depthAveragePixelCount
    samples: np.ndarray  # (N,) pixels the SLC samples in a full window (AVERAGE 25, MEDIAN 9)
    depth_size: tuple  # (width, height) of the depth grid
    center: Optional[dict] = None
    pixel_count_reported: bool = True
    other_algorithm: int = 0  # ROIs where only the other algorithm's depth came (a firmware that chose it)
    timestamp_s: Optional[float] = None


# decode's columns of each algorithm's depth
DEPTH_COLUMNS = {MEDIAN: 4, AVERAGE: 5}
OTHER_ALGORITHM = {AVERAGE: MEDIAN, MEDIAN: AVERAGE}


def _sampled(windows, step):
    w, h = windows[:, 2], windows[:, 3]
    return ((w + step - 1) // step) * ((h + step - 1) // step)


def _denormalized(rows, depth_size):
    """The echoed rects in pixels: a firmware that hands them back normalized gets them scaled."""
    rects = rows[:, :4].copy()
    if len(rects) and np.all(rects[:, 0] + rects[:, 2] <= 1.0 + 1e-6) and np.all(rects[:, 1] + rects[:, 3] <= 1.0 + 1e-6):
        rects *= np.array([depth_size[0], depth_size[1], depth_size[0], depth_size[1]], dtype=np.float64)
    return rects


def resolve(rows, uv, image_size, depth_size=None, sequence=None):
    """
    (DepthPoints, None) when the rows are exactly this frame's ROIs, else (None, (reason, text)). depth_size:
    the size the host built the ROIs for (rois "host"); None takes it from the size marker (rois "camera").
    """
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if len(rows) != len(uv) + EXTRA_ROIS:
        return None, ("count", f"{len(rows)} ROIs for {len(uv)} features + {EXTRA_ROIS}")
    marker = rows[-1, :4]
    if depth_size is None:
        if marker[0] + marker[2] <= 1.0 + 1e-6:
            return None, ("size", f"size marker {marker.tolist()} is not a pixel rect")
        depth_size = (int(round(marker[0])) + 1, int(round(marker[1])) + 1)
    expected = all_windows(uv, image_size, depth_size)
    echoed = _denormalized(rows, depth_size)
    off = np.abs(echoed - expected).max(axis=1) if len(expected) else np.zeros(0)
    bad = np.flatnonzero(off > ROI_TOLERANCE_PX)
    if bad.size:
        k = int(bad[0])
        return None, ("roi", f"{bad.size} ROIs differ from this frame's features, e.g. #{k}: "
                             f"{np.round(echoed[k], 2).tolist()} != {expected[k].tolist()} (depth {depth_size[0]}x"
                             f"{depth_size[1]})")
    n = len(uv)
    other = OTHER_ALGORITHM[ALGORITHM]
    primary, secondary, pixels = rows[:, DEPTH_COLUMNS[ALGORITHM]], rows[:, DEPTH_COLUMNS[other]], rows[:, 6]
    use_other = (primary <= 0) & (secondary > 0)
    depth = np.where(use_other, secondary, primary)
    # Pixels the SLC samples in a full (unclipped) window: what PnP's min_valid_depth_fraction refers to
    samples = np.where(use_other, samples_per_window(WINDOW_RADIUS, STEPS[other]), SAMPLES_PER_WINDOW)
    reported = bool(np.any(pixels[depth > 0] > 0)) if np.any(depth > 0) else True
    c_patch, c_pixel = rows[n], rows[n + 1]
    cx, cy = depth_size[0] // 2, depth_size[1] // 2
    patch_depth = depth[n]
    center = {"x": cx, "y": cy, "pixel_mm": int(round(max(c_pixel[4], c_pixel[5]))), "patch": CENTER_PATCH,
              "median_mm": float(patch_depth) if patch_depth > 0 else None, "valid": int(c_patch[6]),
              "pixels": int(_sampled(expected[n:n + 1], STEPS[ALGORITHM])[0]), "source": "depth_points",
              "algorithm": ALGORITHM.lower() if not use_other[n] else other.lower(),
              "note": f"camera SpatialLocationCalculator, {ALGORITHM} of the valid pixels in the patch"}
    return DepthPoints(sequence=sequence, depth_mm=depth[:n], pixels=pixels[:n].astype(np.int64),
                       samples=samples[:n].astype(np.int64), depth_size=tuple(depth_size), center=center,
                       pixel_count_reported=reported, other_algorithm=int(use_other[:n].sum())), None


def depth_sample(points):
    """The feature depths (mm, 0 = invalid) as a 1-D sample for the depth stream's blank-frame check."""
    return None if points is None else np.asarray(points.depth_mm)


# --- host: pairing by sequence number -----------------------------------------------------------------------

def _seconds(value):
    try:
        return value.total_seconds()
    except AttributeError:
        return None


@dataclass
class Received:
    """A points message as it arrived: decoded once (rows, or the decoding error)."""
    msg: object
    rows: Optional[np.ndarray]
    error: Optional[str] = None


class PointsMatcher:
    """
    Pairs each features message with the points message of the same sequence number. take(sequence) drains the
    points queue, then blocks on it (queue.get(timedelta)) until that sequence arrives, a newer one shows it
    is gone ("dropped"), or wait_s passes ("timeout"). Points left unpaired are orphans. After DEAD_AFTER
    timeouts in a row it stops blocking (only takes what has arrived) until points come again. Each message is
    decoded once on arrival (on_message(rows) then counts its bytes).
    """

    def __init__(self, queue, wait_s, clock=time.monotonic, keep=KEEP_POINTS, on_message=None):
        self.queue = queue
        self.wait_s = float(wait_s)
        self.clock = clock
        self.keep = keep
        self.on_message = on_message  # called with each message's rows (None if unreadable): bytes accounting
        self.pending = {}
        self.newest = None
        self.received = self.orphans = self.gaps = self.gap_frames = 0
        self.timeouts_in_row = 0
        self.waited = deque(maxlen=200)

    @property
    def dead(self):
        return self.timeouts_in_row >= DEAD_AFTER

    def _add(self, msg):
        sequence = int(msg.getSequenceNum())
        self.received += 1
        self.timeouts_in_row = 0
        try:
            entry = Received(msg, decode(msg))
        except Exception as e:
            entry = Received(msg, None, f"{type(e).__name__}: {e}")
        if self.on_message is not None:
            self.on_message(entry.rows)
        if self.newest is not None:
            if sequence > self.newest + GAP_TOLERANCE:
                self.gaps += 1
                self.gap_frames += sequence - self.newest - 1
            elif sequence <= self.newest - 1000:  # a restarted camera counts from 0 again
                self.orphans += len(self.pending)
                self.pending.clear()
        self.newest = sequence if self.newest is None else max(self.newest, sequence)
        self.pending[sequence] = entry
        while len(self.pending) > self.keep:
            self.pending.pop(min(self.pending))
            self.orphans += 1

    def _drain(self):
        for msg in self.queue.tryGetAll():
            self._add(msg)

    def _take(self, sequence):
        entry = self.pending.pop(sequence, None)
        for older in [s for s in self.pending if s < sequence]:
            del self.pending[older]
            self.orphans += 1
        return entry

    def take(self, sequence):
        """(Received or None, reason or None)."""
        start = self.clock()
        self._drain()
        deadline = start + (0.0 if self.dead else self.wait_s)
        while True:
            if sequence in self.pending:
                self.waited.append(self.clock() - start)
                return self._take(sequence), None
            if any(s > sequence for s in self.pending):
                self._take(sequence)
                return None, "dropped"
            remaining = timedelta(seconds=deadline - self.clock())
            # timedelta has microseconds: a remainder below that is the deadline
            if remaining <= timedelta(0):
                self.timeouts_in_row += 1
                return None, "timeout"
            msg = self.queue.get(remaining)
            if msg is not None:
                self._add(msg)

    def wait_ms(self):
        if not self.waited:
            return None, None
        values = np.array(self.waited) * 1000.0
        return round(float(values.mean()), 2), round(float(values.max()), 2)


@dataclass
class PointsReport:
    """Host-side counters of the points transport (DepthPointsReceiver.status)."""
    frames: int = 0
    paired: int = 0
    missing: Counter = field(default_factory=Counter)
    last_problem: Optional[dict] = None
    feature_gaps: int = 0
    feature_gap_frames: int = 0
    other_algorithm: int = 0
    pixel_count_missing: int = 0
    valid: int = 0
    features: int = 0
    timestamp_diff_ms_max: Optional[float] = None
    configs_sent: int = 0
    config_errors: int = 0


class DepthPointsReceiver:
    """
    VIO-loop side of points mode. request(sequence, uv) once per features message (rois "host": sends the tag
    (build_tag) and then the ROI config (build_config) through send, needs the camera's depth size from its
    'size' report), then take(sequence, uv, features_ts) returns (DepthPoints or None, reason or None) for it;
    several requests may come before their takes. on_status(data) with each Script report. status() is what
    /api/status shows under telemetry.depth_points.
    """

    def __init__(self, queue, image_size, roi_source, wait_s, send=None, build_config=None, build_tag=None,
                 thresholds=None, frame_period_s=0.1, clock=time.monotonic, wall_clock=time.time, log=print,
                 on_message=None):
        self.matcher = PointsMatcher(queue, wait_s, clock=clock, on_message=on_message)
        self.image_size = tuple(image_size)
        self.roi_source = roi_source
        self.send = send
        self.build_config = build_config
        self.build_tag = build_tag
        self.thresholds = thresholds
        self.frame_period_s = frame_period_s
        self.wall_clock = wall_clock
        self.log = log
        self.depth_size = None
        self.report = PointsReport()
        self.device = {"errors": 0, "sent": 0, "misses": 0, "last_error": None, "fatal": False, "stats": None,
                       "size": None, "config_sequence": None}
        self._last_feature_sequence = None
        self._requested = set()
        self._logged = Counter()

    # Script reports
    def on_status(self, data):
        status = parse_status(data)
        if status is None:
            self.log(f"Depth points: unreadable status from the camera: {bytes(data)[:80]!r}")
            return
        device = self.device
        device.update(errors=max(device["errors"], status["errors"]), sent=status["sent"], misses=status["misses"])
        kind, now = status["kind"], self.wall_clock()
        if kind == "size":
            size = tuple(status["size"])
            if self.depth_size is not None and size != self.depth_size:
                self.log(f"Depth points: the camera's depth frame is now {size[0]}x{size[1]}")
            self.depth_size = size
            device["size"] = list(size)
        elif kind == "stats":
            device["stats"] = {key: status[key] for key in ("build_ms_mean", "build_ms_max", "slc_ms_mean",
                                                             "slc_ms_max", "size")}
            device["stats"]["unix_time"] = now
        elif kind == "miss":
            device["last_miss"] = {"text": status["text"], "unix_time": now}
        elif kind == "tagging":
            survives = status["header"] == status["tag"]
            device["config_sequence"] = {"tag": status["tag"], "config": status["header"], "survives": survives}
            self.log(f"Depth points: the camera paired its first config by its tag (frame {status['tag']}); the "
                     f"config's own sequence number read {status['header']} on the camera"
                     f"{'' if survives else ' (lost on the link, as on test16b)'}")
        elif kind in ("error", "fatal"):
            fatal = kind == "fatal"
            if fatal or device["last_error"] is None or device["last_error"]["text"] != status["text"]:
                self.log(f"Depth points {'stopped on' if fatal else 'error on'} the camera ({status['errors']} "
                         f"errors): {status['text']}")
            device["last_error"] = {"text": status["text"], "unix_time": now, "fatal": fatal}
            device["fatal"] = device["fatal"] or fatal

    def request(self, sequence, uv):
        """rois "host": send this frame's ROI config. True if sent."""
        if self.roi_source != ROIS_HOST:
            return True
        if self.depth_size is None:
            return False
        try:
            windows = all_windows(uv, self.image_size, self.depth_size)
            config = self.build_config(windows, self.thresholds, sequence)
            self.send(self.build_tag(sequence))
            self.send(config)
        except Exception as e:
            self.report.config_errors += 1
            if self.report.config_errors == 1 or self.report.config_errors % 100 == 0:
                self.log(f"Depth points: ROI config not sent ({self.report.config_errors} times): {e}")
            return False
        self.report.configs_sent += 1
        self._requested.add(int(sequence))
        if len(self._requested) > 64:
            self._requested = {s for s in self._requested if s > sequence - 64}
        return True

    def _missing(self, sequence, reason, text):
        report = self.report
        report.missing[reason] += 1
        report.last_problem = {"reason": reason, "text": text, "sequence": int(sequence),
                               "unix_time": self.wall_clock()}
        self._logged[reason] += 1
        count = self._logged[reason]
        if count == 1 or count % 100 == 0:
            self.log(f"Depth points: frame {sequence} has no depth ({reason}, {count} times): {text}")
        return None, reason

    def take(self, sequence, uv, features_ts=None, requested=True):
        report = self.report
        report.frames += 1
        sequence = int(sequence)
        last = self._last_feature_sequence
        if last is not None and sequence > last + 1:
            report.feature_gaps += 1
            report.feature_gap_frames += sequence - last - 1
        self._last_feature_sequence = sequence
        if not requested:
            return self._missing(sequence, "not_requested",
                                 "the camera's depth frame size is not known yet" if self.depth_size is None
                                 else "the ROI config could not be sent")
        entry, reason = self.matcher.take(sequence)
        if entry is None:
            text = {"dropped": "points for a later frame came, none for this one (dropped on the camera or the link)",
                    "timeout": f"no points within {self.matcher.wait_s * 1000:.0f} ms"}[reason]
            if self.matcher.dead:
                text += " (camera sends no points; not waiting until they come back)"
            return self._missing(sequence, reason, text)
        if entry.rows is None:
            return self._missing(sequence, "unreadable", f"points message not readable: {entry.error}")
        size = self.depth_size if self.roi_source == ROIS_HOST else None
        points, problem = resolve(entry.rows, uv, self.image_size, size, sequence)
        if problem is not None:
            return self._missing(sequence, *problem)
        msg = entry.msg
        points_ts = _seconds(msg.getTimestamp()) if hasattr(msg, "getTimestamp") else None
        if points_ts is not None and features_ts is not None:
            diff_ms = abs(points_ts - features_ts) * 1000.0
            report.timestamp_diff_ms_max = max(report.timestamp_diff_ms_max or 0.0, round(diff_ms, 3))
            if diff_ms > 500.0 * self.frame_period_s:
                return self._missing(sequence, "timestamp", f"the depth frame is {diff_ms:.1f} ms away from the "
                                                            f"features' frame")
            points.timestamp_s = points_ts
        report.paired += 1
        report.other_algorithm += points.other_algorithm
        report.pixel_count_missing += not points.pixel_count_reported
        report.features += len(points.depth_mm)
        report.valid += int(np.count_nonzero(points.depth_mm > 0))
        return points, None

    def status(self):
        report, matcher = self.report, self.matcher
        wait_mean, wait_max = matcher.wait_ms()
        missing = sum(report.missing.values())
        return {
            "transport": POINTS, "rois": self.roi_source, "frames": report.frames, "paired": report.paired,
            "paired_percent": round(100.0 * report.paired / report.frames, 2) if report.frames else None,
            "missing": missing, "missing_by_reason": dict(report.missing), "last_problem": report.last_problem,
            "points_received": matcher.received, "orphan_points": matcher.orphans,
            "points_gaps": matcher.gaps, "points_gap_frames": matcher.gap_frames,
            "feature_gaps": report.feature_gaps, "feature_gap_frames": report.feature_gap_frames,
            "waiting": not matcher.dead, "wait_ms_mean": wait_mean, "wait_ms_max": wait_max,
            "wait_limit_ms": round(matcher.wait_s * 1000.0, 1),
            "valid_percent": round(100.0 * report.valid / report.features, 1) if report.features else None,
            "algorithm": ALGORITHM.lower(), "other_algorithm_rois": report.other_algorithm,
            "pixel_count_missing_frames": report.pixel_count_missing,
            "timestamp_diff_ms_max": report.timestamp_diff_ms_max,
            "configs_sent": report.configs_sent, "config_errors": report.config_errors,
            "depth_size": None if self.depth_size is None else list(self.depth_size),
            "window_px": 2 * WINDOW_RADIUS + 1, "samples_per_window": SAMPLES_PER_WINDOW,
            "depth_buffer_frames": DEPTH_BUFFER,
            "device": dict(self.device),
        }


# --- host: on-demand full depth frames (points mode) ------------------------------------------------------

class FullDepthGate:
    """
    Points mode: full depth frames cross the link only while something on the Pi needs them (the depth probe,
    the host_jpeg depth stream while watched). update(needs) once per frame with {reason: every n-th frame,
    0 = not needed}; the gate forwards every min(n)-th frame. The control goes out when it changes and every
    resend_interval (a lost message heals).
    """

    def __init__(self, send, clock=time.monotonic, log=print, resend_interval=5.0):
        self.send = send
        self.clock = clock
        self.log = log
        self.resend_interval = resend_interval
        self.every = 0
        self.reasons = ()
        self.frames = 0
        self.bytes = 0
        self.since = None
        self._last_send = float("-inf")

    def update(self, needs):
        wanted = {reason: int(every) for reason, every in needs.items() if every and every > 0}
        every = min(wanted.values()) if wanted else 0
        now = self.clock()
        changed = every != self.every
        if changed or now - self._last_send >= self.resend_interval:
            try:
                self.send(every)
            except Exception as e:
                self.log(f"Full depth gate control not sent: {e}")
                return self.every
            self._last_send = now
            if changed:
                if every:
                    self.log(f"Full depth frames on (every {every} frame(s)) for {', '.join(sorted(wanted))}")
                else:
                    self.log("Full depth frames off")
                self.since = time.time() if every else None
            self.every = every
        self.reasons = tuple(sorted(wanted))
        return self.every

    def on_frame(self, nbytes=0):
        self.frames += 1
        self.bytes += int(nbytes)

    def status(self):
        return {"on": self.every > 0, "every": self.every, "reasons": list(self.reasons), "frames": self.frames,
                "bytes": self.bytes, "since": self.since}
