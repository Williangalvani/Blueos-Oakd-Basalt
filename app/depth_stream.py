"""
Live stream of the PnP VIO for the web page: the camera image the VIO tracks its features on, or its depth,
encoded on the OAK-D (or, as a last resort, depth on the Pi).

Device (pnp_vio.build_depth_stream): a Script node takes a StereoDepth output from a non-blocking size-1
queue and, only while the host enabled it, passes every `every`-th frame on to a VideoEncoder node, which
encodes it as MJPEG; only the JPEG crosses the link. The VIO process writes it as-is to
DIRECTORY/depth_stream.jpg; main.py forwards it as multipart/x-mixed-replace. The browser shows the camera
image with the tracked features on top, or colorizes the depth.

Encoding variants (depth_stream_encoding, VARIANTS; the StereoDepth output each one shows is SOURCES):
    mono_features  stereo.rectifiedLeft -> Script (gate only: forwards StereoDepth's own frame) ->
               VideoEncoder. The camera image the FeatureTracker runs on (RAW8, full 0..255), so the page
               draws the tracked features (feature_stream) on it; MONO_ENCODING. Works with any stereo
               setting
    disparity  stereo.disparity -> Script (gate only) -> VideoEncoder. Needs an 8-bit (RAW8) disparity:
               subpixel off, extended off and no post-processing filter after the median
               (stereo_config.disparity_format; with the speckle filter on after the median it is RAW16);
               image: the disparity itself (DISPARITY_ENCODING, see "Disparity encoding" below)
    host_jpeg  nothing extra on the camera: the VIO process converts the depth frame it already has and
               JPEG-encodes it on a worker thread (HostJpegEncoder), capped at HOST_CPU_BUDGET of a core
    auto       (default) mono_features; host_jpeg from the start if the image size does not suit the
               encoder; switches to host_jpeg for the rest of the run (with the reason in the metadata's
               fallback) if the encoder stalls or its frames are blank. auto never picks disparity: it
               depends on the stereo settings, so it is only built when asked for

Why only StereoDepth's own frames work on the camera (firmware a406121b strings, depthai-core v3.3.0
source, test12, test13 and the stereo probe on an OAK-D-W, test14, depthai 3.3.0, 640x400 @ 10 fps), and
why the earlier Script-built depth variants (manip, nv12, yuv400p) were removed:
- A Script's new ImgFrame comes from a DynamicPool of size 0. ImageManip (PlgImageManipV2) needs an input
  pool of at least 2 ("Pool size of the input message is too small... Currently it is 0") and then sends a
  constant image (test12: every JPEG 4668 bytes, every pixel 1). The VideoEncoder drops pool-less frames
  with no log (test12: Script-made NV12 and YUV400p frames, 0 bytes).
- StereoDepth's outputs come from its own pools (StereoDepth.setNumFramesPool, default 3; the pipeline
  raises it), and a Script that forwards the frame it received (the same object) keeps that pool
  (test13: no pool error).
- RAW8 frames go straight into the VideoEncoder (test14: the RAW8 disparity and rectifiedLeft /
  rectifiedRight, 640x400 stride 640, encoded directly into 6.8-32 kB JPEGs), so these variants have no
  ImageManip. A RAW16 frame (stereo.depth, the subpixel disparity, and the disparity with a filter after the
  median) gets "Arrived frame type (14) is not either NV12 or YUV400p (8-bit Gray)" from the encoder and no
  JPEG, and through ImageManip "Processing failed, potentially unsupported config" and a constant image
  (test14 cases A and E: 4671-byte JPEGs, gray 1).
The disparity output carries the same threshold and post-processing filters as stereo.depth (depthai
applies them in the disparity domain before converting to depth).

Disparity encoding (DISPARITY_ENCODING, disparity_scale): gray = disparity in pixels, 0..max_disparity
(95), invalid = 0, so depth_mm = focal_px * baseline_mm / gray. Near is bright; far is dark and coarse
(each gray level is one pixel of disparity: at 4 m and 75 mm baseline with fx 440, about 0.5 m per level;
test14's scene used 0..34 of 95). The page stretches the display (colours or gray by depth) and labels its
legend from the scale in the metadata; GRAY_INVALID_BELOW for this encoding is DISPARITY_INVALID_BELOW (3),
as JPEG rings around 0.

Mono encoding (MONO_ENCODING): gray = the rectified-left camera image, not depth. The tracked features'
positions are on its pixel grid (scaled to the depth grid if decimated, feature_stream.py), and the VIO
publishes the features of the frame each image comes from (feature_stream.FeatureStreamService.publish_for).

VideoEncoder (MJPEG) input requirements, from the RVC2 firmware a406121b (depthai-core v3.3.0) strings and
asserts of its PlgVidEnc, the depthai-core source and test14:
- an 8-bit type: NV12, YUV400p, or RAW8 from a StereoDepth pool (test14); the warning names only NV12 and
  YUV400p ("Arrived frame type ({}) is not either NV12 or YUV400p (8-bit Gray)"). GRAY8 (30) is dropped
  (test10), and RAW16 (14) too (test14);
- width a multiple of 32 and height a multiple of 2 ("Width must be multiple of 32, height multiple of 2
  for MJPEG encoder profile"), at most 16384 x 8192;
- every frame the size of the first one ("Arrived frame ({}, {}) does not match the encoder size"; asserts
  frm->fb.spec.width == FrameInfo.iWidth and height);
- YUV400p needs an extra gray chroma buffer the encoder reserves ("Encoder couldn't reserve extra space for
  gray chroma buffer", maxGrayChromaBufferSize != -1); NV12 does not;
- source, timestamp and sequence metadata present (asserts pSrcMeta, pPtsMeta, pSeqMeta != nullptr);
- 64-byte aligned buffer base and size (asserts ((uint32_t)frm->base & 63) == 0, (frm->size & 63) == 0):
  pool buffers are; a Script's ImgFrame(size) buffer is not guaranteed to be;
- only one of the 'bitstream' and 'out' outputs may be linked;
- with the debug log level it logs "VideoEncoder took '{}' ms." per frame (depth_stream_debug).
WIDTH_ALIGN and HEIGHT_ALIGN check the size before building the pipeline.

8-bit depth encoding (DEPTH_ENCODING: host_jpeg; linear in depth over the stereo threshold range, near
bright):
    0                      invalid (no depth, or outside min_mm..max_mm, as the VIO sees it)
    GRAY_FAR..GRAY_NEAR    max_mm (far) .. min_mm (near)
Depth is quantised to 2**shift mm before the mapping (16 mm for the default 4 m range). JPEG ringing
blurs values near 0, so the page treats gray below GRAY_INVALID_BELOW as invalid.

Gating: main.py touches DIRECTORY/depth_stream.lease while a client streams; the VIO process checks it
at most every poll_interval and tells the Script to start or stop. With nobody watching, the Script
blocks on its control input, so nothing is converted, encoded or sent.

Errors: the Script counts failed frames and sends the count with the last error (throttled) on its 'status'
output, and while streaming its count of frames sent about once a second; the VIO process publishes them in
DIRECTORY/depth_stream.json, which /api/status serves. The encoder drops frames it can't take with only a
device log warning, so the VIO process flags the stream (encoder_stalled) when the Script keeps sending and
no JPEG comes back. HostJpegEncoder sends the same reports, so the flag works for every variant.
Blank frames (blank): for a camera variant, frames byte-identical to the previous one or with a gray
standard deviation below BLANK_STD, for BLANK_S and at least BLANK_MIN_SAMPLES samples, while the host's
own depth (stereo.depth, sampled every EXPECT_INTERVAL_S) has valid pixels that vary. A view with no depth
or one flat depth is constant and right, so it is not blank; a camera image whose depth varies is not
constant either. auto falls back to host_jpeg ("blank frames from <variant>"); an explicit variant only
reports it. Between them, encoder_stalled (no JPEG) and blank (useless JPEGs) cover what test12 and test14
saw from camera frames the encoder can't take.

Device log: DeviceLog keeps the camera's log lines (VideoEncoder and the stream Script first),
which the metadata carries. With depth_stream_debug, DeviceLogDebug raises the device log level to DEBUG
while the stream runs and restores it when the stream stops.

depthai-free so it can be unit tested.
"""

import json
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

import snapshots

FRAME_FILE = "depth_stream.jpg"
META_FILE = "depth_stream.json"
LEASE_FILE = "depth_stream.lease"
LEASE_S = 4.0
GRAY_FAR = 16
GRAY_NEAR = 255
GRAY_INVALID_BELOW = 8
CENTER_PATCH = 5
BOUNDARY = "depthframe"
RATE_WINDOW_S = 5.0
REPORT_INTERVAL_S = 1.0
REPORT_MAX_CHARS = 400
# Without a clock on the device, the Script reports its frame count every this many frames
PROGRESS_EVERY = 5
# encoder_stalled: the Script's count rose for this long since its first report after the last JPEG, with
# its reports no further apart than ENCODER_REPORT_GAP seconds (or frame intervals, for a slow stream)
ENCODER_STALL_S = 3.0
ENCODER_REPORT_GAP = 2.5
ENCODER_STALL_TEXT = "camera converts frames but the encoder outputs nothing"
# Blank frames (test12: ImageManip fed a pool-less Script frame outputs a constant image): a camera JPEG is
# constant if it is byte-identical to the previous one, or decodes (at most every BLANK_SAMPLE_S) to a
# gray standard deviation under BLANK_STD. The stream is blank after constant JPEGs for BLANK_S (at least
# BLANK_MIN_SAMPLES of them) while the host's own depth (sampled every EXPECT_INTERVAL_S, at most
# EXPECT_MAX_AGE_S old) says the image should vary: at least EXPECT_VALID_MIN of it valid and a gray
# standard deviation of EXPECT_GRAY_STD_MIN. A covered lens or an empty view is not blank.
BLANK_S = 2.0
BLANK_MIN_SAMPLES = 3
BLANK_SAMPLE_S = 0.5
BLANK_STD = 0.75
EXPECT_INTERVAL_S = 1.0
EXPECT_MAX_AGE_S = 3.0
EXPECT_VALID_MIN = 0.05
EXPECT_GRAY_STD_MIN = 3.0
# MJPEG profile: width a multiple of 32, height a multiple of 2
WIDTH_ALIGN = 32
HEIGHT_ALIGN = 2

AUTO, MONO_FEATURES, DISPARITY, HOST_JPEG = "auto", "mono_features", "disparity", "host_jpeg"
VARIANTS = (AUTO, MONO_FEATURES, DISPARITY, HOST_JPEG)
DEFAULT_VARIANT = AUTO
# The variants that run on the camera: the Script forwards the StereoDepth frame unchanged into the encoder
CAMERA_VARIANTS = (MONO_FEATURES, DISPARITY)
# The StereoDepth output each variant shows (info "source")
SOURCES = {MONO_FEATURES: "rectifiedLeft", DISPARITY: "disparity", HOST_JPEG: "depth"}
# What the gray values mean (info "encoding")
DEPTH_ENCODING, DISPARITY_ENCODING, MONO_ENCODING = "gray8-depth-linear", "gray8-disparity", "gray8-mono-rectified-left"
# StereoDepth's disparity search range without subpixel and extended disparity (stereo_config sets
# extended off): RAW8 0..95, 0 = invalid
DISPARITY_MAX = 95
# Disparity gray below this is invalid (0) or JPEG ringing around it: under 3 px is > 10 m on an OAK-D
DISPARITY_INVALID_BELOW = 3
# host_jpeg: at most this fraction of one Pi core on average
HOST_CPU_BUDGET = 0.25
HOST_STATS_EVERY = 50
# ImageManip output buffers (depthai-core ImageManipImpl: DEPTHAI_STRIDE/HEIGHT/PLANE_ALIGNMENT)
MANIP_STRIDE_ALIGN = 128
MANIP_HEIGHT_ALIGN = 32
MANIP_PLANE_ALIGN = 128 * 32
MANIP_FRAMES_POOL = 3
DEVICE_LOG_LINES = 30
DEVICE_LOG_OTHER_LINES = 10
DEVICE_LOG_MAX_CHARS = 300
# Device log lines from these node types count as the stream's (besides its own Script)
STREAM_NODE_TYPES = ("VideoEncoder",)


def device_variant(variant):
    """The variant that runs on the camera at the start (auto starts with mono_features), None for host_jpeg."""
    if variant == AUTO:
        return MONO_FEATURES
    return variant if variant in CAMERA_VARIANTS else None


def disparity_problem(disparity_format):
    """
    Why the disparity variant can't run, or None: it needs a RAW8 disparity (stereo_config.disparity_format of the
    applied stereo config). RAW16 neither goes into the encoder nor through ImageManip (test14).
    """
    if disparity_format is None:
        return "the disparity format is unknown"
    if disparity_format.get("raw8"):
        return None
    return f"{disparity_format.get('reason') or 'disparity is RAW16'}; or use mono_features"


def resolve_variant(requested, disparity_problem=None, problem=None):
    """
    (variant built at the start, None for host_jpeg; why it is not the requested one, or None). auto:
    mono_features, or host_jpeg straight away if the size does not suit the encoder (problem). An explicit
    disparity request runs as host_jpeg when the disparity is not RAW8 (disparity_problem, with the reason);
    mono_features and host_jpeg are built as asked.
    """
    variant = device_variant(requested)
    if requested == AUTO and problem:
        return None, problem
    if variant == DISPARITY and disparity_problem:
        return None, disparity_problem
    return variant, None


def encoding_for(variant):
    return {DISPARITY: DISPARITY_ENCODING, MONO_FEATURES: MONO_ENCODING}.get(variant, DEPTH_ENCODING)


def source_for(variant):
    """The StereoDepth output a variant shows, as "stereo.<output>" (None if unknown)."""
    output = SOURCES.get(variant)
    return None if output is None else f"stereo.{output}"


def disparity_scale(focal_px, baseline_m, min_mm, max_mm, max_disparity=DISPARITY_MAX):
    """
    The disparity variant's gray scale: gray = disparity in pixels (0 invalid), depth_mm = depth_times_disparity
    / gray, as the camera computes depth (rectified focal length x baseline). gray_far / gray_near are the
    disparities at max_mm / min_mm (near clamped to max_disparity, the closest the search range reaches).
    """
    k = float(focal_px) * float(baseline_m) * 1000.0
    if not (k > 0):
        return None
    return {"focal_px": round(float(focal_px), 3), "baseline_m": round(float(baseline_m), 5),
            "depth_times_disparity": round(k, 1), "max_disparity": int(max_disparity),
            "gray_far": round(k / max(float(max_mm), 1.0), 2),
            "gray_near": round(min(k / max(float(min_mm), 1.0), float(max_disparity)), 2),
            "nearest_mm": round(k / max_disparity), "gray_invalid_below": DISPARITY_INVALID_BELOW}


def gray_disparity_reference(depth_mm, focal_px, baseline_m, max_disparity=DISPARITY_MAX):
    """The disparity variant's image for a uint16 depth image (numpy): round(f * B / depth), 0 invalid."""
    depth = np.asarray(depth_mm, dtype=np.float64)
    k = float(focal_px) * float(baseline_m) * 1000.0
    with np.errstate(divide="ignore"):
        disparity = np.where(depth > 0, np.round(k / np.maximum(depth, 1.0)), 0)
    return np.clip(disparity, 0, max_disparity).astype(np.uint8)


def size_problem(width, height):
    """Why the MJPEG encoder would refuse width x height, or None."""
    if width % WIDTH_ALIGN or height % HEIGHT_ALIGN:
        return (f"{width}x{height}: the camera's MJPEG encoder needs a width that is a multiple of {WIDTH_ALIGN} "
                f"and an even height")
    return None


def _align(value, alignment):
    return (value + alignment - 1) // alignment * alignment


def manip_output_size(width, height):
    """Bytes of ImageManip's NV12 output frame (depthai-core getAlignedOutputFrameSize for NV12)."""
    stride = _align(width, MANIP_STRIDE_ALIGN)
    return (_align(stride * _align(height, MANIP_HEIGHT_ALIGN), MANIP_PLANE_ALIGN)
            + _align(stride * _align(height // 2, MANIP_HEIGHT_ALIGN // 2), MANIP_PLANE_ALIGN))


def depth_shift(max_mm):
    """Smallest shift s with max_mm >> s <= 254, so 255 is free as the overflow marker."""
    shift = 0
    while (int(max_mm) >> shift) > 254:
        shift += 1
    return shift


def gray_for_depth(depth_mm, min_mm, max_mm):
    """Gray value of one depth (mm) on the stream's scale: 0 invalid, GRAY_FAR at max_mm, GRAY_NEAR at min_mm."""
    if depth_mm <= 0 or depth_mm < min_mm or depth_mm > max_mm:
        return 0
    span = max(max_mm - min_mm, 1)
    return int(round(GRAY_FAR + (GRAY_NEAR - GRAY_FAR) * (max_mm - depth_mm) / span))


def encoding_tables(min_mm, max_mm):
    """
    (shift, t_hi, t_lo, t_out): 256-byte tables for the device conversion of a RAW16 depth pixel with
    high byte h and low byte l: gray = t_out[t_hi[h] | t_lo[l]]. t_hi[h] | t_lo[l] is depth >> shift
    (255 if it overflows); t_out maps it to the gray scale at the centre of its 2**shift mm bin, clamped
    to min_mm..max_mm, and to 0 (invalid) for bins entirely outside it.
    """
    min_mm, max_mm = int(min_mm), int(max_mm)
    shift = depth_shift(max_mm)
    t_hi = bytes(min((h << 8) >> shift, 255) for h in range(256))
    t_lo = bytes(low >> shift for low in range(256))
    out = [0] * 256
    for v in range(1, 255):
        low, high = v << shift, ((v + 1) << shift) - 1
        if high >= min_mm and low <= max_mm:
            centre = min(max((low + high) / 2.0, min_mm), max_mm)
            out[v] = gray_for_depth(centre, min_mm, max_mm)
    return shift, t_hi, t_lo, bytes(out)


def gray_reference(depth_mm, min_mm, max_mm):
    """What the device produces for a uint16 depth image (numpy), for tests and offline checks."""
    _, t_hi, t_lo, t_out = encoding_tables(min_mm, max_mm)
    depth = np.asarray(depth_mm, dtype=np.uint16)
    hi = np.frombuffer(t_hi, np.uint8)[depth >> 8]
    lo = np.frombuffer(t_lo, np.uint8)[depth & 0xFF]
    return np.frombuffer(t_out, np.uint8)[hi | lo]


def stream_divisor(camera_fps, stream_fps):
    """Forward every n-th depth frame for about stream_fps."""
    return max(1, int(round(float(camera_fps) / max(float(stream_fps), 1e-3))))


def device_script(every, stats_every=150, report_interval=REPORT_INTERVAL_S):
    """
    Script node source, a gate: inputs 'depth' (the StereoDepth frame the variant shows, RAW8) and 'control'
    (Buffer, sequence number 1 = stream, 0 = stop); outputs 'gray' (the received frame, sent on unchanged:
    the same message, so it keeps its pool buffer and metadata) for the VideoEncoder and 'status' (Buffer, see
    parse_device_status) for the host. Plain CPython builtins and the on-device message API only (the RVC2
    firmware's Script bindings: Buffer(size in bytes) filled with setData; ImgFrame getters).

    Errors: each failed frame counts; the first one, then at most one per report_interval, is sent on
    'status' with the running count, and the 1st and every 100th also go to node.warn (the host log). While
    streaming it reports its count of frames sent after the first frame, then at most once per
    report_interval, and (status and node.warn) its time per frame every stats_every frames: the measured
    on-device cost.
    """
    return "\n".join([
        f"every = {int(every)}",
        f"STATS_EVERY = {int(stats_every)}",
        f"REPORT_INTERVAL = {float(report_interval)!r}",
        f"REPORT_MAX = {int(REPORT_MAX_CHARS)}",
        f"PROGRESS_EVERY = {int(PROGRESS_EVERY)}",
        "",
        *_script_loop([
            "        width = frame.getWidth()",
            "        height = frame.getHeight()",
            "        out = frame",
        ]),
    ])


def _script_loop(convert):
    """The gate, report and send loop of device_script; convert: the lines making `out` from `frame`."""
    return [
        "try:",
        "    import time",
        "    clock = time.monotonic",
        "except Exception:",
        "    clock = None",
        "",
        "enabled = False",
        "count = 0",
        "errors = 0",
        "sent = 0",
        "busy = 0.0",
        "worst = 0.0",
        "reported = None",
        "progressed = None",
        "",
        "def report(kind, text):",
        "    try:",
        "        data = (kind + '\\t' + str(errors) + '\\t' + str(sent) + '\\t' + text)[:REPORT_MAX].encode()",
        "        message = Buffer(len(data))",
        "        message.setData(data)",
        "        node.outputs['status'].send(message)",
        "    except Exception as e:",
        "        node.warn('depth stream: status not sent: ' + str(e))",
        "",
        "def failed(where, e, kind='error'):",
        "    global errors, reported",
        "    errors += 1",
        "    text = where + ': ' + type(e).__name__ + ': ' + str(e)",
        "    if errors == 1 or errors % 100 == 0 or kind != 'error':",
        "        node.warn('depth stream: ' + text + ' (' + str(errors) + ' errors)')",
        "    now = clock() if clock else None",
        "    if (kind != 'error' or reported is None or (now is None and errors % 100 == 0)",
        "            or (now is not None and now - reported >= REPORT_INTERVAL)):",
        "        reported = now if now is not None else 0.0",
        "        report(kind, text)",
        "",
        "try:",
        "    while True:",
        "        if not enabled:",
        "            enabled = node.inputs['control'].get().getSequenceNum() > 0",
        "            count = every - 1",
        "            continue",
        "        control = node.inputs['control'].tryGet()",
        "        while control is not None:",
        "            enabled = control.getSequenceNum() > 0",
        "            control = node.inputs['control'].tryGet()",
        "        if not enabled:",
        "            continue",
        "        frame = node.inputs['depth'].get()",
        "        count += 1",
        "        if count < every:",
        "            continue",
        "        count = 0",
        "        start = clock() if clock else 0.0",
        *convert,
        "        frame = None",
        "        try:",
        "            node.outputs['gray'].send(out)",
        "        except Exception as e:",
        "            failed('send', e)",
        "            continue",
        "        out = None",
        "        sent += 1",
        "        if clock:",
        "            now = clock()",
        "            took = now - start",
        "            busy += took",
        "            worst = max(worst, took)",
        "            if progressed is None or now - progressed >= REPORT_INTERVAL:",
        "                progressed = now",
        "                report('sent', '')",
        "            if sent % STATS_EVERY == 0:",
        "                mean_ms = round(1000.0 * busy / STATS_EVERY, 1)",
        "                max_ms = round(1000.0 * worst, 1)",
        "                node.warn('depth stream: ' + str(sent) + ' frames, convert + send mean ' + str(mean_ms)",
        "                          + ' ms, max ' + str(max_ms) + ' ms (' + str(width) + 'x' + str(height) + ')')",
        "                report('stats', str(mean_ms) + ' ' + str(max_ms) + ' ' + str(width) + ' ' + str(height))",
        "                busy = 0.0",
        "                worst = 0.0",
        "        elif sent == 1 or sent % PROGRESS_EVERY == 0:",
        "            report('sent', '')",
        "except Exception as e:",
        "    failed('stopped', e, 'fatal')",
        "    raise",
    ]


def parse_device_status(data):
    """
    One 'status' message of device_script: tab-separated kind, error count, frames sent, text. kind is
    'error' (a frame failed; text says where and why), 'fatal' (the Script stopped), 'sent' (progress, no
    text) or 'stats' (text: mean and max convert + send ms, width, height). None if malformed.
    """
    try:
        kind, errors, sent, text = bytes(data).decode("utf-8", "replace").split("\t", 3)
        status = {"kind": kind, "errors": int(errors), "sent": int(sent), "text": text}
        if kind == "stats":
            mean_ms, max_ms, width, height = text.split()
            status.update(convert_ms_mean=float(mean_ms), convert_ms_max=float(max_ms), size=[int(width), int(height)])
    except (TypeError, ValueError):
        return None
    return status


def center_depth(depth, patch=CENTER_PATCH):
    """Raw device depth (mm) at the image centre: the centre pixel and the median of the valid patch x patch."""
    depth = np.asarray(depth)
    if depth.ndim != 2 or not depth.size:
        return None
    height, width = depth.shape
    y, x, r = height // 2, width // 2, patch // 2
    window = depth[max(y - r, 0):y + r + 1, max(x - r, 0):x + r + 1]
    valid = window[window > 0]
    return {"x": x, "y": y, "pixel_mm": int(depth[y, x]), "patch": patch,
            "median_mm": float(np.median(valid)) if valid.size else None,
            "valid": int(valid.size), "pixels": int(window.size)}


def _lease_fresh(path, now, lease_s):
    try:
        return now - path.stat().st_mtime <= lease_s
    except OSError:
        return False


def status_report(kind, errors, sent, text=""):
    """A 'status' message in device_script's format (parse_device_status)."""
    return f"{kind}\t{int(errors)}\t{int(sent)}\t{text}"[:REPORT_MAX_CHARS].encode()


class HostJpegEncoder:
    """
    host_jpeg: the VIO process converts its depth frames to the stream's gray scale and JPEG-encodes them.
    submit(depth, sequence) once per VIO frame (every `every`-th one is taken while enabled); a worker
    thread encodes; drain() returns the JPEGs and reports() the status messages, both on the VIO thread.

    CPU: each encode's thread CPU time c pushes the next start c * (1 / cpu_budget - 1) after its end, so
    the average stays at or below cpu_budget of one core; frames arriving earlier, or while one is being
    encoded, are skipped (skipped_cpu, skipped_busy). Reports like the camera Script's, with the count of
    frames offered (taken or skipped): 'sent' at most every report_interval, 'error' with the reason,
    'stats' (mean and max encode ms) every stats_every frames. So encoder_stalled works the same way, and
    also trips when the worker hangs: frames keep being offered and no JPEG comes back.
    """

    def __init__(self, min_mm, max_mm, quality, every=1, cpu_budget=HOST_CPU_BUDGET, threaded=True,
                 clock=time.monotonic, cpu_clock=time.thread_time, encode=None, stats_every=HOST_STATS_EVERY,
                 report_interval=REPORT_INTERVAL_S):
        _, t_hi, t_lo, t_out = encoding_tables(min_mm, max_mm)
        self._luts = tuple(np.frombuffer(t, np.uint8) for t in (t_hi, t_lo, t_out))
        self.quality = int(quality)
        self.every = max(1, int(every))
        self.cpu_budget = min(max(float(cpu_budget), 0.01), 1.0)
        self.clock = clock
        self.cpu_clock = cpu_clock
        self.encode = encode or self.encode_jpeg
        self.stats_every = stats_every
        self.report_interval = report_interval
        self.enabled = False
        self.count = self.every - 1
        self.offered = self.sent = self.errors = self.frames = 0
        self.skipped_cpu = self.skipped_busy = 0
        self.last_error = None
        self._lock = threading.Condition()
        self._job = None
        self._busy = False
        self._next_start = float("-inf")
        self._results, self._reports = [], []
        self._reported = None
        self._cpu = deque()  # (end, cpu seconds) of the encodes in the last RATE_WINDOW_S
        self._took = []
        self._size = None
        self._closed = False
        self.threaded = threaded
        self._thread = None  # started with the first set_enabled(True)

    def encode_jpeg(self, depth):
        import cv2
        t_hi, t_lo, t_out = self._luts
        depth = np.asarray(depth, dtype=np.uint16)
        gray = t_out[t_hi[depth >> 8] | t_lo[depth & 0xFF]]
        ok, jpeg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError("cv2.imencode returned no image")
        return jpeg.tobytes()

    def set_enabled(self, enabled):
        with self._lock:
            if enabled and not self.enabled:
                self.count = self.every - 1  # the first frame after a start goes out at once
                self._reported = None
            self.enabled = bool(enabled) and not self._closed
            if self.enabled and self.threaded and self._thread is None:
                self._thread = threading.Thread(target=self._worker, name="depth-jpeg", daemon=True)
                self._thread.start()

    def submit(self, depth, sequence=None):
        """True if this frame was taken for encoding."""
        if not self.enabled or depth is None:
            return False
        self.count += 1
        if self.count < self.every:
            return False
        now = self.clock()
        with self._lock:
            self.offered += 1
            if self._reported is None or now - self._reported >= self.report_interval:
                self._reported = now
                self._reports.append(status_report("sent", self.errors, self.offered))
            if self._busy or self._job is not None:
                self.skipped_busy += 1
                self.count = self.every - 1
                return False
            if now < self._next_start:
                self.skipped_cpu += 1
                self.count = self.every - 1  # try again with the next frame
                return False
            self.count = 0
            self.sent += 1
            job = (depth, sequence)
            if self._thread is not None:
                self._job = job
                self._lock.notify()
                return True
            self._busy = True
        self._run(job)
        return True

    def _worker(self):
        while True:
            with self._lock:
                while self._job is None and not self._closed:
                    self._lock.wait()
                if self._closed:
                    return
                job, self._job, self._busy = self._job, None, True
            self._run(job)

    def _run(self, job):
        depth, sequence = job
        cpu0, wall0 = self.cpu_clock(), time.perf_counter()
        jpeg = error = None
        try:
            jpeg = self.encode(depth)
        except Exception as e:
            error = f"encode: {type(e).__name__}: {e}"
        took, cpu = time.perf_counter() - wall0, max(self.cpu_clock() - cpu0, 0.0)
        end = self.clock()
        with self._lock:
            self._busy = False
            self._next_start = end + cpu * (1.0 / self.cpu_budget - 1.0)
            self._cpu.append((end, cpu))
            while self._cpu and end - self._cpu[0][0] > RATE_WINDOW_S:
                self._cpu.popleft()
            if error is not None:
                self.errors += 1
                self.last_error = error
                self._reports.append(status_report("error", self.errors, self.offered, error))
                return
            self.frames += 1
            self._results.append((jpeg, sequence))
            self._took.append(took)
            self._size = tuple(np.shape(depth)[1::-1])
            if len(self._took) >= self.stats_every:
                took_ms = [1000.0 * t for t in self._took]
                self._took = []
                width, height = self._size
                self._reports.append(status_report(
                    "stats", self.errors, self.offered,
                    f"{sum(took_ms) / len(took_ms):.1f} {max(took_ms):.1f} {width} {height}"))

    def drain(self):
        """[(jpeg bytes, sequence)] encoded since the last call."""
        with self._lock:
            results, self._results = self._results, []
        return results

    def reports(self):
        with self._lock:
            reports, self._reports = self._reports, []
        return reports

    def stats(self, now=None):
        now = self.clock() if now is None else now
        with self._lock:
            cpu = sum(c for end, c in self._cpu if now - end <= RATE_WINDOW_S)
            return {"cpu_percent": round(100.0 * cpu / RATE_WINDOW_S, 1),
                    "cpu_budget_percent": round(100.0 * self.cpu_budget, 1), "frames": self.frames,
                    "offered": self.offered, "taken": self.sent, "errors": self.errors,
                    "skipped_cpu": self.skipped_cpu, "skipped_busy": self.skipped_busy}

    def close(self):
        with self._lock:
            self._closed = True
            self.enabled = False
            self._lock.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def _level_name(level):
    return None if level is None else str(getattr(level, "name", level))


class DeviceLog:
    """
    The camera's log lines for /api/status. on_message is the depthai log callback (device.addLogCallback;
    called on depthai's logging thread with a LogMessage: nodeIdName, level, payload). Lines from the
    stream's own nodes (nodes: their "Type(id)" names) or any VideoEncoder, or that mention the
    depth stream, go to `stream` (the last DEVICE_LOG_LINES); the rest to `other`.
    """

    def __init__(self, nodes=(), clock=time.time, max_lines=DEVICE_LOG_LINES, other_lines=DEVICE_LOG_OTHER_LINES):
        self.nodes = set(nodes)
        self.clock = clock
        self._lock = threading.Lock()
        self.stream = deque(maxlen=max_lines)
        self.other = deque(maxlen=other_lines)
        self.total = 0

    def is_stream(self, node, text):
        return node in self.nodes or node.split("(")[0] in STREAM_NODE_TYPES or "depth stream" in text

    def on_message(self, message):
        try:
            node = str(getattr(message, "nodeIdName", "") or "")
            text = str(getattr(message, "payload", "") or "").strip()[:DEVICE_LOG_MAX_CHARS]
            line = {"unix_time": self.clock(), "node": node, "level": _level_name(getattr(message, "level", None)),
                    "text": text}
            with self._lock:
                self.total += 1
                (self.stream if self.is_stream(node, text) else self.other).append(line)
        except Exception:
            pass  # never raise into depthai's logging thread

    def snapshot(self):
        with self._lock:
            return {"stream": list(self.stream), "other": list(self.other), "total": self.total}


class DeviceLogDebug:
    """depth_stream_debug: start() raises the device log level to `level`, stop() restores the previous one."""

    def __init__(self, device, level, log=print):
        self.device = device
        self.level = level
        self.log = log
        self.saved = None
        self.active = False
        self.error = None

    def start(self):
        if self.active:
            return
        try:
            self.saved = self.device.getLogLevel()
            self.device.setLogLevel(self.level)
        except Exception as e:
            self.error = f"device log level not raised: {e}"
            self.log(f"Depth stream debug: {self.error}")
            return
        self.active = True
        self.log(f"Depth stream debug: device log level {_level_name(self.saved)} -> {_level_name(self.level)}")

    def stop(self):
        if not self.active:
            return
        self.active = False
        try:
            self.device.setLogLevel(self.saved)
        except Exception as e:
            self.error = f"device log level not restored: {e}"
            self.log(f"Depth stream debug: {self.error}")
            return
        self.log(f"Depth stream debug: device log level back to {_level_name(self.saved)}")

    def state(self):
        return {"active": self.active, "level": _level_name(self.level), "restore_level": _level_name(self.saved),
                "error": self.error}


class DepthStreamService:
    """
    VIO process side. tick(depth) once per VIO frame: at most every poll_interval one stat of the
    lease, and set_enabled(bool) when it changes (and every resend_interval, in case a control message
    was lost). on_frame(jpeg) with each encoded frame from the device: one atomic write of the JPEG.
    While streaming, the metadata (META_FILE, with the raw depth at the image centre) is rewritten at
    most every meta_interval.

    on_device_status(data) with each Script report, after the encoded frames of the same loop: while
    streaming, encoder_stalled is set (with last_error, logged once) when the Script's frame count rose for
    over ENCODER_STALL_S since its first report after the last JPEG, reported without a gap; the next JPEG
    or the end of the stream clears it.

    Variants: info["variant"] is the one running (info["requested_variant"] the configured one). With a
    host_encoder (HostJpegEncoder), host_jpeg is encoded on the Pi: tick(depth, sequence) feeds it and
    publishes its JPEGs and reports, and set_enabled (the camera's control) is not used. A requested "auto"
    falls back from the camera to host_jpeg the first time the encoder stalls or its frames are blank
    (fallback, logged once); reports and JPEGs from the camera are ignored after that. The metadata's
    encoding and source follow the variant running, so a fallback from mono_features shows depth.

    device_log (DeviceLog) adds the camera's log lines to the metadata; debug (DeviceLogDebug) is started
    while a camera variant streams and stopped when it stops.
    """

    def __init__(self, directory, set_enabled, info=None, lease_s=LEASE_S, poll_interval=0.5, resend_interval=5.0,
                 meta_interval=1.0, clock=time.time, log=print, host_encoder=None, device_log=None, debug=None):
        self.directory = Path(directory)
        self.set_enabled = set_enabled
        self.info = dict(info or {})
        self.host_encoder = host_encoder
        self.device_log = device_log
        self.debug = debug
        self.variant = self.info.get("variant") or ""
        self.requested_variant = self.info.get("requested_variant") or self.variant or None
        self.fallback = None
        self.lease_s = lease_s
        self.poll_interval = poll_interval
        self.resend_interval = resend_interval
        self.meta_interval = meta_interval
        self.clock = clock
        self.log = log
        self.enabled = False
        self.frames = 0
        self.bytes = 0
        self.last_frame = None  # (unix time, bytes, sequence)
        self.center = None
        self.errors = 0
        self.last_error = None
        self.device_stats = None
        self.encoder_stalled = False
        self._stall_base = None  # (unix time, Script frame count) of its first report since the last JPEG
        self._last_report = None
        self.blank = None
        self.bad_frames = 0
        self._last_jpeg = None
        self._last_decode = self._last_expect = float("-inf")
        self._expected = None  # (unix time, valid fraction, gray std, varies) of the host's depth
        self._blank_since, self._blank_samples, self._blank_level = None, 0, None
        reason = self.info.get("variant_reason")
        if reason and self.variant == HOST_JPEG and self.requested_variant not in (HOST_JPEG, None):
            self.fallback = {"from": self.requested_variant, "to": HOST_JPEG, "reason": reason, "detail": None,
                             "unix_time": clock(), "at_start": True}
            log(f"Depth stream: {self.requested_variant} requested, encoding on the Pi (host_jpeg): {reason}")
        fps = float(self.info.get("fps_target") or 0.0)
        self._report_gap = max(ENCODER_REPORT_GAP, ENCODER_REPORT_GAP / fps if fps > 0 else 0.0)
        self.recent = deque()
        self._last_poll = self._last_send = self._last_meta = float("-inf")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            for name in (FRAME_FILE, META_FILE):
                (self.directory / name).unlink(missing_ok=True)  # never show a previous run's frame
        except OSError as e:
            log(f"Depth stream directory {self.directory} unusable: {e}")
        self.write_meta()

    @property
    def on_host(self):
        return self.variant == HOST_JPEG and self.host_encoder is not None

    def tick(self, depth=None, sequence=None, center=None, sample=None):
        """
        depth: this frame's depth (None in points mode, except the on-demand frames host_jpeg asked for);
        center (a center_depth dict) and sample (the feature depths, mm) stand in for it in points mode.
        """
        now = self.clock()
        if now - self._last_poll >= self.poll_interval:
            self._last_poll = now
            wanted = _lease_fresh(self.directory / LEASE_FILE, now, self.lease_s)
            if wanted != self.enabled or now - self._last_send >= self.resend_interval:
                changed = wanted != self.enabled
                if changed:
                    self.log(f"Depth stream {'started: a client is watching' if wanted else 'stopped: no client'}")
                self._send(wanted, now)
                if changed and self.enabled == wanted:
                    self._last_meta = float("-inf")  # publish the change right away
                    if not wanted:
                        self.write_meta(now)
        if self.on_host:
            if self.enabled:
                self.host_encoder.submit(depth, sequence)
            for jpeg, frame_sequence in self.host_encoder.drain():
                self._store_frame(jpeg, frame_sequence)
            for report in self.host_encoder.reports():
                self._on_status(report)
        elif self.enabled and now - self._last_expect >= EXPECT_INTERVAL_S:
            source = depth if depth is not None else sample
            if source is not None:
                self._last_expect = now
                self._expect(source, now)
        if self.enabled and now - self._last_meta >= self.meta_interval:
            if center is not None:
                self.center = center
            elif depth is not None:
                self.center = center_depth(depth)
            self.write_meta(now)

    def _send(self, enabled, now):
        try:
            if self.on_host:
                self.host_encoder.set_enabled(enabled)
            else:
                self.set_enabled(enabled)
        except Exception as e:
            self.log(f"Depth stream control not sent: {e}")
            return
        if enabled != self.enabled:
            self.encoder_stalled = False
            self._stall_base = None
            self._last_jpeg = None
            self._blank_since, self._blank_samples = None, 0
            if self.debug is not None:
                if enabled and not self.on_host:
                    self.debug.start()
                else:
                    self.debug.stop()
        self.enabled = enabled
        self._last_send = now

    def _fall_back(self, reason, now, detail=None):
        """auto: the camera's encoder stalled or sends blank frames; encode on the Pi from now on."""
        previous, self.variant = self.variant, HOST_JPEG
        self.fallback = {"from": previous, "to": HOST_JPEG, "reason": reason, "detail": detail, "unix_time": now}
        self.log(f"Depth stream: falling back from {previous} (camera) to host_jpeg (encoded on the Pi): {reason}")
        try:
            self.set_enabled(False)
        except Exception as e:
            self.log(f"Depth stream control not sent: {e}")
        if self.debug is not None:
            self.debug.stop()
        self.encoder_stalled = False
        self._stall_base = self._last_report = None
        self.host_encoder.set_enabled(self.enabled)

    def on_frame(self, jpeg, sequence=None):
        """An encoded frame from the camera (ignored once encoding moved to the Pi); True if it was published."""
        if self.on_host:
            return False
        data = bytes(jpeg)
        if data[:2] != b"\xff\xd8":  # empty or not a JPEG: not a frame (the stall watch keeps counting)
            self.bad_frames += 1
            if self.bad_frames == 1 or self.bad_frames % 100 == 0:
                self.log(f"Depth stream: {self.bad_frames} frames from the camera encoder were not JPEGs "
                         f"({len(data)} bytes)")
            return False
        if not self._store_frame(data, sequence):
            return False
        self._check_blank(data, self.clock())
        return True

    def _expect(self, depth, now):
        """Whether the host's depth of about now (a frame, or the feature depths) would make a varying stream image."""
        sample = np.asarray(depth)
        if sample.ndim == 2:
            sample = sample[::4, ::4]
        elif sample.ndim != 1:
            return
        if not sample.size:
            return
        valid = float(np.count_nonzero(sample)) / sample.size
        min_mm, max_mm = self.info.get("depth_range_mm") or (1, 65535)
        std = float(gray_reference(sample, min_mm, max_mm).std())
        self._expected = (now, valid, std, valid >= EXPECT_VALID_MIN and std >= EXPECT_GRAY_STD_MIN)

    @staticmethod
    def _gray_level(data):
        """(constant, mean gray) of a JPEG; an undecodable one counts as constant."""
        import cv2
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or not image.size:
            return True, None
        return float(image.std()) < BLANK_STD, round(float(image.mean()), 1)

    def _check_blank(self, data, now):
        identical = data == self._last_jpeg
        self._last_jpeg = data
        if identical:
            constant = True
        elif now - self._last_decode >= BLANK_SAMPLE_S:
            self._last_decode = now
            constant, self._blank_level = self._gray_level(data)
        else:
            return
        if not constant:
            if self.blank is not None:
                self.log(f"Depth stream: frames from {self.variant} are not blank any more")
                self.blank = None
            self._blank_since, self._blank_samples = None, 0
            return
        if self._blank_since is None:
            self._blank_since, self._blank_samples = now, 0
        self._blank_samples += 1
        expected = self._expected
        varies = expected is not None and now - expected[0] <= EXPECT_MAX_AGE_S and expected[3]
        if (self.blank is not None or not varies or self._blank_samples < BLANK_MIN_SAMPLES
                or now - self._blank_since < BLANK_S):
            return
        variant = self.variant or "unknown"
        level = "undecodable" if self._blank_level is None else f"gray {self._blank_level:g}"
        text = (f"blank frames from {variant}: {self._blank_samples} constant JPEGs ({len(data)} bytes, {level}) in "
                f"{now - self._blank_since:.1f} s while the camera depth varies ({100 * expected[1]:.0f} % valid)")
        self.blank = {"unix_time": now, "variant": variant, "frames": self._blank_samples, "bytes": len(data),
                      "gray": self._blank_level, "text": text}
        self.log(f"Depth stream: {text}")
        self.last_error = {"text": text, "unix_time": now, "fatal": False, "errors": self.errors,
                           "frames_sent": None, "variant": variant, "blank": True}
        if self.requested_variant == AUTO and self.host_encoder is not None:
            self._fall_back(f"blank frames from {variant}", now, detail=text)
        self.write_meta(now)

    def _store_frame(self, jpeg, sequence):
        now = self.clock()
        data = jpeg if isinstance(jpeg, bytes) else bytes(jpeg)
        try:
            snapshots.atomic_write(self.directory / FRAME_FILE, data)
        except OSError as e:
            self.log(f"Depth stream frame not written: {e}")
            return False
        self.frames += 1
        self.bytes += len(data)
        self.last_frame = (now, len(data), sequence)
        self.recent.append((now, len(data)))
        while self.recent and now - self.recent[0][0] > RATE_WINDOW_S:
            self.recent.popleft()
        self._stall_base = None
        if self.encoder_stalled:
            self.encoder_stalled = False
            self.log("Depth stream: encoded frames arrive again")
        return True

    def on_device_status(self, data):
        """A 'status' message from the device Script: errors (count, last text and time) or its own timing."""
        if self.on_host:
            return
        self._on_status(data)

    def _on_status(self, data):
        now = self.clock()
        status = parse_device_status(data)
        if status is None:
            self.log(f"Depth stream: unreadable status from the {'Pi encoder' if self.on_host else 'camera'}: "
                     f"{bytes(data)[:80]!r}")
            return
        self.errors = max(self.errors, status["errors"])
        self._watch_encoder(status["sent"], now)
        if status["kind"] == "stats":
            self.device_stats = {"unix_time": now, "frames_sent": status["sent"], "size": status["size"],
                                 "convert_ms_mean": status["convert_ms_mean"],
                                 "convert_ms_max": status["convert_ms_max"]}
        elif status["kind"] != "sent":
            fatal = status["kind"] == "fatal"
            where = "the Pi encoder" if self.on_host else "the camera"
            if fatal or self.last_error is None or self.last_error["text"] != status["text"]:
                self.log(f"Depth stream {'stopped on' if fatal else 'error on'} {where} "
                         f"({status['errors']} errors, {status['sent']} frames sent): {status['text']}")
            self.last_error = {"text": status["text"], "unix_time": now, "fatal": fatal, "errors": status["errors"],
                               "frames_sent": status["sent"], "variant": self.variant or None}
        self.write_meta(now)

    def _watch_encoder(self, sent, now):
        last, self._last_report = self._last_report, now
        if not self.enabled:
            return
        base = self._stall_base
        if base is None or sent < base[1] or last is None or now - last > self._report_gap:
            self._stall_base = (now, sent)  # first report since the last JPEG, a restarted Script, or a pause
            return
        if self.encoder_stalled or sent <= base[1] or now - base[0] <= ENCODER_STALL_S:
            return
        self.encoder_stalled = True
        if self.on_host:
            text = (f"the Pi took {sent - base[1]} frames in {now - base[0]:.1f} s but encoded none "
                    f"(host_jpeg encoder stuck)")
        else:
            text = (f"{ENCODER_STALL_TEXT}: {sent - base[1]} frames sent in {now - base[0]:.1f} s, no JPEG back "
                    f"(variant {self.variant or 'unknown'}; the camera log has the VideoEncoder's reason)")
        self.log(f"Depth stream: {text}")
        self.last_error = {"text": text, "unix_time": now, "fatal": False, "errors": self.errors,
                           "frames_sent": sent, "variant": self.variant or None, "stall": True}
        if self.requested_variant == AUTO and self.host_encoder is not None and not self.on_host:
            self._fall_back(f"no JPEG from {self.variant or 'unknown'} (the camera encoder outputs nothing)", now,
                            detail=text)

    def rates(self, now):
        recent = [(t, size) for t, size in self.recent if now - t <= RATE_WINDOW_S]
        if len(recent) < 2:
            return None, None
        span = max(recent[-1][0] - recent[0][0], 1e-6)
        return (len(recent) - 1) / span, 8.0 * sum(size for _, size in recent[1:]) / span / 1000.0

    def meta(self, now=None):
        now = self.clock() if now is None else now
        fps, kbps = self.rates(now)
        last = self.last_frame
        return {**self.info, "encoding": encoding_for(self.variant), "source": source_for(self.variant),
                "blank": self.blank, "bad_frames": self.bad_frames,
                "pid": os.getpid(), "unix_time": now, "streaming": self.enabled,
                "frames_total": self.frames, "bytes_total": self.bytes, "fps": fps, "kbps": kbps,
                "last_frame": None if last is None else {"unix_time": last[0], "bytes": last[1], "sequence": last[2]},
                "center": self.center, "errors": self.errors, "last_error": self.last_error,
                "encoder_stalled": self.encoder_stalled, "device_stats": self.device_stats,
                "variant": self.variant or None, "requested_variant": self.requested_variant,
                "encoded_on": "pi" if self.on_host else "camera", "fallback": self.fallback,
                "host_encoder": self.host_encoder.stats() if self.on_host else None,
                "debug": self.debug is not None, "debug_state": None if self.debug is None else self.debug.state(),
                "device_log": None if self.device_log is None else self.device_log.snapshot()}

    def write_meta(self, now=None):
        now = self.clock() if now is None else now
        self._last_meta = now
        try:
            snapshots.atomic_write(self.directory / META_FILE, json.dumps(self.meta(now)).encode())
        except OSError as e:
            self.log(f"Depth stream metadata not written: {e}")

    def close(self):
        """The pipeline is stopping (and the Script with it): only record that nothing streams any more."""
        self.enabled = False
        if self.debug is not None:
            self.debug.stop()
        if self.host_encoder is not None:
            self.host_encoder.close()
        self.write_meta()


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(directory):
    """Ask the VIO process to stream for LEASE_S."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / LEASE_FILE).touch()
    except OSError:
        pass


def read_meta(directory):
    try:
        return json.loads((Path(directory) / META_FILE).read_text())
    except (OSError, ValueError):
        return None


class FrameReader:
    """The latest JPEG in DIRECTORY/depth_stream.jpg, returned once per new file and only if fresh."""

    def __init__(self, directory, max_age_s=3.0, clock=time.time):
        self.path = Path(directory) / FRAME_FILE
        self.max_age_s = max_age_s
        self.clock = clock
        self._key = None

    def read_new(self):
        try:
            stat = self.path.stat()
        except OSError:
            return None
        key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        if key == self._key:
            return None
        self._key = key
        if self.clock() - stat.st_mtime > self.max_age_s:
            return None  # left over from a VIO process that stopped
        try:
            data = self.path.read_bytes()
        except OSError:
            return None
        return data if data[:2] == b"\xff\xd8" else None


def multipart_part(jpeg, boundary=BOUNDARY):
    return (f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode()
            + jpeg + b"\r\n")
