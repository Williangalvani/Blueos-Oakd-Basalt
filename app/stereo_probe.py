#!/usr/bin/env python3
"""
Stereo/encoder probe: runs short pipelines on the real camera with a few StereoDepth configurations and
reports, per case, what StereoDepth outputs and whether the camera's MJPEG encoder can encode it.

Takes pnp_vio.py's camera and stereo arguments (main.py builds both command lines from the saved config),
configures StereoDepth with stereo_config.apply_stereo_config, then per case:
- reads the applied config back from stereo.initialConfig (subpixel, extended, LR check, median, every
  post-processing filter, filteringOrder, disparityWidth, companding, disparityShift) and getMaxDisparity(),
  and what stereo_config.disparity_format (the depth stream's disparity precondition) makes of it;
- samples stereo.disparity, stereo.depth, rectifiedLeft and rectifiedRight on small non-blocking host
  queues: ImgFrame type, size, stride, bytes, and value statistics of a few frames;
- encodes sources two ways, source -> ImageManip (NV12, own pool) -> VideoEncoder MJPEG and source ->
  VideoEncoder, and counts the JPEGs, their sizes and the decoded gray spread;
- keeps the camera's warning/error log lines (depth_stream.DeviceLog on device.addLogCallback).

One device open per case: depthai v3 closes the device when a pipeline stops. A failing case records its
exception and the next case runs. The JSON goes to --output after every case (a killed probe leaves the
cases so far) and once to stdout at the end; everything else goes to stderr.

--mode mjpeg_loss (run_mjpeg_loss; the analysis is mjpeg_loss.py): how much depth the MJPEG encoding of the RAW8
disparity loses. Run A: case G's settings (subpixel off, median last, speckle on), the raw disparity and one MJPEG
encoder per --qualities on the same frames, the VIO's FeatureTracker (pnp_vio.create_feature_tracker) and every
n-th stereo.depth frame; then run B: the saved settings (the VIO's, subpixel on), every n-th depth frame and the
features. A quality whose JPEGs do not come with run A's frames (the encoders did not all run at once) is measured
again alone in its own run afterwards.

Run in the container while the VIO is stopped (POST /api/stereo_probe does that), or by hand:
    PYTHONPATH=/app/depthai-python python /app/stereo_probe.py --width 640 --height 400 --fps 15 --cases A,E
    PYTHONPATH=/app/depthai-python python /app/stereo_probe.py --mode mjpeg_loss --qualities 70,95 --seconds 10
The pure helpers below need neither depthai nor a camera.
"""

import argparse
import json
import math
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import depth_stream
import mjpeg_loss
import snapshots
from pnp_odometry import PnPParams
from rectified_intrinsics import as_dict, rectified_for_settings
from stereo_config import (FILTER_RUNS, FILTERING_ORDER_MEDIAN_LAST, add_stereo_arguments,  # noqa: F401
                           apply_stereo_config, disparity_format, enum_name, executed_filters,
                           predicted_max_disparity, read_back, resolve_stereo_settings)

CASES_MODE, MJPEG_LOSS = "cases", "mjpeg_loss"
MODES = (CASES_MODE, MJPEG_LOSS)
SAMPLE_SECONDS = 4.5
WARMUP_S = 1.0
SAMPLES = 3
# The whole probe stops starting cases after this (POST /api/stereo_probe kills it later than that)
TIME_BUDGET_S = 85.0
OPEN_TIMEOUT_S = 25.0
# A case is skipped if less than this is left of the budget
MIN_CASE_S = 8.0
# StereoDepth outputs here feed a host queue, an ImageManip and an encoder at once
STEREO_FRAMES_POOL = 6
HOST_QUEUE_SIZE = 2
ENCODE_QUEUE_SIZE = 4
MAX_SIZES = 60
LOG_LINES = 400
LOG_DISTINCT = 40
POLL_S = 0.02

# mjpeg_loss: capture time per run (after WARMUP_S), and the range POST /api/stereo_probe accepts
LOSS_SECONDS = 10.0
LOSS_SECONDS_RANGE = (3.0, 30.0)
# Per run on top of the warmup and the capture: device open, pipeline start, analysis (opens take 2-10 s)
RUN_OVERHEAD_S = 15.0
# A quality ran alongside the others when its JPEGs pair with at least this share of run A's raw frames
ENCODER_MIN_SHARE = 0.5
LOSS_QUEUE_SIZE = 8
# The disparity feeds the host queue and one encoder per quality, each holding a frame
LOSS_STEREO_POOL = 4
# Raw disparity frames kept for the JPEG comparison (about 256 kB each at 640x400), and depth frames for the
# temporal medians of the subpixel comparison and the conversion check (every n-th frame)
MAX_FRAMES = 300
REFERENCE_FRAMES = 20
# What the VIO gets from main.py by default (pnp_vio.parse_args; corner detector and depth range are not saved)
CORNER_DETECTOR_CHOICES = ("harris", "shi-tomasi")

DISPARITY = "disparity"
STREAMS = (DISPARITY, "depth", "rectifiedLeft", "rectifiedRight")
RECTIFIED = ("rectifiedLeft", "rectifiedRight")
ENCODE_PATHS = ("manip", "direct")
BYTES_PER_PIXEL = {"RAW8": 1, "GRAY8": 1, "YUV400p": 1, "NV12": 1, "RAW16": 2, "GRAYF16": 2}
MEDIAN_ON = "on"  # case override: the saved median, or 7x7 if the saved one is off
NO_FILTERS = {"stereo_speckle_filter": False, "stereo_spatial_filter": False, "stereo_temporal_filter": False,
              "stereo_decimation": 1}


@dataclass(frozen=True)
class Case:
    name: str
    description: str
    overrides: dict
    encode_sources: tuple = (DISPARITY,)


CASES = (
    Case("A", "saved settings, subpixel off", {"stereo_subpixel": False}, (DISPARITY, *RECTIFIED)),
    Case("B", "A with the speckle filter off", {"stereo_subpixel": False, "stereo_speckle_filter": False}),
    Case("C", "A with every post-processing filter and the median off",
         {"stereo_subpixel": False, **NO_FILTERS, "stereo_median": "off"}),
    Case("D", "A with only the median on (median runs last)",
         {"stereo_subpixel": False, **NO_FILTERS, "stereo_median": MEDIAN_ON}),
    Case("E", "saved settings, subpixel on", {"stereo_subpixel": True}, (DISPARITY, *RECTIFIED)),
    Case("F", "preset NONE, subpixel off, no filters",
         {"stereo_preset": "NONE", "stereo_subpixel": False, **NO_FILTERS, "stereo_median": "off"}),
    Case("G", f"A with the speckle filter on and the median last (filteringOrder "
              f"{', '.join(FILTERING_ORDER_MEDIAN_LAST)})",
         {"stereo_subpixel": False, "stereo_speckle_filter": True, "stereo_median": MEDIAN_ON,
          "stereo_median_last": True}),
)
CASES_BY_NAME = {case.name: case for case in CASES}


def parse_case_names(text):
    """'A,c, E' -> ['A', 'C', 'E'] in the given order; None or empty -> every case. ValueError if unknown."""
    if text is None or not str(text).strip():
        return [case.name for case in CASES]
    names = [part.strip().upper() for part in str(text).split(",") if part.strip()]
    unknown = [name for name in names if name not in CASES_BY_NAME]
    if unknown:
        raise ValueError(f"unknown case(s) {', '.join(unknown)}; the cases are {', '.join(CASES_BY_NAME)}")
    return list(dict.fromkeys(names))


def case_settings(saved, case):
    """The resolved stereo settings of a case: the saved ones with the case's overrides."""
    merged = {**saved, **case.overrides}
    if merged.get("stereo_median") == MEDIAN_ON:
        merged["stereo_median"] = saved.get("stereo_median") if saved.get("stereo_median") not in (None, "off") else "7x7"
    return resolve_stereo_settings(merged)


def frame_array(data, type_name, width, height, stride):
    """The image of an ImgFrame's bytes as a (height, width) uint8 or uint16 array (first plane only)."""
    buf = np.asarray(data, dtype=np.uint8).ravel()
    bpp = BYTES_PER_PIXEL.get(type_name)
    if bpp is None:
        bpp = 2 if stride and stride >= 2 * width else 1
    row = width * bpp
    pitch = stride if stride and stride >= row else row
    if width <= 0 or height <= 0 or buf.size < pitch * (height - 1) + row:
        raise ValueError(f"{buf.size} bytes is too short for {width}x{height} {type_name} with stride {stride}")
    if buf.size < pitch * height:
        buf = np.concatenate([buf, np.zeros(pitch * height - buf.size, np.uint8)])
    rows = buf[:pitch * height].reshape(height, pitch)[:, :row]
    return np.ascontiguousarray(rows).view("<u2" if bpp == 2 else np.uint8)


def value_stats(image):
    image = np.asarray(image)
    if not image.size:
        return {"min": None, "max": None, "mean": None, "distinct": 0, "zero_pct": None, "max_bits": None}
    top = int(image.max())
    return {"min": int(image.min()), "max": top, "mean": round(float(image.mean()), 3),
            "distinct": int(np.unique(image).size), "zero_pct": round(100.0 * float(np.mean(image == 0)), 2),
            "max_bits": top.bit_length()}


def frame_info(msg):
    """Type, size, stride, byte count and value statistics of a host-side ImgFrame."""
    info = {}
    try:
        data = msg.getData()
        info.update(type=enum_name(msg.getType()), width=int(msg.getWidth()), height=int(msg.getHeight()),
                    stride=int(msg.getStride()), bytes=int(len(data)), sequence=int(msg.getSequenceNum()))
        info.update(value_stats(frame_array(data, info["type"], info["width"], info["height"], info["stride"])))
    except Exception as e:
        info["error"] = f"{type(e).__name__}: {e}"
    return info


def decode_gray(jpeg):
    """(std, mean) of a JPEG decoded to gray, or None if it does not decode (or OpenCV is missing)."""
    try:
        import cv2
    except ImportError:
        return None
    image = cv2.imdecode(np.frombuffer(bytes(jpeg), np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None or not image.size:
        return None
    return round(float(image.std()), 3), round(float(image.mean()), 2)


def jpeg_summary(sizes, identical, starts_ok, samples, decode=decode_gray):
    """An encode path's result: JPEG count, sizes, repeats and the decoded gray of the sampled JPEGs."""
    decoded = []
    for jpeg in samples:
        gray = decode(jpeg)
        decoded.append({"bytes": len(jpeg), "decoded": gray is not None,
                        "gray_std": gray[0] if gray else None, "gray_mean": gray[1] if gray else None})
    return {"jpegs": len(sizes), "jpeg_header_ok": starts_ok, "identical_to_previous": identical,
            "size_min": min(sizes) if sizes else None, "size_max": max(sizes) if sizes else None,
            "distinct_sizes": len(set(sizes)), "sizes": sizes[:MAX_SIZES], "samples": decoded}


def summarize_log(lines, limit=LOG_DISTINCT):
    """Device log lines grouped by (node, level, text): count and first time, in order of appearance."""
    groups = {}
    for line in lines:
        key = (line.get("node"), line.get("level"), line.get("text"))
        if key not in groups:
            groups[key] = {"node": key[0], "level": key[1], "text": key[2], "count": 0,
                           "first_unix_time": line.get("unix_time")}
        groups[key]["count"] += 1
    distinct = list(groups.values())
    return {"lines": len(lines), "distinct": len(distinct), "groups": distinct[:limit],
            "truncated": len(distinct) > limit}


def case_summary(result):
    """One line per case: disparity type/max and the JPEG counts of each encode path."""
    parts = []
    for sample in (result.get("streams", {}).get(DISPARITY, {}).get("samples") or [])[-1:]:
        parts.append(f"disparity {sample.get('type')} max {sample.get('max')} ({sample.get('distinct')} values)")
    parts.append(f"maxDisparity {result.get('applied', {}).get('maxDisparity')}")
    for source, paths in (result.get("encode") or {}).items():
        for path, encoded in paths.items():
            stds = [s["gray_std"] for s in encoded.get("samples", []) if s.get("gray_std") is not None]
            spread = f", gray std {max(stds):.2f}" if stds else ""
            parts.append(f"{source}/{path} {encoded.get('jpegs', 0)} JPEG{spread}")
    if result.get("error"):
        parts.append(f"error: {result['error']}")
    return "; ".join(parts)


# --- on the camera --------------------------------------------------------------------------------------

def _log(message):
    print(message, file=sys.stderr, flush=True)


def _node_name(node):
    """How the device log names a node: "Type(id)" (as pnp_vio.node_log_name)."""
    try:
        return f"{node.getName()}({node.id})"
    except Exception:
        return None


class _EncodePath:
    def __init__(self, queue):
        self.queue = queue
        self.sizes, self.samples = [], []
        self.identical = self.starts_ok = 0
        self.last = None

    def drain(self):
        for msg in self.queue.tryGetAll():
            data = bytes(msg.getData())
            self.sizes.append(len(data))
            self.starts_ok += data[:2] == b"\xff\xd8"
            self.identical += data == self.last
            self.last = data

    def sample(self):
        if self.last is not None and len(self.samples) < SAMPLES:
            self.samples.append(self.last)


def _build_encoder(p, dai, source, args, manip):
    encoder = p.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(float(args.fps), dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(args.quality)
    encoder.input.setBlocking(False)
    encoder.input.setMaxSize(1)
    nodes = [encoder]
    if manip:
        node = p.create(dai.node.ImageManip)
        node.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        node.setMaxOutputFrameSize(depth_stream.manip_output_size(args.width, args.height))
        node.setNumFramesPool(depth_stream.MANIP_FRAMES_POOL)
        node.inputImage.setBlocking(False)
        node.inputImage.setMaxSize(1)
        source.link(node.inputImage)
        node.out.link(encoder.input)
        nodes.append(node)
    else:
        source.link(encoder.input)
    return encoder.out.createOutputQueue(maxSize=ENCODE_QUEUE_SIZE, blocking=False), nodes


def run_case(case, saved, args, dai, open_device, clock=time.monotonic, sleep=time.sleep):
    """One case on a freshly opened device; returns its result (with "error" if it failed part-way)."""
    settings = case_settings(saved, case)
    result = {"name": case.name, "description": case.description, "settings": settings,
              "encode_sources": list(case.encode_sources), "error": None}
    started = clock()
    device = log_callback = None
    device_log = depth_stream.DeviceLog(clock=time.time, max_lines=LOG_LINES, other_lines=LOG_LINES)
    try:
        device = open_device()
        result["device_open_s"] = round(clock() - started, 2)
        try:
            result["device_name"] = device.getDeviceName()
        except Exception:
            pass
        try:
            device.setLogLevel(dai.LogLevel.WARN)
            log_callback = device.addLogCallback(device_log.on_message)
        except Exception as e:
            result["device_log_error"] = str(e)
        with dai.Pipeline(device) as p:
            left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
            right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
            stereo = p.create(dai.node.StereoDepth)
            apply_stereo_config(stereo, settings, dai)
            stereo.setNumFramesPool(STEREO_FRAMES_POOL)
            left.requestOutput((args.width, args.height)).link(stereo.left)
            right.requestOutput((args.width, args.height)).link(stereo.right)
            result["applied"] = read_back(stereo.initialConfig)
            result["executed_filters"] = executed_filters(result["applied"])
            result["predicted_max_disparity"] = predicted_max_disparity(result["applied"])
            result["disparity_format"] = disparity_format(result["applied"], settings)

            outputs = {name: getattr(stereo, name) for name in STREAMS}
            streams = {name: outputs[name].createOutputQueue(maxSize=HOST_QUEUE_SIZE, blocking=False)
                       for name in STREAMS}
            encode, nodes = {}, []
            for source in case.encode_sources:
                for path in ENCODE_PATHS:
                    queue, made = _build_encoder(p, dai, outputs[source], args, manip=path == "manip")
                    encode[(source, path)] = _EncodePath(queue)
                    nodes += made
            device_log.nodes = {name for name in map(_node_name, nodes) if name}

            p.start()
            run_started = clock()
            result["pipeline_start_s"] = round(run_started - started, 2)
            after_start = read_back(stereo.initialConfig)
            result["changed_at_start"] = {k: v for k, v in after_start.items() if result["applied"].get(k) != v}
            counts = {name: 0 for name in STREAMS}
            latest = {name: None for name in STREAMS}
            samples = {name: [] for name in STREAMS}
            next_sample = run_started + WARMUP_S
            interval = max((args.seconds - WARMUP_S) / SAMPLES, 0.1)
            while clock() - run_started < args.seconds:
                for name, queue in streams.items():
                    msgs = queue.tryGetAll()
                    if msgs:
                        counts[name] += len(msgs)
                        latest[name] = msgs[-1]
                for path in encode.values():
                    path.drain()
                if clock() >= next_sample:
                    next_sample += interval
                    for name in STREAMS:
                        if latest[name] is not None and len(samples[name]) < SAMPLES:
                            samples[name].append(frame_info(latest[name]))
                    for path in encode.values():
                        path.sample()
                sleep(POLL_S)
            for path in encode.values():
                path.drain()
                if not path.samples:
                    path.sample()
            elapsed = clock() - run_started
            result["streams"] = {name: {"frames": counts[name], "fps": round(counts[name] / elapsed, 2),
                                        "samples": samples[name]} for name in STREAMS}
            result["encode"] = {}
            for (source, path_name), path in encode.items():
                result["encode"].setdefault(source, {})[path_name] = jpeg_summary(
                    path.sizes, path.identical, path.starts_ok, path.samples)
    except Exception as e:
        result["error"] = f"{type(e).__name__}: {e}"
        result["traceback"] = traceback.format_exc(limit=6)
        _log(f"Case {case.name} failed: {result['error']}")
    finally:
        if device is not None:
            if log_callback is not None:
                try:
                    device.removeLogCallback(log_callback)
                except Exception:
                    pass
            try:
                device.close()
            except Exception:
                pass
    lines = device_log.snapshot()
    result["device_log"] = summarize_log(sorted(lines["stream"] + lines["other"],
                                                key=lambda line: line.get("unix_time") or 0))
    result["duration_s"] = round(clock() - started, 2)
    result["summary"] = case_summary(result)
    return result


# --- mjpeg_loss mode ------------------------------------------------------------------------------------

def run_cost_s(seconds):
    return WARMUP_S + seconds + RUN_OVERHEAD_S


def mjpeg_loss_budget(seconds, qualities):
    """No run starts later than this: runs A and B, and one run per quality if the encoders do not run at once."""
    return (2 + qualities) * run_cost_s(seconds)


def mjpeg_loss_timeout(seconds, qualities):
    """When POST /api/stereo_probe kills the probe: the budget, a last run with a slow open, and some margin."""
    return mjpeg_loss_budget(seconds, qualities) + OPEN_TIMEOUT_S + WARMUP_S + seconds + 30.0


def reference_every(fps, seconds, frames=REFERENCE_FRAMES):
    """Keep the depth frames whose sequence number is a multiple of this: about `frames` of them per run."""
    return max(1, int(round(fps * seconds / frames)))


def calibration_info(calibration, rectified, problem):
    """The fx (rectified, else raw: as the VIO's disparity stream) and baseline the depth conversion uses."""
    focal = rectified or calibration.raw
    return {"fx": float(focal.fx), "fx_source": "rectified" if rectified is not None else f"raw ({problem})",
            "rectified": as_dict(rectified), "raw": as_dict(calibration.raw), "baseline_m": calibration.baseline_m,
            "baseline_spec_m": calibration.baseline_spec_m,
            "depth_times_disparity_mm": mjpeg_loss.depth_times_disparity_mm(focal.fx, calibration.baseline_m)}


def _device_seconds(msg):
    try:
        return msg.getTimestampDevice().total_seconds()
    except Exception:
        return None


def _record(msg, kind, keep_image):
    record = {"seq": int(msg.getSequenceNum()), "ts": _device_seconds(msg)}
    if kind == "features":
        features = list(msg.trackedFeatures)
        record["ids"] = np.array([int(f.id) for f in features], dtype=np.int64)
        record["uv"] = np.array([(f.position.x, f.position.y) for f in features], dtype=np.float64).reshape(-1, 2)
    elif kind == "jpeg":
        record["data"] = bytes(msg.getData())
    else:
        record["type"] = enum_name(msg.getType())
        record["image"] = None
        if keep_image:
            record["image"] = frame_array(msg.getData(), record["type"], int(msg.getWidth()), int(msg.getHeight()),
                                          int(msg.getStride()))
    return record


def capture_run(name, description, settings, args, dai, open_device, vio, qualities=(), disparity=True,
                clock=time.monotonic, sleep=time.sleep):
    """
    One mjpeg_loss pipeline run on a freshly opened device: StereoDepth with `settings`, the VIO's FeatureTracker
    on rectifiedLeft, and to the host the features, every n-th depth frame and (disparity) every disparity frame,
    raw and through one MJPEG encoder per quality. Returns (result, records); records holds what came after the
    warmup.
    """
    result = {"name": name, "description": description, "settings": settings, "qualities": list(qualities),
              "error": None}
    records = {"disparity": [], "depth": [], "features": [], "jpeg": {q: [] for q in qualities}}
    every = reference_every(args.fps, args.seconds)
    started = clock()
    device = log_callback = None
    device_log = depth_stream.DeviceLog(clock=time.time, max_lines=LOG_LINES, other_lines=LOG_LINES)
    try:
        device = open_device()
        result["device_open_s"] = round(clock() - started, 2)
        try:
            result["device_name"] = device.getDeviceName()
        except Exception:
            pass
        try:
            device.setLogLevel(dai.LogLevel.WARN)
            log_callback = device.addLogCallback(device_log.on_message)
        except Exception as e:
            result["device_log_error"] = str(e)
        calibration = vio.load_calibration(device, argparse.Namespace(
            width=args.width, height=args.height, intrinsics_from="rectified", gyro_gate=0))
        rectified, problem = rectified_for_settings(calibration, settings, (args.width, args.height))
        result["calibration"] = calibration_info(calibration, rectified, problem)
        with dai.Pipeline(device) as p:
            left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
            right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
            stereo = p.create(dai.node.StereoDepth)
            apply_stereo_config(stereo, settings, dai)
            stereo.setNumFramesPool(LOSS_STEREO_POOL + len(qualities))
            left.requestOutput((args.width, args.height)).link(stereo.left)
            right.requestOutput((args.width, args.height)).link(stereo.right)
            tracker = vio.create_feature_tracker(p, args)
            stereo.rectifiedLeft.link(tracker.inputImage)
            result["applied"] = read_back(stereo.initialConfig)
            result["executed_filters"] = executed_filters(result["applied"])
            result["predicted_max_disparity"] = predicted_max_disparity(result["applied"])
            result["disparity_format"] = disparity_format(result["applied"], settings)

            queues = {"features": tracker.outputFeatures, "depth": stereo.depth}
            if disparity:
                queues["disparity"] = stereo.disparity
            queues = {kind: out.createOutputQueue(maxSize=LOSS_QUEUE_SIZE, blocking=False)
                      for kind, out in queues.items()}
            encoders, nodes = {}, []
            for quality in qualities:
                encoder = p.create(dai.node.VideoEncoder)
                encoder.setDefaultProfilePreset(float(args.fps), dai.VideoEncoderProperties.Profile.MJPEG)
                encoder.setQuality(quality)
                encoder.input.setBlocking(False)
                encoder.input.setMaxSize(1)
                stereo.disparity.link(encoder.input)
                encoders[quality] = encoder.out.createOutputQueue(maxSize=LOSS_QUEUE_SIZE, blocking=False)
                nodes.append(encoder)
            device_log.nodes = {n for n in map(_node_name, nodes + [stereo, tracker]) if n}

            p.start()
            run_started = clock()
            result["pipeline_start_s"] = round(run_started - started, 2)
            warm_until = run_started + WARMUP_S
            counts = {kind: 0 for kind in queues}
            jpeg_counts = {quality: 0 for quality in qualities}
            images = {"disparity": 0, "depth": 0}
            while clock() < warm_until + args.seconds:
                warm = clock() >= warm_until
                for kind, queue in queues.items():
                    for msg in queue.tryGetAll():
                        counts[kind] += 1
                        if not warm:
                            continue
                        keep = False
                        if kind == "disparity":
                            keep = images["disparity"] < MAX_FRAMES
                        elif kind == "depth":
                            keep = (msg.getSequenceNum() % every == 0
                                    and images["depth"] < 2 * REFERENCE_FRAMES)
                        record = _record(msg, kind, keep)
                        if keep:
                            images[kind] += 1
                        records[kind].append(record)
                for quality, queue in encoders.items():
                    for msg in queue.tryGetAll():
                        jpeg_counts[quality] += 1
                        if warm:
                            records["jpeg"][quality].append(_record(msg, "jpeg", False))
                sleep(POLL_S)
            result["received"] = {**counts, **{f"jpeg_q{q}": n for q, n in jpeg_counts.items()}}
            result["captured"] = {kind: len(records[kind]) for kind in queues}
            result["captured"].update({f"jpeg_q{q}": len(records["jpeg"][q]) for q in qualities})
            result["fps"] = {kind: round(n / args.seconds, 2) for kind, n in result["captured"].items()}
            result["images_kept"] = images
            result["depth_every"] = every
    except Exception as e:
        result["error"] = f"{type(e).__name__}: {e}"
        result["traceback"] = traceback.format_exc(limit=6)
        _log(f"mjpeg_loss run {name} failed: {result['error']}")
    finally:
        if device is not None:
            if log_callback is not None:
                try:
                    device.removeLogCallback(log_callback)
                except Exception:
                    pass
            try:
                device.close()
            except Exception:
                pass
    lines = device_log.snapshot()
    result["device_log"] = summarize_log(sorted(lines["stream"] + lines["other"],
                                                key=lambda line: line.get("unix_time") or 0))
    result["duration_s"] = round(clock() - started, 2)
    return result, records


def _analyse(result, function, *args):
    """function(*args), or None with the exception recorded in result["analysis_error"]."""
    try:
        return function(*args)
    except Exception as e:
        result["analysis_error"] = f"{type(e).__name__}: {e}"
        result["analysis_traceback"] = traceback.format_exc(limit=6)
        _log(f"mjpeg_loss analysis of {result.get('name')} failed: {result['analysis_error']}")
        return None


def _references(records, kind, every):
    """The kept frames of every n-th sequence number (the temporal medians' frames)."""
    return [r for r in records[kind] if r.get("image") is not None and r["seq"] % every == 0][:REFERENCE_FRAMES]


def encoded_share(analysis, quality):
    """Share of the run's raw disparity frames that got a paired JPEG of this quality (0 without frames)."""
    if not analysis:
        return 0.0
    frames = analysis["frames"]["disparity"]
    paired = analysis["pairing"].get(f"q{quality}", {}).get("paired", 0)
    return paired / frames if frames else 0.0


def run_summary(result):
    parts = [f"{result['name']}: {result.get('captured', {})}"]
    if result.get("disparity_format"):
        parts.append(f"disparity raw8 {result['disparity_format'].get('raw8')}")
    for key in ("error", "analysis_error"):
        if result.get(key):
            parts.append(f"{key}: {result[key]}")
    return "; ".join(parts)


def run_mjpeg_loss(args, saved, dai, open_device, vio, clock=time.monotonic, sleep=time.sleep, wall_clock=time.time):
    """The mjpeg_loss document; written to --output after every run."""
    qualities = mjpeg_loss.parse_qualities(args.qualities)
    budget = args.budget if args.budget is not None else mjpeg_loss_budget(args.seconds, len(qualities))
    params = PnPParams(min_depth_m=args.min_depth, max_depth_m=args.max_depth)
    image_size = (args.width, args.height)
    period = 1.0 / max(float(args.fps), 1.0)
    every = reference_every(args.fps, args.seconds)
    settings_a = case_settings(saved, CASES_BY_NAME["G"])
    started = clock()
    document = {
        "probe": "stereo", "mode": MJPEG_LOSS, "version": 1, "started_at": wall_clock(), "complete": False,
        "depthai_version": getattr(dai, "__version__", None),
        "camera": {"sockets": ["CAM_B", "CAM_C"], "width": args.width, "height": args.height, "fps": args.fps},
        "qualities": qualities, "seconds": args.seconds, "warmup_s": WARMUP_S, "budget_s": budget,
        "feature_tracker": {"num_features": args.num_features, "max_features": args.max_features,
                            "corner_detector": args.corner_detector},
        "depth_rules": {"rules": list(mjpeg_loss.RULES), "primary": mjpeg_loss.PRIMARY_RULE,
                        "range_m": [args.min_depth, args.max_depth],
                        "window": 2 * mjpeg_loss.depth_points.WINDOW_RADIUS + 1,
                        "min_valid_fraction": params.min_valid_depth_fraction,
                        "decode_invalid_below": mjpeg_loss.DECODE_RULES, "primary_decode": mjpeg_loss.PRIMARY_DECODE},
        "saved_stereo": saved, "calibration": None, "runs": [], "skipped": [], "mjpeg": {},
        "raw_rates_mbps": {"disparity_raw8": mjpeg_loss.raw_mbps(args.width, args.height, 1, args.fps),
                           "depth_raw16": mjpeg_loss.raw_mbps(args.width, args.height, 2, args.fps)},
    }
    write_output(args.output, document)

    def time_left():
        return budget - (clock() - started)

    def run(name, description, settings, run_qualities, with_disparity):
        _log(f"Stereo probe mjpeg_loss: run {name} ({description})")
        result, records = capture_run(name, description, settings, args, dai, open_device, vio, run_qualities,
                                      with_disparity, clock=clock, sleep=sleep)
        if document["calibration"] is None and result.get("calibration", {}).get("depth_times_disparity_mm"):
            document["calibration"] = result["calibration"]
        document["runs"].append(result)
        return result, records

    def k_mm(result):
        return (result.get("calibration") or {}).get("depth_times_disparity_mm") \
            or (document["calibration"] or {}).get("depth_times_disparity_mm")

    def analyse_a(result, records, run_qualities):
        k = k_mm(result)
        if result["error"] or not k:
            if not result["error"]:
                result["analysis_error"] = "no fx * baseline from the calibration"
            return None
        analysis = _analyse(result, mjpeg_loss.analyse_mjpeg, records, run_qualities, k, image_size, params, args.fps)
        if analysis:
            result["frames"], result["pairing"] = analysis["frames"], analysis["pairing"]
            result["raw"] = analysis["raw"]
        return analysis

    def store(analysis, result, quality, alone):
        entry = analysis["qualities"][quality]
        document["mjpeg"][str(quality)] = {"run": result["name"], "encoders_in_run": len(result["qualities"]),
                                           "alone": alone, **entry}

    a_result, a_records = run("A", "case G settings (subpixel off, median last): raw disparity + MJPEG per quality",
                              settings_a, qualities, True)
    a_analysis = analyse_a(a_result, a_records, qualities)
    reference = {"result": a_result, "disparity": _references(a_records, "disparity", every),
                 "depth": _references(a_records, "depth", every)}
    raw8 = bool(a_analysis) and a_analysis["frames"]["disparity_types"] == ["RAW8"]
    parallel = [q for q in qualities if encoded_share(a_analysis, q) >= ENCODER_MIN_SHARE]
    for quality in parallel:
        store(a_analysis, a_result, quality, len(qualities) == 1)
    del a_records
    write_output(args.output, document)

    b_result, b_records = run("B", "saved settings (the VIO's): RAW16 depth + features", saved, (), False)
    b_depth, b_features = _references(b_records, "depth", every), b_records["features"]
    del b_records
    write_output(args.output, document)

    failed = [q for q in qualities if q not in parallel]
    sequential = []
    if failed and a_result["error"] is None and a_analysis and not raw8:
        document["skipped"] += [{"name": f"A-q{q}", "reason": "run A's disparity was not RAW8, so no encoder "
                                                               "takes it; another run would not help"} for q in failed]
        failed = []
    for quality in failed:
        name = f"A-q{quality}"
        if time_left() < run_cost_s(args.seconds):
            document["skipped"].append({"name": name, "reason": f"time budget: {time_left():.1f} s left"})
            continue
        result, records = run(name, f"run A with only the q{quality} encoder", settings_a, [quality], True)
        analysis = analyse_a(result, records, [quality])
        if analysis and analysis["frames"]["disparity"]:
            store(analysis, result, quality, True)
            sequential.append(quality)
            if not reference["disparity"]:
                reference = {"result": result, "disparity": _references(records, "disparity", every),
                             "depth": _references(records, "depth", every)}
        del records
        write_output(args.output, document)

    document["encoders"] = encoders_info(qualities, parallel, sequential, a_result, a_analysis)
    k = (document["calibration"] or {}).get("depth_times_disparity_mm")
    if k:
        if reference["disparity"] and reference["depth"]:
            document["conversion_check"] = _analyse(reference["result"], mjpeg_loss.conversion_check,
                                                    reference["disparity"], reference["depth"], k, period)
        holder = {"name": "subpixel"}
        document["subpixel"] = _analyse(holder, mjpeg_loss.subpixel_loss, reference["disparity"], reference["depth"],
                                        b_depth, b_features, k, image_size, params, period) or holder
        document["subpixel"].update(
            reference_run=reference["result"]["name"], b_run_error=b_result["error"],
            subpixel=b_result["settings"].get("stereo_subpixel"),
            method="camera at rest: per-pixel temporal medians of run A (integer disparity, fx*B/d on the host; and "
                   "A's own depth output) and run B (the VIO's subpixel depth) over about "
                   f"{REFERENCE_FRAMES} frames each, compared at the features of B's kept frames (B = reference). "
                   "per_frame: each B frame vs A's median map (B's noise included). Limits: different frames and "
                   "auto-exposure; feature ids restart with each run so they cannot be matched by id; anything "
                   "that moved between the runs shows up as error; the medians hide A's and B's noise "
                   "(reported separately).")
        bits = b_result["settings"].get("stereo_subpixel_bits", 3)
        document["quantisation"] = {
            "subpixel_bits": bits, "formula": "dz ~ z^2 / (fx * B) per disparity pixel; / 2^bits with subpixel",
            "theory": mjpeg_loss.quantisation_table(k, bits),
            "measured_integer": (reference["result"].get("raw") or {}).get("steps")}
    for result in document["runs"]:
        result["summary"] = run_summary(result)
    document["summary_table"] = "\n".join(mjpeg_loss.summary_lines(document))
    document["complete"] = True
    document["duration_s"] = round(clock() - started, 2)
    write_output(args.output, document)
    return document


def encoders_info(qualities, parallel, sequential, a_result, a_analysis):
    """How many encoders ran at once and which qualities were measured alone (in sequence, same static scene)."""
    shares = {f"q{q}": round(encoded_share(a_analysis, q), 3) for q in qualities}
    if len(parallel) == len(qualities):
        note = f"all {len(qualities)} encoders ran at once on run A's frames"
    else:
        reason = a_result["error"] or f"JPEG share of the raw frames {shares}"
        note = (f"{len(parallel)} of {len(qualities)} encoders ran at once on run A ({reason}); "
                f"measured alone, one run each in sequence on the same static scene: {sequential or 'none'}")
    return {"requested": qualities, "parallel": parallel, "sequential": sequential, "jpeg_share_run_a": shares,
            "min_share": ENCODER_MIN_SHARE, "note": note}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Stereo/encoder probe on the OAK-D (stop the VIO first)")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--mode", choices=MODES, default=CASES_MODE,
                        help="cases: the StereoDepth/encoder cases; mjpeg_loss: how much depth MJPEG loses")
    parser.add_argument("--quality", type=int, default=70, help="MJPEG quality, 1-100 (cases)")
    parser.add_argument("--cases", default=None, help=f"Comma-separated, default all: {','.join(CASES_BY_NAME)}")
    parser.add_argument("--qualities", default=None,
                        help=f"mjpeg_loss: comma-separated MJPEG qualities, default "
                             f"{','.join(map(str, mjpeg_loss.DEFAULT_QUALITIES))}")
    parser.add_argument("--seconds", type=float, default=None,
                        help=f"Sampling time per case ({SAMPLE_SECONDS}) or per mjpeg_loss run ({LOSS_SECONDS})")
    parser.add_argument("--budget", type=float, default=None,
                        help=f"No case or run starts after this many seconds (cases: {TIME_BUDGET_S}; mjpeg_loss: "
                             "mjpeg_loss_budget)")
    parser.add_argument("--device-timeout", type=float, default=OPEN_TIMEOUT_S)
    parser.add_argument("--output", type=Path, default=None, help="Also write the JSON here (after every case)")
    # mjpeg_loss: the VIO's FeatureTracker and depth range (pnp_vio.py's options and defaults)
    parser.add_argument("--num-features", type=int, default=300)
    parser.add_argument("--max-features", type=int, default=0)
    parser.add_argument("--corner-detector", choices=CORNER_DETECTOR_CHOICES, default="harris")
    parser.add_argument("--min-depth", type=float, default=0.2, help="m")
    parser.add_argument("--max-depth", type=float, default=15.0, help="m")
    add_stereo_arguments(parser)
    args = parser.parse_args(argv)
    if args.seconds is None:
        args.seconds = LOSS_SECONDS if args.mode == MJPEG_LOSS else SAMPLE_SECONDS
    if args.budget is None and args.mode == CASES_MODE:
        args.budget = TIME_BUDGET_S
    return args


def write_output(path, document):
    if path is None:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        snapshots.atomic_write(path, json.dumps(document, indent=1).encode())
    except OSError as e:
        _log(f"Probe output not written to {path}: {e}")


def main(argv=None, dai=None, open_device=None, clock=time.monotonic, sleep=time.sleep, wall_clock=time.time,
         vio=None):
    args = parse_args(argv)
    try:
        if args.mode == MJPEG_LOSS:
            mjpeg_loss.parse_qualities(args.qualities)
            if not (math.isfinite(args.seconds) and args.seconds > 0):
                raise ValueError(f"--seconds must be above 0, got {args.seconds}")
        names = parse_case_names(args.cases)
    except ValueError as e:
        _log(str(e))
        return 2
    saved = resolve_stereo_settings(args)
    if dai is None:
        import depthai as dai
    if open_device is None:
        from device_open import open_oak_device

        def open_device():
            return open_oak_device(args.device_timeout, log=_log)

    if args.mode == MJPEG_LOSS:
        if vio is None:
            import pnp_vio as vio
        document = run_mjpeg_loss(args, saved, dai, open_device, vio, clock=clock, sleep=sleep, wall_clock=wall_clock)
        _log("Stereo probe mjpeg_loss:\n" + document["summary_table"])
        print(json.dumps(document), flush=True)
        return 0

    started = clock()
    document = {"probe": "stereo", "version": 1, "started_at": wall_clock(), "complete": False,
                "depthai_version": getattr(dai, "__version__", None),
                "camera": {"sockets": ["CAM_B", "CAM_C"], "width": args.width, "height": args.height,
                           "fps": args.fps, "quality": args.quality},
                "seconds_per_case": args.seconds, "saved_stereo": saved, "cases": [], "skipped": []}
    write_output(args.output, document)
    for name in names:
        left = args.budget - (clock() - started)
        if left < MIN_CASE_S:
            document["skipped"].append({"name": name, "reason": f"time budget: {left:.1f} s left"})
            continue
        _log(f"Stereo probe: case {name} ({CASES_BY_NAME[name].description})")
        result = run_case(CASES_BY_NAME[name], saved, args, dai, open_device, clock=clock, sleep=sleep)
        _log(f"Stereo probe: case {name}: {result['summary']}")
        document["cases"].append(result)
        document["duration_s"] = round(clock() - started, 2)
        write_output(args.output, document)
    document["complete"] = True
    document["duration_s"] = round(clock() - started, 2)
    write_output(args.output, document)
    print(json.dumps(document), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
