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

Run in the container while the VIO is stopped (POST /api/stereo_probe does that), or by hand:
    PYTHONPATH=/app/depthai-python python /app/stereo_probe.py --width 640 --height 400 --fps 15 --cases A,E
The pure helpers below need neither depthai nor a camera.
"""

import argparse
import json
import sys
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import depth_stream
import snapshots
from stereo_config import (FILTER_RUNS, FILTERING_ORDER_MEDIAN_LAST, add_stereo_arguments,  # noqa: F401
                           apply_stereo_config, disparity_format, enum_name, executed_filters,
                           predicted_max_disparity, read_back, resolve_stereo_settings)

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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Stereo/encoder probe on the OAK-D (stop the VIO first)")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--quality", type=int, default=70, help="MJPEG quality, 1-100")
    parser.add_argument("--cases", default=None, help=f"Comma-separated, default all: {','.join(CASES_BY_NAME)}")
    parser.add_argument("--seconds", type=float, default=SAMPLE_SECONDS, help="Sampling time per case")
    parser.add_argument("--budget", type=float, default=TIME_BUDGET_S, help="No case starts after this many seconds")
    parser.add_argument("--device-timeout", type=float, default=OPEN_TIMEOUT_S)
    parser.add_argument("--output", type=Path, default=None, help="Also write the JSON here (after every case)")
    add_stereo_arguments(parser)
    return parser.parse_args(argv)


def write_output(path, document):
    if path is None:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        snapshots.atomic_write(path, json.dumps(document, indent=1).encode())
    except OSError as e:
        _log(f"Probe output not written to {path}: {e}")


def main(argv=None, dai=None, open_device=None, clock=time.monotonic, sleep=time.sleep, wall_clock=time.time):
    args = parse_args(argv)
    try:
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
