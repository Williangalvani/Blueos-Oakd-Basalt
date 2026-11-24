#!/usr/bin/env python3
"""
BlueOS OAK-D PnP VIO (option D)

All pixel work runs on the OAK-D: stereo depth and LK feature tracking on the rectified left image, and
(--depth-transport points, the default) the depth at the tracked features (depth_points.py). The host solves
frame-to-frame PnP with a gyro rotation prior (pnp_odometry.py), then sends VISION_POSITION_DELTA over
MAVLink. --depth-transport full_frame sends every depth frame and looks the depth up on the host.
"""

import argparse
import math
import os
import signal
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path
from typing import Optional

import depthai as dai
import numpy as np
from loguru import logger

import depth_points
import depth_probe
import depth_stream
import feature_stream
import autopilot_attitude
import link_stats
import pose_stream
import snapshots
import vio_telemetry
from autopilot_params import AutopilotParams
from device_open import DEFAULT_TIMEOUT, DeviceOpenTimeout, DeviceRestartWatch, open_oak_device
from mavlink_sender import MavlinkSender
from pnp_odometry import (Intrinsics, PnPOdometry, PnPParams, camera_to_body_frd, camera_to_body_rotation,
                          depth_from_points, integrate_gyro, intrinsics_from_frame, lever_arm_correction,
                          matrix_to_rotvec, refracted_intrinsics)
from rectified_intrinsics import StereoCalibration, as_list, choose_intrinsics, rectified_for_settings
from stereo_config import (DEFAULT_LEVER_ARM_M, DEFAULT_REFRACTIVE_INDEX, INTRINSICS_SOURCES, add_stereo_arguments,
                           apply_stereo_config, disparity_format, effective_stereo_config, parse_depth_scale,
                           parse_intrinsics_override, parse_lever_arm, parse_refractive_index, read_back,
                           resolve_stereo_settings, stereo_warnings)
from tracking_recovery import FeatureTrackerReinit, TrackingRecovery, feature_id_counts
from vio_log import LOG_ONLY_BANNER, env_forces_log_only, open_vio_log, resolve_log_only, setup_autopilot_output

SUMMARY_INTERVAL = 10.0
# Gyro history kept before the older of the PnP reference and the current frame
GYRO_BUFFER_SECONDS = 2.0
# Hard cap (about 50 s at 200 Hz) for a reference held with --max-reference-age 0
GYRO_BUFFER_MAX_SAMPLES = 10000
SNAPSHOT_RGB_SIZE = (640, 400)
SNAPSHOT_RGB_FPS = 5.0
# StereoDepth's default frames per output pool (StereoDepthProperties::numFramesPool)
STEREO_FRAMES_POOL = 3

CORNER_DETECTORS = {
    "harris": dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS,
    "shi-tomasi": dai.FeatureTrackerConfig.CornerDetector.Type.SHI_THOMASI,
}

INTRINSICS_SOCKETS = {
    "cam_b": dai.CameraBoardSocket.CAM_B,
    "cam_c": dai.CameraBoardSocket.CAM_C,
}


def _argument_type(parse):
    def convert(text):
        try:
            return parse(text)
        except ValueError as e:
            raise argparse.ArgumentTypeError(str(e))
    return convert


def parse_stream_fps(text):
    value = float(text)
    if not (math.isfinite(value) and 0 < value <= 60):
        raise ValueError(f"depth stream fps must be above 0 and at most 60, got {text!r}")
    return value


def parse_stream_quality(text):
    value = int(text)
    if not 1 <= value <= 100:
        raise ValueError(f"depth stream quality must be between 1 and 100, got {text!r}")
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='BlueOS OAK-D PnP VIO')

    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--camera-angle', type=float, default=0.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=400)

    parser.add_argument('--num-features', type=int, default=300,
                        help='Target number of features tracked on the camera')
    parser.add_argument('--max-features', type=int, default=0,
                        help='Hard cap on the features detected per frame (FeatureTracker numMaxFeatures); 0 = '
                             "depthai's automatic maximum")
    parser.add_argument('--corner-detector', choices=sorted(CORNER_DETECTORS), default='harris')
    parser.add_argument('--intrinsics-from', choices=INTRINSICS_SOURCES, default='rectified',
                        help='Rectified-left camera matrix: "rectified" computes it from the CAM_B calibration and the '
                             'stereo alpha (what the device rectifies to), "frame" reads the depth frame (which '
                             'carries the raw calibration on RVC2), cam_b/cam_c use the raw calibration '
                             '(see NOTES_OPTION_D.md)')
    parser.add_argument('--intrinsics-override', type=_argument_type(parse_intrinsics_override), default=None,
                        metavar='FX,FY,CX,CY',
                        help='Camera matrix in pixels on the width x height grid; wins over --intrinsics-from')
    parser.add_argument('--depth-scale', type=_argument_type(parse_depth_scale), default=1.0,
                        help='Extra trim on the device depth before back-projection, on top of --refractive-index; '
                             'translation scales with it; 0.2-5')
    parser.add_argument('--refractive-index', type=_argument_type(parse_refractive_index),
                        default=DEFAULT_REFRACTIVE_INDEX,
                        help='Medium in front of the flat port (1.333 fresh water, 1.0 in air), 1-2. The calibration is '
                             'in air: PnP multiplies fx, fy of the camera matrix in use and the device depth by it')
    parser.add_argument('--lever-arm', action=argparse.BooleanOptionalAction, default=False,
                        help='Diagnostics only: also log each delta corrected for the camera lever arm '
                             '(lever_delta_* step fields). Never sent: the autopilot applies VISO_POS_X/Y/Z itself')
    parser.add_argument('--lever-arm-m', type=_argument_type(parse_lever_arm), default=DEFAULT_LEVER_ARM_M,
                        metavar='X,Y,Z',
                        help='Camera position relative to the IMU, body FRD, m (use --lever-arm-m=X,Y,Z if X is '
                             'negative); same as VISO_POS_X/Y/Z')

    parser.add_argument('--min-inliers', type=int, default=15)
    parser.add_argument('--reprojection-error', type=float, default=2.0, help='RANSAC threshold (px)')
    parser.add_argument('--gyro-gate', type=float, default=5.0,
                        help='Reject frames whose PnP rotation differs from the gyro by more than this (deg); 0 disables the gyro prior')
    parser.add_argument('--max-reference-age', type=float, default=1.0,
                        help='After a failed frame, keep solving against the last good frame for up to this long (s), '
                             'so the next delta covers the gap; 0 removes the cap')
    parser.add_argument('--min-depth', type=float, default=0.2, help='m')
    parser.add_argument('--max-depth', type=float, default=15.0, help='m')
    parser.add_argument('--recovery-frames', type=int, default=20,
                        help='Consecutive failed frames before resetting the PnP reference and re-initialising '
                             'the on-device feature tracker (0 disables)')

    parser.add_argument('--snapshots', action=argparse.BooleanOptionalAction, default=True,
                        help='On-demand camera snapshots for /api/snapshot (no bandwidth until requested)')
    parser.add_argument('--snapshot-encoding', choices=snapshots.ENCODINGS, default=snapshots.CAMERA_JPEG,
                        help='camera_jpeg: rect_left, rect_right and rgb snapshots JPEG-encoded on the camera; raw: '
                             'sent raw, PNG-encoded on the Pi (the fallback). depth snapshots are always raw')
    parser.add_argument('--snapshot-dir', type=Path, default=snapshots.SNAPSHOT_DIR)
    parser.add_argument('--telemetry-dir', type=Path, default=None,
                        help='Live status and plot history for the web page (default: <snapshot-dir>/telemetry)')
    parser.add_argument('--depth-stream', action=argparse.BooleanOptionalAction, default=True,
                        help='Live depth for /api/stream/depth, converted to 8 bit and MJPEG-encoded on the camera; '
                             'sent only while a client watches')
    parser.add_argument('--depth-stream-fps', type=_argument_type(parse_stream_fps), default=5.0,
                        help='Depth stream rate; the camera sends every round(fps / this)-th depth frame')
    parser.add_argument('--depth-stream-quality', type=_argument_type(parse_stream_quality), default=70,
                        help='Depth stream JPEG quality, 1-100')
    parser.add_argument('--depth-stream-encoding', choices=depth_stream.VARIANTS, default=depth_stream.DEFAULT_VARIANT,
                        help='What the live stream shows and where it is encoded (see depth_stream.py): '
                             'mono_features (the rectified-left camera image the features are tracked on, '
                             'straight into the camera encoder; the browser draws the features), disparity (the '
                             'StereoDepth disparity straight into the camera encoder; needs a RAW8 disparity: '
                             'subpixel and extended off and no filter after the median), manip (a Script GRAY8 '
                             'depth frame through ImageManip), nv12 or yuv400p (the Script builds the encoder '
                             'frame), host_jpeg (the Pi encodes the depth), auto (mono_features, then host_jpeg '
                             'if the camera sends no or blank frames)')
    parser.add_argument('--depth-stream-debug', action=argparse.BooleanOptionalAction, default=False,
                        help='While the depth stream runs, raise the camera log level to DEBUG; the stream nodes\' '
                             'log lines appear in /api/status depth_stream.device_log')
    parser.add_argument('--depth-transport', choices=depth_points.TRANSPORTS, default=depth_points.DEFAULT_TRANSPORT,
                        help='points: the camera computes the depth at the tracked features (SpatialLocationCalculator, '
                             'a 5x5 median per feature) and sends only those; full depth frames cross the link only '
                             'while the depth probe or a watched host_jpeg stream needs them. full_frame: every depth '
                             'frame goes to the Pi, which looks the depth up itself (the previous behaviour)')
    parser.add_argument('--depth-points-rois', choices=depth_points.ROI_SOURCES,
                        default=depth_points.DEFAULT_ROI_SOURCE,
                        help='points only: who builds the per-feature ROI list: host (the Pi, from the features it '
                             'receives anyway; ~12 kB/frame up the link) or camera (slow: a Script on the camera\'s '
                             'Leon CPU, 111 ms per frame of ~255 features on test16b)')

    parser.add_argument('--pose-attitude-source', choices=pose_stream.ATTITUDE_MODES, default='autopilot',
                        help='Attitude of the 3D pose viewer: autopilot (ArduPilot ATTITUDE read from '
                             'mavlink2rest, the gyro-integrated attitude while it is missing or stale) or gyro '
                             '(gyro-integrated only, drifts). The MAVLink output never uses it')
    parser.add_argument('--vehicle-ip', default='127.0.0.1')
    parser.add_argument('--quality-rate', type=float, default=1.0)
    parser.add_argument('--device-timeout', type=float, default=DEFAULT_TIMEOUT,
                        help='Seconds to keep waiting for the OAK device to be discoverable and open')
    parser.add_argument('--log-only', action='store_true',
                        help='Passive logger: no MAVLink output and no autopilot parameter changes '
                             '(also forced by OAKD_LOG_ONLY=1)')

    add_stereo_arguments(parser)
    return parser.parse_args(argv)


def timestamp_seconds(msg, exposure_middle=False):
    if exposure_middle:
        try:
            return msg.getTimestampDevice(dai.CameraExposureOffset.MIDDLE).total_seconds()
        except TypeError:
            pass
    return msg.getTimestampDevice().total_seconds()


class GyroBuffer:
    """Device-timestamped gyro samples (rad/s, IMU frame)."""

    def __init__(self):
        self.samples = deque(maxlen=GYRO_BUFFER_MAX_SAMPLES)

    def add(self, imu_data):
        """Returns the number of packets."""
        packets = imu_data.packets
        for packet in packets:
            g = packet.gyroscope
            self.samples.append((g.getTimestampDevice().total_seconds(), g.x, g.y, g.z))
        return len(packets)

    def prune(self, before):
        while self.samples and self.samples[0][0] < before:
            self.samples.popleft()

    def rotation(self, t0, t1, R_cam_imu):
        if not self.samples:
            return None
        data = np.array(self.samples)
        order = np.argsort(data[:, 0], kind="stable")
        return integrate_gyro(data[order, 0], data[order, 1:], t0, t1, R_cam_imu)


class Stats:
    def __init__(self, temperature=None, link=None):
        # Returns the last sampled chip temperature (°C) or None; sampled off the VIO loop by Telemetry
        self.temperature = temperature
        # Returns the camera link summary (link_stats.LinkMonitor.summary), from the samples Telemetry takes
        self.link = link
        # Returns the depth transport / camera load part of the line (depth_summary)
        self.depth_summary = None
        self.reset()

    def reset(self):
        self.start = time.monotonic()
        self.frames = 0
        self.ok = 0
        self.proc_ms = []
        self.features = []
        self.inliers = []
        self.reasons = Counter()
        self.recoveries = 0
        self.bridged = 0
        self.reference_resets = 0

    def add(self, frame, proc_ms):
        self.frames += 1
        self.proc_ms.append(proc_ms)
        self.features.append(frame.features)
        if frame.pose.ok:
            self.ok += 1
            self.inliers.append(frame.pose.inliers)
            self.bridged += frame.gap_frames > 0
        else:
            self.reasons[frame.pose.reason] += 1
            self.reference_resets += frame.reference_reset

    def chip_temperature(self):
        value = self.temperature() if self.temperature is not None else None
        return "" if value is None else f", chip {value:.1f}°C"

    def link_summary(self):
        text = "" if self.link is None else f", {self.link()}"
        if self.depth_summary is not None:
            try:
                text += self.depth_summary()
            except Exception as e:
                text += f", depth summary failed: {e}"
        return text

    def maybe_report(self):
        elapsed = time.monotonic() - self.start
        if elapsed < SUMMARY_INTERVAL:
            return
        if self.frames:
            mean_inliers = np.mean(self.inliers) if self.inliers else 0.0
            recoveries = f", recoveries {self.recoveries}" if self.recoveries else ""
            logger.info(
                f"📊 {elapsed:.1f}s: {self.frames / elapsed:.1f} fps, ok {self.ok}/{self.frames}, "
                f"proc mean {np.mean(self.proc_ms):.1f} ms max {np.max(self.proc_ms):.1f} ms, "
                f"mean features {np.mean(self.features):.0f}, mean inliers {mean_inliers:.0f}, "
                f"failures {dict(self.reasons)}, gaps bridged {self.bridged}, reference resets "
                f"{self.reference_resets}{recoveries}{self.chip_temperature()}{self.link_summary()}")
        else:
            logger.warning(f"📊 {elapsed:.1f}s: no frames received{self.chip_temperature()}{self.link_summary()}")
        self.reset()


def depth_summary(status, camera_load=None):
    """The depth transport and camera load part of the 10-s log line."""
    if status.get("transport") == depth_points.POINTS:
        text = (f", depth points {status.get('paired')}/{status.get('frames')} paired"
                f"{', missing ' + str(status.get('missing_by_reason')) if status.get('missing') else ''}"
                f", feature gaps {status.get('feature_gap_frames')}, points gaps {status.get('points_gap_frames')}")
    else:
        text = f", depth full frames, feature gaps {status.get('feature_gap_frames')}"
    if camera_load:
        css, mss = camera_load.get("leon_css_cpu_percent"), camera_load.get("leon_mss_cpu_percent")
        if css is not None or mss is not None:
            text += f", Leon CSS {css}% MSS {mss}%"
    return text


def log_device_info(device):
    info = {}
    for label, getter in (("name", "getDeviceName"), ("IMU", "getConnectedIMU"), ("USB", "getUsbSpeed")):
        try:
            info[label] = getattr(device, getter)()
        except Exception as e:
            info[label] = f"unavailable ({e})"
    logger.info(f"Device: {info['name']}, IMU: {info['IMU']}, USB speed: {info['USB']}")
    return {label: str(value) for label, value in info.items()}


def format_intrinsics(intrinsics):
    if intrinsics is None:
        return "none"
    return f"fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f} cx={intrinsics.cx:.1f} cy={intrinsics.cy:.1f}"


def load_calibration(device, args):
    """
    StereoCalibration: raw CAM_B (stereo left) intrinsics and distortion on the width x height grid,
    the raw socket matrix --intrinsics-from cam_b/cam_c asks for, the stereo baseline and the IMU -> CAM_B rotation.
    """
    calib = device.readCalibration()
    left = dai.CameraBoardSocket.CAM_B
    raw = Intrinsics.from_matrix(calib.getCameraIntrinsics(left, args.width, args.height))
    logger.info(f"Calibration intrinsics (cam_b @ {args.width}x{args.height}): {format_intrinsics(raw)}")
    calibration = StereoCalibration(raw=raw, sockets={"cam_b": raw})
    try:
        calibration.distortion = [float(v) for v in calib.getDistortionCoefficients(left)]
        logger.info(f"CAM_B distortion coefficients: {np.round(calibration.distortion, 5).tolist()}")
    except Exception as e:
        logger.warning(f"No CAM_B distortion coefficients ({e})")
    if args.intrinsics_from == "cam_c":
        calibration.sockets["cam_c"] = Intrinsics.from_matrix(
            calib.getCameraIntrinsics(INTRINSICS_SOCKETS["cam_c"], args.width, args.height))
        logger.info(f"Calibration intrinsics (cam_c): {format_intrinsics(calibration.sockets['cam_c'])}")
    try:
        # Default useSpecTranslation=True is the board design value (CalibrationHandler.hpp)
        calibration.baseline_m = calib.getBaselineDistance(dai.CameraBoardSocket.CAM_C, left, False) / 100.0
        calibration.baseline_spec_m = calib.getBaselineDistance(dai.CameraBoardSocket.CAM_C, left, True) / 100.0
        logger.info(f"Stereo baseline: {calibration.baseline_m * 100:.3f} cm calibrated, "
                    f"{calibration.baseline_spec_m * 100:.3f} cm design")
    except Exception as e:
        logger.warning(f"No stereo baseline ({e})")

    if args.gyro_gate > 0:
        try:
            extrinsics = np.array(calib.getImuToCameraExtrinsics(left, True))
            calibration.R_cam_imu = extrinsics[:3, :3]
            logger.info(f"IMU -> CAM_B rotation:\n{np.array2string(calibration.R_cam_imu, precision=3)}")
        except Exception as e:
            logger.warning(f"No IMU-to-camera extrinsics ({e}); gyro prior disabled")
    return calibration


def frame_intrinsics(depth_msg, image_size):
    """(Intrinsics or None, problem) from the depth frame's transformation, on the image_size grid."""
    if depth_msg is None:
        return None, ("no depth frame on the Pi to read its camera matrix from (--depth-transport points; "
                      "full_frame has one)")
    try:
        transformation = depth_msg.getTransformation()
        intrinsics = intrinsics_from_frame(transformation.getIntrinsicMatrix(), transformation.getSize(), image_size)
    except Exception as e:
        return None, f"no depth frame camera matrix ({e})"
    if intrinsics is None:
        return None, f"implausible depth frame camera matrix for {image_size[0]}x{image_size[1]}"
    return intrinsics, None


def select_intrinsics(args, calibration, depth_msg, rectified, rectified_problem=None):
    """
    IntrinsicsChoice for the first depth frame. --intrinsics-override wins; the default "rectified"
    uses the K computed from the calibration and only reads the frame matrix to log whether it matches.
    """
    override = None if args.intrinsics_override is None else Intrinsics(*args.intrinsics_override)
    frame, frame_problem = frame_intrinsics(depth_msg, (args.width, args.height))
    return choose_intrinsics(args.intrinsics_from, calibration.raw, rectified, frame, frame_problem, override,
                             calibration.sockets if args.intrinsics_from in INTRINSICS_SOCKETS else None,
                             rectified_problem)


def camera_status(args, calibration, rectified, choice, stereo_settings):
    """
    What /api/status and /api/focal_check read about the camera matrix in use. intrinsics is the in-air
    matrix the device's depth is consistent with (what the focal check compares); PnP runs with
    effective_intrinsics and the depth times total_depth_scale.
    """
    return {
        "pid": os.getpid(),
        "unix_time": time.time(),
        "image_size": [args.width, args.height],
        "source": choice.source,
        "requested_source": args.intrinsics_from,
        "intrinsics": as_list(choice.intrinsics),
        "effective_intrinsics": as_list(refracted_intrinsics(choice.intrinsics, args.refractive_index)),
        "raw": as_list(calibration.raw),
        "rectified": as_list(rectified),
        "frame": as_list(choice.frame),
        "frame_matches": choice.frame_matches,
        "override": None if args.intrinsics_override is None else list(args.intrinsics_override),
        "alpha": stereo_settings["stereo_alpha"],
        "distortion_correction": stereo_settings["stereo_distortion_correction"],
        "baseline_m": calibration.baseline_m,
        "baseline_spec_m": calibration.baseline_spec_m,
        "depth_scale": args.depth_scale,
        "refractive_index": args.refractive_index,
        "total_depth_scale": args.depth_scale * args.refractive_index,
        "depth_range_mm": [stereo_settings["stereo_depth_min_mm"], stereo_settings["stereo_depth_max_mm"]],
        "warnings": choice.warnings,
    }


def lever_arm_fields(position_delta, R_gyro, R_pnp, args):
    """
    Step fields for --lever-arm, logged only: the body (IMU) delta, the camera delta minus the rotation's
    lever-arm term r - R^T r, with R the gyro rotation over the delta's interval (the PnP one without a gyro
    prior), both in the optical frame. None for frames without a delta.
    """
    if position_delta is None:
        return {"lever_delta_position": None, "lever_delta_correction": None, "lever_delta_rotation": None}
    source, R = ("gyro", R_gyro) if R_gyro is not None else ("pnp", R_pnp)
    correction = lever_arm_correction(camera_to_body_rotation(R, args.camera_angle), args.lever_arm_m)
    return {"lever_delta_position": np.asarray(position_delta) - correction, "lever_delta_correction": correction,
            "lever_delta_rotation": source}


def configure_feature_tracker(config, args, maintainer=True):
    config.setCornerDetector(CORNER_DETECTORS[args.corner_detector])
    config.setNumTargetFeatures(args.num_features)
    if args.max_features > 0:
        # numMaxFeatures is only reachable through a whole CornerDetector (no setter); its other fields are the
        # defaults the two calls above leave
        detector = dai.FeatureTrackerConfig.CornerDetector()
        detector.type = CORNER_DETECTORS[args.corner_detector]
        detector.numTargetFeatures = args.num_features
        detector.numMaxFeatures = args.max_features
        config.setCornerDetector(detector)
    config.setMotionEstimator(True)
    config.setOpticalFlow()  # Lucas-Kanade; keeps feature ids across frames
    config.setFeatureMaintainer(maintainer)  # default minimum distance between features (50)
    return config


def snapshot_script(available, encoders=None):
    """
    Device-side gate: forward one frame of the requested stream per request message (snapshots.request_number):
    to the stream's JPEG encoder output (encoders: stream -> output name), or raw to 'snapshot' when asked so
    or when the stream has no encoder.
    """
    return "\n".join([
        f"streams = {list(snapshots.STREAMS)!r}",
        f"available = {sorted(available)!r}",
        f"encoders = {dict(sorted((encoders or {}).items()))!r}",
        "while True:",
        "    number = node.inputs['request'].get().getSequenceNum()",
        "    index, raw = number % len(streams), number >= len(streams)",
        "    if 0 <= number < 2 * len(streams) and streams[index] in available:",
        "        name = streams[index]",
        "        output = 'snapshot' if raw or name not in encoders else encoders[name]",
        "        node.outputs[output].send(node.inputs[name].get())",
    ])


@dataclass
class SnapshotGate:
    request_queue: object
    raw_queue: object
    encoded_queues: tuple  # the JPEG encoders' output queues
    streams: tuple
    unavailable: dict
    encoded: tuple  # streams the camera JPEG-encodes
    not_encoded: dict  # stream -> why it crosses raw


@dataclass
class PipelineHandles:
    sync_queue: Optional[object]  # full_frame: features + depth groups
    imu_queue: object
    feature_config_queue: object
    stereo_effective: dict
    depth_transport: str = depth_points.FULL_FRAME
    # points: the features straight from the FeatureTracker, the camera's depth at them and its Script's reports
    features_queue: Optional[object] = None
    points_queue: Optional[object] = None
    points_status_queue: Optional[object] = None
    rois_queue: Optional[object] = None  # rois "host": the ROI configs up
    # points: the on-demand full depth gate (control up, frames down)
    full_depth_control_queue: Optional[object] = None
    full_depth_queue: Optional[object] = None
    snapshot_request_queue: Optional[object] = None
    snapshot_queue: Optional[object] = None  # raw frames
    snapshot_streams: tuple = ()
    snapshot_unavailable: dict = field(default_factory=dict)
    snapshot_encoded_queues: tuple = ()  # the camera's JPEGs
    snapshot_encoded: tuple = ()
    snapshot_not_encoded: dict = field(default_factory=dict)
    depth_stream_control_queue: Optional[object] = None
    depth_stream_queue: Optional[object] = None
    depth_stream_status_queue: Optional[object] = None
    # Empty: no depth stream. info["variant"] says where it is encoded (host_jpeg: no camera nodes)
    depth_stream_info: dict = field(default_factory=dict)
    # "Type(id)" names of the stream's camera nodes, as the device log names them
    depth_stream_nodes: tuple = ()


def snapshot_encoder(p, fps):
    encoder = p.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(snapshots.JPEG_QUALITY)
    encoder.input.setBlocking(False)
    encoder.input.setMaxSize(1)
    return encoder


def build_snapshot_gate(p, device, stereo, args):
    """
    Script node holding the latest frame of each stream in non-blocking size-1 queues; it forwards a
    single frame per host request, so the snapshot outputs cost no link bandwidth until asked.
    camera_jpeg: rect_left and rect_right (StereoDepth's pooled RAW8 frames, as the mono_features stream
    sends them) go through one MJPEG encoder ('mono'), rgb (NV12) through another ('color'); depth is raw.
    """
    sources = {"rect_left": stereo.rectifiedLeft, "rect_right": stereo.rectifiedRight, "depth": stereo.depth}
    unavailable = {}
    if dai.CameraBoardSocket.CAM_A in device.getConnectedCameras():
        rgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=SNAPSHOT_RGB_FPS)
        sources["rgb"] = rgb.requestOutput(SNAPSHOT_RGB_SIZE, dai.ImgFrame.Type.NV12)
    else:
        unavailable["rgb"] = "this device has no CAM_A (RGB) camera"

    not_encoded = {}
    groups = {}
    if args.snapshot_encoding == snapshots.RAW:
        not_encoded = {s: "Snapshot Encoding is raw" for s in snapshots.CAMERA_JPEG_STREAMS if s in sources}
    else:
        problem = depth_stream.size_problem(args.width, args.height)
        if problem:
            not_encoded.update(rect_left=problem, rect_right=problem)
        else:
            groups["mono"] = (("rect_left", "rect_right"), float(args.fps))
        if "rgb" in sources:
            groups["color"] = (("rgb",), SNAPSHOT_RGB_FPS)
    encoders = {stream: output for output, (streams, _) in groups.items() for stream in streams}

    script = p.create(dai.node.Script)
    for name, output in sources.items():
        output.link(script.inputs[name])
        script.inputs[name].setBlocking(False)
        script.inputs[name].setMaxSize(1)
    script.inputs["request"].setBlocking(False)
    script.inputs["request"].setMaxSize(4)
    script.setScript(snapshot_script(sources, encoders))

    encoded_queues = []
    for output, (_, fps) in groups.items():
        encoder = snapshot_encoder(p, fps)
        script.outputs[output].link(encoder.input)
        encoded_queues.append(encoder.out.createOutputQueue(maxSize=2, blocking=False))
    return SnapshotGate(
        request_queue=script.inputs["request"].createInputQueue(maxSize=4, blocking=False),
        raw_queue=script.outputs["snapshot"].createOutputQueue(maxSize=2, blocking=False),
        encoded_queues=tuple(encoded_queues), streams=tuple(sources), unavailable=unavailable,
        encoded=tuple(encoders), not_encoded=not_encoded)


def node_log_name(node):
    """How the device log names a node (LogMessage.nodeIdName): "Type(id)"."""
    try:
        return f"{node.getName()}({node.id})"
    except Exception:
        return None


def build_depth_stream(p, stereo, args, stereo_settings):
    """
    Script node gate (depth_stream.device_script) feeding an MJPEG VideoEncoder, the Script waiting on its
    control input until the host enables it, then passing every n-th frame on:
    - mono_features (auto at the start): stereo.rectifiedLeft (RAW8, the image the FeatureTracker sees) ->
      the Script forwards the frame unchanged -> VideoEncoder;
    - disparity (explicit, only with a RAW8 disparity: stereo_config.disparity_format of the applied config):
      stereo.disparity -> the Script forwards it -> VideoEncoder. Both encode StereoDepth's own pooled frames,
      which the encoder takes as RAW8 (test14); RAW16 would be dropped, so it runs as host_jpeg instead;
    - manip: stereo.depth -> the Script converts to a GRAY8 frame -> ImageManip (NV12) -> VideoEncoder;
    - nv12, yuv400p: stereo.depth -> the Script builds the encoder's frame -> VideoEncoder.
    The Script-made frames have no pool: on an OAK-D-W ImageManip made them a constant image and the
    encoder dropped them (test12); they stay for diagnosis. The encoder and host queues are non-blocking,
    so a slow consumer only drops stream frames. The Script's 'status' output carries its errors and timing
    (depth_stream.parse_device_status). host_jpeg adds nothing on the camera (the queues are None).

    Returns (control queue, JPEG queue, status queue, info, node log names).
    """
    every = depth_stream.stream_divisor(args.fps, args.depth_stream_fps)
    fps = args.fps / every
    min_mm, max_mm = stereo_settings["stereo_depth_min_mm"], stereo_settings["stereo_depth_max_mm"]
    requested = args.depth_stream_encoding
    problem = depth_stream.size_problem(args.width, args.height)
    fmt = disparity_format(read_back(stereo.initialConfig), stereo_settings)
    disparity_problem = depth_stream.disparity_problem(fmt)
    decimation = int(stereo_settings.get("stereo_decimation") or 1)
    if not disparity_problem and decimation > 1:
        disparity_problem = depth_stream.size_problem(args.width // decimation, args.height // decimation)
    variant, reason = depth_stream.resolve_variant(requested, disparity_problem, problem)
    shift = depth_stream.depth_shift(max_mm)
    info = {"encoding": depth_stream.encoding_for(variant), "codec": "mjpeg", "fps_target": fps, "every": every,
            "camera_fps": args.fps, "quality": args.depth_stream_quality, "image_size": [args.width, args.height],
            "depth_range_mm": [min_mm, max_mm], "quantization_mm": 1 << shift,
            "gray_far": depth_stream.GRAY_FAR, "gray_near": depth_stream.GRAY_NEAR,
            "gray_invalid_below": depth_stream.GRAY_INVALID_BELOW, "requested_variant": requested,
            "variant": variant or depth_stream.HOST_JPEG, "variant_reason": reason, "size_problem": problem,
            "source": depth_stream.source_for(variant or depth_stream.HOST_JPEG), "disparity_format": fmt,
            "disparity_problem": disparity_problem}
    if variant is None:
        return None, None, None, info, ()

    frame_type = depth_stream.SCRIPT_FRAME_TYPES[variant]
    script = p.create(dai.node.Script)
    sources = {depth_stream.MONO_FEATURES: stereo.rectifiedLeft, depth_stream.DISPARITY: stereo.disparity}
    sources.get(variant, stereo.depth).link(script.inputs["depth"])
    script.inputs["depth"].setBlocking(False)
    script.inputs["depth"].setMaxSize(1)
    script.inputs["control"].setBlocking(False)
    script.inputs["control"].setMaxSize(4)
    script.setScript(depth_stream.device_script(every, min_mm, max_mm, frame_type=frame_type))
    if variant == depth_stream.DISPARITY:
        info["disparity_max"] = int(fmt["max_disparity"])

    encoder = p.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(args.depth_stream_quality)
    encoder.input.setBlocking(False)
    encoder.input.setMaxSize(1)
    nodes = [script, encoder]
    if variant == depth_stream.MANIP:
        manip = p.create(dai.node.ImageManip)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        manip.setMaxOutputFrameSize(depth_stream.manip_output_size(args.width, args.height))
        manip.setNumFramesPool(depth_stream.MANIP_FRAMES_POOL)
        manip.inputImage.setBlocking(False)
        manip.inputImage.setMaxSize(1)
        script.outputs["gray"].link(manip.inputImage)
        manip.out.link(encoder.input)
        nodes.append(manip)
    else:
        script.outputs["gray"].link(encoder.input)

    control_queue = script.inputs["control"].createInputQueue(maxSize=4, blocking=False)
    frame_queue = encoder.out.createOutputQueue(maxSize=2, blocking=False)
    status_queue = script.outputs["status"].createOutputQueue(maxSize=8, blocking=False)
    names = tuple(name for name in map(node_log_name, nodes) if name)
    return control_queue, frame_queue, status_queue, info, names


def build_depth_points(p, stereo, feature_tracker, args):
    """
    points: the Script (depth_points.device_script) pairing each request with its depth frame, and the
    SpatialLocationCalculator it feeds in lockstep (blocking size-1 inputs both ways, inputConfig waiting for a
    message, so exactly one config and one depth frame are in flight). The Script's depth input is
    non-blocking and holds the newest DEPTH_BUFFER frames (a request comes up to WAIT_PERIODS after its frame).
    rois "host": the tags and configs come from the host's 'rois' queue (ROIS_QUEUE: REQUESTS_AHEAD of each);
    "camera": the Script builds the configs from the FeatureTracker output.

    Returns (points queue, status queue, rois queue or None).
    """
    thresholds = depth_points.thresholds_mm(args.min_depth, args.max_depth)
    script = p.create(dai.node.Script)
    slc = p.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(script.inputs["depth"])
    script.inputs["depth"].setBlocking(False)
    script.inputs["depth"].setMaxSize(depth_points.DEPTH_BUFFER)
    rois_queue = None
    if args.depth_points_rois == depth_points.ROIS_CAMERA:
        feature_tracker.outputFeatures.link(script.inputs["features"])
        script.inputs["features"].setBlocking(False)
        script.inputs["features"].setMaxSize(2)
    else:
        script.inputs["rois"].setBlocking(False)
        script.inputs["rois"].setMaxSize(depth_points.ROIS_QUEUE)
        rois_queue = script.inputs["rois"].createInputQueue(maxSize=depth_points.ROIS_QUEUE, blocking=False)
    for name, slc_input in (("config", slc.inputConfig), (depth_points.SLC_DEPTH_OUT, slc.inputDepth)):
        script.outputs[name].link(slc_input)
        slc_input.setBlocking(True)
        slc_input.setMaxSize(1)
    slc.inputConfig.setWaitForMessage(True)
    for name, output in (("spatial", slc.out), ("passthrough", slc.passthroughDepth)):
        output.link(script.inputs[name])
        script.inputs[name].setBlocking(True)
        script.inputs[name].setMaxSize(1)
    script.setScript(depth_points.device_script(args.depth_points_rois, (args.width, args.height), thresholds))
    points_queue = script.outputs["points"].createOutputQueue(maxSize=8, blocking=False)
    status_queue = script.outputs["status"].createOutputQueue(maxSize=16, blocking=False)
    return points_queue, status_queue, rois_queue


def build_full_depth_gate(p, stereo):
    """points: a Script forwarding every n-th stereo.depth frame while the host asks (depth_points.gate_script)."""
    script = p.create(dai.node.Script)
    stereo.depth.link(script.inputs["depth"])
    script.inputs["depth"].setBlocking(False)
    script.inputs["depth"].setMaxSize(1)
    script.inputs["control"].setBlocking(False)
    script.inputs["control"].setMaxSize(4)
    script.setScript(depth_points.gate_script())
    control_queue = script.inputs["control"].createInputQueue(maxSize=4, blocking=False)
    frame_queue = script.outputs[depth_points.GATE_OUT].createOutputQueue(maxSize=2, blocking=False)
    return control_queue, frame_queue


def build_pipeline(p, device, args, stereo_settings):
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    feature_tracker = p.create(dai.node.FeatureTracker)
    points = args.depth_transport == depth_points.POINTS

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    apply_stereo_config(stereo, stereo_settings, dai)

    feature_tracker.setHardwareResources(1, 2)
    configure_feature_tracker(feature_tracker.initialConfig, args)

    left.requestOutput((args.width, args.height)).link(stereo.left)
    right.requestOutput((args.width, args.height)).link(stereo.right)
    stereo.rectifiedLeft.link(feature_tracker.inputImage)

    handles = PipelineHandles(
        sync_queue=None,
        imu_queue=imu.out.createOutputQueue(maxSize=200, blocking=False),
        feature_config_queue=feature_tracker.inputConfig.createInputQueue(maxSize=4, blocking=False),
        stereo_effective=effective_stereo_config(stereo, stereo_settings),
        depth_transport=args.depth_transport,
    )
    # Non-blocking: if the host falls behind, old frames are dropped and the next delta spans the gap
    if points:
        handles.features_queue = feature_tracker.outputFeatures.createOutputQueue(maxSize=4, blocking=False)
        (handles.points_queue, handles.points_status_queue,
         handles.rois_queue) = build_depth_points(p, stereo, feature_tracker, args)
        handles.full_depth_control_queue, handles.full_depth_queue = build_full_depth_gate(p, stereo)
    else:
        sync = p.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(seconds=0.5 / args.fps))
        feature_tracker.outputFeatures.link(sync.inputs["features"])
        stereo.depth.link(sync.inputs["depth"])
        handles.sync_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)
    if args.snapshots:
        shots = build_snapshot_gate(p, device, stereo, args)
        handles.snapshot_request_queue, handles.snapshot_queue = shots.request_queue, shots.raw_queue
        handles.snapshot_streams, handles.snapshot_unavailable = shots.streams, shots.unavailable
        handles.snapshot_encoded_queues, handles.snapshot_encoded = shots.encoded_queues, shots.encoded
        handles.snapshot_not_encoded = shots.not_encoded
    else:
        handles.snapshot_unavailable = {s: "snapshots are disabled (--no-snapshots)" for s in snapshots.STREAMS}
    if args.depth_stream:
        (handles.depth_stream_control_queue, handles.depth_stream_queue, handles.depth_stream_status_queue,
         handles.depth_stream_info, handles.depth_stream_nodes) = build_depth_stream(p, stereo, args, stereo_settings)
    # One extra frame per stereo output pool for the frame each gate queue holds, plus the one the
    # depth stream Script is converting; the forwarding variants (mono_features, disparity) send
    # StereoDepth's own frame on, which then also waits in (and is read by) the encoder (so does a
    # rectified snapshot on its way to the mono encoder). points: the depth points Script's queue, the
    # frame it keeps for the next request, the SLC's input and passthrough, and the full depth gate's frame
    stream_extra = 0
    if handles.depth_stream_control_queue is not None:
        forwarding = handles.depth_stream_info.get("variant") in depth_stream.FORWARD_VARIANTS
        stream_extra = 3 if forwarding else 2
    points_extra = depth_points.POOL_FRAMES + 1 if points else 0
    snapshot_extra = 0
    if args.snapshots:
        snapshot_extra = 2 if "rect_left" in handles.snapshot_encoded else 1
    extra = snapshot_extra + stream_extra + points_extra
    if extra:
        stereo.setNumFramesPool(STEREO_FRAMES_POOL + extra)
    return handles


def snapshot_frame(msg, stream):
    return msg.getCvFrame() if stream == "rgb" else msg.getFrame()


def body_yaw(R_cam, camera_angle):
    """Yaw (rad, about body z down) of a camera-frame rotation R_01."""
    R = camera_to_body_rotation(R_cam, camera_angle)
    return math.atan2(R[1, 0], R[0, 0])


@dataclass
class OverlayState:
    """References to the last processed frame's features, kept for the snapshot overlay (no copies)."""
    ids: object = None
    uv: object = None
    previous_ids: object = None
    previous_uv: object = None
    matched_index: object = None
    inlier_index: object = None
    sequence: Optional[int] = None
    ok: Optional[bool] = None
    reason: Optional[str] = None

    def update(self, ids, uv, frame, sequence):
        self.previous_ids, self.previous_uv = self.ids, self.uv
        self.ids, self.uv, self.sequence = ids, uv, sequence
        self.matched_index, self.inlier_index = frame.matched_index, frame.pose.inlier_index
        self.ok, self.reason = frame.pose.ok, frame.pose.reason

    def builder(self, image_size):
        """A function building this frame's overlay later (on the snapshot writer thread), or None."""
        if self.ids is None:
            return None
        values = (self.ids, self.uv, self.matched_index, self.inlier_index, self.previous_ids, self.previous_uv,
                  self.sequence, image_size, self.ok, self.reason)
        return lambda: snapshots.build_overlay(*values)


def tracked_arrays(features_msg):
    """(ids int64 (N,), uv float64 (N, 2)) of a TrackedFeatures message."""
    tracked = features_msg.trackedFeatures
    ids = np.fromiter((f.id for f in tracked), dtype=np.int64, count=len(tracked))
    uv = np.array([(f.position.x, f.position.y) for f in tracked], dtype=np.float64).reshape(-1, 2)
    return ids, uv


@dataclass
class FrameInput:
    """One frame for the PnP: the features and either the depth frame (full_frame) or the depth at them (points)."""
    sequence: int
    t: float
    ids: np.ndarray
    uv: np.ndarray
    depth_shape: tuple  # (height, width) of the depth grid
    depth: Optional[np.ndarray] = None
    depth_msg: Optional[object] = None
    points: Optional[object] = None  # depth_points.DepthPoints
    depth_problem: Optional[str] = None  # points: why this frame has no depth (it gets none, never older depth)


class SequenceGaps:
    """Frames missing between consecutive sequence numbers (a restarted camera starts again)."""

    def __init__(self):
        self.last = None
        self.gaps = self.frames = 0

    def add(self, sequence):
        if self.last is not None and sequence > self.last + 1:
            self.gaps += 1
            self.frames += sequence - self.last - 1
        self.last = sequence


class FullFrameSource:
    """full_frame: Sync groups of the features and the depth frame, every frame over the link."""

    def __init__(self, handles, link_streams):
        self.queue = handles.sync_queue
        self.link_streams = link_streams
        self.gaps = SequenceGaps()
        self.depth_frames = 0

    def get(self):
        return self.queue.get()

    @staticmethod
    def stamp(group):
        depth_msg = group["depth"]
        return depth_msg.getSequenceNum(), timestamp_seconds(depth_msg, exposure_middle=True)

    def frame(self, group, sequence, t):
        features_msg, depth_msg = group["features"], group["depth"]
        if features_msg.getSequenceNum() != sequence:
            logger.warning(f"Sequence mismatch: features {features_msg.getSequenceNum()} depth {sequence}")
        ids, uv = tracked_arrays(features_msg)
        depth = depth_msg.getFrame()
        self.link_streams.add("depth", link_stats.frame_bytes(link_stats.data_bytes(depth_msg)))
        self.link_streams.add("features", link_stats.features_bytes(ids))
        self.link_streams.add("sync_group", link_stats.MESSAGE_GROUP_OVERHEAD)
        self.gaps.add(sequence)
        self.depth_frames += 1
        return FrameInput(sequence, t, ids, uv, depth.shape[:2], depth=depth, depth_msg=depth_msg)

    def on_status(self):
        pass

    def status(self):
        return {"transport": depth_points.FULL_FRAME, "frames": self.depth_frames, "feature_gaps": self.gaps.gaps,
                "feature_gap_frames": self.gaps.frames}


@dataclass
class FetchedFeatures:
    """A features message PointsSource took from the queue, with its depth already requested."""
    msg: object
    ids: np.ndarray
    uv: np.ndarray
    requested: bool


class PointsSource:
    """
    points: the features straight from the FeatureTracker, then the camera's depth at them
    (depth_points.DepthPointsReceiver: rois "host" sends each frame's tag and ROI config). Each features message
    is requested as soon as it is fetched: get() takes the messages already queued and requests them, then
    hands out the oldest, so when the Pi falls behind the camera already works on the next frames while the Pi
    solves this one. At most REQUESTS_AHEAD are requested and not yet handed out (what the camera's 'rois' queue
    holds); queued messages beyond that are dropped unrequested (host_behind_dropped). t is the features'
    device timestamp: the end of the exposure (a TrackedFeatures message has no exposure time), the same
    for every frame, so the deltas' dt are unchanged.
    """

    def __init__(self, handles, link_streams, args, decimation=1, log=print):
        self.handles = handles
        self.queue = handles.features_queue
        self.link_streams = link_streams
        self.image_size = (args.width, args.height)
        self.fallback_size = (args.width // max(decimation, 1), args.height // max(decimation, 1))
        self.ahead = deque()
        self.ahead_max = self.behind_dropped = 0
        thresholds = depth_points.thresholds_mm(args.min_depth, args.max_depth)
        rois_queue = handles.rois_queue

        def send(message):
            rois_queue.send(message)

        def counted(rows):
            link_streams.add("depth_points", link_stats.spatial_data_bytes(0 if rows is None else len(rows)))

        self.receiver = depth_points.DepthPointsReceiver(
            handles.points_queue, self.image_size, args.depth_points_rois,
            wait_s=depth_points.wait_s(args.fps, args.depth_points_rois),
            send=send if rois_queue is not None else None,
            build_config=lambda windows, limits, sequence: depth_points.make_config(dai, windows, limits, sequence),
            build_tag=lambda sequence: depth_points.make_tag(dai, sequence),
            thresholds=thresholds, frame_period_s=1.0 / max(args.fps, 1), log=log, on_message=counted)

    def _fetched(self, msg):
        self.on_status()
        ids, uv = tracked_arrays(msg)
        self.link_streams.add("features", link_stats.features_bytes(ids))
        receiver = self.receiver
        requested = receiver.request(msg.getSequenceNum(), uv)
        if requested and receiver.roi_source == depth_points.ROIS_HOST:
            self.link_streams.add("depth_points_rois", link_stats.points_request_bytes(len(ids) + depth_points.EXTRA_ROIS))
        return FetchedFeatures(msg, ids, uv, requested)

    def get(self):
        if not self.ahead:
            self.ahead.append(self._fetched(self.queue.get()))
        more = self.queue.tryGetAll()
        room = depth_points.REQUESTS_AHEAD - len(self.ahead)
        if len(more) > room:
            # Never more requests in flight than the camera's 'rois' queue holds: the newest that fit are taken
            self.behind_dropped += len(more) - room
            more = more[len(more) - room:] if room > 0 else []
        for msg in more:
            self.ahead.append(self._fetched(msg))
        self.ahead_max = max(self.ahead_max, len(self.ahead))
        return self.ahead.popleft()

    @staticmethod
    def stamp(item):
        return item.msg.getSequenceNum(), item.msg.getTimestampDevice().total_seconds()

    def on_status(self):
        if self.handles.points_status_queue is None:
            return
        for report in self.handles.points_status_queue.tryGetAll():
            data = report.getData()
            self.link_streams.add("depth_points_status", link_stats.buffer_bytes(len(data)))
            self.receiver.on_status(data)

    def frame(self, item, sequence, t):
        self.on_status()
        receiver = self.receiver
        try:
            features_ts = item.msg.getTimestamp().total_seconds()
        except Exception:
            features_ts = None
        points, problem = receiver.take(sequence, item.uv, features_ts, requested=item.requested)
        size = points.depth_size if points is not None else (receiver.depth_size or self.fallback_size)
        return FrameInput(sequence, t, item.ids, item.uv, (size[1], size[0]), points=points, depth_problem=problem)

    def status(self):
        return {**self.receiver.status(), "requested_ahead_max": self.ahead_max,
                "host_behind_dropped": self.behind_dropped}


def frame_depths(frame_in, params):
    """(N,) metres per feature for PnPOdometry.process_depths (points mode; NaN where there is none)."""
    points = frame_in.points
    if points is None:
        return np.full(len(frame_in.ids), np.nan)
    return depth_from_points(points.depth_mm, points.pixels, points.samples, params, points.pixel_count_reported)


def _raise_system_exit(signum, frame):
    # The supervisor stops us with SIGTERM; exit through the finally blocks so the log gets its exit line
    raise SystemExit(128 + signum)


def main(argv=None):
    args = parse_args(argv)
    log_only = resolve_log_only(args.log_only)
    stereo_settings = resolve_stereo_settings(args)
    signal.signal(signal.SIGTERM, _raise_system_exit)

    print("🚀 Starting OAK-D PnP VIO")
    print(f"📷 Camera: {args.width}x{args.height} @ {args.fps} FPS")
    print(f"📐 Camera Angle: {args.camera_angle}°")
    print(f"🔧 Features on camera: {args.num_features} ({args.corner_detector}), "
          f"min inliers {args.min_inliers}, gyro gate {args.gyro_gate}°, max reference age {args.max_reference_age} s, "
          f"recovery after {args.recovery_frames} failures")
    print(f"🌊 Stereo: {', '.join(f'{k[7:]}={v}' for k, v in stereo_settings.items())}")
    print(f"📏 Intrinsics from {args.intrinsics_from}"
          f"{f', override {args.intrinsics_override}' if args.intrinsics_override else ''}, "
          f"refractive index {args.refractive_index}{' (in air)' if args.refractive_index == 1.0 else ''}, "
          f"depth scale {args.depth_scale}")
    if args.lever_arm:
        print(f"📐 Lever arm diagnostics on (logged only, not sent): camera at {list(args.lever_arm_m)} m FRD from the IMU")
    print(f"📡 MAVLink: {'disabled (log-only)' if log_only else args.vehicle_ip}")

    data_log = open_vio_log()
    logger.info(f"Data log: {data_log.path}")
    data_log.event("start", algorithm="pnp", args=vars(args), stereo=stereo_settings, log_only=log_only,
                   log_only_forced_by_env=env_forces_log_only(), depth_scale=args.depth_scale,
                   refractive_index=args.refractive_index, intrinsics_from=args.intrinsics_from,
                   intrinsics_override=args.intrinsics_override, lever_arm=args.lever_arm,
                   lever_arm_m=list(args.lever_arm_m))
    exit_code = 0
    try:
        run(args, log_only, data_log, stereo_settings)
    except SystemExit as e:
        exit_code = e.code
        raise
    except BaseException as e:
        exit_code = 1
        data_log.event("error", error=repr(e))
        raise
    finally:
        data_log.event("exit", exit_code=exit_code)
        data_log.close()


def run(args, log_only, data_log, stereo_settings):
    telemetry = vio_telemetry.Telemetry(args.telemetry_dir or args.snapshot_dir / "telemetry", fps=args.fps,
                                        min_inliers=args.min_inliers, mode="log-only" if log_only else "sending",
                                        log=logger.warning, link=link_stats.LinkMonitor())
    telemetry.set_phase("opening_device")
    telemetry.start()
    try:
        run_pipeline(args, log_only, data_log, stereo_settings, telemetry)
    finally:
        telemetry.close()


def run_pipeline(args, log_only, data_log, stereo_settings, telemetry):
    """
    Opens the camera and runs sessions (run_session) on it. When the camera restarted under a session
    (depthai's XLink reconnect), the pipeline is torn down and the camera reopened, in this process: the
    MAVLink output, the data log, the telemetry and the integrated pose carry on.
    """
    def log_warning(message):
        logger.warning(message)
        data_log.event("warning", message=message)

    mavlink = setup_autopilot_output(log_only, args.vehicle_ip, AutopilotParams, MavlinkSender,
                                     log=log_warning)
    if mavlink:
        telemetry.output_error = lambda: mavlink.last_error

    params = PnPParams(
        min_inliers=args.min_inliers,
        reprojection_error_px=args.reprojection_error,
        gyro_gate_deg=args.gyro_gate if args.gyro_gate > 0 else 180.0,
        max_reference_age_s=args.max_reference_age,
        min_depth_m=args.min_depth,
        max_depth_m=args.max_depth,
        depth_correction=args.depth_scale,
        refractive_index=args.refractive_index,
    )

    def log_device_open(message):
        logger.info(message)
        data_log.event("device_open", message=message)

    attitude = None
    if args.pose_attitude_source == pose_stream.AUTOPILOT:
        attitude = autopilot_attitude.from_mavlink2rest(args.vehicle_ip, log=logger.info)
    pose_out = pose_stream.PoseStreamService(telemetry.directory, args.camera_angle, log=logger.info,
                                             attitude=attitude)
    telemetry.pose_status = pose_out.status
    reconnects = 0
    try:
        while True:
            try:
                device = open_oak_device(args.device_timeout, log=log_device_open)
            except DeviceOpenTimeout as e:
                logger.error(f"❌ {e}")
                data_log.event("error", error=str(e))
                sys.exit(2)
            telemetry.temperature = device.getChipTemperature
            telemetry.camera_load = lambda device=device: vio_telemetry.read_camera_load(device)
            telemetry.set_phase("configuring")
            reason = run_session(device, args, log_only, data_log, stereo_settings, telemetry, mavlink, params,
                                 pose_out, log_warning)
            if reason is None:
                return
            telemetry.temperature = None
            telemetry.camera_load = None
            telemetry.link.clear_device(f"camera reconnecting: {reason}")
            reconnects += 1
            logger.warning(f"🔌 {reason}: restarting the pipeline (reconnect #{reconnects}) so snapshots, the depth "
                           f"stream and the feature tracker settings reach the camera again")
            data_log.event("reconnect", count=reconnects, reason=reason)
            telemetry.set_phase("opening_device", reconnects=reconnects,
                                last_reconnect={"unix_time": time.time(), "reason": reason})
    finally:
        pose_out.close()
        if mavlink:
            mavlink.close()


def run_session(device, args, log_only, data_log, stereo_settings, telemetry, mavlink, params, pose_out,
                log_warning):
    """
    One pipeline on the open device until it stops (returns None) or the camera restarts under it
    (DeviceRestartWatch; returns the reason after closing the device). A restarted camera's frames never
    reach the PnP: the session ends at the first one, and the next session starts from a new reference.
    """
    quality_interval = 1.0 / args.quality_rate if args.quality_rate > 0 else float('inf')
    restart = DeviceRestartWatch(log=logger.warning)
    try:
        device.setMaxReconnectionAttempts(1, restart.on_reconnection)
    except Exception as e:
        logger.warning(f"No depthai reconnection callback ({e}); a camera restart is noticed from its frames only")

    writer = stream = log_callback = link_log_callback = None
    link_streams = telemetry.link.streams
    try:
        with dai.Pipeline(device) as p:
            handles = build_pipeline(p, device, args, stereo_settings)
            warnings = stereo_warnings(stereo_settings, args.intrinsics_from)
            logger.info(f"Stereo config (effective): {handles.stereo_effective}")
            for message in warnings:
                log_warning(f"Stereo: {message}")
            data_log.event("stereo_config", requested=stereo_settings, effective=handles.stereo_effective,
                           warnings=warnings)
            device_info = log_device_info(device)
            link_info = link_stats.device_link_info(device, device_info.get("USB"))
            telemetry.link.set_device(device.getProfilingData, **link_info)
            data_log.event("device", **device_info, link=link_info)
            try:
                link_log_callback = device.addLogCallback(
                    lambda message: link_streams.add("device_log", link_stats.log_bytes(message.payload)))
            except Exception as e:
                logger.warning(f"Camera link: camera log messages not counted ({e})")
            calibration = load_calibration(device, args)
            R_cam_imu = calibration.R_cam_imu
            rectified, rectified_problem = rectified_for_settings(calibration, stereo_settings, (args.width, args.height))
            logger.info(f"Rectified intrinsics (alpha {stereo_settings['stereo_alpha']}, distortion correction "
                        f"{stereo_settings['stereo_distortion_correction']}): {format_intrinsics(rectified)}"
                        f"{f' ({rectified_problem})' if rectified_problem else ''}")
            data_log.event("calibration", intrinsics=as_list(calibration.raw), distortion=calibration.distortion,
                           rectified=as_list(rectified), rectified_problem=rectified_problem,
                           alpha=stereo_settings["stereo_alpha"],
                           distortion_correction=stereo_settings["stereo_distortion_correction"],
                           baseline_m=calibration.baseline_m, baseline_spec_m=calibration.baseline_spec_m,
                           R_cam_imu=R_cam_imu)
            snapshots.clear_camera_status(args.snapshot_dir)
            odometry = None  # created on the first frame, once the rectified intrinsics are known
            probe = depth_probe.DepthProbeService(
                args.snapshot_dir / "depth_probe",
                (stereo_settings["stereo_depth_min_mm"], stereo_settings["stereo_depth_max_mm"]),
                args.depth_scale, (args.width, args.height), log=logger.warning,
                refractive_index=args.refractive_index)
            gyro = GyroBuffer()
            stats = Stats(lambda: telemetry.chip_temperature, link=telemetry.link.summary)
            tracking = None
            last_quality_time = 0.0
            previous_ids = set()
            overlay = OverlayState()

            recovery = TrackingRecovery(args.recovery_frames)
            reinit = FeatureTrackerReinit(lambda maintainer: handles.feature_config_queue.send(
                configure_feature_tracker(dai.FeatureTrackerConfig(), args, maintainer)))

            def send_snapshot_request(stream, raw):
                request = dai.Buffer()
                request.setSequenceNum(snapshots.request_number(stream, raw))
                handles.snapshot_request_queue.send(request)
                link_streams.add("control", link_stats.BUFFER_OVERHEAD)

            writer = snapshots.SnapshotWriter(
                args.snapshot_dir, (stereo_settings["stereo_depth_min_mm"], stereo_settings["stereo_depth_max_mm"]),
                log=logger.warning)
            snapshot_service = snapshots.SnapshotService(
                handles.snapshot_streams, send_snapshot_request, writer, args.snapshot_dir,
                unavailable=handles.snapshot_unavailable, log=logger.warning,
                overlay=lambda: overlay.builder((args.width, args.height)),
                encoded=handles.snapshot_encoded, not_encoded=handles.snapshot_not_encoded)
            data_log.event("snapshots", streams=list(handles.snapshot_streams),
                           unavailable=handles.snapshot_unavailable, encoding=args.snapshot_encoding,
                           encoded_on_camera=list(handles.snapshot_encoded),
                           not_encoded=handles.snapshot_not_encoded)

            def send_depth_stream_control(enabled):
                control = dai.Buffer()
                control.setSequenceNum(1 if enabled else 0)
                handles.depth_stream_control_queue.send(control)

            info = handles.depth_stream_info
            if info and info.get("variant") == depth_stream.DISPARITY:
                # The camera's depth = rectified fx * baseline / disparity
                focal = (rectified or calibration.raw).fx
                info["disparity"] = depth_stream.disparity_scale(
                    focal, calibration.baseline_m, *info["depth_range_mm"],
                    info.get("disparity_max", depth_stream.DISPARITY_MAX))
            points_mode = handles.depth_transport == depth_points.POINTS
            if info:
                on_camera = handles.depth_stream_control_queue is not None
                host = device_log = debug = None
                if not on_camera or info.get("requested_variant") == depth_stream.AUTO:
                    # points: the full depth gate already sends only every n-th frame, and only while it streams
                    host = depth_stream.HostJpegEncoder(*info["depth_range_mm"], args.depth_stream_quality,
                                                        1 if points_mode else info.get("every", 1))
                info["depth_frames"] = "on_demand" if points_mode else "every_frame"
                if on_camera:
                    device_log = depth_stream.DeviceLog(handles.depth_stream_nodes)
                    try:
                        log_callback = device.addLogCallback(device_log.on_message)
                    except Exception as e:
                        logger.warning(f"Depth stream: no camera log callback ({e})")
                        device_log = None
                    if args.depth_stream_debug:
                        debug = depth_stream.DeviceLogDebug(device, dai.LogLevel.DEBUG, log=logger.info)
                stream = depth_stream.DepthStreamService(telemetry.directory, send_depth_stream_control, info,
                                                         log=logger.info, host_encoder=host, device_log=device_log,
                                                         debug=debug)
                where = "on the camera" if on_camera else "on the Pi"
                logger.info(f"Depth stream: on demand, {info['fps_target']:.1f} fps MJPEG quality "
                            f"{args.depth_stream_quality}, {info.get('variant')} (encoded {where}; requested "
                            f"{info.get('requested_variant')}){', camera debug log while streaming' if debug else ''}")
                if info.get("size_problem"):
                    log_warning(f"Depth stream: {info['size_problem']}")
                elif info.get("variant_reason") and info.get("requested_variant") != depth_stream.AUTO:
                    log_warning(f"Depth stream: {info['variant_reason']}")
            data_log.event("depth_stream", enabled=stream is not None, debug=args.depth_stream_debug, **info)
            features_out = feature_stream.FeatureStreamService(telemetry.directory, (args.width, args.height),
                                                               log=logger.info)

            gate = None
            if points_mode:
                source = PointsSource(handles, link_streams, args,
                                      int(stereo_settings.get("stereo_decimation") or 1), log=logger.warning)
                if handles.full_depth_control_queue is not None:
                    def send_gate_control(every):
                        control = dai.Buffer()
                        control.setSequenceNum(int(every))
                        handles.full_depth_control_queue.send(control)
                        link_streams.add("control", link_stats.BUFFER_OVERHEAD)
                    gate = depth_points.FullDepthGate(send_gate_control, log=logger.info)
                logger.info(f"Depth transport: points (camera SpatialLocationCalculator, ROIs built on the "
                            f"{args.depth_points_rois}); full depth frames only on demand")
            else:
                source = FullFrameSource(handles, link_streams)
                logger.info("Depth transport: full_frame (every depth frame to the Pi)")
            data_log.event("depth_transport", transport=handles.depth_transport,
                           rois=args.depth_points_rois if points_mode else None,
                           wait_s=depth_points.wait_s(args.fps, args.depth_points_rois) if points_mode else None,
                           depth_buffer_frames=depth_points.DEPTH_BUFFER if points_mode else None)
            telemetry.depth_points_status = lambda: {**source.status(),
                                                     "full_depth": gate.status() if gate is not None else None}
            stats.depth_summary = lambda: depth_summary(source.status(), telemetry.camera_load_value)

            p.start()
            telemetry.set_phase("running")
            print("✅ Pipeline started!")
            if log_only:
                logger.warning(f"⚠️ {LOG_ONLY_BANNER}")
            else:
                print("📡 Sending VIO data via MAVLink...")

            while p.isRunning() and not restart.triggered:
                item = source.get()
                if restart.triggered:
                    break
                start = time.perf_counter()
                sequence, t = source.stamp(item)
                if restart.on_frame(sequence, t):
                    break
                reinit.step()

                for imu_data in handles.imu_queue.tryGetAll():
                    link_streams.add("imu", link_stats.imu_bytes(gyro.add(imu_data)))

                frame_in = source.frame(item, sequence, t)
                item = None
                ids, uv = frame_in.ids, frame_in.uv

                if odometry is None:
                    choice = select_intrinsics(args, calibration, frame_in.depth_msg, rectified, rectified_problem)
                    for message in choice.warnings:
                        log_warning(f"Intrinsics: {message}")
                    intrinsics = choice.intrinsics
                    logger.info(f"Intrinsics ({choice.source}): {format_intrinsics(intrinsics)}; raw cam_b "
                                f"{format_intrinsics(calibration.raw)}, rectified {format_intrinsics(rectified)}, "
                                f"depth frame {format_intrinsics(choice.frame)} (matches {choice.frame_matches}), "
                                f"alpha {stereo_settings['stereo_alpha']}")
                    status = camera_status(args, calibration, rectified, choice, stereo_settings)
                    logger.info(f"Refractive index {args.refractive_index} x depth scale {args.depth_scale}: PnP uses "
                                f"{format_intrinsics(refracted_intrinsics(intrinsics, args.refractive_index))}, "
                                f"depth x {status['total_depth_scale']:.4g}")
                    data_log.event("intrinsics", **{key: status[key] for key in (
                        "source", "requested_source", "intrinsics", "effective_intrinsics", "raw", "rectified", "frame",
                        "frame_matches", "override", "alpha", "distortion_correction", "baseline_m", "depth_scale",
                        "refractive_index", "total_depth_scale", "warnings")})
                    try:
                        snapshots.write_camera_status(status, args.snapshot_dir)
                    except OSError as e:
                        logger.warning(f"Camera status not written: {e}")
                    odometry = PnPOdometry(intrinsics, params, image_size=(args.width, args.height))
                    probe.set_intrinsics(intrinsics, choice.source)

                id_list = ids.tolist()
                new_features, tracked_features = feature_id_counts(previous_ids, id_list)
                previous_ids = set(id_list)

                # The reference is held across failed frames, so the prior spans the whole gap
                reference_t = odometry.prev_timestamp
                R_prior = None
                if R_cam_imu is not None and reference_t is not None:
                    R_prior = gyro.rotation(reference_t, t, R_cam_imu)
                gyro.prune((t if reference_t is None else min(t, reference_t)) - GYRO_BUFFER_SECONDS)

                if frame_in.depth is not None:
                    frame = odometry.process(ids, uv, frame_in.depth, t, R_prior)
                else:
                    frame = odometry.process_depths(ids, uv, frame_depths(frame_in, params), t, R_prior,
                                                    depth_missing=frame_in.depth_problem is not None)
                pose = frame.pose
                overlay.update(ids, uv, frame, sequence)
                features_out.tick(uv, frame_in.depth_shape, sequence, pose.ok, pose.reason,
                                  frame.depth_valid, frame.matched_index, pose.inlier_index)

                angle_delta = position_delta = time_delta_us = yaw_delta = rotation_delta = None
                sent = False
                if pose.ok:
                    # Exactly what goes (or would go) into VISION_POSITION_DELTA
                    angle_delta = camera_to_body_frd(pose.angle_delta, args.camera_angle)
                    position_delta = camera_to_body_frd(pose.position_delta, args.camera_angle)
                    time_delta_us = frame.dt * 1e6
                    yaw_delta = body_yaw(R_prior, args.camera_angle) if R_prior is not None else angle_delta[2]
                    rotation_delta = (matrix_to_rotvec(camera_to_body_rotation(R_prior, args.camera_angle))
                                      if R_prior is not None else angle_delta)
                    if mavlink:
                        sent = mavlink.send_vision_position_delta(
                            angle_delta=angle_delta,
                            position_delta=position_delta,
                            time_delta_us=time_delta_us,
                            confidence=pose.confidence,
                        )

                now_tracking = pose.ok
                if pose.reason == "no_reference":
                    now_tracking = tracking  # first frame: nothing to say yet
                else:
                    failed_frames = recovery.threshold
                    if recovery.update(pose.ok):
                        odometry.reset()
                        reinit.start()
                        stats.recoveries += 1
                        telemetry.record_recovery()
                        logger.warning(f"🔄 Tracking recovery #{recovery.count} after {failed_frames} failed frames "
                                       f"({pose.reason}, features {frame.features}, depth {frame.valid_depth}): "
                                       f"PnP reference reset, feature tracker re-initialised; "
                                       f"next after {recovery.threshold} more failures")
                        data_log.event("recovery", count=recovery.count, failed_frames=failed_frames,
                                       reason=pose.reason, features=frame.features, valid_depth=frame.valid_depth,
                                       matches=pose.matches, next_after=recovery.threshold)

                snapshot_service.tick()
                if handles.snapshot_queue is not None:
                    snapshot_msg = handles.snapshot_queue.tryGet()
                    if snapshot_msg is not None:
                        nbytes = link_stats.frame_bytes(link_stats.data_bytes(snapshot_msg))
                        link_streams.add("snapshot", nbytes)
                        snapshot_service.on_frame(lambda stream, m=snapshot_msg: snapshot_frame(m, stream),
                                                  timestamp_seconds(snapshot_msg), snapshot_msg.getSequenceNum(),
                                                  encoded=False, nbytes=nbytes)
                for encoded_queue in handles.snapshot_encoded_queues:
                    jpeg = encoded_queue.tryGet()
                    if jpeg is not None:
                        nbytes = link_stats.encoded_bytes(link_stats.data_bytes(jpeg))
                        link_streams.add("snapshot", nbytes)
                        snapshot_service.on_frame(lambda stream, m=jpeg: bytes(m.getData()), timestamp_seconds(jpeg),
                                                  jpeg.getSequenceNum(), encoded=True, nbytes=nbytes)
                probe.tick()
                stream_depth, stream_sequence = frame_in.depth, sequence
                if frame_in.depth is not None:
                    probe.on_depth(frame_in.depth, t, sequence)
                elif handles.full_depth_queue is not None:
                    # points: the full frames the gate forwarded (each its own frame, not this one)
                    for msg in handles.full_depth_queue.tryGetAll():
                        nbytes = link_stats.frame_bytes(link_stats.data_bytes(msg))
                        link_streams.add("depth_on_demand", nbytes)
                        if gate is not None:
                            gate.on_frame(nbytes)
                        stream_depth, stream_sequence = msg.getFrame(), msg.getSequenceNum()
                        probe.on_depth(stream_depth, timestamp_seconds(msg, exposure_middle=True), stream_sequence)
                if stream is not None:
                    if frame_in.depth is not None:
                        stream.tick(frame_in.depth, sequence)
                    else:
                        points = frame_in.points
                        stream.tick(stream_depth, stream_sequence, center=points.center if points else None,
                                    sample=depth_points.depth_sample(points))
                    encoded = handles.depth_stream_queue.tryGetAll() if handles.depth_stream_queue is not None else ()
                    for jpeg in encoded:
                        link_streams.add("depth_stream", link_stats.encoded_bytes(link_stats.data_bytes(jpeg)))
                    if encoded:
                        image_sequence = encoded[-1].getSequenceNum()
                        if (stream.on_frame(encoded[-1].getData(), image_sequence)
                                and stream.variant == depth_stream.MONO_FEATURES):
                            features_out.publish_for(image_sequence)
                    if handles.depth_stream_status_queue is not None:
                        for report in handles.depth_stream_status_queue.tryGetAll():
                            data = report.getData()
                            link_streams.add("depth_stream_status", link_stats.buffer_bytes(len(data)))
                            stream.on_device_status(data)
                if gate is not None:
                    watched_host_jpeg = stream is not None and stream.on_host and stream.enabled
                    gate.update({"depth_probe": 1 if probe.active is not None else 0,
                                 "host_jpeg": int(stream.info.get("every") or 1) if watched_host_jpeg else 0})
                proc_ms = (time.perf_counter() - start) * 1000.0

                lever = lever_arm_fields(position_delta, R_prior, pose.rotation, args) if args.lever_arm else {}
                data_log.step(
                    device_timestamp=t,
                    sequence=sequence,
                    depth_source="frame" if frame_in.depth is not None else "points",
                    depth_problem=frame_in.depth_problem,
                    time_delta_us=time_delta_us,
                    position_delta=position_delta,
                    angle_delta=angle_delta,
                    confidence=pose.confidence,
                    features=frame.features,
                    new_features=new_features,
                    tracked_features=tracked_features,
                    valid_depth=frame.valid_depth,
                    matches=pose.matches,
                    inliers=pose.inliers,
                    reprojection_rms_px=pose.reprojection_rms_px,
                    gyro_disagreement_deg=pose.gyro_disagreement_deg,
                    proc_ms=proc_ms,
                    status="ok" if pose.ok else "rejected",
                    reason=pose.reason or None,
                    sent=sent,
                    reference_age_s=frame.reference_age_s,
                    gap_frames=frame.gap_frames,
                    reference_reset=frame.reference_reset,
                    reference_carried=frame.reference_carried,
                    **lever,
                )
                now_s = time.time()
                telemetry.record_frame(now_s, pose.ok, pose.reason, frame.features, frame.valid_depth,
                                       pose.matches, pose.inliers, proc_ms, frame.dt, position_delta, yaw_delta,
                                       frame.gap_frames, frame.reference_reset, sent)
                pose_out.record(now_s, pose.ok, position_delta, rotation_delta)

                now = time.monotonic()
                if now_tracking is not None and now_tracking != tracking:
                    if mavlink:
                        mavlink.send_named_value_float("VIO_TRACK", 1.0 if now_tracking else 0.0)
                    if not now_tracking:
                        logger.warning(f"Tracking lost: {pose.reason} (matches {pose.matches}, inliers {pose.inliers})")
                    else:
                        logger.info("Tracking (re)established")
                    data_log.event("tracking", tracking=now_tracking, reason=pose.reason or None)
                    tracking = now_tracking
                if mavlink and now - last_quality_time >= quality_interval:
                    last_quality_time = now
                    mavlink.send_named_value_float("VIO_TRACK", 1.0 if tracking else 0.0)
                    mavlink.send_named_value_float("VIO_INLIER", pose.inliers)
                    mavlink.send_named_value_float("VIO_FEAT", frame.features)
                    mavlink.send_named_value_float("VIO_CONF", pose.confidence)
                    mavlink.send_named_value_float("VIO_PROCMS", proc_ms)

                logger.debug(
                    f"frame t={t:.3f} dt={frame.dt if frame.dt is not None else float('nan'):.3f}s "
                    f"feat={frame.features} (new {new_features}) depth={frame.valid_depth} match={pose.matches} "
                    f"inl={pose.inliers} gyro_diff={pose.gyro_disagreement_deg:.2f}° gap={frame.gap_frames}"
                    f"{' reference reset' if frame.reference_reset else ''} "
                    f"{'ok' if pose.ok else pose.reason} proc={proc_ms:.1f}ms")
                stats.add(frame, proc_ms)
                stats.maybe_report()
    finally:
        if stream is not None:
            stream.close()
        for callback in (log_callback, link_log_callback):
            if callback is not None:
                try:
                    device.removeLogCallback(callback)
                except Exception:
                    pass
        if writer is not None:
            writer.close()
    if not restart.triggered:
        return None
    try:
        device.close()  # leaving the pipeline closed it already; this only makes sure
    except Exception as e:
        logger.warning(f"Closing the camera after its restart: {e}")
    return restart.reason


if __name__ == "__main__":
    main()
