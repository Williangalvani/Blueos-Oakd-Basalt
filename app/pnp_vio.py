#!/usr/bin/env python3
"""
BlueOS OAK-D PnP VIO (option D)

All pixel work runs on the OAK-D: stereo depth and LK feature tracking on the rectified left image.
The host only looks up depth at the tracked features and solves frame-to-frame PnP with a gyro
rotation prior (pnp_odometry.py), then sends VISION_POSITION_DELTA over MAVLink.
"""

import argparse
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

import snapshots
from autopilot_params import AutopilotParams
from device_open import DEFAULT_TIMEOUT, DeviceOpenTimeout, open_oak_device
from mavlink_sender import MavlinkSender
from pnp_odometry import (Intrinsics, PnPOdometry, PnPParams, camera_to_body_frd, integrate_gyro,
                          intrinsics_from_frame)
from stereo_config import (add_stereo_arguments, apply_stereo_config, effective_stereo_config,
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

CORNER_DETECTORS = {
    "harris": dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS,
    "shi-tomasi": dai.FeatureTrackerConfig.CornerDetector.Type.SHI_THOMASI,
}

INTRINSICS_SOCKETS = {
    "cam_b": dai.CameraBoardSocket.CAM_B,
    "cam_c": dai.CameraBoardSocket.CAM_C,
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='BlueOS OAK-D PnP VIO')

    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--camera-angle', type=float, default=0.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=400)

    parser.add_argument('--num-features', type=int, default=300,
                        help='Target number of features tracked on the camera')
    parser.add_argument('--corner-detector', choices=sorted(CORNER_DETECTORS), default='harris')
    parser.add_argument('--intrinsics-from', choices=['frame', *sorted(INTRINSICS_SOCKETS)], default='frame',
                        help='Rectified-left camera matrix: "frame" reads it from the depth frame (matches alpha '
                             'scaling), cam_b/cam_c use the raw calibration (see NOTES_OPTION_D.md)')

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
    parser.add_argument('--snapshot-dir', type=Path, default=snapshots.SNAPSHOT_DIR)

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
        for packet in imu_data.packets:
            g = packet.gyroscope
            self.samples.append((g.getTimestampDevice().total_seconds(), g.x, g.y, g.z))

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
    def __init__(self, device=None):
        self.device = device
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
        if self.device is None:
            return ""
        try:
            return f", chip {self.device.getChipTemperature().average:.1f}°C"
        except Exception:
            return ""

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
                f"{self.reference_resets}{recoveries}{self.chip_temperature()}")
        else:
            logger.warning(f"📊 {elapsed:.1f}s: no frames received{self.chip_temperature()}")
        self.reset()


def log_device_info(device):
    info = {}
    for label, getter in (("name", "getDeviceName"), ("IMU", "getConnectedIMU"), ("USB", "getUsbSpeed")):
        try:
            info[label] = getattr(device, getter)()
        except Exception as e:
            info[label] = f"unavailable ({e})"
    logger.info(f"Device: {info['name']}, IMU: {info['IMU']}, USB speed: {info['USB']}")
    return {label: str(value) for label, value in info.items()}


def load_calibration(device, args):
    """Raw calibration intrinsics (cam_b unless --intrinsics-from cam_c) and the IMU -> CAM_B rotation."""
    calib = device.readCalibration()
    socket = args.intrinsics_from if args.intrinsics_from in INTRINSICS_SOCKETS else "cam_b"
    K = calib.getCameraIntrinsics(INTRINSICS_SOCKETS[socket], args.width, args.height)
    intrinsics = Intrinsics.from_matrix(K)
    logger.info(f"Calibration intrinsics ({socket} @ {args.width}x{args.height}): "
                f"fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f} cx={intrinsics.cx:.1f} cy={intrinsics.cy:.1f}")

    R_cam_imu = None
    if args.gyro_gate > 0:
        try:
            extrinsics = np.array(calib.getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_B, True))
            R_cam_imu = extrinsics[:3, :3]
            logger.info(f"IMU -> CAM_B rotation:\n{np.array2string(R_cam_imu, precision=3)}")
        except Exception as e:
            logger.warning(f"No IMU-to-camera extrinsics ({e}); gyro prior disabled")
    return intrinsics, R_cam_imu


def select_intrinsics(args, calibration_intrinsics, depth_msg):
    """
    (intrinsics, source, problem). With --intrinsics-from frame the camera matrix comes from the depth
    frame's transformation: the rectified-left grid the features and depth are on, including alpha
    scaling. Falls back to the calibration (problem says why) if the frame carries no usable matrix.
    """
    if args.intrinsics_from != "frame":
        return calibration_intrinsics, args.intrinsics_from, None
    try:
        transformation = depth_msg.getTransformation()
        intrinsics = intrinsics_from_frame(transformation.getIntrinsicMatrix(), transformation.getSize(),
                                           (args.width, args.height))
        problem = None if intrinsics else f"implausible depth frame camera matrix for {args.width}x{args.height}"
    except Exception as e:
        intrinsics, problem = None, f"no depth frame camera matrix ({e})"
    if intrinsics is None:
        return calibration_intrinsics, "cam_b (fallback)", problem
    return intrinsics, "frame", None


def configure_feature_tracker(config, args, maintainer=True):
    config.setCornerDetector(CORNER_DETECTORS[args.corner_detector])
    config.setNumTargetFeatures(args.num_features)
    config.setMotionEstimator(True)
    config.setOpticalFlow()  # Lucas-Kanade; keeps feature ids across frames
    config.setFeatureMaintainer(maintainer)  # default minimum distance between features (50)
    return config


def snapshot_script(available):
    """Device-side gate: forward one frame of the requested stream per request message."""
    return "\n".join([
        f"streams = {list(snapshots.STREAMS)!r}",
        f"available = {sorted(available)!r}",
        "while True:",
        "    index = node.inputs['request'].get().getSequenceNum()",
        "    if 0 <= index < len(streams) and streams[index] in available:",
        "        node.outputs['snapshot'].send(node.inputs[streams[index]].get())",
    ])


@dataclass
class PipelineHandles:
    sync_queue: object
    imu_queue: object
    feature_config_queue: object
    stereo_effective: dict
    snapshot_request_queue: Optional[object] = None
    snapshot_queue: Optional[object] = None
    snapshot_streams: tuple = ()
    snapshot_unavailable: dict = field(default_factory=dict)


def build_snapshot_gate(p, device, stereo):
    """
    Script node holding the latest frame of each stream in non-blocking size-1 queues; it forwards a
    single frame per host request, so the snapshot outputs cost no link bandwidth until asked.
    """
    sources = {"rect_left": stereo.rectifiedLeft, "rect_right": stereo.rectifiedRight, "depth": stereo.depth}
    unavailable = {}
    if dai.CameraBoardSocket.CAM_A in device.getConnectedCameras():
        rgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=SNAPSHOT_RGB_FPS)
        sources["rgb"] = rgb.requestOutput(SNAPSHOT_RGB_SIZE, dai.ImgFrame.Type.NV12)
    else:
        unavailable["rgb"] = "this device has no CAM_A (RGB) camera"

    script = p.create(dai.node.Script)
    for name, output in sources.items():
        output.link(script.inputs[name])
        script.inputs[name].setBlocking(False)
        script.inputs[name].setMaxSize(1)
    script.inputs["request"].setBlocking(False)
    script.inputs["request"].setMaxSize(4)
    script.setScript(snapshot_script(sources))
    # One extra frame per stereo output pool for the frame each gate queue holds
    stereo.setNumFramesPool(4)

    request_queue = script.inputs["request"].createInputQueue(maxSize=4, blocking=False)
    snapshot_queue = script.outputs["snapshot"].createOutputQueue(maxSize=2, blocking=False)
    return request_queue, snapshot_queue, tuple(sources), unavailable


def build_pipeline(p, device, args, stereo_settings):
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    feature_tracker = p.create(dai.node.FeatureTracker)
    sync = p.create(dai.node.Sync)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    apply_stereo_config(stereo, stereo_settings, dai)

    feature_tracker.setHardwareResources(1, 2)
    configure_feature_tracker(feature_tracker.initialConfig, args)

    sync.setSyncThreshold(timedelta(seconds=0.5 / args.fps))

    left.requestOutput((args.width, args.height)).link(stereo.left)
    right.requestOutput((args.width, args.height)).link(stereo.right)
    stereo.rectifiedLeft.link(feature_tracker.inputImage)
    feature_tracker.outputFeatures.link(sync.inputs["features"])
    stereo.depth.link(sync.inputs["depth"])

    handles = PipelineHandles(
        # Non-blocking: if the host falls behind, old frames are dropped and the next delta spans the gap
        sync_queue=sync.out.createOutputQueue(maxSize=4, blocking=False),
        imu_queue=imu.out.createOutputQueue(maxSize=200, blocking=False),
        feature_config_queue=feature_tracker.inputConfig.createInputQueue(maxSize=4, blocking=False),
        stereo_effective=effective_stereo_config(stereo, stereo_settings),
    )
    if args.snapshots:
        (handles.snapshot_request_queue, handles.snapshot_queue, handles.snapshot_streams,
         handles.snapshot_unavailable) = build_snapshot_gate(p, device, stereo)
    else:
        handles.snapshot_unavailable = {s: "snapshots are disabled (--no-snapshots)" for s in snapshots.STREAMS}
    return handles


def snapshot_frame(msg, stream):
    return msg.getCvFrame() if stream == "rgb" else msg.getFrame()


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
    print(f"📡 MAVLink: {'disabled (log-only)' if log_only else args.vehicle_ip}")

    data_log = open_vio_log()
    logger.info(f"Data log: {data_log.path}")
    data_log.event("start", algorithm="pnp", args=vars(args), stereo=stereo_settings, log_only=log_only,
                   log_only_forced_by_env=env_forces_log_only())
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
    def log_warning(message):
        logger.warning(message)
        data_log.event("warning", message=message)

    mavlink = setup_autopilot_output(log_only, args.vehicle_ip, AutopilotParams, MavlinkSender,
                                     log=log_warning)

    params = PnPParams(
        min_inliers=args.min_inliers,
        reprojection_error_px=args.reprojection_error,
        gyro_gate_deg=args.gyro_gate if args.gyro_gate > 0 else 180.0,
        max_reference_age_s=args.max_reference_age,
        min_depth_m=args.min_depth,
        max_depth_m=args.max_depth,
    )
    quality_interval = 1.0 / args.quality_rate if args.quality_rate > 0 else float('inf')

    def log_device_open(message):
        logger.info(message)
        data_log.event("device_open", message=message)

    try:
        device = open_oak_device(args.device_timeout, log=log_device_open)
    except DeviceOpenTimeout as e:
        logger.error(f"❌ {e}")
        data_log.event("error", error=str(e))
        if mavlink:
            mavlink.close()
        sys.exit(2)

    writer = None
    try:
        with dai.Pipeline(device) as p:
            handles = build_pipeline(p, device, args, stereo_settings)
            warnings = stereo_warnings(stereo_settings, args.intrinsics_from)
            logger.info(f"Stereo config (effective): {handles.stereo_effective}")
            for message in warnings:
                log_warning(f"Stereo: {message}")
            data_log.event("stereo_config", requested=stereo_settings, effective=handles.stereo_effective,
                           warnings=warnings)
            data_log.event("device", **log_device_info(device))
            calibration_intrinsics, R_cam_imu = load_calibration(device, args)
            data_log.event("calibration",
                           intrinsics=[calibration_intrinsics.fx, calibration_intrinsics.fy,
                                       calibration_intrinsics.cx, calibration_intrinsics.cy],
                           R_cam_imu=R_cam_imu)
            odometry = None  # created on the first frame, once the rectified intrinsics are known
            gyro = GyroBuffer()
            stats = Stats(device)
            tracking = None
            last_quality_time = 0.0
            previous_ids = set()

            recovery = TrackingRecovery(args.recovery_frames)
            reinit = FeatureTrackerReinit(lambda maintainer: handles.feature_config_queue.send(
                configure_feature_tracker(dai.FeatureTrackerConfig(), args, maintainer)))

            def send_snapshot_request(stream):
                request = dai.Buffer()
                request.setSequenceNum(snapshots.STREAMS.index(stream))
                handles.snapshot_request_queue.send(request)

            writer = snapshots.SnapshotWriter(
                args.snapshot_dir, (stereo_settings["stereo_depth_min_mm"], stereo_settings["stereo_depth_max_mm"]),
                log=logger.warning)
            snapshot_service = snapshots.SnapshotService(
                handles.snapshot_streams, send_snapshot_request, writer, args.snapshot_dir,
                unavailable=handles.snapshot_unavailable, log=logger.warning)
            data_log.event("snapshots", streams=list(handles.snapshot_streams),
                           unavailable=handles.snapshot_unavailable)

            p.start()
            print("✅ Pipeline started!")
            if log_only:
                logger.warning(f"⚠️ {LOG_ONLY_BANNER}")
            else:
                print("📡 Sending VIO data via MAVLink...")

            while p.isRunning():
                group = handles.sync_queue.get()
                start = time.perf_counter()
                reinit.step()

                for imu_data in handles.imu_queue.tryGetAll():
                    gyro.add(imu_data)

                features_msg = group["features"]
                depth_msg = group["depth"]
                if features_msg.getSequenceNum() != depth_msg.getSequenceNum():
                    logger.warning(f"Sequence mismatch: features {features_msg.getSequenceNum()} "
                                   f"depth {depth_msg.getSequenceNum()}")

                if odometry is None:
                    intrinsics, source, problem = select_intrinsics(args, calibration_intrinsics, depth_msg)
                    if problem:
                        log_warning(f"{problem}; using the CAM_B calibration, which is wrong if alpha scaling is set")
                    logger.info(f"Intrinsics ({source}): fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f} "
                                f"cx={intrinsics.cx:.1f} cy={intrinsics.cy:.1f}")
                    data_log.event("intrinsics", source=source,
                                   intrinsics=[intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy])
                    odometry = PnPOdometry(intrinsics, params, image_size=(args.width, args.height))

                t = timestamp_seconds(depth_msg, exposure_middle=True)
                tracked = features_msg.trackedFeatures
                ids = np.fromiter((f.id for f in tracked), dtype=np.int64, count=len(tracked))
                uv = np.array([(f.position.x, f.position.y) for f in tracked], dtype=np.float64).reshape(-1, 2)
                depth = depth_msg.getFrame()
                id_list = ids.tolist()
                new_features, tracked_features = feature_id_counts(previous_ids, id_list)
                previous_ids = set(id_list)

                # The reference is held across failed frames, so the prior spans the whole gap
                reference_t = odometry.prev_timestamp
                R_prior = None
                if R_cam_imu is not None and reference_t is not None:
                    R_prior = gyro.rotation(reference_t, t, R_cam_imu)
                gyro.prune((t if reference_t is None else min(t, reference_t)) - GYRO_BUFFER_SECONDS)

                frame = odometry.process(ids, uv, depth, t, R_prior)
                pose = frame.pose

                angle_delta = position_delta = time_delta_us = None
                sent = False
                if pose.ok:
                    # Exactly what goes (or would go) into VISION_POSITION_DELTA
                    angle_delta = camera_to_body_frd(pose.angle_delta, args.camera_angle)
                    position_delta = camera_to_body_frd(pose.position_delta, args.camera_angle)
                    time_delta_us = frame.dt * 1e6
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
                        snapshot_service.on_frame(lambda stream, m=snapshot_msg: snapshot_frame(m, stream),
                                                  timestamp_seconds(snapshot_msg), snapshot_msg.getSequenceNum())
                proc_ms = (time.perf_counter() - start) * 1000.0

                data_log.step(
                    device_timestamp=t,
                    sequence=depth_msg.getSequenceNum(),
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
                )

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
        if writer is not None:
            writer.close()
        if mavlink:
            mavlink.close()


if __name__ == "__main__":
    main()
