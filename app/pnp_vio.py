#!/usr/bin/env python3
"""
BlueOS OAK-D PnP VIO (option D)

All pixel work runs on the OAK-D: stereo depth and LK feature tracking on the rectified left image.
The host only looks up depth at the tracked features and solves frame-to-frame PnP with a gyro
rotation prior (pnp_odometry.py), then sends VISION_POSITION_DELTA over MAVLink.
"""

import argparse
import time
from collections import Counter, deque
from datetime import timedelta

import depthai as dai
import numpy as np
from loguru import logger

from autopilot_params import AutopilotParams
from mavlink_sender import MavlinkSender
from pnp_odometry import Intrinsics, PnPOdometry, PnPParams, camera_to_body_frd, integrate_gyro

SUMMARY_INTERVAL = 10.0
GYRO_BUFFER_SECONDS = 2.0

CORNER_DETECTORS = {
    "harris": dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS,
    "shi-tomasi": dai.FeatureTrackerConfig.CornerDetector.Type.SHI_THOMASI,
}

INTRINSICS_SOCKETS = {
    "cam_b": dai.CameraBoardSocket.CAM_B,
    "cam_c": dai.CameraBoardSocket.CAM_C,
}


def parse_args():
    parser = argparse.ArgumentParser(description='BlueOS OAK-D PnP VIO')

    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--camera-angle', type=float, default=0.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=400)

    parser.add_argument('--num-features', type=int, default=300,
                        help='Target number of features tracked on the camera')
    parser.add_argument('--corner-detector', choices=sorted(CORNER_DETECTORS), default='harris')
    parser.add_argument('--intrinsics-from', choices=sorted(INTRINSICS_SOCKETS), default='cam_b',
                        help='Calibration used as the rectified-left camera matrix (see NOTES_OPTION_D.md)')

    parser.add_argument('--min-inliers', type=int, default=15)
    parser.add_argument('--reprojection-error', type=float, default=2.0, help='RANSAC threshold (px)')
    parser.add_argument('--gyro-gate', type=float, default=5.0,
                        help='Reject frames whose PnP rotation differs from the gyro by more than this (deg); 0 disables the gyro prior')
    parser.add_argument('--min-depth', type=float, default=0.2, help='m')
    parser.add_argument('--max-depth', type=float, default=15.0, help='m')

    parser.add_argument('--vehicle-ip', default='127.0.0.1')
    parser.add_argument('--quality-rate', type=float, default=1.0)

    return parser.parse_args()


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
        self.samples = deque()

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

    def add(self, frame, proc_ms):
        self.frames += 1
        self.proc_ms.append(proc_ms)
        self.features.append(frame.features)
        if frame.pose.ok:
            self.ok += 1
            self.inliers.append(frame.pose.inliers)
        else:
            self.reasons[frame.pose.reason] += 1

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
            logger.info(
                f"📊 {elapsed:.1f}s: {self.frames / elapsed:.1f} fps, ok {self.ok}/{self.frames}, "
                f"proc mean {np.mean(self.proc_ms):.1f} ms max {np.max(self.proc_ms):.1f} ms, "
                f"mean features {np.mean(self.features):.0f}, mean inliers {mean_inliers:.0f}, "
                f"failures {dict(self.reasons)}{self.chip_temperature()}")
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


def load_calibration(device, args):
    calib = device.readCalibration()
    K = calib.getCameraIntrinsics(INTRINSICS_SOCKETS[args.intrinsics_from], args.width, args.height)
    intrinsics = Intrinsics.from_matrix(K)
    logger.info(f"Intrinsics ({args.intrinsics_from} @ {args.width}x{args.height}): "
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


def build_pipeline(p, args):
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
    imu = p.create(dai.node.IMU)
    stereo = p.create(dai.node.StereoDepth)
    feature_tracker = p.create(dai.node.FeatureTracker)
    sync = p.create(dai.node.Sync)

    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    # Depth on the same pixel grid as stereo.rectifiedLeft, which the feature tracker runs on
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

    feature_tracker.setHardwareResources(1, 2)
    config = feature_tracker.initialConfig
    config.setCornerDetector(CORNER_DETECTORS[args.corner_detector])
    config.setNumTargetFeatures(args.num_features)
    config.setMotionEstimator(True)
    config.setOpticalFlow()  # Lucas-Kanade; keeps feature ids across frames
    config.setFeatureMaintainer(True)  # default minimum distance between features (50)

    sync.setSyncThreshold(timedelta(seconds=0.5 / args.fps))

    left.requestOutput((args.width, args.height)).link(stereo.left)
    right.requestOutput((args.width, args.height)).link(stereo.right)
    stereo.rectifiedLeft.link(feature_tracker.inputImage)
    feature_tracker.outputFeatures.link(sync.inputs["features"])
    stereo.depth.link(sync.inputs["depth"])

    # Non-blocking: if the host falls behind, old frames are dropped and the next delta spans the gap
    sync_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)
    imu_queue = imu.out.createOutputQueue(maxSize=200, blocking=False)
    return sync_queue, imu_queue


def main():
    args = parse_args()

    print("🚀 Starting OAK-D PnP VIO")
    print(f"📷 Camera: {args.width}x{args.height} @ {args.fps} FPS")
    print(f"📐 Camera Angle: {args.camera_angle}°")
    print(f"🔧 Features on camera: {args.num_features} ({args.corner_detector}), "
          f"min inliers {args.min_inliers}, gyro gate {args.gyro_gate}°")
    print(f"📡 MAVLink: {args.vehicle_ip}")

    AutopilotParams(args.vehicle_ip).configure_visual_odometry()

    mavlink = MavlinkSender(f"ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY")
    if not mavlink.connect():
        logger.warning("MAVLink websocket not connected yet; will keep retrying")

    params = PnPParams(
        min_inliers=args.min_inliers,
        reprojection_error_px=args.reprojection_error,
        gyro_gate_deg=args.gyro_gate if args.gyro_gate > 0 else 180.0,
        min_depth_m=args.min_depth,
        max_depth_m=args.max_depth,
    )
    quality_interval = 1.0 / args.quality_rate if args.quality_rate > 0 else float('inf')

    with dai.Pipeline() as p:
        sync_queue, imu_queue = build_pipeline(p, args)
        device = p.getDefaultDevice()
        log_device_info(device)
        intrinsics, R_cam_imu = load_calibration(device, args)
        odometry = PnPOdometry(intrinsics, params)
        gyro = GyroBuffer()
        stats = Stats(device)
        tracking = None
        last_quality_time = 0.0

        p.start()
        print("✅ Pipeline started!")
        print("📡 Sending VIO data via MAVLink...")

        while p.isRunning():
            group = sync_queue.get()
            start = time.perf_counter()

            for imu_data in imu_queue.tryGetAll():
                gyro.add(imu_data)

            features_msg = group["features"]
            depth_msg = group["depth"]
            if features_msg.getSequenceNum() != depth_msg.getSequenceNum():
                logger.warning(f"Sequence mismatch: features {features_msg.getSequenceNum()} "
                               f"depth {depth_msg.getSequenceNum()}")

            t = timestamp_seconds(depth_msg, exposure_middle=True)
            tracked = features_msg.trackedFeatures
            ids = np.fromiter((f.id for f in tracked), dtype=np.int64, count=len(tracked))
            uv = np.array([(f.position.x, f.position.y) for f in tracked], dtype=np.float64).reshape(-1, 2)
            depth = depth_msg.getFrame()

            R_prior = None
            if R_cam_imu is not None and odometry.prev_timestamp is not None:
                R_prior = gyro.rotation(odometry.prev_timestamp, t, R_cam_imu)
            gyro.prune(t - GYRO_BUFFER_SECONDS)

            frame = odometry.process(ids, uv, depth, t, R_prior)
            pose = frame.pose

            if pose.ok:
                mavlink.send_vision_position_delta(
                    angle_delta=camera_to_body_frd(pose.angle_delta, args.camera_angle),
                    position_delta=camera_to_body_frd(pose.position_delta, args.camera_angle),
                    time_delta_us=frame.dt * 1e6,
                    confidence=pose.confidence,
                )

            now_tracking = pose.ok
            if pose.reason == "no_reference":
                now_tracking = tracking  # first frame: nothing to say yet
            proc_ms = (time.perf_counter() - start) * 1000.0

            now = time.monotonic()
            if now_tracking is not None and now_tracking != tracking:
                mavlink.send_named_value_float("VIO_TRACK", 1.0 if now_tracking else 0.0)
                if not now_tracking:
                    logger.warning(f"Tracking lost: {pose.reason} (matches {pose.matches}, inliers {pose.inliers})")
                else:
                    logger.info("Tracking (re)established")
                tracking = now_tracking
            if now - last_quality_time >= quality_interval:
                last_quality_time = now
                mavlink.send_named_value_float("VIO_TRACK", 1.0 if tracking else 0.0)
                mavlink.send_named_value_float("VIO_INLIER", pose.inliers)
                mavlink.send_named_value_float("VIO_FEAT", frame.features)
                mavlink.send_named_value_float("VIO_CONF", pose.confidence)
                mavlink.send_named_value_float("VIO_PROCMS", proc_ms)

            logger.debug(
                f"frame t={t:.3f} dt={frame.dt if frame.dt is not None else float('nan'):.3f}s "
                f"feat={frame.features} depth={frame.valid_depth} match={pose.matches} inl={pose.inliers} "
                f"gyro_diff={pose.gyro_disagreement_deg:.2f}° {'ok' if pose.ok else pose.reason} "
                f"proc={proc_ms:.1f}ms")
            stats.add(frame, proc_ms)
            stats.maybe_report()

    mavlink.close()


if __name__ == "__main__":
    main()
