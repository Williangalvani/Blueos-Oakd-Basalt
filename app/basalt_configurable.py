#!/usr/bin/env python3
"""
BlueOS OAK-D Basalt VIO with configurable parameters
"""

import signal
import sys
import time
import argparse
import depthai as dai
from mavlink_node import MavlinkNode
from autopilot_params import AutopilotParams
from device_open import DEFAULT_TIMEOUT, DeviceOpenTimeout, open_oak_device
from vio_log import LOG_ONLY_BANNER, env_forces_log_only, open_vio_log, resolve_log_only


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='BlueOS OAK-D Basalt VIO')
    
    # Camera settings
    parser.add_argument('--fps', type=int, default=3)
    parser.add_argument('--camera-angle', type=float, default=0.0)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=200)
    
    # VIO algorithm settings
    parser.add_argument('--grid-size', type=int, default=80)
    parser.add_argument('--flow-iterations', type=int, default=3)
    parser.add_argument('--flow-levels', type=int, default=2)
    parser.add_argument('--max-states', type=int, default=2)
    parser.add_argument('--max-kfs', type=int, default=4)
    parser.add_argument('--vio-iterations', type=int, default=3)
    parser.add_argument('--skip-frames', type=int, default=1)
    
    # MAVLink settings
    parser.add_argument('--vehicle-ip', default='127.0.0.1')
    parser.add_argument('--quality-rate', type=float, default=1.0)
    parser.add_argument('--device-timeout', type=float, default=DEFAULT_TIMEOUT,
                        help='Seconds to keep waiting for the OAK device to be discoverable and open')
    parser.add_argument('--log-only', action='store_true',
                        help='No MAVLink output and no autopilot parameter changes (also forced by OAKD_LOG_ONLY=1). '
                             'Basalt poses are not written to the data log, only start/device/exit events')
    
    return parser.parse_args()


def _raise_system_exit(signum, frame):
    raise SystemExit(128 + signum)


def main():
    args = parse_args()
    log_only = resolve_log_only(args.log_only)
    signal.signal(signal.SIGTERM, _raise_system_exit)

    data_log = open_vio_log()
    print(f"📝 Data log: {data_log.path}", flush=True)
    data_log.event("start", algorithm="basalt", args=vars(args), log_only=log_only,
                   log_only_forced_by_env=env_forces_log_only())
    exit_code = 0
    try:
        run(args, log_only, data_log)
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


def run(args, log_only, data_log):
    print("🚀 Starting OAK-D Basalt VIO")
    print(f"📷 Camera: {args.width}x{args.height} @ {args.fps} FPS")
    print(f"📐 Camera Angle: {args.camera_angle}°")
    print(f"🔧 VIO: grid={args.grid_size}, iterations={args.flow_iterations}, levels={args.flow_levels}")
    print(f"📡 MAVLink: {'disabled (log-only)' if log_only else args.vehicle_ip}")

    if log_only:
        print(f"⚠️ {LOG_ONLY_BANNER}", flush=True)
        data_log.event("warning", message=LOG_ONLY_BANNER)
    else:
        AutopilotParams(args.vehicle_ip).configure_visual_odometry()

    def log_device_open(message):
        print(message, flush=True)
        data_log.event("device_open", message=message)

    try:
        device = open_oak_device(args.device_timeout, log=log_device_open)
    except DeviceOpenTimeout as e:
        print(f"❌ {e}", flush=True)
        data_log.event("error", error=str(e))
        sys.exit(2)

    with dai.Pipeline(device) as p:
        # Define sources
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=float(args.fps))
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=float(args.fps))
        imu = p.create(dai.node.IMU)
        odom = p.create(dai.node.BasaltVIO)
        
        # Configure VIO with parameters
        odom.vioConfig.optical_flow_detection_grid_size = args.grid_size
        odom.vioConfig.optical_flow_max_iterations = args.flow_iterations
        odom.vioConfig.optical_flow_levels = args.flow_levels
        odom.vioConfig.vio_max_states = args.max_states
        odom.vioConfig.vio_max_kfs = args.max_kfs
        odom.vioConfig.vio_max_iterations = args.vio_iterations
        odom.vioConfig.optical_flow_skip_frames = args.skip_frames
        
        print("⚡ VIO configured")
        
        # Create MAVLink publisher (not at all in log-only mode, so nothing reaches mavlink2rest)
        mavlink_publisher = None
        if not log_only:
            mavlink_url = f"ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
            mavlink_publisher = MavlinkNode(
                mavlink_host=mavlink_url, 
                camera_angle=args.camera_angle, 
                quality_rate=args.quality_rate
            )
        
        # Configure IMU
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        
        # Linking
        left.requestOutput((args.width, args.height)).link(odom.left)
        right.requestOutput((args.width, args.height)).link(odom.right)
        imu.out.link(odom.imu)
        if mavlink_publisher is not None:
            odom.transform.link(mavlink_publisher.inputTrans)
            odom.quality.link(mavlink_publisher.inputQuality)
        
        p.start()
        print("✅ Pipeline started!")
        if log_only:
            print(f"⚠️ {LOG_ONLY_BANNER}", flush=True)
        else:
            print("📡 Sending VIO data via MAVLink...")
        
        while p.isRunning():
            time.sleep(0.1)


if __name__ == "__main__":
    main()
