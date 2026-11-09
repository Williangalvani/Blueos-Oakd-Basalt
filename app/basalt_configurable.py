#!/usr/bin/env python3
"""
BlueOS OAK-D Basalt VIO with configurable parameters
"""

import signal
import time
import argparse
import depthai as dai
from mavlink_node import MavlinkNode
from autopilot_params import AutopilotParams


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
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    print(f"🚀 Starting OAK-D Basalt VIO")
    print(f"📷 Camera: {args.width}x{args.height} @ {args.fps} FPS")
    print(f"📐 Camera Angle: {args.camera_angle}°")
    print(f"🔧 VIO: grid={args.grid_size}, iterations={args.flow_iterations}, levels={args.flow_levels}")
    print(f"📡 MAVLink: {args.vehicle_ip}")

    AutopilotParams(args.vehicle_ip).configure_visual_odometry()
    
    with dai.Pipeline() as p:
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
        
        print(f"⚡ VIO configured")
        
        # Create MAVLink publisher
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
        odom.transform.link(mavlink_publisher.inputTrans)
        odom.quality.link(mavlink_publisher.inputQuality)
        
        p.start()
        print("✅ Pipeline started!")
        print("📡 Sending VIO data via MAVLink...")
        
        while p.isRunning():
            time.sleep(0.1)


if __name__ == "__main__":
    main()
