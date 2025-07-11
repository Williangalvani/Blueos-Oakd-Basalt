#!/usr/bin/env python3
"""
BlueOS OAK-D Basalt VIO with MAVLink output

Publishes the following MAVLink messages:
- ATT_POS_MOCAP: Motion capture attitude and position data (world frame)
- VISION_SPEED_ESTIMATE: Vision-based velocity estimates (body frame)

Usage examples:
  python basalt.py                                    # Local BlueOS (127.0.0.1:6040)
  python basalt.py --vehicle-ip 192.168.1.100        # Remote BlueOS on LAN
  python basalt.py --ip blueos.local                  # Using hostname
  python basalt.py --ip 192.168.1.100 --port 6041    # Custom port
  python basalt.py --fps 30 --camera-angle 45        # Custom FPS and camera angle
"""

import signal
import time
import argparse
import depthai as dai
from mavlink_node import MavlinkNode

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='BlueOS OAK-D Basalt VIO with MAVLink output')
    parser.add_argument('--vehicle-ip', '--ip', 
                       default='127.0.0.1',
                       help='IP address of the vehicle/BlueOS instance (default: 127.0.0.1)')
    parser.add_argument('--fps',
                       type=int,
                       default=60,
                       help='FPS of the camera (default: 60)')
    parser.add_argument('--camera-angle',
                       type=float,
                       default=0.0,
                       help='Camera angle in degrees (0 = forward, 45 = 45° down, etc.)')
    return parser.parse_args()

# Parse command line arguments
args = parse_args()

print(f"🚀 Starting BlueOS OAK-D Basalt VIO")
print(f"📡 Vehicle IP: {args.vehicle_ip}")
print(f"📷 Camera FPS: {args.fps}")
print(f"📐 Camera Angle: {args.camera_angle}°")

print(f"🌐 WebSocket URL: ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY")

# Create pipeline

with dai.Pipeline() as p:
    fps = float(args.fps)
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)

    # Create MAVLink publisher instead of RerunNode
    mavlink_url = f"ws://{args.vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY"
    mavlink_publisher = MavlinkNode(mavlink_host=mavlink_url, camera_angle=args.camera_angle)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
    imu.out.link(odom.imu)
    # Link outputs to MAVLink publisher instead of rerun viewer
    odom.transform.link(mavlink_publisher.inputTrans)
    # odom.passthrough.link(mavlink_publisher.inputImg)
    # odom.quality.link(mavlink_publisher.inputQuality)
    quality = odom.quality.createOutputQueue(maxSize=8, blocking=False)
    p.start()
    while p.isRunning():
        quality_data = quality.tryGet()
        if quality_data is not None:
            print(f"Timestamp: {quality_data.getTimestamp()}")
            print(f"Tracking Status: {'TRACKING' if quality_data.isTracking else 'LOST'}")
            print(f"Active 3D Points: {quality_data.numActivePoints}")
            print(f"Landmarks in Map: {quality_data.numLandmarks}")
            print(f"Total Observations: {quality_data.numObservations}")
            print(f"Keyframes: {quality_data.numKeyframes}")
            print(f"States: {quality_data.numStates}")
            print(f"Avg Tracking Quality: {quality_data.avgTrackingQuality:.2%}")
            print(f"Processing Time: {quality_data.processingTimeMs:.1f}ms")
            
            if quality_data.numTrackedFeatures:
                print(f"Tracked Features per Camera: {quality_data.numTrackedFeatures}")
                total_features = sum(quality_data.numTrackedFeatures)
                print(f"Total Tracked Features: {total_features}")
            
            print("-" * 50)
        time.sleep(0.01)