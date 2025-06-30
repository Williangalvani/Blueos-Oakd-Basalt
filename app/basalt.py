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
                       default=60,
                       help='FPS of the camera (default: 60)')
    return parser.parse_args()

# Parse command line arguments
args = parse_args()

print(f"🚀 Starting BlueOS OAK-D Basalt VIO")
print(f"📡 Vehicle IP: {args.vehicle_ip}")

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
    mavlink_publisher = MavlinkNode(mavlink_host=mavlink_url)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Linking
    left.requestOutput((width, height)).link(odom.left)
    right.requestOutput((width, height)).link(odom.right)
    imu.out.link(odom.imu)
    # Link outputs to MAVLink publisher instead of rerun viewer
    # odom.passthrough.link(mavlink_publisher.inputImg)
    odom.transform.link(mavlink_publisher.inputTrans)
    p.start()
    while p.isRunning():
        time.sleep(0.01)