#!/bin/bash

# Entrypoint script for BlueOS OAK-D Basalt VIO
# Reads environment variables and passes them as command line arguments

# Default values
DEFAULT_FPS=10
DEFAULT_CAMERA_ANGLE=0.0
DEFAULT_VEHICLE_IP="docker.host.internal"

# Read environment variables with defaults
FPS=${FPS:-$DEFAULT_FPS}
CAMERA_ANGLE=${CAMERA_ANGLE:-$DEFAULT_CAMERA_ANGLE}
VEHICLE_IP=${VEHICLE_IP:-$DEFAULT_VEHICLE_IP}

echo "🚀 Starting BlueOS OAK-D Basalt VIO with environment variables:"
echo "📷 FPS: $FPS"
echo "📐 Camera Angle: $CAMERA_ANGLE°"
echo "📡 Vehicle IP: $VEHICLE_IP"

# Execute the Python script with the environment variables as arguments
exec python /app/basalt.py \
    --fps "$FPS" \
    --camera-angle "$CAMERA_ANGLE" \
    --vehicle-ip "$VEHICLE_IP" 