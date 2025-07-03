#!/bin/bash

# Function to handle shutdown
cleanup() {
    echo "Shutting down services..."
    pkill nginx
    pkill ttyd
    tmux kill-server
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

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

# Kill any existing tmux session with the same name
tmux kill-session -t basalt-vio 2>/dev/null || true

# Create a new tmux session named "basalt-vio" in detached mode
tmux new-session -d -s basalt-vio

# Send the command to run the Python script in the tmux session
tmux send-keys -t basalt-vio:0.0 "cd /app && python basalt.py --fps $FPS --camera-angle $CAMERA_ANGLE --vehicle-ip $VEHICLE_IP" C-m

# Start nginx in the background
nginx -c /etc/nginx/nginx.conf &
nginx_pid=$!

# Start ttyd server on port 7681 (nginx will proxy from 8000 to 7681)
ttyd -p 7681 tmux attach-session -t basalt-vio &
ttyd_pid=$!

echo "✅ Basalt VIO started in tmux session 'basalt-vio'"
echo "📋 Access the terminal at: http://localhost:8000"
echo "📋 To detach from session: Ctrl+B, then D"

# Wait for both processes
wait $nginx_pid $ttyd_pid 