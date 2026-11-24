#!/bin/bash

# Function to handle shutdown
cleanup() {
    echo "Shutting down services..."
    pkill -f "uvicorn"
    pkill -f "pnp_vio"
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

export PYTHONPATH=/app/depthai-python

echo "🚀 Starting OAK-D VIO Extension"
echo "📋 Web UI available at http://localhost:8095"

# Start the FastAPI web server
cd /app
/app/venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 8095
