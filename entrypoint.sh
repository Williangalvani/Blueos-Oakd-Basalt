#!/bin/bash

export PYTHONPATH=/app/depthai-python

echo "🚀 Starting OAK-D VIO Extension"
echo "📋 Web UI available at http://localhost:8095"

# exec: uvicorn gets SIGTERM from docker stop itself; its shutdown stops the VIO process (main.lifespan)
cd /app
exec /app/venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 8095
