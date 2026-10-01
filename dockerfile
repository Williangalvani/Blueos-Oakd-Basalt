# =============================================================================
# Runtime image - uses pre-built builder image for fast iteration
# 
# First, build the builder image ONCE (takes ~1hr):
#   docker build -f dockerfile.builder -t blueos-oakd-basalt-builder .
#
# Then build runtime image quickly:
#   docker build -t blueos-oakd-basalt .
# =============================================================================
ARG BUILDER_IMAGE=blueos-oakd-basalt-builder

FROM ${BUILDER_IMAGE} AS builder

# =============================================================================
# Runtime stage - minimal image
# =============================================================================
FROM debian:trixie-slim

WORKDIR /app

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-venv \
    libusb-1.0-0 libudev1 \
    libopencv-core410 libopencv-imgproc410 libopencv-imgcodecs410 \
    libboost-filesystem1.83.0 libboost-program-options1.83.0 \
    tmux \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install ttyd
COPY install_ttyd.sh .
RUN ./install_ttyd.sh && rm install_ttyd.sh

# Copy built depthai Python bindings from builder
COPY --from=builder /app/depthai-core/build/bindings/python /app/depthai-python

# Copy any required shared libraries that were built
COPY --from=builder /app/depthai-core/build/libdepthai*.so* /usr/local/lib/
RUN ldconfig

# Create virtual environment and install Python deps
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"
ENV PYTHONPATH="/app/depthai-python"

COPY requirements.txt /app/requirements.txt
RUN /app/venv/bin/pip install --no-cache-dir -r /app/requirements.txt

# Copy application code
COPY app /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Labels
LABEL version="0.0.1"
LABEL permissions='{\
  "HostConfig": {\
    "NetworkMode": "host",\
    "Binds":["/dev/:/dev/"],\
    "DeviceCgroupRules": [\
      "c 189:* rmw"\
    ],\
    "Env": ["VEHICLE_IP=127.0.0.1","FPS=10","CAMERA_ANGLE=0.0"]\
  }\
}'
LABEL authors='[{"name": "Willian Galvani", "email": "willian@bluerobotics.com"}]'
LABEL company='{"about": "", "name": "Blue Robotics", "email": "support@bluerobotics.com"}'
LABEL type="device-integration"
LABEL readme='https://raw.githubusercontent.com/williangalvani/Blueos-Oakd-Basalt/{tag}/README.md'
LABEL links='{"source": "https://github.com/williangalvani/Blueos-Oakd-Basalt"}'
LABEL requirements="core >= 1.1"

# Environment variables
ENV FPS=10
ENV CAMERA_ANGLE=0.0
ENV VEHICLE_IP=127.0.0.1

CMD ["/app/entrypoint.sh"]
