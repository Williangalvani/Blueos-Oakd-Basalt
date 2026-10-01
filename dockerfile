# =============================================================================
# STAGE 1: Builder - compile depthai with basalt support
# =============================================================================
FROM debian:trixie-slim AS builder

WORKDIR /app

# Install build dependencies in one layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential curl make wget \
    pkg-config zip \
    libudev-dev libusb-1.0-0-dev \
    libboost-dev python3-dev python3-numpy \
    libssl3 openssl linux-libc-dev libopencv-dev \
    python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Copy and build depthai-core with basalt support
COPY depthai-core /app/depthai-core

RUN --mount=type=cache,target=/root/.cache/vcpkg \
    cd depthai-core && cmake -S . -B build \
    -DDEPTHAI_BUILD_PYTHON=ON \
    -DDEPTHAI_BASALT_SUPPORT=ON \
    -DDEPTHAI_OPENCV_SUPPORT=ON \
    -DVCPKG_MANIFEST_FEATURES="basalt;python-bindings;usb;backward" \
    -DCMAKE_BUILD_TYPE=Release

RUN --mount=type=cache,target=/root/.cache/vcpkg \
    --mount=type=cache,target=/root/.ccache \
    cd depthai-core && cmake --build build --parallel $(nproc)

# =============================================================================
# STAGE 2: Runtime - minimal image with only runtime dependencies
# =============================================================================
FROM debian:trixie-slim

WORKDIR /app

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-venv \
    libusb-1.0-0 libudev1 \
    libopencv-core406t64 libopencv-imgproc406t64 libopencv-imgcodecs406t64 \
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
