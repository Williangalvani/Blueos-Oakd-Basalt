# =============================================================================
# Builder image - build this ONCE and reuse: 
#   docker build -f dockerfile.builder -t blueos-oakd-basalt-builder .
# =============================================================================
FROM debian@sha256:a99cfc517144bc59b1978475ec53b46ecabec7e43635402ee5b77cc54cd1b20a

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential curl make wget \
    pkg-config zip gfortran \
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
    -DDEPTHAI_RTABMAP_SUPPORT=ON \
    -DDEPTHAI_PCL_SUPPORT=ON \
    -DDEPTHAI_OPENCV_SUPPORT=ON \
    -DVCPKG_MANIFEST_FEATURES="basalt;rtabmap;pcl;python-bindings;usb;backward" \
    -DCMAKE_BUILD_TYPE=Release

RUN --mount=type=cache,target=/root/.cache/vcpkg \
    --mount=type=cache,target=/root/.ccache \
    cd depthai-core && cmake --build build --parallel $(nproc)

# The built artifacts are at:
# - /app/depthai-core/build/bindings/python (Python bindings)
# - /app/depthai-core/build/libdepthai*.so* (shared libs)
