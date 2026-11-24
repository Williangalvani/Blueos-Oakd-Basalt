# =============================================================================
# Builder image: depthai-core with the Python bindings. Build it once and reuse it
# (hours under QEMU); BuildKit picks up dockerfile.builder.dockerignore:
#   git submodule update --init depthai-core
#   docker buildx build --platform linux/arm64 -f dockerfile.builder \
#     -t williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap --push .
#
# The published williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap, which the
# runtime dockerfile uses, was built with Basalt, RTAB-Map and PCL on; PnP uses none
# of them and the runtime copies none of their libraries, so they are off here. Not
# rebuilt since this change.
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

# Copy and build depthai-core
COPY depthai-core /app/depthai-core

RUN --mount=type=cache,target=/root/.cache/vcpkg \
    cd depthai-core && cmake -S . -B build \
    -DDEPTHAI_BUILD_PYTHON=ON \
    -DDEPTHAI_BASALT_SUPPORT=OFF \
    -DDEPTHAI_RTABMAP_SUPPORT=OFF \
    -DDEPTHAI_PCL_SUPPORT=OFF \
    -DDEPTHAI_OPENCV_SUPPORT=ON \
    -DVCPKG_MANIFEST_FEATURES="python-bindings;usb;backward" \
    -DCMAKE_BUILD_TYPE=Release

RUN --mount=type=cache,target=/root/.cache/vcpkg \
    --mount=type=cache,target=/root/.ccache \
    cd depthai-core && cmake --build build --parallel $(nproc)

# What the runtime dockerfile copies:
# - build/bindings/python/depthai.cpython-313-aarch64-linux-gnu.so
# - build/vcpkg_installed/arm64-linux/lib/libusb-1.0.so
# - build/_deps/dynamic_calibration-src/lib/libdynamic_calibration.so
