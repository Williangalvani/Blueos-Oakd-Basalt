# =============================================================================
# Runtime image: the depthai Python module and the two libraries it links, copied
# out of a prebuilt builder image (dockerfile.builder), plus the app.
#
#   docker buildx build --platform linux/arm64 \
#     --build-arg BUILDER_IMAGE=williangalvani/blueos-oakd-basalt-builder:arm64-rtabmap \
#     -t williangalvani/blueos-oakd-basalt:arm64-rtabmap .
# =============================================================================
ARG BUILDER_IMAGE=blueos-oakd-basalt-builder

FROM ${BUILDER_IMAGE} AS builder

# =============================================================================
# Runtime stage - minimal image
# =============================================================================
FROM debian:trixie-slim

WORKDIR /app

# depthai.so NEEDS libgomp, libusb-1.0.so and libdynamic_calibration.so (both copied below) and libc/libstdc++.
# Its embedded TBB dlopens libtbbmalloc.so.2 when present; the hardware-tested images had it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-venv \
    libusb-1.0-0 libudev1 \
    libgomp1 libtbbmalloc2 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/depthai-core/build/bindings/python/depthai.cpython-313-aarch64-linux-gnu.so /app/depthai-python/
COPY --from=builder /app/depthai-core/build/vcpkg_installed/arm64-linux/lib/libusb-1.0.so /usr/local/lib/
COPY --from=builder /app/depthai-core/build/_deps/dynamic_calibration-src/lib/libdynamic_calibration.so /usr/local/lib/
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
LABEL version="0.1.0"
LABEL permissions='{\
  "ExposedPorts": {"8095/tcp": {}},\
  "HostConfig": {\
    "NetworkMode": "host",\
    "Binds":["/dev/:/dev/", "/usr/blueos/extensions/oakd-vio:/app/data"],\
    "DeviceCgroupRules": [\
      "c 189:* rmw"\
    ]\
  }\
}'
LABEL authors='[{"name": "Willian Galvani", "email": "willian@bluerobotics.com"}]'
LABEL company='{"about": "", "name": "Blue Robotics", "email": "support@bluerobotics.com"}'
LABEL type="device-integration"
LABEL webpage='http://\{\{HOST\}\}:8095'
LABEL readme='https://raw.githubusercontent.com/williangalvani/Blueos-Oakd-Basalt/{tag}/README.md'
LABEL links='{"source": "https://github.com/williangalvani/Blueos-Oakd-Basalt"}'
LABEL requirements="core >= 1.1"

CMD ["/app/entrypoint.sh"]
