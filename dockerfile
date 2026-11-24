# =============================================================================
# Runtime image: Debian, depthai 3.3.0 from PyPI (the official manylinux aarch64
# wheel, which bundles libdepthaicore, libusb, libgomp, FFmpeg and
# libdynamic_calibration) and the app.
#
#   docker buildx build --platform linux/arm64 \
#     -t williangalvani/blueos-oakd-basalt:arm64-rtabmap .
# =============================================================================
FROM debian:trixie-slim

WORKDIR /app

# The depthai wheel needs only libc, libstdc++ and libgcc_s from the system, all in trixie-slim.
# Its embedded TBB dlopens libtbbmalloc.so.2 when present; the hardware-tested images had it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-venv \
    libtbbmalloc2 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Create virtual environment and install Python deps
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"

# The depthai wheel carries its extension module for every CPython from 3.9 to 3.14 (about 10 MB each);
# only this Python's is kept, and ldd checks that everything it links resolves. Not `python -c "import
# depthai"`: depthai's firmware unpack threads can abort a process that exits right after the import.
COPY requirements.txt /app/requirements.txt
RUN /app/venv/bin/pip install --no-cache-dir -r /app/requirements.txt \
    && so=$(/app/venv/bin/python -c "import sysconfig; print(sysconfig.get_path('platlib') + '/depthai' + sysconfig.get_config_var('EXT_SUFFIX'))") \
    && find "${so%/*}" -maxdepth 1 -name 'depthai.cpython-*.so' ! -path "$so" -delete \
    && ldd "$so" > /tmp/depthai-ldd \
    && ! grep 'not found' /tmp/depthai-ldd \
    && rm /tmp/depthai-ldd

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
