FROM debian:trixie-slim

# Install Python and other dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN apt update && apt install -y  --no-install-recommends git cmake build-essential curl make libudev-dev libusb-1.0-0-dev wget && rm -rf /var/lib/apt/lists/*

COPY install_ttyd.sh .

RUN ./install_ttyd.sh
# for building
RUN apt update && apt install -y pkg-config zip libboost-dev python3-dev python3-numpy libssl3 openssl linux-libc-dev libopencv-dev tmux gdb nano && rm -rf /var/lib/apt/lists/*
RUN export PYTHONPATH=/app/depthai-core/build/bindings/python
RUN echo "bataata"
COPY depthai-core /app/depthai-core


RUN cd depthai-core && cmake -S . -B build -DDEPTHAI_BUILD_PYTHON=ON -DDEPTHAI_BASALT_SUPPORT=ON || echo "Failed to configure CMake"
RUN cd depthai-core && ./build/vcpkg/vcpkg install --x-feature=basalt
RUN cd depthai-core && ./build/vcpkg/vcpkg list
RUN cd depthai-core && cmake -S . -B build \
    -DCMAKE_PREFIX_PATH=/app/depthai-core/vcpkg_installed/x64-linux \
    -DDEPTHAI_BUILD_PYTHON=ON \
    -DDEPTHAI_BASALT_SUPPORT=ON || echo "Failed to configure CMake"
RUN cd depthai-core && cmake --build build --parallel 5

# Create and activate virtual environment
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"

COPY requirements.txt /app/requirements.txt
RUN /app/venv/bin/pip install -r /app/requirements.txt

COPY app /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

LABEL version="0.0.1"

ARG IMAGE_NAME=OAKD_BASALT

LABEL permissions='\
{\
  "HostConfig": {\
    "NetworkMode": "host",\
    "Binds":["/dev/:/dev/"],\
    "DeviceCgroupRules": [\
      "c 189:* rmw"\
    ],\
    "Env": ["VEHICLE_IP=127.0.0.1","FPS=10","CAMERA_ANGLE=0.0"],\
  }\
}'

LABEL authors='[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'

LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="device-integration"

LABEL readme='https://raw.githubusercontent.com/williangalvani/Blueos-Oakd-Basalt/{tag}/README.md'
LABEL links='{\
        "source": "https://github.com/williangalvani/Blueos-Oakd-Basalt"\
    }'
LABEL requirements="core >= 1.1"

# Environment variables with defaults
ENV FPS=10
ENV CAMERA_ANGLE=0.0
ENV VEHICLE_IP=127.0.0.1

CMD ["/app/entrypoint.sh"]
