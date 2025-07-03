FROM python:3.12-slim

WORKDIR /app

RUN apt update && apt install -y  --no-install-recommends git cmake build-essential curl make libudev-dev libusb-1.0-0-dev wget && rm -rf /var/lib/apt/lists/*

COPY install_ttyd.sh .

RUN ./install_ttyd.sh


# Install Python dependencies first (needed for building)
RUN pip install numpy==2.3.1

# for bulding
RUN apt update && apt install -y pkg-config zip libboost-dev libopencv-dev && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive --branch metrics https://github.com/williangalvani/depthai-core

RUN cd depthai-core && cmake -S . -B build -DDEPTHAI_BUILD_PYTHON=ON
RUN cd depthai-core && cmake --build build --parallel $(nproc)


COPY requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

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

ENTRYPOINT ["/app/entrypoint.sh"]