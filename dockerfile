FROM python:3.12-slim

WORKDIR /app

RUN apt update && apt install -y git cmake build-essential curl make libudev-dev libusb-1.0-0-dev


# RUN git clone --branch develop https://github.com/luxonis/depthai-core
# RUN cd depthai-core && git submodule update --init --recursive

# RUN cd depthai-core && cmake -S . -B build
# RUN cd depthai-core && cmake --build build --parallel $(nproc)
RUN python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ depthai==3.0.0rc2

COPY requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

COPY app /app

LABEL version="0.0.1"

ARG IMAGE_NAME=OAKD_VINS

LABEL permissions='\
{\
  "ExposedPorts": {\
    "8000/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/dev/:/dev/"],\
    "DeviceCgroupRules": [\
      "c 189:* rmw"\
    ],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "8000/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
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

LABEL readme='https://raw.githubusercontent.com/williangalvani/Blueos-oakd-vins/{tag}/README.md'
LABEL links='{\
        "source": "https://github.com/williangalvani/Blueos-oakd-vins"\
    }'
LABEL requirements="core >= 1.1"

CMD ["python", "/app/basalt.py", "--fps", "10", "--vehicle-ip", "docker.host.internal"]