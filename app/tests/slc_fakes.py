"""
Strict fakes of depthai's SpatialLocationCalculator (SLC) types and computation, shared by test_depth_points.py
and the points-mode runs in test_tracking_recovery.py.

They follow depthai v3.3.0 (SpatialLocationCalculatorImpl / SpatialLocationCalculatorConfig.hpp) and the RVC2
firmware as test16b saw it: the AUTO step (1 for AVERAGE, 2 for MEDIAN: every 2nd pixel in each direction from
the ROI's top-left), only the computed algorithm's depth filled (the other stays 0), MEDIAN / MODE computed as
AVERAGE on a depth frame that does not come straight from StereoDepth (the firmware's "Defaulting to
'AVERAGE/MEAN'"; ours always come through the pairing Script), thresholds exclusive (lower < depth < upper), a
Rect that is normalized when built without the flag and x + w <= 1, y + h <= 1 (Rect.hpp isNormalized), ROIs
copied into the config by value (addROI pushes a copy). The output carries no sequence number or timestamp of
its own: whoever uses slc_compute sets them (or deliberately does not).

over_link is what the camera receives for a host message: a SpatialLocationCalculatorConfig's sequence number
reads 0 there (test16b: every host config), a plain Buffer's survives (the gate controls since test15).
"""
import copy
from datetime import timedelta
from types import SimpleNamespace

import numpy as np


class Point2f:
    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)


class Size2f:
    def __init__(self, width, height):
        self.width, self.height = float(width), float(height)


class Rect:
    """dai.Rect: Rect(x, y, w, h, normalized) or Rect(x, y, w, h) (normalized if it fits in 0..1) or (Point2f, Size2f)."""

    def __init__(self, *args):
        if len(args) == 2 and isinstance(args[0], Point2f):
            args = (args[0].x, args[0].y, args[1].width, args[1].height)
        if len(args) == 4:
            x, y, w, h = (float(a) for a in args)
            normalized = x + w <= 1.0 and y + h <= 1.0
        elif len(args) == 5 and isinstance(args[4], bool):
            x, y, w, h = (float(a) for a in args[:4])
            normalized = args[4]
        else:
            raise TypeError(f"Rect(): incompatible constructor arguments {args!r}")
        self.x, self.y, self.width, self.height, self.normalized = x, y, w, h, normalized


class Thresholds:
    def __init__(self):
        self.lowerThreshold, self.upperThreshold = 0, 65535


class Algorithm:
    AVERAGE, MIN, MAX, MODE, MEDIAN = range(5)


class ConfigData:
    def __init__(self):
        self.roi = Rect(0.0, 0.0, 0.0, 0.0, False)
        self.depthThresholds = Thresholds()
        self.calculationAlgorithm = Algorithm.MEDIAN


class HostBuffer:
    """dai.Buffer on the host: Buffer() (the camera Script's Buffer(size) is the tests' own)."""

    def __init__(self):
        self.sequence = 0

    def setSequenceNum(self, sequence):
        self.sequence = int(sequence)

    def getSequenceNum(self):
        return self.sequence


class Config:
    def __init__(self):
        self.rois, self.sequence = [], 0

    def addROI(self, data):
        self.rois.append(copy.deepcopy(data))  # C++: pushes a copy

    def getConfigData(self):
        return copy.deepcopy(self.rois)

    def setSequenceNum(self, sequence):
        self.sequence = int(sequence)

    def getSequenceNum(self):
        return self.sequence


FAKE_DAI = SimpleNamespace(SpatialLocationCalculatorConfig=Config, SpatialLocationCalculatorConfigData=ConfigData,
                           SpatialLocationCalculatorAlgorithm=Algorithm, Rect=Rect, Buffer=HostBuffer)


def over_link(message):
    """The message as the camera's Script receives it from a host input queue."""
    received = copy.deepcopy(message)
    if isinstance(received, Config):
        received.sequence = 0
    return received


class SpatialData:
    def __init__(self, locations):
        self.locations, self.sequence, self.timestamp = locations, 0, timedelta(0)

    def getSpatialLocations(self):
        return self.locations

    def setSequenceNum(self, sequence):
        self.sequence = sequence

    def getSequenceNum(self):
        return self.sequence

    def setTimestamp(self, timestamp):
        self.timestamp = timestamp

    def getTimestamp(self):
        return self.timestamp


STEPS = {Algorithm.AVERAGE: 1, Algorithm.MEDIAN: 2}


def slc_compute(config, depth, from_stereo=False):
    """
    The SLC on one depth frame: per ROI the echoed rect (pixels), the computed algorithm's depth (AVERAGE or
    MEDIAN, AUTO step), the other 0, and the count of valid samples. from_stereo: the frame comes straight from
    StereoDepth (else MEDIAN is computed as AVERAGE).
    """
    h, w = depth.shape
    locations = []
    for data in config.rois:
        r = data.roi
        x, y, rw, rh = r.x, r.y, r.width, r.height
        if r.normalized:
            x, y, rw, rh = x * w, y * h, rw * w, rh * h
        x, y, rw, rh = int(x), int(y), int(rw), int(rh)
        if x < 0 or y < 0 or rw <= 0 or rh <= 0 or x + rw > w or y + rh > h:
            raise ValueError(f"ROI {x, y, rw, rh} outside the {w}x{h} depth frame")
        algorithm = data.calculationAlgorithm
        if algorithm not in STEPS:
            raise NotImplementedError(f"fake SLC: algorithm {algorithm}")
        if algorithm == Algorithm.MEDIAN and not from_stereo:
            algorithm = Algorithm.AVERAGE
        step = STEPS[algorithm]
        patch = depth[y:y + rh:step, x:x + rw:step].astype(np.float64).ravel()
        lower, upper = data.depthThresholds.lowerThreshold, data.depthThresholds.upperThreshold
        valid = patch[(patch > lower) & (patch < upper)]
        value = (float(np.median(valid)) if algorithm == Algorithm.MEDIAN else float(valid.mean())) \
            if valid.size else 0.0
        locations.append(SimpleNamespace(
            config=SimpleNamespace(roi=SimpleNamespace(x=float(x), y=float(y), width=float(rw), height=float(rh))),
            depthMedian=value if algorithm == Algorithm.MEDIAN else 0.0,
            depthAverage=value if algorithm == Algorithm.AVERAGE else 0.0,
            depthAveragePixelCount=int(valid.size)))
    return SpatialData(locations)


def answer(config, depth, t, sequence=None):
    """The camera's re-tagged output for one config (what the Script sends the Pi)."""
    spatial = slc_compute(config, depth)
    spatial.setSequenceNum(config.getSequenceNum() if sequence is None else sequence)
    spatial.setTimestamp(timedelta(seconds=t))
    return spatial
