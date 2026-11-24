"""
Camera link bandwidth: what the camera sends the host (down) and the host sends the camera (up), measured
from depthai's XLink counters, and an estimate per host output queue. depthai-free so it can be unit tested.

Measured: device.getProfilingData() (depthai v3.3.0) returns the link's cumulative numBytesRead (camera ->
host) and numBytesWritten (host -> camera). It is a host-local read of XLink's per-link counters (no RPC),
which XLink updates on every packet read or written, with or without DEPTHAI_PROFILING. They count the
packet payloads of every XLink stream (output and input queues, RPC, logs, timesync, watchdog): per message
its data, serialized metadata and a TRAILER_BYTES trailer (StreamMessageParser::serializeMetadata: datatype 4 B,
metadata size 4 B, a 16-byte end marker). XLink's own event headers and the USB/TCP framing are
not counted, so the wire carries a little more. A new connection (camera reconnect) starts at 0.

LinkMonitor.sample() runs on the telemetry publisher thread about once a second (never on the VIO loop):
rates between consecutive samples, the mean over WINDOW_S and the peak 1-s rate over PEAK_WINDOW_S. It
keeps at most PEAK_WINDOW_S / SAMPLE_INTERVAL_S + 1 samples.

Estimated: StreamCounter.add() on the VIO loop (a lock and two additions per message) counts the messages
and bytes the host takes from each output queue: an ImgFrame's len(getData()), an EncodedFrame's JPEG, a
Buffer's data, each plus a header estimate; TrackedFeatures and IMU packets from a per-item estimate. The
estimates follow the libnop encoding of the depthai v3.3.0 structs (a struct 2 B, a float 5 B, a bool 1 B,
unsigned integers 1 B below 128, 2 below 256, 3 below 65536, else 5, std::array<uint8_t, 32> 34 B, Timestamp
about 12 B):
- TrackedFeature (position x, y and its two normalized flags, id, age, harrisScore, trackingError, the 32-byte
  descriptor): 60 B + the id's width + the age's (features_bytes takes the real ids; the age is taken as 1 B,
  under 128 frames). The descriptor is always serialized whole (34 B of the ~64) and RVC2's FeatureTracker
  leaves it zero; depthai has no setting to leave it or any other field out, so the only way to a smaller
  message is fewer features (num_features, max_features);
- IMUPacket (accelerometer, gyroscope, magnetic field, rotation vector; the last two empty here): about 156 B;
- ImgFrame metadata (timestamps, Specs x2, CameraSettings, ImgTransformation with four 3x3 matrices and 14
  distortion coefficients): about 500 B; EncodedFrame metadata about 450 B (each + the trailer).
UP_STREAMS are what the host sends the camera (input queues), summed separately (streams_up_sum_mbps).
Messages a non-blocking host queue drops before the VIO reads them crossed the link but are not counted;
with the protocol traffic (RPC, logs, timesync) they make up the measured total minus the sum.
"""

import threading
import time
from collections import deque, namedtuple

import numpy as np

SOURCE = "xlink profiling"
SAMPLE_INTERVAL_S = 1.0
WINDOW_S = 10.0
PEAK_WINDOW_S = 60.0
REFERENCE_MBPS = 100.0
STUCK_AT_ZERO_SAMPLES = 3
_EPS = 1e-6

# Bytes per message on top of its data: serialized metadata + the trailer (see the module docstring)
TRAILER_BYTES = 24
IMG_FRAME_OVERHEAD = 516
ENCODED_FRAME_OVERHEAD = 466
BUFFER_OVERHEAD = 56
TRACKED_FEATURE_FIXED_BYTES = 60
FEATURE_AGE_BYTES = 1
# A feature with an id of 128..65535 (3 B)
TRACKED_FEATURE_BYTES = TRACKED_FEATURE_FIXED_BYTES + 3 + FEATURE_AGE_BYTES
TRACKED_FEATURES_OVERHEAD = 64
IMU_PACKET_BYTES = 156
IMU_DATA_OVERHEAD = 56
MESSAGE_GROUP_OVERHEAD = 80
LOG_MESSAGE_OVERHEAD = 64
# SpatialLocations: the echoed config (Rect, thresholds, algorithm, step) about 45 B, three float depths, two
# uint16, the pixel count and a Point3f
SPATIAL_LOCATION_BYTES = 90
SPATIAL_DATA_OVERHEAD = 64
# SpatialLocationCalculatorConfigData sent up
SPATIAL_CONFIG_ROI_BYTES = 45
SPATIAL_CONFIG_OVERHEAD = 56
# libnop's unsigned integer widths: below each limit, the width at the same index (above the last: 9 B)
_UINT_LIMITS = np.array([128, 256, 65536, 2 ** 32], dtype=np.uint64)
_UINT_WIDTHS = np.array([1, 2, 3, 5, 9], dtype=np.int64)

# Host output queues in display order, with what they carry
STREAMS = {
    "depth": "stereo.depth via Sync (RAW16 depth the PnP reads; depth_transport full_frame)",
    "depth_points": "depth at the tracked features from the camera's SpatialLocationCalculator (points; estimate "
                    "per ROI)",
    "features": "FeatureTracker tracked features (estimate per feature)",
    "sync_group": "Sync MessageGroup header (estimate; full_frame)",
    "imu": "IMU accelerometer + gyroscope (estimate per packet)",
    "depth_on_demand": "full depth frames in points mode, only while the depth probe or a watched host_jpeg stream "
                       "needs them",
    "snapshot": "on-demand snapshots (only when requested; images JPEG from the camera, depth raw)",
    "depth_stream": "live camera / depth stream JPEG (only while a client watches)",
    "depth_stream_status": "depth stream Script status reports",
    "depth_points_status": "depth points Script status reports",
    "device_log": "camera log messages",
    "depth_points_rois": "UP: tag + ROI config per frame, built on the Pi (estimate per ROI)",
    "control": "UP: gate, snapshot request and tracker control messages (estimate)",
}
# Streams the host sends to the camera
UP_STREAMS = frozenset({"depth_points_rois", "control"})

PROTOCOLS = {
    "X_LINK_USB_VSC": "USB",
    "X_LINK_USB_CDC": "USB",
    "X_LINK_TCP_IP": "TCP/IP (PoE)",
    "X_LINK_PCIE": "PCIe",
    "X_LINK_IPC": "IPC",
}

Sample = namedtuple("Sample", "t read written streams")


def data_bytes(msg):
    return len(msg.getData())


def frame_bytes(data_length):
    return int(data_length) + IMG_FRAME_OVERHEAD


def encoded_bytes(data_length):
    return int(data_length) + ENCODED_FRAME_OVERHEAD


def buffer_bytes(data_length):
    return int(data_length) + BUFFER_OVERHEAD


def uint_bytes(values):
    """libnop's encoded size of each unsigned integer."""
    values = np.asarray(values, dtype=np.uint64).reshape(-1)
    return _UINT_WIDTHS[np.searchsorted(_UINT_LIMITS, values, side="right")]


def features_bytes(ids):
    """A TrackedFeatures message: from its feature ids (their encoded widths), or a count (3-byte ids)."""
    if np.ndim(ids) == 0:
        return TRACKED_FEATURES_OVERHEAD + int(ids) * TRACKED_FEATURE_BYTES
    widths = uint_bytes(ids)
    return TRACKED_FEATURES_OVERHEAD + len(widths) * (TRACKED_FEATURE_FIXED_BYTES + FEATURE_AGE_BYTES) + \
        int(widths.sum())


def imu_bytes(packets):
    return IMU_DATA_OVERHEAD + int(packets) * IMU_PACKET_BYTES


def spatial_data_bytes(rois):
    return SPATIAL_DATA_OVERHEAD + int(rois) * SPATIAL_LOCATION_BYTES


def spatial_config_bytes(rois):
    return SPATIAL_CONFIG_OVERHEAD + int(rois) * SPATIAL_CONFIG_ROI_BYTES


def points_request_bytes(rois):
    """One frame's tag (a Buffer) and ROI config."""
    return BUFFER_OVERHEAD + spatial_config_bytes(rois)


def log_bytes(payload):
    return len(payload or "") + LOG_MESSAGE_OVERHEAD


def mbps(nbytes, seconds):
    return nbytes * 8.0 / 1e6 / seconds


def device_link_info(device, usb_speed):
    """
    usb_speed: str(device.getUsbSpeed()) as pnp_vio.log_device_info read it (an RPC, read once per session),
    e.g. "UsbSpeed.SUPER", or its "unavailable (...)" text. The protocol and platform are host-side.
    """
    speed = None
    if usb_speed and not str(usb_speed).startswith("unavailable"):
        speed = str(usb_speed).split(".")[-1]
    protocol = raw = platform = None
    try:
        value = device.getDeviceInfo().protocol
        raw = getattr(value, "name", None)
        raw = raw if isinstance(raw, str) else str(value).split(".")[-1]
        protocol = PROTOCOLS.get(raw, raw)
    except Exception:
        pass
    try:
        platform = device.getPlatformAsString()
        platform = platform if isinstance(platform, str) else None
    except Exception:
        pass
    return {"usb_speed": speed, "protocol": protocol, "protocol_raw": raw, "platform": platform}


class StreamCounter:
    """Messages and estimated bytes per host output queue since the start; add() is called on the VIO loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._totals = {}

    def add(self, name, nbytes, messages=1):
        with self._lock:
            entry = self._totals.get(name)
            if entry is None:
                self._totals[name] = [messages, nbytes]
            else:
                entry[0] += messages
                entry[1] += nbytes

    def totals(self):
        with self._lock:
            return {name: (entry[0], entry[1]) for name, entry in self._totals.items()}


def _stream_order(name):
    return (list(STREAMS).index(name), name) if name in STREAMS else (len(STREAMS), name)


class LinkMonitor:
    """
    set_device() when a camera session starts, clear_device() when it ends; sample() about every
    SAMPLE_INTERVAL_S and status() / summary() on the publisher thread. streams: the StreamCounter the VIO
    loop feeds.
    """

    def __init__(self, clock=time.monotonic, source=SOURCE, mock=False):
        self.clock = clock
        self.source = source
        self.mock = mock
        self.interval_s = SAMPLE_INTERVAL_S
        self.streams = StreamCounter()
        self.samples = deque()
        self._lock = threading.Lock()
        self._profiling = None
        self._generation = 0
        self._info = {}
        self._reason = "no camera open"

    def set_device(self, profiling, usb_speed=None, protocol=None, protocol_raw=None, platform=None):
        """profiling: device.getProfilingData (or a stand-in returning numBytesRead / numBytesWritten)."""
        with self._lock:
            self._generation += 1
            self._profiling = profiling
            self._info = {"usb_speed": usb_speed, "protocol": protocol, "protocol_raw": protocol_raw,
                          "platform": platform}
            self._reason = None
            self.samples.clear()

    def clear_device(self, reason="no camera open"):
        with self._lock:
            self._generation += 1
            self._profiling = None
            self._info = {}
            self._reason = reason
            self.samples.clear()

    def sample(self):
        with self._lock:
            profiling, generation = self._profiling, self._generation
        read = written = error = None
        if profiling is not None:
            try:
                data = profiling()
                read, written = int(data.numBytesRead), int(data.numBytesWritten)
            except Exception as e:
                read = written = None
                error = f"getProfilingData() failed: {e}"
        now = self.clock()
        streams = self.streams.totals()
        with self._lock:
            if generation != self._generation:
                return  # the camera changed while reading
            if profiling is not None:
                self._reason = error
            last = next((s for s in reversed(self.samples) if s.read is not None), None)
            if read is not None and last is not None and (read < last.read or written < last.written):
                self.samples.clear()  # a new connection counts from 0
            self.samples.append(Sample(now, read, written, streams))
            while self.samples and self.samples[0].t < now - PEAK_WINDOW_S - _EPS:
                self.samples.popleft()

    def _availability(self, samples):
        if self._profiling is None or not samples:
            return False, self._reason or "no camera open"
        if samples[-1].read is None:
            return False, self._reason or "getProfilingData() gave no numbers"
        numeric = [s for s in samples if s.read is not None]
        if len(numeric) >= STUCK_AT_ZERO_SAMPLES and all(s.read == 0 and s.written == 0 for s in numeric):
            return False, ("getProfilingData() counters stay at 0 (XLink profiling not counting on this build?); "
                           "the per-stream estimates are still shown")
        return True, None

    def status(self):
        with self._lock:
            samples = list(self.samples)
            info = dict(self._info)
            available, reason = self._availability(samples)
        result = {
            "available": available, "reason": reason, "source": self.source, "mock": self.mock,
            "usb_speed": info.get("usb_speed"), "protocol": info.get("protocol"),
            "protocol_raw": info.get("protocol_raw"), "platform": info.get("platform"),
            "reference_mbps": REFERENCE_MBPS, "sample_interval_s": self.interval_s, "window_s": WINDOW_S,
            "peak_window_s": PEAK_WINDOW_S, "samples": len(samples),
            "down_mbps": None, "up_mbps": None, "down_mbps_10s": None, "up_mbps_10s": None, "window_span_s": None,
            "peak_down_mbps_60s": None, "peak_up_mbps_60s": None, "totals_bytes": None,
            "share_of_100mbps": None, "peak_share_of_100mbps": None, "unaccounted_mbps": None,
        }
        result.update(self._stream_rates(samples))
        if not available:
            return result

        numeric = [s for s in samples if s.read is not None]
        latest = numeric[-1]
        result["totals_bytes"] = {"down": latest.read, "up": latest.written}
        pairs = [(b.t, mbps(b.read - a.read, b.t - a.t), mbps(b.written - a.written, b.t - a.t))
                 for a, b in zip(numeric, numeric[1:]) if b.t > a.t]
        if not pairs:
            return result
        result["down_mbps"], result["up_mbps"] = pairs[-1][1], pairs[-1][2]
        first = next(s for s in numeric if s.t >= latest.t - WINDOW_S - _EPS)
        if latest.t > first.t:
            span = latest.t - first.t
            result["window_span_s"] = span
            result["down_mbps_10s"] = mbps(latest.read - first.read, span)
            result["up_mbps_10s"] = mbps(latest.written - first.written, span)
        recent = [p for p in pairs if p[0] >= latest.t - PEAK_WINDOW_S - _EPS]
        result["peak_down_mbps_60s"] = max(p[1] for p in recent)
        result["peak_up_mbps_60s"] = max(p[2] for p in recent)
        if result["down_mbps_10s"] is not None:
            result["share_of_100mbps"] = result["down_mbps_10s"] / REFERENCE_MBPS
            if result["streams_sum_mbps"] is not None:
                result["unaccounted_mbps"] = result["down_mbps_10s"] - result["streams_sum_mbps"]
        result["peak_share_of_100mbps"] = result["peak_down_mbps_60s"] / REFERENCE_MBPS
        return result

    @staticmethod
    def _stream_rates(samples):
        empty = {"streams": [], "streams_window_s": None, "streams_sum_mbps": None, "streams_up_sum_mbps": None}
        if len(samples) < 2:
            return empty
        latest = samples[-1]
        first = next(s for s in samples if s.t >= latest.t - WINDOW_S - _EPS)
        span = latest.t - first.t
        if span <= 0:
            return empty
        streams = []
        for name in sorted(latest.streams, key=_stream_order):
            messages, nbytes = latest.streams[name]
            before = first.streams.get(name, (0, 0))
            messages, nbytes = messages - before[0], nbytes - before[1]
            streams.append({"name": name, "description": STREAMS.get(name, ""), "mbps": mbps(nbytes, span),
                            "msgs_per_s": messages / span, "bytes_per_msg": nbytes / messages if messages else None,
                            "direction": "up" if name in UP_STREAMS else "down"})
        down = [s["mbps"] for s in streams if s["direction"] == "down"]
        up = [s["mbps"] for s in streams if s["direction"] == "up"]
        return {"streams": streams, "streams_window_s": span, "streams_sum_mbps": sum(down),
                "streams_up_sum_mbps": sum(up) if up else None}

    def summary(self):
        """The link part of the VIO's periodic log line."""
        status = self.status()
        if not status["available"]:
            return f"link n/a ({status['reason']})"
        if status["down_mbps_10s"] is None:
            return "link waiting for samples"
        where = " ".join(v for v in (status["protocol"], status["usb_speed"]) if v)
        streams = status["streams_sum_mbps"]
        return (f"link down {status['down_mbps_10s']:.1f} up {status['up_mbps_10s']:.1f} Mbps "
                f"({status['window_span_s']:.0f} s mean), peak {status['peak_down_mbps_60s']:.1f}, "
                f"{100 * status['share_of_100mbps']:.0f}% of {REFERENCE_MBPS:.0f} Mbps"
                f"{f', streams est. {streams:.1f}' if streams is not None else ''}{f', {where}' if where else ''}")
