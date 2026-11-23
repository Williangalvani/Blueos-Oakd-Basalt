"""
Live depth stream of the PnP VIO for the web page, encoded on the OAK-D.

Device (pnp_vio.build_depth_stream): a Script node takes stereo.depth (the same thresholded, filtered
depth the VIO uses) from a non-blocking size-1 queue and, only while the host enabled it, every
`every`-th frame turns the RAW16 millimetres into an 8-bit image with C-speed byte operations (strided
byte slices, bytes.translate tables, one big-integer OR per chunk of rows; see device_script). A
VideoEncoder node encodes that GRAY8 frame as MJPEG and only the JPEG crosses the link. The VIO process
writes it as-is to DIRECTORY/depth_stream.jpg; main.py forwards it as multipart/x-mixed-replace.
Nothing is decoded or re-encoded on the Pi; the browser colorizes the gray image.

8-bit encoding (linear in depth over the stereo threshold range, near bright):
    0                      invalid (no depth, or outside min_mm..max_mm, as the VIO sees it)
    GRAY_FAR..GRAY_NEAR    max_mm (far) .. min_mm (near)
Depth is quantised to 2**shift mm before the mapping (16 mm for the default 4 m range). JPEG ringing
blurs values near 0, so the page treats gray below GRAY_INVALID_BELOW as invalid.

Gating: main.py touches DIRECTORY/depth_stream.lease while a client streams; the VIO process checks it
at most every poll_interval and tells the Script to start or stop. With nobody watching, the Script
blocks on its control input, so nothing is converted, encoded or sent.

depthai-free so it can be unit tested.
"""

import json
import os
import time
from collections import deque
from pathlib import Path

import numpy as np

import snapshots

FRAME_FILE = "depth_stream.jpg"
META_FILE = "depth_stream.json"
LEASE_FILE = "depth_stream.lease"
LEASE_S = 4.0
GRAY_FAR = 16
GRAY_NEAR = 255
GRAY_INVALID_BELOW = 8
# Device-side conversion works on this many rows at a time to keep the Script's heap small
CHUNK_ROWS = 50
CENTER_PATCH = 5
BOUNDARY = "depthframe"
RATE_WINDOW_S = 5.0


def depth_shift(max_mm):
    """Smallest shift s with max_mm >> s <= 254, so 255 is free as the overflow marker."""
    shift = 0
    while (int(max_mm) >> shift) > 254:
        shift += 1
    return shift


def gray_for_depth(depth_mm, min_mm, max_mm):
    """Gray value of one depth (mm) on the stream's scale: 0 invalid, GRAY_FAR at max_mm, GRAY_NEAR at min_mm."""
    if depth_mm <= 0 or depth_mm < min_mm or depth_mm > max_mm:
        return 0
    span = max(max_mm - min_mm, 1)
    return int(round(GRAY_FAR + (GRAY_NEAR - GRAY_FAR) * (max_mm - depth_mm) / span))


def encoding_tables(min_mm, max_mm):
    """
    (shift, t_hi, t_lo, t_out): 256-byte tables for the device conversion of a RAW16 depth pixel with
    high byte h and low byte l: gray = t_out[t_hi[h] | t_lo[l]]. t_hi[h] | t_lo[l] is depth >> shift
    (255 if it overflows); t_out maps it to the gray scale at the centre of its 2**shift mm bin, clamped
    to min_mm..max_mm, and to 0 (invalid) for bins entirely outside it.
    """
    min_mm, max_mm = int(min_mm), int(max_mm)
    shift = depth_shift(max_mm)
    t_hi = bytes(min((h << 8) >> shift, 255) for h in range(256))
    t_lo = bytes(low >> shift for low in range(256))
    out = [0] * 256
    for v in range(1, 255):
        low, high = v << shift, ((v + 1) << shift) - 1
        if high >= min_mm and low <= max_mm:
            centre = min(max((low + high) / 2.0, min_mm), max_mm)
            out[v] = gray_for_depth(centre, min_mm, max_mm)
    return shift, t_hi, t_lo, bytes(out)


def gray_reference(depth_mm, min_mm, max_mm):
    """What the device produces for a uint16 depth image (numpy), for tests and offline checks."""
    _, t_hi, t_lo, t_out = encoding_tables(min_mm, max_mm)
    depth = np.asarray(depth_mm, dtype=np.uint16)
    hi = np.frombuffer(t_hi, np.uint8)[depth >> 8]
    lo = np.frombuffer(t_lo, np.uint8)[depth & 0xFF]
    return np.frombuffer(t_out, np.uint8)[hi | lo]


def stream_divisor(camera_fps, stream_fps):
    """Forward every n-th depth frame for about stream_fps."""
    return max(1, int(round(float(camera_fps) / max(float(stream_fps), 1e-3))))


def device_script(every, min_mm, max_mm, chunk_rows=CHUNK_ROWS, stats_every=150):
    """
    Script node source: inputs 'depth' (RAW16 mm) and 'control' (Buffer, sequence number 1 = stream,
    0 = stop); output 'gray' (GRAY8 ImgFrame) for the VideoEncoder. Plain CPython builtins only.
    While streaming it logs (node.warn, so it reaches the host log) its conversion time every
    stats_every frames: the measured on-device cost.
    """
    _, t_hi, t_lo, t_out = encoding_tables(min_mm, max_mm)
    return "\n".join([
        f"every = {int(every)}",
        f"chunk_rows = {int(chunk_rows)}",
        f"STATS_EVERY = {int(stats_every)}",
        f"t_hi = {t_hi!r}",
        f"t_lo = {t_lo!r}",
        f"t_out = {t_out!r}",
        "",
        "def to_gray(frame):",
        "    width = frame.getWidth()",
        "    height = frame.getHeight()",
        "    data = frame.getData()",
        "    row = width * 2",
        "    stride = len(data) // height if height else row",
        "    parts = []",
        "    for top in range(0, height, chunk_rows):",
        "        rows = min(chunk_rows, height - top)",
        "        if stride == row:",
        "            raw = bytes(data[top * row:(top + rows) * row])",
        "        else:",
        "            raw = b''.join([bytes(data[r * stride:r * stride + row]) for r in range(top, top + rows)])",
        "        hi = int.from_bytes(raw[1::2].translate(t_hi), 'little')",
        "        lo = int.from_bytes(raw[0::2].translate(t_lo), 'little')",
        "        parts.append((hi | lo).to_bytes(rows * width, 'little').translate(t_out))",
        "    return width, height, b''.join(parts)",
        "",
        "def gray_frame(src, width, height, gray):",
        "    out = ImgFrame()",
        "    try:",
        "        out.setMetadata(src)",
        "    except Exception:",
        "        pass",
        "    out.setData(gray)",
        "    out.setWidth(width)",
        "    out.setHeight(height)",
        "    try:",
        "        out.setStride(width)",
        "    except Exception:",
        "        pass",
        "    out.setType(ImgFrame.Type.GRAY8)",
        "    for name in ('setTimestamp', 'setTimestampDevice', 'setSequenceNum'):",
        "        try:",
        "            getattr(out, name)(getattr(src, 'g' + name[1:])())",
        "        except Exception:",
        "            pass",
        "    return out",
        "",
        "try:",
        "    import time",
        "    clock = time.monotonic",
        "except Exception:",
        "    clock = None",
        "",
        "enabled = False",
        "count = 0",
        "errors = 0",
        "sent = 0",
        "busy = 0.0",
        "worst = 0.0",
        "while True:",
        "    if not enabled:",
        "        enabled = node.inputs['control'].get().getSequenceNum() > 0",
        "        count = every - 1",
        "        continue",
        "    control = node.inputs['control'].tryGet()",
        "    while control is not None:",
        "        enabled = control.getSequenceNum() > 0",
        "        control = node.inputs['control'].tryGet()",
        "    if not enabled:",
        "        continue",
        "    frame = node.inputs['depth'].get()",
        "    count += 1",
        "    if count < every:",
        "        continue",
        "    count = 0",
        "    try:",
        "        start = clock() if clock else 0.0",
        "        width, height, gray = to_gray(frame)",
        "        node.outputs['gray'].send(gray_frame(frame, width, height, gray))",
        "        sent += 1",
        "        if clock:",
        "            took = clock() - start",
        "            busy += took",
        "            worst = max(worst, took)",
        "            if sent % STATS_EVERY == 0:",
        "                node.warn('depth stream: ' + str(sent) + ' frames, convert + send mean '",
        "                          + str(round(1000.0 * busy / STATS_EVERY, 1)) + ' ms, max '",
        "                          + str(round(1000.0 * worst, 1)) + ' ms (' + str(width) + 'x' + str(height) + ')')",
        "                busy = 0.0",
        "                worst = 0.0",
        "    except Exception as e:",
        "        errors += 1",
        "        if errors == 1 or errors % 100 == 0:",
        "            node.warn('depth stream: ' + str(e) + ' (' + str(errors) + ' errors)')",
        "    frame = None",
    ])


def center_depth(depth, patch=CENTER_PATCH):
    """Raw device depth (mm) at the image centre: the centre pixel and the median of the valid patch x patch."""
    depth = np.asarray(depth)
    if depth.ndim != 2 or not depth.size:
        return None
    height, width = depth.shape
    y, x, r = height // 2, width // 2, patch // 2
    window = depth[max(y - r, 0):y + r + 1, max(x - r, 0):x + r + 1]
    valid = window[window > 0]
    return {"x": x, "y": y, "pixel_mm": int(depth[y, x]), "patch": patch,
            "median_mm": float(np.median(valid)) if valid.size else None,
            "valid": int(valid.size), "pixels": int(window.size)}


def _lease_fresh(path, now, lease_s):
    try:
        return now - path.stat().st_mtime <= lease_s
    except OSError:
        return False


class DepthStreamService:
    """
    VIO process side. tick(depth) once per VIO frame: at most every poll_interval one stat of the
    lease, and set_enabled(bool) when it changes (and every resend_interval, in case a control message
    was lost). on_frame(jpeg) with each encoded frame from the device: one atomic write of the JPEG.
    While streaming, the metadata (META_FILE, with the raw depth at the image centre) is rewritten at
    most every meta_interval.
    """

    def __init__(self, directory, set_enabled, info=None, lease_s=LEASE_S, poll_interval=0.5, resend_interval=5.0,
                 meta_interval=1.0, clock=time.time, log=print):
        self.directory = Path(directory)
        self.set_enabled = set_enabled
        self.info = dict(info or {})
        self.lease_s = lease_s
        self.poll_interval = poll_interval
        self.resend_interval = resend_interval
        self.meta_interval = meta_interval
        self.clock = clock
        self.log = log
        self.enabled = False
        self.frames = 0
        self.bytes = 0
        self.last_frame = None  # (unix time, bytes, sequence)
        self.center = None
        self.recent = deque()
        self._last_poll = self._last_send = self._last_meta = float("-inf")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            for name in (FRAME_FILE, META_FILE):
                (self.directory / name).unlink(missing_ok=True)  # never show a previous run's frame
        except OSError as e:
            log(f"Depth stream directory {self.directory} unusable: {e}")
        self.write_meta()

    def tick(self, depth=None):
        now = self.clock()
        if now - self._last_poll >= self.poll_interval:
            self._last_poll = now
            wanted = _lease_fresh(self.directory / LEASE_FILE, now, self.lease_s)
            if wanted != self.enabled or now - self._last_send >= self.resend_interval:
                changed = wanted != self.enabled
                if changed:
                    self.log(f"Depth stream {'started: a client is watching' if wanted else 'stopped: no client'}")
                self._send(wanted, now)
                if changed and self.enabled == wanted:
                    self._last_meta = float("-inf")  # publish the change right away
                    if not wanted:
                        self.write_meta(now)
        if self.enabled and now - self._last_meta >= self.meta_interval:
            if depth is not None:
                self.center = center_depth(depth)
            self.write_meta(now)

    def _send(self, enabled, now):
        try:
            self.set_enabled(enabled)
        except Exception as e:
            self.log(f"Depth stream control not sent: {e}")
            return
        self.enabled = enabled
        self._last_send = now

    def on_frame(self, jpeg, sequence=None):
        now = self.clock()
        data = bytes(jpeg)
        try:
            snapshots.atomic_write(self.directory / FRAME_FILE, data)
        except OSError as e:
            self.log(f"Depth stream frame not written: {e}")
            return False
        self.frames += 1
        self.bytes += len(data)
        self.last_frame = (now, len(data), sequence)
        self.recent.append((now, len(data)))
        while self.recent and now - self.recent[0][0] > RATE_WINDOW_S:
            self.recent.popleft()
        return True

    def rates(self, now):
        recent = [(t, size) for t, size in self.recent if now - t <= RATE_WINDOW_S]
        if len(recent) < 2:
            return None, None
        span = max(recent[-1][0] - recent[0][0], 1e-6)
        return (len(recent) - 1) / span, 8.0 * sum(size for _, size in recent[1:]) / span / 1000.0

    def meta(self, now=None):
        now = self.clock() if now is None else now
        fps, kbps = self.rates(now)
        last = self.last_frame
        return {**self.info, "pid": os.getpid(), "unix_time": now, "streaming": self.enabled,
                "frames_total": self.frames, "bytes_total": self.bytes, "fps": fps, "kbps": kbps,
                "last_frame": None if last is None else {"unix_time": last[0], "bytes": last[1], "sequence": last[2]},
                "center": self.center}

    def write_meta(self, now=None):
        now = self.clock() if now is None else now
        self._last_meta = now
        try:
            snapshots.atomic_write(self.directory / META_FILE, json.dumps(self.meta(now)).encode())
        except OSError as e:
            self.log(f"Depth stream metadata not written: {e}")

    def close(self):
        """The pipeline is stopping (and the Script with it): only record that nothing streams any more."""
        self.enabled = False
        self.write_meta()


# --- main.py side ---------------------------------------------------------------------------------

def touch_lease(directory):
    """Ask the VIO process to stream for LEASE_S."""
    directory = Path(directory)
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / LEASE_FILE).touch()
    except OSError:
        pass


def read_meta(directory):
    try:
        return json.loads((Path(directory) / META_FILE).read_text())
    except (OSError, ValueError):
        return None


class FrameReader:
    """The latest JPEG in DIRECTORY/depth_stream.jpg, returned once per new file and only if fresh."""

    def __init__(self, directory, max_age_s=3.0, clock=time.time):
        self.path = Path(directory) / FRAME_FILE
        self.max_age_s = max_age_s
        self.clock = clock
        self._key = None

    def read_new(self):
        try:
            stat = self.path.stat()
        except OSError:
            return None
        key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        if key == self._key:
            return None
        self._key = key
        if self.clock() - stat.st_mtime > self.max_age_s:
            return None  # left over from a VIO process that stopped
        try:
            data = self.path.read_bytes()
        except OSError:
            return None
        return data if data[:2] == b"\xff\xd8" else None


def multipart_part(jpeg, boundary=BOUNDARY):
    return (f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode()
            + jpeg + b"\r\n")
