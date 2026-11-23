import json
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_stream  # noqa: E402


# --- 8-bit encoding -------------------------------------------------------------------------------

@pytest.mark.parametrize("max_mm, shift", [(254, 0), (255, 1), (4000, 4), (15000, 6), (65535, 9)])
def test_depth_shift_keeps_255_free(max_mm, shift):
    assert depth_stream.depth_shift(max_mm) == shift
    assert max_mm >> shift <= 254


def test_gray_is_linear_in_depth_near_bright_invalid_black():
    # 16 mm bins: 4001 shares 4000's bin (the device's threshold filter has zeroed it anyway), 4016 does not
    depth = np.array([0, 100, 287, 300, 1000, 2150, 3000, 4000, 4016, 9000, 65535], np.uint16)
    gray = depth_stream.gray_reference(depth, 300, 4000).tolist()
    assert gray[0] == gray[1] == gray[2] == 0 and gray[-3:] == [0, 0, 0]  # no depth, below min, above max
    assert gray[3] >= 250 and gray[7] <= depth_stream.GRAY_FAR + 2
    valid = depth_stream.gray_reference(np.arange(300, 4001, dtype=np.uint16), 300, 4000).astype(int)
    assert valid.min() >= depth_stream.GRAY_FAR and valid.max() <= depth_stream.GRAY_NEAR
    assert (np.diff(valid) <= 0).all()  # monotonic: farther is never brighter
    # Linear: gray -> depth back within one 16 mm bin plus one gray level
    step_mm = 3700 / (depth_stream.GRAY_NEAR - depth_stream.GRAY_FAR)
    back = 4000 - (valid - depth_stream.GRAY_FAR) * step_mm
    assert np.abs(back - np.arange(300, 4001)).max() <= 16 + step_mm


def test_pixels_just_inside_the_range_stay_valid():
    gray = depth_stream.gray_reference(np.array([301, 3999], np.uint16), 300, 4000)
    assert (gray > 0).all()


def test_wide_range_uses_coarser_bins():
    gray = depth_stream.gray_reference(np.array([200, 15000, 15100], np.uint16), 200, 15000)
    assert gray[0] >= 250 and 0 < gray[1] <= depth_stream.GRAY_FAR + 2 and gray[2] == 0


def test_stream_divisor():
    assert depth_stream.stream_divisor(15, 5) == 3
    assert depth_stream.stream_divisor(10, 5) == 2
    assert depth_stream.stream_divisor(3, 5) == 1
    assert depth_stream.stream_divisor(15, 0.5) == 30


# --- the device Script, run on the host with fake Script inputs ------------------------------------

class Done(BaseException):
    """Ends a fake Script run; not an Exception, so the Script's own error handling does not catch it."""


class Control:
    def __init__(self, on):
        self.on = on

    def getSequenceNum(self):
        return 1 if self.on else 0


class FakeInput:
    def __init__(self, items=(), after=None):
        self.items = list(items)
        self.after = after  # called with each item handed out

    def get(self):
        if not self.items:
            raise Done()
        item = self.items.pop(0)
        if self.after:
            self.after(item)
        return item

    def tryGet(self):
        return self.items.pop(0) if self.items else None


class FakeOutput:
    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(message)


class DepthFrame:
    def __init__(self, depth, sequence, padding=0):
        self.depth, self.sequence = depth, sequence
        rows = depth.astype("<u2").view(np.uint8).reshape(depth.shape[0], -1)
        self.data = np.pad(rows, ((0, 0), (0, padding))).tobytes()

    def getWidth(self):
        return self.depth.shape[1]

    def getHeight(self):
        return self.depth.shape[0]

    def getData(self):
        return memoryview(self.data)

    def getSequenceNum(self):
        return self.sequence

    def getTimestamp(self):
        return self.sequence * 0.1


class DeviceBuffer:
    """
    A message as the camera's Script runtime has it, not the host's dai one: the constructor only takes the
    size in bytes (lpb.ImgFrame(arg0: int), the error on test9 for ImgFrame()), setData refuses more data
    than that, and only the methods in API exist.
    """
    API = frozenset({"getData", "setData", "getSequenceNum", "setSequenceNum", "getTimestamp", "setTimestamp"})

    def __init__(self, *args, **kwargs):
        if kwargs or len(args) != 1 or not isinstance(args[0], int) or isinstance(args[0], bool) or args[0] < 0:
            raise TypeError(f"__init__(): incompatible constructor arguments ... {type(self).__name__}(arg0: int)")
        self.size = args[0]
        self.calls = {}

    def __getattr__(self, name):
        if name not in type(self).API:
            raise AttributeError(f"'lpb.{type(self).__name__}' object has no attribute '{name}'")

        def call(value):
            if name == "setData" and len(value) > self.size:
                raise RuntimeError(f"Buffer size ({self.size}) too small to set specified data ({len(value)})")
            self.calls[name] = value
        return call


class Buffer(DeviceBuffer):
    pass


class ImgFrame(DeviceBuffer):
    API = depth_stream.DEVICE_IMGFRAME_API

    class Type:
        GRAY8 = "GRAY8"


def test_the_fake_device_messages_reject_what_the_camera_rejects():
    with pytest.raises(TypeError, match="incompatible constructor"):
        ImgFrame()
    with pytest.raises(AttributeError):
        ImgFrame(4).setStride(2)
    with pytest.raises(RuntimeError, match="too small"):
        ImgFrame(4).setData(b"12345")


def run_script(code, depth_items, control_items, after=None, clock=None):
    node = type("Node", (), {})()
    control = control_items if isinstance(control_items, FakeInput) else FakeInput(control_items)
    node.inputs = {"depth": FakeInput(depth_items, after), "control": control}
    node.outputs = {"gray": FakeOutput(), "status": FakeOutput()}
    node.warnings = []
    node.warn = node.warnings.append
    if clock is not None:
        code = code.replace("    clock = time.monotonic", "    clock = fake_clock")
    with pytest.raises(Done):
        exec(compile(code, "depth_stream_script", "exec"),
             {"node": node, "ImgFrame": ImgFrame, "Buffer": Buffer, "fake_clock": clock})
    return node


def reports(node):
    return [depth_stream.parse_device_status(message.calls["setData"]) for message in node.outputs["status"].sent]


def depth_image(seed=0, height=400, width=640):
    depth = np.random.default_rng(seed).integers(0, 4500, (height, width)).astype(np.uint16)
    depth[depth < 250] = 0
    return depth


def test_script_converts_like_the_reference_and_sends_every_nth_frame():
    frames = [DepthFrame(depth_image(n), n) for n in range(9)]
    code = depth_stream.device_script(3, 300, 4000)
    node = run_script(code, frames, [Control(True)])
    sent = node.outputs["gray"].sent
    # The first frame goes out right away, then every 3rd
    assert [f.calls["setSequenceNum"] for f in sent] == [0, 3, 6]
    for out in sent:
        source = frames[out.calls["setSequenceNum"]]
        assert out.size == 640 * 400 and len(out.calls["setData"]) == out.size
        assert out.calls["setType"] == "GRAY8"
        assert (out.calls["setWidth"], out.calls["setHeight"]) == (640, 400)
        assert out.calls["setTimestamp"] == source.getTimestamp()
        gray = np.frombuffer(out.calls["setData"], np.uint8).reshape(400, 640)
        np.testing.assert_array_equal(gray, depth_stream.gray_reference(source.depth, 300, 4000))
    assert node.warnings == [] and node.outputs["status"].sent == []


def script_calls(code):
    """(constructor calls by name, every obj.method(...) call as (obj name, method)) in the Script source."""
    import ast
    constructors, methods = {}, set()
    for call in ast.walk(ast.parse(code)):
        if not isinstance(call, ast.Call):
            continue
        if isinstance(call.func, ast.Name):
            constructors.setdefault(call.func.id, []).append(call)
        elif isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name):
            methods.add((call.func.value.id, call.func.attr))
    return constructors, methods


def test_script_uses_only_the_on_device_message_api():
    """The bug on test9: ImgFrame() does not exist in the camera's Script runtime, only ImgFrame(size)."""
    import ast
    code = depth_stream.device_script(3, 300, 4000)
    constructors, methods = script_calls(code)
    (img_frame,) = constructors["ImgFrame"]
    assert not img_frame.keywords and len(img_frame.args) == 1
    size = img_frame.args[0]
    assert isinstance(size, ast.BinOp) and isinstance(size.op, ast.Mult)
    assert (size.left.id, size.right.id) == ("width", "height")
    (buffer,) = constructors["Buffer"]
    assert len(buffer.args) == 1 and ast.unparse(buffer.args[0]) == "len(data)"
    # Every call on an ImgFrame (the output 'out', the input 'frame' / 'src') exists on the device
    on_frames = {method for name, method in methods if name in ("out", "frame", "src")}
    assert on_frames <= depth_stream.DEVICE_IMGFRAME_API, on_frames - depth_stream.DEVICE_IMGFRAME_API
    assert {"setData", "setType", "setWidth", "setHeight", "setTimestamp", "setSequenceNum"} <= on_frames
    assert {method for name, method in methods if name == "message"} == {"setData"}
    for missing_on_device in ("setStride", "setMetadata", "setTimestampDevice", "setFrame", "setCvFrame"):
        assert missing_on_device not in code
    assert "ImgFrame.Type.GRAY8" in code


def test_script_reports_errors_on_its_status_output():
    class Broken(DepthFrame):
        def getData(self):
            raise RuntimeError("no data")

    frames = [Broken(depth_image(0, 20, 32), 0), DepthFrame(depth_image(1, 20, 32), 1)]
    node = run_script(depth_stream.device_script(1, 300, 4000), frames, [Control(True)])
    assert [f.calls["setSequenceNum"] for f in node.outputs["gray"].sent] == [1]  # keeps going
    assert len(node.warnings) == 1 and "no data" in node.warnings[0]
    (report,) = reports(node)
    assert report["kind"] == "error" and report["errors"] == 1 and report["sent"] == 0
    assert report["text"] == "convert: RuntimeError: no data"


def test_script_error_reports_are_throttled():
    class Broken(DepthFrame):
        def getData(self):
            raise RuntimeError("no data %d" % self.sequence)

    frames = [Broken(depth_image(0, 20, 32), n) for n in range(250)]
    t = [100.0]

    def clock():  # read twice per frame: 10 ms per frame
        t[0] += 0.005
        return t[0]

    node = run_script(depth_stream.device_script(1, 300, 4000), frames, [Control(True)], clock=clock)
    # 250 errors over 2.5 s: the first right away, then about one a second, with the running count
    counts = [r["errors"] for r in reports(node)]
    assert counts[0] == 1 and len(counts) == 3 and counts == sorted(counts) and counts[-1] <= 250
    assert reports(node)[-1]["text"].startswith("convert: RuntimeError: no data ")
    assert len(node.warnings) == 3  # the 1st, 100th and 200th


def test_script_sends_the_frame_when_only_its_metadata_fails():
    class NoTimestamp(DepthFrame):
        def getTimestamp(self):
            raise TypeError("incompatible function arguments")

    node = run_script(depth_stream.device_script(1, 300, 4000), [NoTimestamp(depth_image(0, 20, 32), 7)],
                      [Control(True)])
    (out,) = node.outputs["gray"].sent
    assert out.calls["setType"] == "GRAY8" and "setSequenceNum" not in out.calls
    (report,) = reports(node)
    assert report["kind"] == "error" and report["text"].startswith("metadata: TypeError")


def test_script_reports_when_it_stops():
    class Dead(FakeInput):
        def get(self):
            raise RuntimeError("input closed")

    node = type("Node", (), {})()
    node.inputs = {"depth": FakeInput(), "control": Dead()}
    node.outputs = {"gray": FakeOutput(), "status": FakeOutput()}
    node.warnings = []
    node.warn = node.warnings.append
    with pytest.raises(RuntimeError, match="input closed"):
        exec(depth_stream.device_script(1, 300, 4000), {"node": node, "ImgFrame": ImgFrame, "Buffer": Buffer})
    (report,) = reports(node)
    assert report["kind"] == "fatal" and report["text"] == "stopped: RuntimeError: input closed"
    assert "input closed" in node.warnings[0]


def test_script_status_survives_a_failing_status_output():
    class Broken(DepthFrame):
        def getData(self):
            raise RuntimeError("no data")

    class Refusing(FakeOutput):
        def send(self, message):
            raise RuntimeError("queue full")

    node = type("Node", (), {})()
    node.inputs = {"depth": FakeInput([Broken(depth_image(0, 8, 8), 0), DepthFrame(depth_image(1, 8, 8), 1)]),
                   "control": FakeInput([Control(True)])}
    node.outputs = {"gray": FakeOutput(), "status": Refusing()}
    node.warnings = []
    node.warn = node.warnings.append
    with pytest.raises(Done):
        exec(depth_stream.device_script(1, 300, 4000), {"node": node, "ImgFrame": ImgFrame, "Buffer": Buffer})
    assert len(node.outputs["gray"].sent) == 1 and "status not sent: queue full" in node.warnings[-1]


def test_script_handles_padded_rows():
    depth = depth_image(3, height=120, width=200)
    node = run_script(depth_stream.device_script(1, 300, 4000, chunk_rows=50), [DepthFrame(depth, 0, padding=48)],
                      [Control(True)])
    (out,) = node.outputs["gray"].sent
    np.testing.assert_array_equal(np.frombuffer(out.calls["setData"], np.uint8).reshape(120, 200),
                                  depth_stream.gray_reference(depth, 300, 4000))


def test_script_waits_for_the_host_and_stops_when_told():
    frames = [DepthFrame(depth_image(n, 40, 64), n) for n in range(10)]
    control = FakeInput([Control(False), Control(True)])

    def stop_after_frame_4(frame):
        if frame.sequence == 4:
            control.items.append(Control(False))

    code = depth_stream.device_script(2, 300, 4000)
    node = run_script(code, frames, control, after=stop_after_frame_4)
    assert len(node.inputs["depth"].items) == 5  # never read past the stop
    # Off until enabled, sends frames 0, 2, 4, then blocks on the control input again once disabled
    assert [f.calls["setSequenceNum"] for f in node.outputs["gray"].sent] == [0, 2, 4]


def test_script_reports_its_cost():
    frames = [DepthFrame(depth_image(n, 40, 64), n) for n in range(4)]
    node = run_script(depth_stream.device_script(1, 300, 4000, stats_every=2), frames, [Control(True)])
    assert len(node.warnings) == 2 and "convert + send mean" in node.warnings[0] and "64x40" in node.warnings[0]
    stats = reports(node)
    assert [(r["kind"], r["sent"], r["errors"], r["size"]) for r in stats] == [("stats", 2, 0, [64, 40]),
                                                                               ("stats", 4, 0, [64, 40])]
    assert stats[0]["convert_ms_max"] >= stats[0]["convert_ms_mean"] >= 0.0


def test_script_conversion_cost_on_this_host():
    """Documents the per-frame work (it is far slower on the camera's CPU; the Script logs that)."""
    namespace = {}
    exec(depth_stream.device_script(3, 300, 4000).split("try:\n    import time")[0], namespace)
    frame = DepthFrame(depth_image(), 0)
    start = time.perf_counter()
    for _ in range(10):
        namespace["to_gray"](frame)
    assert (time.perf_counter() - start) / 10 < 0.05


# --- VIO process side -----------------------------------------------------------------------------

class Clock:
    def __init__(self, t=None):
        self.t = time.time() if t is None else t

    def __call__(self):
        return self.t


def test_service_streams_only_while_leased(tmp_path):
    (tmp_path / depth_stream.FRAME_FILE).write_bytes(b"\xff\xd8old")
    sent, clock, logs = [], Clock(), []
    service = depth_stream.DepthStreamService(tmp_path, sent.append, {"fps_target": 5.0}, clock=clock,
                                              log=logs.append)
    assert not (tmp_path / depth_stream.FRAME_FILE).exists()  # a previous run's frame is gone
    assert depth_stream.read_meta(tmp_path)["streaming"] is False

    service.tick()
    assert sent == [False]  # tells the device once, in case it was left on
    depth = np.full((400, 640), 1500, np.uint16)
    depth[200, 320] = 0
    depth_stream.touch_lease(tmp_path)
    clock.t += 0.6
    service.tick(depth)
    assert sent == [False, True] and service.enabled
    meta = depth_stream.read_meta(tmp_path)
    assert meta["streaming"] is True and meta["fps_target"] == 5.0
    assert meta["center"] == {"x": 320, "y": 200, "pixel_mm": 0, "patch": 5, "median_mm": 1500.0, "valid": 24,
                              "pixels": 25}
    clock.t += 0.1
    service.tick(depth)
    assert sent == [False, True]  # polled at most every 0.5 s

    for n in range(3):
        clock.t += 0.2
        assert service.on_frame(np.frombuffer(b"\xff\xd8jpeg%d" % n, np.uint8), sequence=n)
    assert (tmp_path / depth_stream.FRAME_FILE).read_bytes() == b"\xff\xd8jpeg2"
    meta = service.meta()
    assert meta["frames_total"] == 3 and meta["last_frame"]["sequence"] == 2
    assert meta["fps"] == pytest.approx(5.0) and meta["kbps"] > 0

    # Re-sent while streaming, in case a control message was lost
    clock.t += 5.0
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick(depth)
    assert sent == [False, True, True]

    # The lease runs out: stop
    clock.t += depth_stream.LEASE_S + 1
    service.tick(depth)
    assert sent[-1] is False and not service.enabled
    assert depth_stream.read_meta(tmp_path)["streaming"] is False
    assert logs == ["Depth stream started: a client is watching", "Depth stream stopped: no client"]
    service.close()
    assert sent[-1] is False and depth_stream.read_meta(tmp_path)["streaming"] is False


def test_service_survives_a_failing_control_queue(tmp_path):
    logs = []

    def broken(enabled):
        raise RuntimeError("queue closed")

    service = depth_stream.DepthStreamService(tmp_path, broken, log=logs.append)
    depth_stream.touch_lease(tmp_path)
    service.tick()
    assert not service.enabled and "queue closed" in logs[-1]


def test_parse_device_status():
    assert depth_stream.parse_device_status(b"error\t2\t5\tconvert: TypeError: a\tb") == {
        "kind": "error", "errors": 2, "sent": 5, "text": "convert: TypeError: a\tb"}
    stats = depth_stream.parse_device_status(np.frombuffer(b"stats\t0\t150\t41.5 60.2 640 400", np.uint8))
    assert stats["convert_ms_mean"] == 41.5 and stats["convert_ms_max"] == 60.2 and stats["size"] == [640, 400]
    for bad in (b"", b"error\tx\t0\tt", b"stats\t0\t1\tnot numbers", b"\xff\xfe"):
        assert depth_stream.parse_device_status(bad) is None


def test_service_publishes_the_device_errors(tmp_path):
    clock, logs = Clock(1000.0), []
    service = depth_stream.DepthStreamService(tmp_path, lambda enabled: None, clock=clock, log=logs.append)
    meta = depth_stream.read_meta(tmp_path)
    assert meta["errors"] == 0 and meta["last_error"] is None and meta["device_stats"] is None

    text = "convert: TypeError: __init__(): incompatible constructor arguments"
    service.on_device_status(f"error\t1\t0\t{text}".encode())
    clock.t += 1.0
    service.on_device_status(f"error\t57\t0\t{text}".encode())
    meta = depth_stream.read_meta(tmp_path)  # written right away, not only once a second while streaming
    assert meta["errors"] == 57
    assert meta["last_error"] == {"text": text, "unix_time": 1001.0, "fatal": False, "errors": 57, "frames_sent": 0}
    assert len(logs) == 1 and text in logs[0]  # the same error is logged once

    service.on_device_status(b"stats\t57\t150\t41.5 60.2 640 400")
    meta = depth_stream.read_meta(tmp_path)
    assert meta["device_stats"] == {"unix_time": 1001.0, "frames_sent": 150, "size": [640, 400],
                                    "convert_ms_mean": 41.5, "convert_ms_max": 60.2}
    assert meta["last_error"]["text"] == text  # kept

    service.on_device_status(b"fatal\t58\t150\tstopped: RuntimeError: input closed")
    meta = depth_stream.read_meta(tmp_path)
    assert meta["errors"] == 58 and meta["last_error"]["fatal"] is True and "stopped on the camera" in logs[-1]
    service.on_device_status(b"garbage")
    assert "unreadable status" in logs[-1] and depth_stream.read_meta(tmp_path)["errors"] == 58


def test_center_depth_ignores_invalid_pixels():
    depth = np.zeros((10, 10), np.uint16)
    assert depth_stream.center_depth(depth)["median_mm"] is None
    depth[4:7, 4:7] = [[1000, 1100, 0], [1200, 1300, 0], [0, 0, 0]]
    center = depth_stream.center_depth(depth)
    assert center["pixel_mm"] == 1300 and center["median_mm"] == 1150.0 and center["valid"] == 4
    assert depth_stream.center_depth(np.zeros(5)) is None


# --- main.py side ---------------------------------------------------------------------------------

def test_frame_reader_returns_each_fresh_frame_once(tmp_path):
    clock = Clock()
    reader = depth_stream.FrameReader(tmp_path, clock=clock)
    assert reader.read_new() is None
    path = tmp_path / depth_stream.FRAME_FILE
    depth_stream.snapshots.atomic_write(path, b"\xff\xd8one")
    assert reader.read_new() == b"\xff\xd8one" and reader.read_new() is None
    depth_stream.snapshots.atomic_write(path, b"\xff\xd8two")
    assert reader.read_new() == b"\xff\xd8two"
    depth_stream.snapshots.atomic_write(path, b"not a jpeg")
    assert reader.read_new() is None
    depth_stream.snapshots.atomic_write(path, b"\xff\xd8old")
    clock.t += 10
    assert reader.read_new() is None  # left over from a stopped VIO process


def test_multipart_part():
    part = depth_stream.multipart_part(b"\xff\xd8abc")
    assert part == b"--depthframe\r\nContent-Type: image/jpeg\r\nContent-Length: 5\r\n\r\n\xff\xd8abc\r\n"
    json.dumps(depth_stream.device_script(3, 300, 4000))
