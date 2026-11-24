import enum
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

    class Type(enum.IntEnum):
        """The values the device logs (GRAY8 is 'frame type (30)' in the VideoEncoder warning on test10)."""
        YUV400p = 4
        RAW16 = 14
        NV12 = 22
        GRAY8 = 30


class FakeVideoEncoder(FakeOutput):
    """
    The Script's 'gray' output linked to the camera's VideoEncoder (PlgVidEnc in the RVC2 firmware): it
    takes NV12 or YUV400p only and drops anything else with the firmware's warning, once per frame (test10:
    GRAY8 frames sent, nothing encoded). The first frame sets the encoder size.
    """

    def __init__(self):
        super().__init__()
        self.encoded = []
        self.warnings = []
        self.size = None

    def send(self, frame):
        super().send(frame)
        kind = frame.calls.get("setType")
        width, height = frame.calls.get("setWidth"), frame.calls.get("setHeight")
        if kind not in {getattr(ImgFrame.Type, name) for name in depth_stream.DEVICE_ENCODER_INPUT_TYPES}:
            number = 0 if kind is None else int(kind)
            self.warnings.append(f"[VideoEncoder(11)] [warning] Arrived frame type ({number}) is not either NV12 or "
                                 f"YUV400p (8-bit Gray)")
            return
        self.size = self.size or (width, height)
        if (width, height) != self.size:
            self.warnings.append(f"Arrived frame ({width}, {height}) does not match the encoder size {self.size}")
            return
        planes = width * height * (3 if kind == ImgFrame.Type.NV12 else 2) // 2
        assert len(frame.calls["setData"]) == planes <= frame.size, "wrong plane size for the frame type"
        self.encoded.append(frame)


def test_the_fake_device_messages_reject_what_the_camera_rejects():
    with pytest.raises(TypeError, match="incompatible constructor"):
        ImgFrame()
    with pytest.raises(AttributeError):
        ImgFrame(4).setStride(2)
    with pytest.raises(RuntimeError, match="too small"):
        ImgFrame(4).setData(b"12345")


def gray_img_frame(kind, width=64, height=40):
    frame = ImgFrame(width * height)
    frame.setData(bytes(width * height))
    frame.setType(kind)
    frame.setWidth(width)
    frame.setHeight(height)
    return frame


def test_the_fake_encoder_drops_gray8_like_the_camera():
    encoder = FakeVideoEncoder()
    encoder.send(gray_img_frame(ImgFrame.Type.GRAY8))
    assert encoder.encoded == []
    assert encoder.warnings == ["[VideoEncoder(11)] [warning] Arrived frame type (30) is not either NV12 or "
                                "YUV400p (8-bit Gray)"]
    encoder.send(gray_img_frame(ImgFrame.Type.YUV400p))
    assert len(encoder.encoded) == 1 and len(encoder.warnings) == 1


def run_script(code, depth_items, control_items, after=None, clock=None, gray=None):
    node = type("Node", (), {})()
    control = control_items if isinstance(control_items, FakeInput) else FakeInput(control_items)
    node.inputs = {"depth": FakeInput(depth_items, after), "control": control}
    node.outputs = {"gray": gray or FakeVideoEncoder(), "status": FakeOutput()}
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


def errors_reported(node):
    return [r for r in reports(node) if r["kind"] != "sent"]


def depth_image(seed=0, height=400, width=640):
    depth = np.random.default_rng(seed).integers(0, 4500, (height, width)).astype(np.uint16)
    depth[depth < 250] = 0
    return depth


def test_script_converts_like_the_reference_and_sends_every_nth_frame():
    frames = [DepthFrame(depth_image(n), n) for n in range(9)]
    code = depth_stream.device_script(3, 300, 4000)
    node = run_script(code, frames, [Control(True)])
    encoder = node.outputs["gray"]
    # The first frame goes out right away, then every 3rd; the encoder takes all of them
    assert [f.calls["setSequenceNum"] for f in encoder.sent] == [0, 3, 6]
    assert encoder.encoded == encoder.sent and encoder.warnings == []
    for out in encoder.sent:
        source = frames[out.calls["setSequenceNum"]]
        assert out.size == 640 * 400 and len(out.calls["setData"]) == out.size
        assert out.calls["setType"] == ImgFrame.Type.YUV400p
        assert (out.calls["setWidth"], out.calls["setHeight"]) == (640, 400)
        assert out.calls["setTimestamp"] == source.getTimestamp()
        gray = np.frombuffer(out.calls["setData"], np.uint8).reshape(400, 640)
        np.testing.assert_array_equal(gray, depth_stream.gray_reference(source.depth, 300, 4000))
    assert node.warnings == []
    assert reports(node)[0] == {"kind": "sent", "errors": 0, "sent": 1, "text": ""}
    assert {r["kind"] for r in reports(node)} == {"sent"}


def test_gray8_into_the_encoder_reproduces_test10():
    """HEAD before this fix: the Script sends GRAY8, counts every frame as sent, and nothing is encoded."""
    code = depth_stream.device_script(1, 300, 4000).replace("ImgFrame.Type.YUV400p", "ImgFrame.Type.GRAY8")
    node = run_script(code, [DepthFrame(depth_image(n, 40, 64), n) for n in range(5)], [Control(True)])
    encoder = node.outputs["gray"]
    assert len(encoder.sent) == 5 and encoder.encoded == []
    assert encoder.warnings == ["[VideoEncoder(11)] [warning] Arrived frame type (30) is not either NV12 or "
                                "YUV400p (8-bit Gray)"] * 5
    assert node.warnings == [] and reports(node)[0]["sent"] == 1  # no error on the camera's side


def test_script_reports_frames_sent_about_once_a_second():
    frames = [DepthFrame(depth_image(n, 20, 32), n) for n in range(25)]
    t = [100.0]

    def clock():  # read twice per frame: 0.125 s per frame
        t[0] += 0.0625
        return t[0]

    node = run_script(depth_stream.device_script(1, 300, 4000), frames, [Control(True)], clock=clock)
    progress = [r["sent"] for r in reports(node) if r["kind"] == "sent"]
    assert progress == [1, 9, 17, 25]
    assert len(node.outputs["gray"].encoded) == 25


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
    # The frame type is one the camera's VideoEncoder takes (it drops GRAY8: test10)
    types = {node.attr for node in ast.walk(ast.parse(code)) if isinstance(node, ast.Attribute)
             and ast.unparse(node.value) == "ImgFrame.Type"}
    assert types == {"YUV400p"} and types <= depth_stream.DEVICE_ENCODER_INPUT_TYPES


def test_script_reports_errors_on_its_status_output():
    class Broken(DepthFrame):
        def getData(self):
            raise RuntimeError("no data")

    frames = [Broken(depth_image(0, 20, 32), 0), DepthFrame(depth_image(1, 20, 32), 1)]
    node = run_script(depth_stream.device_script(1, 300, 4000), frames, [Control(True)])
    assert [f.calls["setSequenceNum"] for f in node.outputs["gray"].sent] == [1]  # keeps going
    assert len(node.warnings) == 1 and "no data" in node.warnings[0]
    (report,) = errors_reported(node)
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
    assert out.calls["setType"] == ImgFrame.Type.YUV400p and "setSequenceNum" not in out.calls
    (report,) = errors_reported(node)
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
    stats = [r for r in reports(node) if r["kind"] == "stats"]
    assert [(r["kind"], r["sent"], r["errors"], r["size"]) for r in stats] == [("stats", 2, 0, [64, 40]),
                                                                               ("stats", 4, 0, [64, 40])]
    assert stats[0]["convert_ms_max"] >= stats[0]["convert_ms_mean"] >= 0.0


class FakeImageManip(FakeOutput):
    """
    The manip variant's ImageManip (depthai v3, frame type NV12) between the Script and the encoder: it takes
    the Script's GRAY8 frame (stride 0 counts as the width) and sends an NV12 frame of the same size, from
    its own pool of manip_output_size buffers, with the gray plane as Y and neutral chroma.
    """

    def __init__(self, encoder):
        super().__init__()
        self.encoder = encoder

    def send(self, frame):
        super().send(frame)
        width, height = frame.calls["setWidth"], frame.calls["setHeight"]
        assert frame.calls["setType"] in (ImgFrame.Type.GRAY8, ImgFrame.Type.YUV400p, ImgFrame.Type.NV12)
        out = ImgFrame(depth_stream.manip_output_size(width, height))
        out.setData(bytes(frame.calls["setData"])[:width * height] + b"\x80" * (width * height // 2))
        out.setType(ImgFrame.Type.NV12)
        out.setWidth(width)
        out.setHeight(height)
        self.encoder.send(out)


def test_manip_output_size_matches_depthai():
    # 640x400 NV12: Y 640 x 416 rows, UV 640 x 208 rows, each plane rounded up to 4096 bytes
    assert depth_stream.manip_output_size(640, 400) == 401408
    assert depth_stream.manip_output_size(640, 400) >= 640 * 400 * 3 // 2


def test_size_problem():
    assert depth_stream.size_problem(640, 400) is None and depth_stream.size_problem(1280, 800) is None
    assert "multiple of 32" in depth_stream.size_problem(600, 400)
    assert depth_stream.size_problem(640, 401) is not None


def test_device_variant():
    assert depth_stream.device_variant("auto") == "manip"
    assert [depth_stream.device_variant(v) for v in ("manip", "nv12", "yuv400p")] == ["manip", "nv12", "yuv400p"]
    assert depth_stream.device_variant("host_jpeg") is None and depth_stream.device_variant("bogus") is None


@pytest.mark.parametrize("variant", ["manip", "nv12", "yuv400p"])
def test_each_camera_variant_reaches_the_encoder(variant):
    """Strict fakes: the device message API, the encoder's frame types and plane sizes, ImageManip for manip."""
    frame_type = depth_stream.SCRIPT_FRAME_TYPES[variant]
    frames = [DepthFrame(depth_image(n, 40, 64), n) for n in range(6)]
    encoder = FakeVideoEncoder()
    gray = FakeImageManip(encoder) if variant == "manip" else encoder
    node = run_script(depth_stream.device_script(2, 300, 4000, frame_type=frame_type), frames, [Control(True)],
                      gray=gray)
    assert [f.calls["setSequenceNum"] for f in gray.sent] == [0, 2, 4]
    assert all(f.calls["setType"] == getattr(ImgFrame.Type, frame_type) for f in gray.sent)
    assert len(encoder.encoded) == 3 and encoder.warnings == []
    for out, sent in zip(encoder.encoded, gray.sent):
        data = bytes(out.calls["setData"])
        y = np.frombuffer(data[:64 * 40], np.uint8).reshape(40, 64)
        np.testing.assert_array_equal(y, depth_stream.gray_reference(frames[sent.calls["setSequenceNum"]].depth,
                                                                     300, 4000))
        if out.calls["setType"] == ImgFrame.Type.NV12:
            assert len(data) == 64 * 40 * 3 // 2 and set(data[64 * 40:]) == {128}
        else:
            assert len(data) == 64 * 40
    assert node.warnings == [] and errors_reported(node) == []


def test_nv12_frames_are_sized_for_both_planes():
    node = run_script(depth_stream.device_script(1, 300, 4000, frame_type="NV12"),
                      [DepthFrame(depth_image(0, 400, 640), 0)], [Control(True)])
    (out,) = node.outputs["gray"].encoded
    assert out.size == len(out.calls["setData"]) == 640 * 400 * 3 // 2


@pytest.mark.parametrize("variant, size", [("manip", "width * height"), ("yuv400p", "width * height"),
                                           ("nv12", "size + size // 2")])
def test_each_variant_script_uses_only_the_on_device_message_api(variant, size):
    import ast
    frame_type = depth_stream.SCRIPT_FRAME_TYPES[variant]
    code = depth_stream.device_script(3, 300, 4000, frame_type=frame_type)
    json.dumps(code)
    constructors, methods = script_calls(code)
    (img_frame,) = constructors["ImgFrame"]
    assert not img_frame.keywords and ast.unparse(img_frame.args[0]) == size
    on_frames = {method for name, method in methods if name in ("out", "frame", "src")}
    assert on_frames <= depth_stream.DEVICE_IMGFRAME_API
    types = {node.attr for node in ast.walk(ast.parse(code)) if isinstance(node, ast.Attribute)
             and ast.unparse(node.value) == "ImgFrame.Type"}
    assert types == {frame_type}
    # Only the ImageManip variant may send a type the encoder does not take itself
    assert (frame_type in depth_stream.DEVICE_ENCODER_INPUT_TYPES) == (variant != "manip")
    with pytest.raises(ValueError):
        depth_stream.device_script(3, 300, 4000, frame_type="RAW8")


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
    assert depth_stream.parse_device_status(b"sent\t0\t150\t") == {"kind": "sent", "errors": 0, "sent": 150,
                                                                    "text": ""}
    for bad in (b"", b"error\tx\t0\tt", b"stats\t0\t1\tnot numbers", b"\xff\xfe"):
        assert depth_stream.parse_device_status(bad) is None


def test_service_publishes_the_device_errors(tmp_path):
    clock, logs = Clock(1000.0), []
    service = depth_stream.DepthStreamService(tmp_path, lambda enabled: None, clock=clock, log=logs.append)
    meta = depth_stream.read_meta(tmp_path)
    assert meta["errors"] == 0 and meta["last_error"] is None and meta["device_stats"] is None
    assert meta["encoder_stalled"] is False

    text = "convert: TypeError: __init__(): incompatible constructor arguments"
    service.on_device_status(f"error\t1\t0\t{text}".encode())
    clock.t += 1.0
    service.on_device_status(f"error\t57\t0\t{text}".encode())
    meta = depth_stream.read_meta(tmp_path)  # written right away, not only once a second while streaming
    assert meta["errors"] == 57
    assert meta["last_error"] == {"text": text, "unix_time": 1001.0, "fatal": False, "errors": 57, "frames_sent": 0,
                                  "variant": None}
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


def streaming_service(tmp_path, clock, logs):
    service = depth_stream.DepthStreamService(tmp_path, lambda enabled: None, clock=clock, log=logs.append)
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    assert service.enabled
    return service


def test_service_flags_frames_sent_but_nothing_encoded(tmp_path):
    """test10: the Script sent 150 frames (GRAY8) and the encoder dropped every one; the host saw nothing."""
    clock, logs = Clock(1000.0), []
    service = streaming_service(tmp_path, clock, logs)
    assert depth_stream.read_meta(tmp_path)["encoder_stalled"] is False
    for sent in range(1, 5):  # one progress report a second, no encoded frame: 3 s after the first one ...
        clock.t += 1.0
        service.on_device_status(f"sent\t0\t{sent * 5}\t".encode())
        assert depth_stream.read_meta(tmp_path)["encoder_stalled"] is False
    clock.t += 0.5  # ... and more
    service.on_device_status(b"sent\t0\t22\t")
    meta = depth_stream.read_meta(tmp_path)
    assert meta["encoder_stalled"] is True and meta["errors"] == 0
    assert meta["last_error"]["text"].startswith(depth_stream.ENCODER_STALL_TEXT)
    assert meta["last_error"]["frames_sent"] == 22 and meta["last_error"]["unix_time"] == 1004.5
    assert len(logs) == 2 and depth_stream.ENCODER_STALL_TEXT in logs[-1]
    clock.t += 1.0
    service.on_device_status(b"sent\t0\t27\t")
    assert len(logs) == 2  # logged once

    # Encoded frames again: cleared, the last error stays as a record
    service.on_frame(b"\xff\xd8jpeg", 1)
    meta = service.meta()
    assert meta["encoder_stalled"] is False and meta["last_error"]["text"].startswith(depth_stream.ENCODER_STALL_TEXT)
    assert "encoded frames arrive again" in logs[-1]

    # Stalled again, then the stream stops: not streaming, so not stalled
    for sent in (30, 35, 40, 45):
        clock.t += 1.2
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
    assert service.meta()["encoder_stalled"] is True
    clock.t += depth_stream.LEASE_S + 1
    service.tick()
    assert not service.enabled and depth_stream.read_meta(tmp_path)["encoder_stalled"] is False


def test_service_does_not_flag_a_working_or_idle_encoder(tmp_path):
    clock, logs = Clock(1000.0), []
    service = streaming_service(tmp_path, clock, logs)
    for n in range(1, 30):  # 5 fps, a progress report a second, each frame encoded
        clock.t += 0.2
        os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
        service.tick()
        if n % 5 == 0:
            service.on_device_status(f"sent\t0\t{n}\t".encode())
        service.on_frame(b"\xff\xd8jpeg", n)
    assert service.meta()["encoder_stalled"] is False
    clock.t += 0.5
    service.on_device_status(b"sent\t0\t30\t")  # the report of a frame that already arrived
    # The Script sent nothing for 10 s (no depth frames), then one frame whose JPEG is not in yet
    clock.t += 10.0
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    service.on_device_status(b"sent\t0\t31\t")
    assert service.meta()["encoder_stalled"] is False and service.last_error is None
    assert logs == ["Depth stream started: a client is watching"]


def test_service_flag_survives_a_restarted_script(tmp_path):
    clock, logs = Clock(1000.0), []
    service = streaming_service(tmp_path, clock, logs)
    for sent in (500, 505, 3, 8, 13):  # the Script's count starts over after 505
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
        assert service.meta()["encoder_stalled"] is False
        clock.t += 1.0
    clock.t += 0.5
    service.on_device_status(b"sent\t0\t20\t")
    assert service.meta()["encoder_stalled"] is True


def test_service_waits_longer_for_reports_of_a_slow_stream(tmp_path):
    clock, logs = Clock(1000.0), []
    service = depth_stream.DepthStreamService(tmp_path, lambda enabled: None, {"fps_target": 0.5}, clock=clock,
                                              log=logs.append)
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    for sent in (1, 2, 3):  # one frame, and so one report, every 2 s and a bit
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
        clock.t += 2.6
    assert service.meta()["encoder_stalled"] is True


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


# --- host_jpeg: encoded on the Pi -----------------------------------------------------------------

def test_host_encoder_matches_the_camera_scale():
    import cv2
    clock = Clock(100.0)
    encoder = depth_stream.HostJpegEncoder(300, 4000, 95, every=2, threaded=False, clock=clock)
    depth = depth_image(1, 40, 64)
    assert not encoder.submit(depth, 0)  # not enabled
    encoder.set_enabled(True)
    taken = []
    for n in range(5):
        clock.t += 1.0
        taken.append(encoder.submit(depth, n))
    assert taken == [True, False, True, False, True]  # every 2nd, the first one at once
    results = encoder.drain()
    assert [sequence for _, sequence in results] == [0, 2, 4] and encoder.drain() == []
    gray = cv2.imdecode(np.frombuffer(results[0][0], np.uint8), cv2.IMREAD_GRAYSCALE)
    reference = depth_stream.gray_reference(depth, 300, 4000).astype(int)
    assert gray.shape == (40, 64) and np.abs(gray.astype(int) - reference).mean() < 4
    reports = [depth_stream.parse_device_status(r) for r in encoder.reports()]
    assert [r["kind"] for r in reports] == ["sent"] * 3 and [r["sent"] for r in reports] == [1, 2, 3]
    assert encoder.stats()["frames"] == 3 and encoder.stats()["cpu_budget_percent"] == 25.0
    encoder.close()
    assert not encoder.enabled


def test_host_encoder_keeps_to_its_cpu_budget():
    clock, cpu = Clock(100.0), [0.0]

    def encode(depth):
        cpu[0] += 0.1  # 0.1 s of CPU per frame
        return b"\xff\xd8x"

    encoder = depth_stream.HostJpegEncoder(300, 4000, 70, cpu_budget=0.25, threaded=False, clock=clock,
                                           cpu_clock=lambda: cpu[0], encode=encode, stats_every=2)
    encoder.set_enabled(True)
    taken = []
    for n in range(9):  # a frame every 0.25 s
        taken.append(encoder.submit(np.zeros((4, 32), np.uint16), n))
        clock.t += 0.25
    # After a 0.1 s encode the next one waits 0.3 s: every other frame, 20 % of a core
    assert taken == [True, False] * 4 + [True]
    stats = encoder.stats()
    assert stats["skipped_cpu"] == 4 and stats["frames"] == 5 and stats["offered"] == 9
    assert stats["cpu_percent"] <= 25.0
    reports = [depth_stream.parse_device_status(r) for r in encoder.reports()]
    assert {r["kind"] for r in reports} == {"sent", "stats"}
    assert [r["size"] for r in reports if r["kind"] == "stats"] == [[32, 4], [32, 4]]


def test_host_encoder_reports_its_errors():
    def encode(depth):
        raise RuntimeError("no memory")

    encoder = depth_stream.HostJpegEncoder(300, 4000, 70, threaded=False, clock=Clock(1.0), encode=encode)
    encoder.set_enabled(True)
    assert encoder.submit(np.zeros((4, 32), np.uint16), 0)
    assert encoder.drain() == []
    kinds = [depth_stream.parse_device_status(r) for r in encoder.reports()]
    assert kinds[-1] == {"kind": "error", "errors": 1, "sent": 1, "text": "encode: RuntimeError: no memory"}


def test_host_encoder_thread_skips_frames_while_busy():
    import threading
    release, started = threading.Event(), threading.Event()

    def encode(depth):
        started.set()
        release.wait(5)
        return b"\xff\xd8x"

    encoder = depth_stream.HostJpegEncoder(300, 4000, 70, clock=Clock(1.0), encode=encode)
    try:
        encoder.set_enabled(True)
        assert encoder.submit(np.zeros((4, 32), np.uint16), 0)
        assert started.wait(5)
        assert not encoder.submit(np.zeros((4, 32), np.uint16), 1)
        assert encoder.stats()["skipped_busy"] == 1
        release.set()
        deadline = time.time() + 5
        results = []
        while not results and time.time() < deadline:
            results = encoder.drain()
            time.sleep(0.01)
        assert results == [(b"\xff\xd8x", 0)]
    finally:
        release.set()
        encoder.close()


def host_service(tmp_path, clock, logs, encode=None, info=None, **kwargs):
    host = depth_stream.HostJpegEncoder(300, 4000, 70, threaded=False, clock=clock,
                                        encode=encode or (lambda depth: b"\xff\xd8host"))
    controls = []
    info = info or {"fps_target": 5.0, "variant": "host_jpeg", "requested_variant": "host_jpeg"}
    service = depth_stream.DepthStreamService(tmp_path, controls.append, info, clock=clock, log=logs.append,
                                              host_encoder=host, **kwargs)
    return service, host, controls


def test_host_jpeg_service_streams_from_the_pi(tmp_path):
    clock, logs = Clock(1000.0), []
    service, host, controls = host_service(tmp_path, clock, logs)
    assert service.on_host and service.meta()["encoded_on"] == "pi"
    depth = np.full((40, 64), 1500, np.uint16)
    service.tick(depth, 0)
    assert not service.enabled and not host.enabled
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    for n in range(1, 6):
        clock.t += 0.6
        service.tick(depth, n)
    assert service.enabled and host.enabled and controls == []  # the camera's control is not used
    assert (tmp_path / depth_stream.FRAME_FILE).read_bytes() == b"\xff\xd8host"
    meta = depth_stream.read_meta(tmp_path)
    assert meta["variant"] == "host_jpeg" and meta["encoded_on"] == "pi" and meta["frames_total"] == 5
    assert meta["host_encoder"]["frames"] == 5 and meta["encoder_stalled"] is False
    assert service.on_frame(b"\xff\xd8camera", 9) is False  # nothing from the camera in this variant
    service.close()
    assert not host.enabled


def test_host_jpeg_flags_a_stuck_pi_encoder(tmp_path):
    clock, logs = Clock(1000.0), []
    service, host, _ = host_service(tmp_path, clock, logs)
    host.threaded, host._job = True, ("stuck", None)  # the worker took a frame and never finished it
    host._thread = object()
    depth_stream.touch_lease(tmp_path)
    for n in range(25):
        os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
        service.tick(np.zeros((4, 32), np.uint16), n)
        clock.t += 0.2
    meta = depth_stream.read_meta(tmp_path)
    assert meta["encoder_stalled"] is True and meta["last_error"]["variant"] == "host_jpeg"
    assert "the Pi" in meta["last_error"]["text"] and meta["host_encoder"]["skipped_busy"] >= 20


def test_auto_falls_back_to_the_pi_when_the_camera_encoder_stalls(tmp_path):
    clock, logs = Clock(1000.0), []
    debug_device = FakeLogDevice()
    debug = depth_stream.DeviceLogDebug(debug_device, "DEBUG", log=logs.append)
    service, host, controls = host_service(
        tmp_path, clock, logs, info={"fps_target": 5.0, "variant": "manip", "requested_variant": "auto"},
        debug=debug)
    assert not service.on_host and service.meta()["encoded_on"] == "camera"
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    assert controls == [True] and debug_device.level == "DEBUG" and not host.enabled
    for sent in (5, 10, 15, 20, 25):  # the camera Script sends, the encoder returns nothing
        clock.t += 1.0
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
    meta = depth_stream.read_meta(tmp_path)
    assert controls == [True, False] and debug_device.level == "WARN"  # camera off, debug level restored
    assert meta["variant"] == "host_jpeg" and meta["requested_variant"] == "auto" and meta["encoded_on"] == "pi"
    assert meta["fallback"]["from"] == "manip" and meta["fallback"]["to"] == "host_jpeg"
    assert depth_stream.ENCODER_STALL_TEXT in meta["fallback"]["reason"]
    assert meta["encoder_stalled"] is False and meta["last_error"]["variant"] == "manip"
    assert any("falling back from manip" in line for line in logs)
    assert host.enabled
    # From now on the frames come from the Pi; late camera frames and reports are ignored
    assert service.on_frame(b"\xff\xd8camera", 1) is False
    service.on_device_status(b"error\t3\t30\tlate")
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    clock.t += 0.6
    service.tick(np.full((40, 64), 1500, np.uint16), 7)
    assert (tmp_path / depth_stream.FRAME_FILE).read_bytes() == b"\xff\xd8host"
    assert service.meta()["errors"] == 0 and service.meta()["last_frame"]["sequence"] == 7


def test_a_fixed_camera_variant_does_not_fall_back(tmp_path):
    clock, logs = Clock(1000.0), []
    service = depth_stream.DepthStreamService(tmp_path, lambda on: None,
                                              {"fps_target": 5.0, "variant": "nv12", "requested_variant": "nv12"},
                                              clock=clock, log=logs.append)
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    for sent in (5, 10, 15, 20, 25):
        clock.t += 1.0
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
    meta = service.meta()
    assert meta["encoder_stalled"] is True and meta["variant"] == "nv12" and meta["fallback"] is None
    assert "variant nv12" in meta["last_error"]["text"] and meta["last_error"]["stall"] is True


# --- depth_stream_debug: the camera's log ---------------------------------------------------------

class FakeLogDevice:
    def __init__(self, level="WARN", fail=False):
        self.level, self.fail, self.calls = level, fail, []

    def getLogLevel(self):
        return self.level

    def setLogLevel(self, level):
        self.calls.append(level)
        if self.fail:
            raise RuntimeError("device gone")
        self.level = level


class LogMessage:
    def __init__(self, node, payload, level="DEBUG"):
        self.nodeIdName, self.payload, self.level = node, payload, level


def test_device_log_keeps_the_stream_nodes_lines_bounded():
    clock = Clock(5.0)
    log = depth_stream.DeviceLog(("Script(12)",), clock=clock, max_lines=3, other_lines=2)
    log.on_message(LogMessage("VideoEncoder(13)", "VideoEncoder took '4' ms."))
    log.on_message(LogMessage("Script(12)", "depth stream: convert + send mean 40 ms"))
    log.on_message(LogMessage("StereoDepth(3)", "unrelated"))
    log.on_message(LogMessage("ImageManip(14)", "x" * 1000, "WARN"))
    log.on_message(LogMessage("Script(2)", "the snapshot gate"))
    log.on_message(LogMessage("Camera(0)", "third other"))
    snap = log.snapshot()
    assert snap["total"] == 6
    assert [line["node"] for line in snap["stream"]] == ["VideoEncoder(13)", "Script(12)", "ImageManip(14)"]
    assert len(snap["stream"][-1]["text"]) == depth_stream.DEVICE_LOG_MAX_CHARS
    assert snap["stream"][0] == {"unix_time": 5.0, "node": "VideoEncoder(13)", "level": "DEBUG",
                                 "text": "VideoEncoder took '4' ms."}
    assert [line["node"] for line in snap["other"]] == ["Script(2)", "Camera(0)"]  # bounded: the newest two
    log.on_message(LogMessage("VideoEncoder(13)", "more"))
    assert [line["text"] for line in log.snapshot()["stream"]][-1] == "more" and len(log.snapshot()["stream"]) == 3
    json.dumps(log.snapshot())


def test_device_log_never_raises_into_depthai():
    class Broken:
        @property
        def payload(self):
            raise RuntimeError("bad message")

    log = depth_stream.DeviceLog()
    log.on_message(Broken())
    assert log.snapshot() == {"stream": [], "other": [], "total": 0}
    log.on_message(None)
    assert log.snapshot()["total"] == 1 and log.snapshot()["stream"] == []


def test_device_log_debug_raises_and_restores_the_level():
    device, logs = FakeLogDevice("WARN"), []
    debug = depth_stream.DeviceLogDebug(device, "DEBUG", log=logs.append)
    debug.start()
    debug.start()
    assert device.calls == ["DEBUG"] and debug.state() == {"active": True, "level": "DEBUG",
                                                           "restore_level": "WARN", "error": None}
    debug.stop()
    debug.stop()
    assert device.calls == ["DEBUG", "WARN"] and device.level == "WARN" and not debug.active
    assert logs == ["Depth stream debug: device log level WARN -> DEBUG",
                    "Depth stream debug: device log level back to WARN"]

    broken = depth_stream.DeviceLogDebug(FakeLogDevice(fail=True), "DEBUG", log=logs.append)
    broken.start()
    assert not broken.active and "device gone" in broken.state()["error"]


def test_service_raises_the_log_level_only_while_streaming(tmp_path):
    clock, logs = Clock(1000.0), []
    device = FakeLogDevice("INFO")
    device_log = depth_stream.DeviceLog(("Script(12)",), clock=clock)
    service = depth_stream.DepthStreamService(
        tmp_path, lambda on: None, {"variant": "manip", "requested_variant": "manip"}, clock=clock,
        log=logs.append, device_log=device_log, debug=depth_stream.DeviceLogDebug(device, "DEBUG", log=logs.append))
    service.tick()
    assert device.calls == []
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    clock.t += 0.6
    service.tick()
    assert device.level == "DEBUG"
    device_log.on_message(LogMessage("VideoEncoder(13)", "VideoEncoder took '4' ms."))
    clock.t += 1.1
    service.tick()
    meta = depth_stream.read_meta(tmp_path)
    assert meta["debug"] is True and meta["debug_state"]["active"] is True
    assert meta["device_log"]["stream"][0]["text"] == "VideoEncoder took '4' ms."
    clock.t += depth_stream.LEASE_S + 1
    service.tick()
    assert device.level == "INFO" and depth_stream.read_meta(tmp_path)["debug_state"]["active"] is False
    service.close()
    assert device.calls == ["DEBUG", "INFO"]
