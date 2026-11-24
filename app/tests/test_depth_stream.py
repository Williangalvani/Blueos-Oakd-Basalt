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


# Frames of StereoDepth's outputs come from its pool (numFramesPool, 3 + extras); a message the Script makes
# itself has none (test12)
STEREO_POOL = 5
# ImgFrame methods of the Script runtime in the RVC2 firmware (depthai 3.3.0, device side a406121b);
# a subset of the host's dai.ImgFrame. Its constructor is ImgFrame(size in bytes).
DEVICE_IMGFRAME_API = frozenset({
    "getData", "setData", "getWidth", "setWidth", "getHeight", "setHeight", "setSize", "getType", "setType",
    "getTimestamp", "getTimestampDevice", "setTimestamp", "getSequenceNum", "setSequenceNum",
    "getInstanceNum", "setInstanceNum", "getCategory", "setCategory", "getTransformation", "setTransformation",
})
# Frame types the RVC2 VideoEncoder takes; it drops others with "Arrived frame type (N) is not either NV12 or
# YUV400p (8-bit Gray)" (GRAY8 is N = 30 on test10, RAW16 N = 14 on test14). RAW8 from StereoDepth's pool
# (the disparity, rectifiedLeft/Right) encodes too (test14).
ENCODER_INPUT_TYPES = frozenset({"NV12", "YUV400p", "RAW8"})


class DepthFrame:
    """A stereo.depth frame (RAW16 mm) as the Script receives it: from StereoDepth's pool."""
    pool = STEREO_POOL

    def __init__(self, depth, sequence, padding=0):
        self.depth, self.sequence = depth, sequence
        rows = depth.astype("<u2").view(np.uint8).reshape(depth.shape[0], -1)
        self.data = np.pad(rows, ((0, 0), (0, padding))).tobytes()

    def getType(self):
        return ImgFrame.Type.RAW16

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


class DisparityFrame(DepthFrame):
    """
    A RAW8 stereo.disparity frame (max disparity <= 255: subpixel and extended off, the median last; test14
    cases B-D, F), disparity in px, from StereoDepth's pool.
    """

    def __init__(self, disparity, sequence):
        self.depth, self.sequence = disparity, sequence
        self.data = np.asarray(disparity, np.uint8).tobytes()

    def getType(self):
        return ImgFrame.Type.RAW8


class RectifiedFrame(DisparityFrame):
    """A stereo.rectifiedLeft frame (RAW8 camera image, the FeatureTracker's input), from StereoDepth's pool."""


class Raw16DisparityFrame(DepthFrame):
    """
    A RAW16 stereo.disparity frame (test14 case A, the saved settings with subpixel off: the speckle filter
    runs after the median, so getMaxDisparity is 8192 and the frame 16-bit), from StereoDepth's pool.
    """


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
        self.pool = 0  # firmware side, not in the Script API

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
    API = DEVICE_IMGFRAME_API

    class Type(enum.IntEnum):
        """The values the device logs (GRAY8 is 'frame type (30)' in the VideoEncoder warning on test10)."""
        YUV400p = 4
        RAW16 = 14
        RAW8 = 18
        NV12 = 22
        GRAY8 = 30


class FakeVideoEncoder(FakeOutput):
    """
    The camera's VideoEncoder (PlgVidEnc in the RVC2 firmware) at the end of the stream: it takes NV12,
    YUV400p and RAW8 (test14: StereoDepth's RAW8 disparity and rectified images encode directly) and drops
    anything else with the firmware's warning, once per frame (test10: GRAY8 frames sent, nothing encoded;
    test14: RAW16, "frame type (14)"). The first frame sets the encoder size.
    """

    def __init__(self):
        super().__init__()
        self.encoded = []
        self.warnings = []
        self.size = None

    def send(self, frame):
        super().send(frame)
        kind, width, height = frame.getType(), frame.getWidth(), frame.getHeight()
        if kind not in {getattr(ImgFrame.Type, name) for name in ENCODER_INPUT_TYPES}:
            self.warnings.append(f"[VideoEncoder(11)] [warning] Arrived frame type ({int(kind)}) is not either NV12 "
                                 f"or YUV400p (8-bit Gray)")
            return
        assert frame.pool >= 1
        self.size = self.size or (width, height)
        if (width, height) != self.size:
            self.warnings.append(f"Arrived frame ({width}, {height}) does not match the encoder size {self.size}")
            return
        assert len(bytes(frame.getData())) == width * height, "wrong plane size for RAW8"
        self.encoded.append(frame)


def test_the_fake_device_messages_reject_what_the_camera_rejects():
    with pytest.raises(TypeError, match="incompatible constructor"):
        Buffer()
    with pytest.raises(AttributeError):
        Buffer(4).setStride(2)
    with pytest.raises(RuntimeError, match="too small"):
        Buffer(4).setData(b"12345")


def test_the_fake_encoder_takes_pooled_raw8_and_drops_raw16_with_a_warning():
    """test14: RAW8 disparity and rectifiedLeft frames from StereoDepth's pool encode; RAW16 gets a warning."""
    encoder = FakeVideoEncoder()
    encoder.send(DisparityFrame(disparity_image(0), 0))
    encoder.send(RectifiedFrame(mono_image(1), 1))
    assert len(encoder.encoded) == 2 and encoder.warnings == []
    encoder.send(Raw16DisparityFrame(depth_image(2, 40, 64), 2))
    assert len(encoder.encoded) == 2
    assert encoder.warnings == ["[VideoEncoder(11)] [warning] Arrived frame type (14) is not either NV12 or "
                                "YUV400p (8-bit Gray)"]


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


def disparity_image(seed=0, height=40, width=64):
    disparity = np.random.default_rng(seed).integers(8, 96, (height, width)).astype(np.uint8)
    disparity[:, :4] = 0  # the left edge band has no depth
    return disparity


def mono_image(seed=0, height=40, width=64):
    return np.random.default_rng(seed).integers(0, 256, (height, width)).astype(np.uint8)


def mono_frames(n, height=40, width=64):
    return [RectifiedFrame(mono_image(k, height, width), k) for k in range(n)]


class RefusingEncoder(FakeVideoEncoder):
    """An encoder input that refuses the frames whose sequence number `refuse` accepts."""

    def __init__(self, refuse):
        super().__init__()
        self.refuse = refuse

    def send(self, frame):
        if self.refuse(frame.getSequenceNum()):
            raise RuntimeError(f"queue full {frame.getSequenceNum()}")
        super().send(frame)


def encoded_y(encoder):
    """The images the fake encoder took."""
    return [np.frombuffer(bytes(f.getData()), np.uint8).reshape(f.getHeight(), f.getWidth()) for f in encoder.encoded]


@pytest.mark.parametrize("make", [lambda n: DisparityFrame(disparity_image(n), n),
                                  lambda n: RectifiedFrame(mono_image(n), n)], ids=["disparity", "mono_features"])
def test_the_gate_sends_every_nth_stereodepth_raw8_frame_straight_to_the_encoder(make):
    """The Script only gates; the encoder gets the pooled StereoDepth frame itself."""
    frames = [make(n) for n in range(6)]
    encoder = FakeVideoEncoder()
    node = run_script(depth_stream.device_script(2), frames, [Control(True)], gray=encoder)
    # The first frame goes out right away, then every 2nd
    assert [f.getSequenceNum() for f in encoder.sent] == [0, 2, 4]
    assert all(sent is frames[sent.sequence] for sent in encoder.sent)  # the same message: its pool and metadata
    assert encoder.warnings == [] and len(encoder.encoded) == 3
    for y, sent in zip(encoded_y(encoder), encoder.sent):
        np.testing.assert_array_equal(y, sent.depth)  # the data as it is: disparity px or the camera's gray
    assert node.warnings == [] and errors_reported(node) == []
    assert reports(node)[0] == {"kind": "sent", "errors": 0, "sent": 1, "text": ""}


def test_a_raw16_disparity_through_the_gate_encodes_nothing():
    """Why auto never picks disparity with the saved settings: the RAW16 frames get the encoder's warning."""
    frames = [Raw16DisparityFrame(depth_image(n, 40, 64), n) for n in range(3)]
    encoder = FakeVideoEncoder()
    run_script(depth_stream.device_script(1), frames, [Control(True)], gray=encoder)
    assert len(encoder.sent) == 3 and encoder.encoded == []
    assert len(encoder.warnings) == 3 and "frame type (14)" in encoder.warnings[0]


def test_the_disparity_gate_waits_for_the_control():
    frames = [DisparityFrame(disparity_image(n), n) for n in range(4)]
    encoder = FakeVideoEncoder()
    node = run_script(depth_stream.device_script(1), frames, [Control(False), Control(True)], gray=encoder)
    assert len(encoder.encoded) == 4 and [r["sent"] for r in reports(node) if r["kind"] == "sent"][0] == 1


def test_script_reports_frames_sent_about_once_a_second():
    t = [100.0]

    def clock():  # read twice per frame: 0.125 s per frame
        t[0] += 0.0625
        return t[0]

    node = run_script(depth_stream.device_script(1), mono_frames(25), [Control(True)], clock=clock)
    progress = [r["sent"] for r in reports(node) if r["kind"] == "sent"]
    assert progress == [1, 9, 17, 25]
    assert len(node.outputs["gray"].sent) == 25


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
    code = depth_stream.device_script(3)
    json.dumps(code)
    constructors, methods = script_calls(code)
    assert "ImgFrame" not in constructors  # no new frame: the received one goes on
    (buffer,) = constructors["Buffer"]
    assert len(buffer.args) == 1 and ast.unparse(buffer.args[0]) == "len(data)"
    frame_methods = {method for obj, method in methods if obj == "frame"}
    assert frame_methods <= DEVICE_IMGFRAME_API and {"getWidth", "getHeight"} <= frame_methods
    assert {method for name, method in methods if name == "message"} == {"setData"}
    for missing_on_device in ("setStride", "setMetadata", "setTimestampDevice", "setFrame", "setCvFrame"):
        assert missing_on_device not in code


def test_script_reports_errors_on_its_status_output():
    encoder = RefusingEncoder(lambda sequence: sequence == 0)
    node = run_script(depth_stream.device_script(1), mono_frames(2), [Control(True)], gray=encoder)
    assert [f.getSequenceNum() for f in encoder.sent] == [1]  # keeps going
    assert len(node.warnings) == 1 and "queue full 0" in node.warnings[0]
    (report,) = errors_reported(node)
    assert report["kind"] == "error" and report["errors"] == 1 and report["sent"] == 0
    assert report["text"] == "send: RuntimeError: queue full 0"


def test_script_error_reports_are_throttled():
    t = [100.0]

    def clock():  # read twice per frame: 10 ms per frame
        t[0] += 0.005
        return t[0]

    node = run_script(depth_stream.device_script(1), mono_frames(250, 8, 32), [Control(True)], clock=clock,
                      gray=RefusingEncoder(lambda sequence: True))
    # 250 errors over 2.5 s: the first right away, then about one a second, with the running count
    counts = [r["errors"] for r in reports(node)]
    assert counts[0] == 1 and len(counts) == 3 and counts == sorted(counts) and counts[-1] <= 250
    assert reports(node)[-1]["text"].startswith("send: RuntimeError: queue full ")
    assert len(node.warnings) == 3  # the 1st, 100th and 200th


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
        exec(depth_stream.device_script(1), {"node": node, "ImgFrame": ImgFrame, "Buffer": Buffer})
    (report,) = reports(node)
    assert report["kind"] == "fatal" and report["text"] == "stopped: RuntimeError: input closed"
    assert "input closed" in node.warnings[0]


def test_script_status_survives_a_failing_status_output():
    class Refusing(FakeOutput):
        def send(self, message):
            raise RuntimeError("queue full")

    node = type("Node", (), {})()
    node.inputs = {"depth": FakeInput(mono_frames(2, 8, 32)), "control": FakeInput([Control(True)])}
    node.outputs = {"gray": RefusingEncoder(lambda sequence: sequence == 0), "status": Refusing()}
    node.warnings = []
    node.warn = node.warnings.append
    with pytest.raises(Done):
        exec(depth_stream.device_script(1), {"node": node, "ImgFrame": ImgFrame, "Buffer": Buffer})
    assert len(node.outputs["gray"].sent) == 1 and "status not sent: queue full" in node.warnings[-1]


def test_script_waits_for_the_host_and_stops_when_told():
    frames = mono_frames(10)
    control = FakeInput([Control(False), Control(True)])

    def stop_after_frame_4(frame):
        if frame.sequence == 4:
            control.items.append(Control(False))

    node = run_script(depth_stream.device_script(2), frames, control, after=stop_after_frame_4)
    assert len(node.inputs["depth"].items) == 5  # never read past the stop
    # Off until enabled, sends frames 0, 2, 4, then blocks on the control input again once disabled
    assert [f.getSequenceNum() for f in node.outputs["gray"].sent] == [0, 2, 4]


def test_script_reports_its_cost():
    node = run_script(depth_stream.device_script(1, stats_every=2), mono_frames(4), [Control(True)])
    assert len(node.warnings) == 2 and "convert + send mean" in node.warnings[0] and "64x40" in node.warnings[0]
    stats = [r for r in reports(node) if r["kind"] == "stats"]
    assert [(r["kind"], r["sent"], r["errors"], r["size"]) for r in stats] == [("stats", 2, 0, [64, 40]),
                                                                               ("stats", 4, 0, [64, 40])]
    assert stats[0]["convert_ms_max"] >= stats[0]["convert_ms_mean"] >= 0.0


def test_manip_output_size_matches_depthai():
    # 640x400 NV12: Y 640 x 416 rows, UV 640 x 208 rows, each plane rounded up to 4096 bytes
    assert depth_stream.manip_output_size(640, 400) == 401408
    assert depth_stream.manip_output_size(640, 400) >= 640 * 400 * 3 // 2


def test_size_problem():
    assert depth_stream.size_problem(640, 400) is None and depth_stream.size_problem(1280, 800) is None
    assert "multiple of 32" in depth_stream.size_problem(600, 400)
    assert depth_stream.size_problem(640, 401) is not None


def test_device_variant():
    assert depth_stream.device_variant("auto") == "mono_features"
    assert [depth_stream.device_variant(v) for v in ("mono_features", "disparity")] == ["mono_features", "disparity"]
    assert depth_stream.device_variant("host_jpeg") is None and depth_stream.device_variant("bogus") is None
    assert depth_stream.VARIANTS == ("auto", "mono_features", "disparity", "host_jpeg")
    assert depth_stream.CAMERA_VARIANTS == ("mono_features", "disparity")


def saved_disparity_format(**changes):
    """stereo_config.disparity_format of the default settings with changes, nothing read back from a camera."""
    import stereo_config
    return stereo_config.disparity_format(None, stereo_config.resolve_stereo_settings(changes))


def test_disparity_problem_follows_the_8192_rule():
    # The saved defaults (speckle on, after the median): RAW16, with the reason and what to change
    problem = depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=False))
    assert problem.startswith("disparity is RAW16 (max disparity 8192) with the speckle filter on (it runs after "
                              "the median); turn the speckle filter off") and problem.endswith("or use mono_features")
    assert "stereo_median_last" in problem
    # RAW8: speckle off, or the median moved last (probe cases B and G)
    assert depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=False,
                                                                 stereo_speckle_filter=False)) is None
    assert depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=False,
                                                                 stereo_median_last=True)) is None
    assert "Subpixel on" in depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=True,
                                                                                  stereo_speckle_filter=False))
    assert depth_stream.disparity_problem(None) == "the disparity format is unknown"


def test_resolve_variant_starts_with_mono_features_and_needs_raw8_for_disparity():
    resolve = depth_stream.resolve_variant
    raw16 = depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=False))
    # auto: mono_features whatever the disparity format; never the disparity
    assert resolve("auto") == ("mono_features", None)
    assert resolve("auto", raw16) == ("mono_features", None)
    assert resolve("auto", raw16, problem="600x400: ...") == (None, "600x400: ...")
    # An explicit disparity request: only with a RAW8 disparity, else host_jpeg with the reason
    assert resolve("disparity") == ("disparity", None)
    assert resolve("disparity", raw16) == (None, raw16)
    # An explicit mono_features is built as asked (a size problem is only logged)
    assert resolve("mono_features", raw16, problem="600x400: ...") == ("mono_features", None)
    assert resolve("host_jpeg") == (None, None)


def test_encoding_and_source_of_each_variant():
    assert depth_stream.encoding_for("disparity") == "gray8-disparity"
    assert depth_stream.encoding_for("mono_features") == "gray8-mono-rectified-left"
    assert {depth_stream.encoding_for(v) for v in ("host_jpeg", None)} == {"gray8-depth-linear"}
    assert depth_stream.source_for("mono_features") == "stereo.rectifiedLeft"
    assert depth_stream.source_for("disparity") == "stereo.disparity"
    assert depth_stream.source_for("host_jpeg") == "stereo.depth"
    assert depth_stream.source_for(None) is None


def test_disparity_scale_and_reference():
    scale = depth_stream.disparity_scale(440.0, 0.075, 300, 4000)
    assert scale["depth_times_disparity"] == 33000.0 and scale["max_disparity"] == 95
    assert scale["gray_far"] == 8.25 and scale["gray_near"] == 95.0 and scale["nearest_mm"] == 347
    assert depth_stream.disparity_scale(0.0, 0.075, 300, 4000) is None
    depth = np.array([0, 200, 347, 1000, 4000, 20000], np.uint16)
    assert depth_stream.gray_disparity_reference(depth, 440.0, 0.075).tolist() == [0, 95, 95, 33, 8, 2]


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
    json.dumps(depth_stream.device_script(3))


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
        tmp_path, clock, logs, info={"fps_target": 5.0, "variant": "mono_features", "requested_variant": "auto"},
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
    assert meta["fallback"]["from"] == "mono_features" and meta["fallback"]["to"] == "host_jpeg"
    assert meta["fallback"]["reason"] == "no JPEG from mono_features (the camera encoder outputs nothing)"
    assert depth_stream.ENCODER_STALL_TEXT in meta["fallback"]["detail"]
    assert meta["encoder_stalled"] is False and meta["last_error"]["variant"] == "mono_features"
    assert any("falling back from mono_features" in line for line in logs)
    assert host.enabled
    # From now on the frames come from the Pi; late camera frames and reports are ignored
    assert service.on_frame(b"\xff\xd8camera", 1) is False
    service.on_device_status(b"error\t3\t30\tlate")
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    clock.t += 0.6
    service.tick(np.full((40, 64), 1500, np.uint16), 7)
    assert (tmp_path / depth_stream.FRAME_FILE).read_bytes() == b"\xff\xd8host"
    assert service.meta()["errors"] == 0 and service.meta()["last_frame"]["sequence"] == 7


# --- blank frames ---------------------------------------------------------------------------------

def camera_jpegs(kind, n, height=40, width=64):
    """
    JPEGs as the strict fakes' camera makes them through the gate: disparity and mono_features (pooled
    StereoDepth frames) vary; blank is a constant camera image (test12: every JPEG the same, all pixels 1).
    """
    import cv2
    encoder = FakeVideoEncoder()
    if kind == "disparity":
        frames = [DisparityFrame(disparity_image(k, height, width), k) for k in range(n)]
    elif kind == "blank":
        frames = [RectifiedFrame(np.ones((height, width), np.uint8), k) for k in range(n)]
    else:
        frames = mono_frames(n, height, width)
    run_script(depth_stream.device_script(1), frames, [Control(True)], gray=encoder)
    return [cv2.imencode(".jpg", y, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes() for y in encoded_y(encoder)]


def blank_service(tmp_path, clock, logs, variant="mono_features", requested="auto"):
    info = {"fps_target": 5.0, "variant": variant, "requested_variant": requested, "depth_range_mm": [300, 4000]}
    service, host, controls = host_service(tmp_path, clock, logs, info=info)
    depth_stream.touch_lease(tmp_path)
    return service, host, controls


def stream_for(service, tmp_path, clock, jpegs, depth, seconds, step=0.2):
    """Camera frames at 5 fps with the host's depth each frame; returns when (s after the start) it fell back."""
    start, n = clock.t, 0
    while clock.t - start < seconds:
        os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
        service.tick(depth, n)
        if service.on_host:
            return clock.t - start
        service.on_frame(jpegs[n % len(jpegs)], n)
        n += 1
        clock.t += step
    return None


def test_auto_falls_back_on_blank_frames_within_a_few_seconds(tmp_path):
    clock, logs = Clock(1000.0), []
    service, host, controls = blank_service(tmp_path, clock, logs)
    jpegs = camera_jpegs("blank", 3)
    assert len(set(jpegs)) == 1  # byte-identical, as on test12 (4668 bytes each at 640x400)
    after = stream_for(service, tmp_path, clock, jpegs, depth_image(1, 40, 64), 6.0)
    assert after is not None and after <= depth_stream.BLANK_S + 1.0
    meta = depth_stream.read_meta(tmp_path)
    assert (meta["fallback"]["reason"] == "blank frames from mono_features"
            and meta["fallback"]["from"] == "mono_features")
    assert "constant JPEGs" in meta["fallback"]["detail"] and "gray 1" in meta["fallback"]["detail"]
    assert meta["blank"]["variant"] == "mono_features" and meta["last_error"]["blank"] is True
    assert meta["variant"] == "host_jpeg" and meta["encoded_on"] == "pi" and meta["encoding"] == "gray8-depth-linear"
    assert controls == [True, False] and host.enabled
    assert any("falling back from mono_features" in line and "blank frames from mono_features" in line
               for line in logs)


@pytest.mark.parametrize("variant, encoding", [("disparity", "gray8-disparity"),
                                               ("mono_features", "gray8-mono-rectified-left")])
def test_camera_frames_are_not_blank(tmp_path, variant, encoding):
    clock, logs = Clock(1000.0), []
    service, host, controls = blank_service(tmp_path, clock, logs, variant=variant)
    jpegs = camera_jpegs(variant, 6)
    assert len(set(jpegs)) == 6
    assert stream_for(service, tmp_path, clock, jpegs, depth_image(1, 40, 64), 8.0) is None
    meta = service.meta()
    assert meta["blank"] is None and meta["fallback"] is None and meta["variant"] == variant
    assert meta["encoding"] == encoding and meta["frames_total"] == 40


def test_constant_frames_of_a_view_without_depth_are_not_blank(tmp_path):
    """A covered lens or open water: no depth anywhere, so a constant image is right."""
    import cv2
    clock, logs = Clock(1000.0), []
    service, host, controls = blank_service(tmp_path, clock, logs)
    black = cv2.imencode(".jpg", np.zeros((40, 64), np.uint8))[1].tobytes()
    assert stream_for(service, tmp_path, clock, [black], np.zeros((40, 64), np.uint16), 6.0) is None
    assert service.blank is None and service.fallback is None
    # A flat wall filling the view (all valid, one depth): constant too, and not blank either
    wall = np.full((40, 64), 1500, np.uint16)
    gray = cv2.imencode(".jpg", depth_stream.gray_reference(wall, 300, 4000))[1].tobytes()
    assert stream_for(service, tmp_path, clock, [gray], wall, 6.0) is None and service.blank is None


def test_a_requested_variant_reports_blank_frames_without_falling_back_and_recovers(tmp_path):
    clock, logs = Clock(1000.0), []
    service, host, controls = blank_service(tmp_path, clock, logs, requested="mono_features")
    depth = depth_image(1, 40, 64)
    assert stream_for(service, tmp_path, clock, camera_jpegs("blank", 2), depth, 4.0) is None
    meta = service.meta()
    assert meta["blank"]["text"].startswith("blank frames from mono_features") and meta["fallback"] is None
    assert meta["variant"] == "mono_features" and meta["encoded_on"] == "camera"
    # Real frames again (e.g. the camera recovered): no longer blank
    clock.t += 1.0
    stream_for(service, tmp_path, clock, camera_jpegs("mono_features", 3), depth, 1.0)
    assert service.blank is None and any("not blank any more" in line for line in logs)


def test_empty_frames_are_not_frames_and_still_stall(tmp_path):
    clock, logs = Clock(1000.0), []
    service, host, controls = blank_service(tmp_path, clock, logs, variant="disparity")
    depth_stream.touch_lease(tmp_path)
    service.tick()
    for sent in (5, 10, 15, 20, 25):
        clock.t += 1.0
        assert service.on_frame(b"", sent) is False
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
    meta = depth_stream.read_meta(tmp_path)
    assert meta["bad_frames"] == 5 and meta["frames_total"] == 0
    assert meta["fallback"]["reason"] == "no JPEG from disparity (the camera encoder outputs nothing)"
    assert any("were not JPEGs" in line for line in logs)


def test_a_raw16_disparity_request_starts_on_the_pi_and_says_why(tmp_path):
    clock, logs = Clock(1000.0), []
    reason = depth_stream.disparity_problem(saved_disparity_format(stereo_subpixel=False))
    info = {"fps_target": 5.0, "variant": "host_jpeg", "requested_variant": "disparity", "variant_reason": reason}
    service, host, controls = host_service(tmp_path, clock, logs, info=info)
    meta = service.meta()
    assert meta["fallback"] == {"from": "disparity", "to": "host_jpeg", "reason": reason,
                                "detail": None, "unix_time": 1000.0, "at_start": True}
    assert "speckle filter on (it runs after the median)" in reason
    assert meta["source"] == "stereo.depth" and meta["encoding"] == "gray8-depth-linear"
    assert service.on_host and any("disparity requested, encoding on the Pi" in line for line in logs)
    # host_jpeg asked for: nothing to explain
    plain, _, _ = host_service(tmp_path / "plain", clock, logs,
                               info={"variant": "host_jpeg", "requested_variant": "host_jpeg", "variant_reason": None})
    assert plain.fallback is None


def test_a_fixed_camera_variant_does_not_fall_back(tmp_path):
    clock, logs = Clock(1000.0), []
    service = depth_stream.DepthStreamService(tmp_path, lambda on: None,
                                              {"fps_target": 5.0, "variant": "mono_features",
                                               "requested_variant": "mono_features"},
                                              clock=clock, log=logs.append)
    depth_stream.touch_lease(tmp_path)
    os.utime(tmp_path / depth_stream.LEASE_FILE, (clock.t, clock.t))
    service.tick()
    for sent in (5, 10, 15, 20, 25):
        clock.t += 1.0
        service.on_device_status(f"sent\t0\t{sent}\t".encode())
    meta = service.meta()
    assert meta["encoder_stalled"] is True and meta["variant"] == "mono_features" and meta["fallback"] is None
    assert "variant mono_features" in meta["last_error"]["text"] and meta["last_error"]["stall"] is True


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
    log.on_message(LogMessage("VideoEncoder(13)", "x" * 1000, "WARN"))
    log.on_message(LogMessage("Script(2)", "the snapshot gate"))
    log.on_message(LogMessage("Camera(0)", "third other"))
    snap = log.snapshot()
    assert snap["total"] == 6
    assert [line["node"] for line in snap["stream"]] == ["VideoEncoder(13)", "Script(12)", "VideoEncoder(13)"]
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
        tmp_path, lambda on: None, {"variant": "mono_features", "requested_variant": "mono_features"}, clock=clock,
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
