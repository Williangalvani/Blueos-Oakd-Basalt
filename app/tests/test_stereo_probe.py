import enum
import json
import os
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import stereo_probe  # noqa: E402
import vio_log  # noqa: E402
from stereo_config import STEREO_DEFAULTS, resolve_stereo_settings  # noqa: E402

SAVED = resolve_stereo_settings(STEREO_DEFAULTS)


# --- pure helpers ---------------------------------------------------------------------------------------

def test_case_names():
    assert stereo_probe.parse_case_names(None) == ["A", "B", "C", "D", "E", "F"]
    assert stereo_probe.parse_case_names(" ") == ["A", "B", "C", "D", "E", "F"]
    assert stereo_probe.parse_case_names("e, a,E") == ["E", "A"]
    with pytest.raises(ValueError, match="unknown case"):
        stereo_probe.parse_case_names("A,Z")


def test_case_settings():
    by_name = {case.name: stereo_probe.case_settings(SAVED, case) for case in stereo_probe.CASES}
    a = by_name["A"]
    assert a == {**SAVED, "stereo_subpixel": False}
    assert by_name["B"] == {**a, "stereo_speckle_filter": False}
    for name in "CDF":
        s = by_name[name]
        assert not (s["stereo_speckle_filter"] or s["stereo_spatial_filter"] or s["stereo_temporal_filter"])
        assert s["stereo_decimation"] == 1 and s["stereo_subpixel"] is False
    assert by_name["C"]["stereo_median"] == "off" and by_name["F"]["stereo_median"] == "off"
    assert by_name["D"]["stereo_median"] == "7x7"
    assert by_name["E"]["stereo_subpixel"] is True and by_name["E"]["stereo_preset"] == "ROBOTICS"
    assert by_name["F"]["stereo_preset"] == "NONE"
    # D keeps a saved median other than off, and picks 7x7 when the saved one is off
    d = stereo_probe.CASES_BY_NAME["D"]
    assert stereo_probe.case_settings({**SAVED, "stereo_median": "5x5"}, d)["stereo_median"] == "5x5"
    assert stereo_probe.case_settings({**SAVED, "stereo_median": "off"}, d)["stereo_median"] == "7x7"
    # A and E also encode the rectified images; the others the disparity only
    assert [c.name for c in stereo_probe.CASES if len(c.encode_sources) == 3] == ["A", "E"]
    assert all(c.encode_sources[0] == "disparity" for c in stereo_probe.CASES)


def readback(**changes):
    rb = {"enableSubpixel": False, "subpixelFractionalBits": 3, "enableExtended": False, "disparityShift": 0,
          "disparityWidth": "DISPARITY_96", "enableCompanding": False, "median": "KERNEL_7x7",
          "speckleFilter": True, "spatialFilter": False, "temporalFilter": False, "decimationFactor": 1,
          "filteringOrder": ["MEDIAN", "DECIMATION", "SPECKLE", "SPATIAL", "TEMPORAL"]}
    rb.update(changes)
    return rb


@pytest.mark.parametrize("changes, filters, expected", [
    ({}, ["MEDIAN", "SPECKLE"], 8192.0),  # test13: speckle after median -> 13-bit disparity
    ({"speckleFilter": False}, ["MEDIAN"], 95.0),
    ({"speckleFilter": False, "median": "MEDIAN_OFF"}, [], 95.0),
    ({"enableSubpixel": True, "speckleFilter": False}, ["MEDIAN"], 760.0),
    ({"enableSubpixel": True}, ["MEDIAN", "SPECKLE"], 8192.0),
    ({"speckleFilter": False, "median": "MEDIAN_OFF", "decimationFactor": 2}, ["DECIMATION"], 8192.0),
    ({"filteringOrder": ["SPECKLE", "MEDIAN", "DECIMATION", "SPATIAL", "TEMPORAL"]}, ["SPECKLE", "MEDIAN"], 95.0),
    ({"speckleFilter": False, "disparityWidth": "DISPARITY_64"}, ["MEDIAN"], 63.0),
    ({"speckleFilter": False, "enableCompanding": True}, ["MEDIAN"], 175.0),
    ({"speckleFilter": False, "enableExtended": True, "disparityShift": 5}, ["MEDIAN"], 200.0),
])
def test_max_disparity_follows_depthai(changes, filters, expected):
    rb = readback(**changes)
    assert stereo_probe.executed_filters(rb) == filters
    assert stereo_probe.predicted_max_disparity(rb) == pytest.approx(expected)


def test_max_disparity_needs_a_full_read_back():
    assert stereo_probe.predicted_max_disparity({}) is None
    assert stereo_probe.predicted_max_disparity(readback(filteringOrder="unavailable (x)")) is None
    assert stereo_probe.predicted_max_disparity(readback(subpixelFractionalBits="unavailable (x)",
                                                         enableSubpixel=True)) is None


Median = enum.Enum("MedianFilter", "MEDIAN_OFF KERNEL_3x3 KERNEL_5x5 KERNEL_7x7")
Filter = enum.Enum("Filter", "MEDIAN DECIMATION SPECKLE SPATIAL TEMPORAL")
DisparityWidth = enum.Enum("DisparityWidth", "DISPARITY_64 DISPARITY_96")
DepthAlign = enum.Enum("DepthAlign", "RECTIFIED_RIGHT RECTIFIED_LEFT CENTER")


class FakeStereoConfig:
    """The fields of dai.StereoDepthConfig the probe and stereo_config touch, with v3.3.0's defaults."""

    def __init__(self):
        on = lambda value=False, **kw: SimpleNamespace(enable=value, **kw)  # noqa: E731
        self.algorithmControl = SimpleNamespace(
            enableSubpixel=True, subpixelFractionalBits=5, enableExtended=False, enableLeftRightCheck=True,
            leftRightCheckThreshold=10, disparityShift=0, depthAlign=DepthAlign.RECTIFIED_RIGHT)
        self.costMatching = SimpleNamespace(confidenceThreshold=245, disparityWidth=DisparityWidth.DISPARITY_96,
                                            enableCompanding=False)
        self.costAggregation = SimpleNamespace(horizontalPenaltyCostP1=250, horizontalPenaltyCostP2=500)
        self.postProcessing = SimpleNamespace(
            median=Median.KERNEL_5x5, speckleFilter=on(speckleRange=50), spatialFilter=on(), temporalFilter=on(),
            decimationFilter=SimpleNamespace(decimationFactor=1), adaptiveMedianFilter=on(), holeFilling=on(),
            thresholdFilter=SimpleNamespace(minRange=0, maxRange=65535),
            brightnessFilter=SimpleNamespace(minBrightness=0, maxBrightness=256),
            filteringOrder=[Filter.MEDIAN, Filter.DECIMATION, Filter.SPECKLE, Filter.SPATIAL, Filter.TEMPORAL])

    def setLeftRightCheckThreshold(self, value):
        self.algorithmControl.leftRightCheckThreshold = value

    def setConfidenceThreshold(self, value):
        self.costMatching.confidenceThreshold = value

    def setMedianFilter(self, value):
        self.postProcessing.median = value

    def getMaxDisparity(self):
        return stereo_probe.predicted_max_disparity(stereo_probe.read_back(SimpleNamespace(
            algorithmControl=self.algorithmControl, costMatching=self.costMatching,
            postProcessing=self.postProcessing, getMaxDisparity=lambda: 0.0)))


def test_read_back_turns_the_config_into_json():
    config = FakeStereoConfig()
    rb = stereo_probe.read_back(config)
    assert rb["median"] == "KERNEL_5x5" and rb["disparityWidth"] == "DISPARITY_96" and rb["depthAlign"] == "RECTIFIED_RIGHT"
    assert rb["filteringOrder"] == ["MEDIAN", "DECIMATION", "SPECKLE", "SPATIAL", "TEMPORAL"]
    assert rb["enableSubpixel"] is True and rb["subpixelFractionalBits"] == 5 and rb["speckleFilter"] is False
    assert rb["thresholdFilter"] == [0, 65535] and rb["holeFilling"] is False
    assert rb["maxDisparity"] == 95.0 * 32
    json.dumps(rb)
    del config.costMatching
    rb = stereo_probe.read_back(config)
    assert rb["disparityWidth"].startswith("unavailable") and rb["enableSubpixel"] is True


def test_enum_name():
    assert stereo_probe.enum_name(Median.KERNEL_7x7) == "KERNEL_7x7"
    assert stereo_probe.enum_name("MedianFilter.KERNEL_3x3") == "KERNEL_3x3"
    assert stereo_probe.enum_name(True) is True and stereo_probe.enum_name(3) == 3
    assert stereo_probe.enum_name(None) is None


def test_frame_array_honours_stride_and_depth():
    image = np.arange(12, dtype=np.uint8).reshape(3, 4)
    padded = np.zeros((3, 8), np.uint8)
    padded[:, :4] = image
    assert np.array_equal(stereo_probe.frame_array(padded.ravel(), "RAW8", 4, 3, 8), image)
    # The last row may stop at the image width
    assert np.array_equal(stereo_probe.frame_array(padded.ravel()[:-4], "GRAY8", 4, 3, 8), image)
    wide = np.array([[1, 8191, 0], [300, 2, 65535]], dtype="<u2")
    data = np.zeros((2, 8), np.uint8)
    data[:, :6] = wide.view(np.uint8)
    out = stereo_probe.frame_array(data.ravel(), "RAW16", 3, 2, 8)
    assert out.dtype == np.dtype("<u2") and np.array_equal(out, wide)
    # Unknown type: two bytes per pixel when the stride says so
    assert np.array_equal(stereo_probe.frame_array(data.ravel(), "WEIRD", 3, 2, 8), wide)
    with pytest.raises(ValueError, match="too short"):
        stereo_probe.frame_array(np.zeros(10, np.uint8), "RAW16", 3, 2, 8)


def test_value_stats():
    stats = stereo_probe.value_stats(np.array([[0, 0, 5], [5, 95, 8191]], np.uint16))
    assert stats == {"min": 0, "max": 8191, "mean": 1382.667, "distinct": 4, "zero_pct": 33.33, "max_bits": 13}
    assert stereo_probe.value_stats(np.zeros(0))["distinct"] == 0


class FakeFrame:
    def __init__(self, type_name, image, stride=None, sequence=0):
        self.type, self.image, self.sequence = type_name, np.asarray(image), sequence
        self.stride = stride or self.image.shape[1] * self.image.itemsize

    def getData(self):
        rows = self.image.astype(self.image.dtype.newbyteorder("<")).view(np.uint8)
        padded = np.zeros((rows.shape[0], self.stride), np.uint8)
        padded[:, :rows.shape[1]] = rows
        return padded.ravel()

    def getType(self):
        return SimpleNamespace(name=self.type)

    def getWidth(self):
        return self.image.shape[1]

    def getHeight(self):
        return self.image.shape[0]

    def getStride(self):
        return self.stride

    def getSequenceNum(self):
        return self.sequence


def test_frame_info():
    info = stereo_probe.frame_info(FakeFrame("RAW16", np.full((4, 6), 8191, np.uint16), stride=16, sequence=7))
    assert info["type"] == "RAW16" and (info["width"], info["height"], info["stride"], info["bytes"]) == (6, 4, 16, 64)
    assert info["max"] == 8191 and info["distinct"] == 1 and info["max_bits"] == 13 and info["sequence"] == 7
    broken = FakeFrame("RAW8", np.zeros((4, 6), np.uint8))
    broken.getStride = lambda: (_ for _ in ()).throw(RuntimeError("gone"))
    assert stereo_probe.frame_info(broken)["error"] == "RuntimeError: gone"


def jpeg(image):
    cv2 = pytest.importorskip("cv2")
    return cv2.imencode(".jpg", np.asarray(image, np.uint8))[1].tobytes()


def test_jpeg_summary_decodes_the_samples():
    flat, ramp = jpeg(np.full((40, 64), 7)), jpeg(np.tile(np.arange(64) * 4, (40, 1)))
    summary = stereo_probe.jpeg_summary([len(flat)] * 3 + [len(ramp)], 2, 4, [flat, ramp, b"not a jpeg"])
    assert summary["jpegs"] == 4 and summary["identical_to_previous"] == 2 and summary["jpeg_header_ok"] == 4
    assert summary["distinct_sizes"] == 2 and summary["size_min"] == min(len(flat), len(ramp))
    flat_s, ramp_s, bad = summary["samples"]
    assert flat_s["gray_std"] < 0.5 and abs(flat_s["gray_mean"] - 7) < 1
    assert ramp_s["gray_std"] > 30 and bad == {"bytes": 10, "decoded": False, "gray_std": None, "gray_mean": None}
    empty = stereo_probe.jpeg_summary([], 0, 0, [])
    assert empty["jpegs"] == 0 and empty["size_min"] is None and empty["samples"] == []


def test_summarize_log_groups_repeats():
    err = {"node": "ImageManip(12)", "level": "ERR", "text": "Processing failed, potentially unsupported config"}
    lines = [{**err, "unix_time": 1.0}, {"node": "", "level": "WARN", "text": "x", "unix_time": 1.5},
             {**err, "unix_time": 2.0}]
    summary = stereo_probe.summarize_log(lines)
    assert summary["lines"] == 3 and summary["distinct"] == 2 and not summary["truncated"]
    assert summary["groups"][0] == {**err, "count": 2, "first_unix_time": 1.0}
    many = [{"node": "n", "level": "ERR", "text": str(i)} for i in range(5)]
    assert stereo_probe.summarize_log(many, limit=3)["truncated"] is True


# --- run_case on a fake depthai -------------------------------------------------------------------------

class FakeQueue:
    def __init__(self, produce):
        self.produce = produce

    def tryGetAll(self):
        return self.produce()


class FakeInput:
    def __init__(self, owner):
        self.owner, self.source = owner, None

    def setBlocking(self, value):
        pass

    def setMaxSize(self, value):
        pass


class FakeOutput:
    def __init__(self, owner, frame=None):
        self.owner, self.frame = owner, frame

    def link(self, target):
        target.source = self

    def createOutputQueue(self, maxSize, blocking):
        assert blocking is False and maxSize <= 4
        return FakeQueue(self.owner.produce if self.frame is None else lambda: [self.frame()])


class FakeCamera:
    def build(self, socket, sensorFps):
        self.socket = socket
        return self

    def requestOutput(self, size):
        return FakeOutput(self)


class FakeStereoDepth:
    def __init__(self):
        self.initialConfig = FakeStereoConfig()
        self.left, self.right, self.pool = FakeInput(self), FakeInput(self), None
        self.disparity = FakeOutput(self, lambda: FakeFrame("RAW16" if self.wide() else "RAW8", self.disparity_image()))
        self.depth = FakeOutput(self, lambda: FakeFrame("RAW16", np.full((4, 64), 1000, np.uint16)))
        self.rectifiedLeft = FakeOutput(self, lambda: FakeFrame("GRAY8", np.tile(np.arange(64), (4, 1))))
        self.rectifiedRight = FakeOutput(self, lambda: FakeFrame("GRAY8", np.tile(np.arange(64), (4, 1))))

    def wide(self):
        return self.initialConfig.getMaxDisparity() > 255

    def disparity_image(self):
        return np.tile(np.arange(64) * (100 if self.wide() else 1), (4, 1)).astype(np.uint16 if self.wide() else np.uint8)

    def setDefaultProfilePreset(self, mode):
        self.preset = mode

    def setExtendedDisparity(self, value):
        self.initialConfig.algorithmControl.enableExtended = value

    def setLeftRightCheck(self, value):
        self.initialConfig.algorithmControl.enableLeftRightCheck = value

    def setSubpixel(self, value):
        self.initialConfig.algorithmControl.enableSubpixel = value

    def setSubpixelFractionalBits(self, value):
        self.initialConfig.algorithmControl.subpixelFractionalBits = value

    def setDepthAlign(self, value):
        self.initialConfig.algorithmControl.depthAlign = value

    def setNumFramesPool(self, value):
        self.pool = value

    def enableDistortionCorrection(self, value):
        pass

    def setAlphaScaling(self, value):
        pass

    def setRectifyEdgeFillColor(self, value):
        pass


class FakeImageManip:
    """Like test13: converts 8-bit input to NV12, outputs a constant image for anything wider."""
    count = 0

    def __init__(self):
        FakeImageManip.count += 1
        self.id = 10 + FakeImageManip.count
        self.initialConfig = SimpleNamespace(setFrameType=lambda t: setattr(self, "frame_type", t))
        self.inputImage, self.out = FakeInput(self), FakeOutput(self)

    def getName(self):
        return "ImageManip"

    def setMaxOutputFrameSize(self, size):
        self.max_size = size

    def setNumFramesPool(self, value):
        self.pool = value


class FakeVideoEncoder:
    def __init__(self):
        self.id = 30
        self.input, self.out = FakeInput(self), FakeOutput(self)

    def getName(self):
        return "VideoEncoder"

    def setDefaultProfilePreset(self, fps, profile):
        self.fps, self.profile = fps, profile

    def setQuality(self, quality):
        self.quality = quality

    def produce(self):
        source = self.input.source
        if isinstance(source.owner, FakeImageManip):
            assert source.owner.pool >= 2 and source.owner.frame_type == "NV12"
            frame = source.owner.inputImage.source.frame()
            if frame.type == "RAW16":
                source.owner.device.log("ImageManip(12)", "ERR", "Processing failed, potentially unsupported config")
                return [SimpleNamespace(getData=lambda: FLAT_JPEG)]
            return [SimpleNamespace(getData=lambda: jpeg(frame.image))]
        return []  # the encoder drops GRAY8 / RAW frames


FLAT_JPEG = b"\xff\xd8" + b"\x00" * 100


class FakeDevice:
    def __init__(self):
        self.callbacks, self.closed, self.level = {}, False, None

    def getDeviceName(self):
        return "OAK-D-W"

    def setLogLevel(self, level):
        self.level = level

    def addLogCallback(self, callback):
        self.callbacks[len(self.callbacks)] = callback
        return len(self.callbacks) - 1

    def removeLogCallback(self, handle):
        del self.callbacks[handle]

    def log(self, node, level, text):
        for callback in list(self.callbacks.values()):
            callback(SimpleNamespace(nodeIdName=node, level=SimpleNamespace(name=level), payload=text))

    def close(self):
        self.closed = True


class FakePipeline:
    def __init__(self, device):
        self.device, self.nodes, self.started = device, [], False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.device.close()

    def create(self, cls):
        node = cls()
        node.device = self.device
        self.nodes.append(node)
        return node

    def start(self):
        self.started = True


def fake_dai():
    return SimpleNamespace(
        __version__="3.3.0-fake",
        LogLevel=SimpleNamespace(WARN="WARN"),
        CameraBoardSocket=SimpleNamespace(CAM_B="CAM_B", CAM_C="CAM_C"),
        ImgFrame=SimpleNamespace(Type=SimpleNamespace(NV12="NV12")),
        VideoEncoderProperties=SimpleNamespace(Profile=SimpleNamespace(MJPEG="MJPEG")),
        MedianFilter=Median,
        StereoDepthConfig=SimpleNamespace(AlgorithmControl=SimpleNamespace(DepthAlign=DepthAlign)),
        node=SimpleNamespace(Camera=FakeCamera, StereoDepth=SimpleNamespace(PresetMode=SimpleNamespace(
            ROBOTICS="ROBOTICS", NONE="NONE")), ImageManip=FakeImageManip, VideoEncoder=FakeVideoEncoder),
        Pipeline=FakePipeline,
    )


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


@pytest.fixture
def dai():
    pytest.importorskip("cv2")
    module = fake_dai()
    module.node.StereoDepth = type("StereoDepth", (FakeStereoDepth,), {"PresetMode": module.node.StereoDepth.PresetMode})
    return module


def probe_args(*extra):
    return stereo_probe.parse_args(["--width", "64", "--height", "4", "--seconds", "2", *extra])


def test_run_case_a_reads_back_samples_and_encodes(dai):
    devices = []
    clock = FakeClock()
    result = stereo_probe.run_case(stereo_probe.CASES_BY_NAME["A"], SAVED, probe_args(), dai,
                                   lambda: devices.append(FakeDevice()) or devices[-1], clock=clock, sleep=clock.sleep)
    assert result["error"] is None, result.get("traceback")
    assert devices[0].closed and devices[0].callbacks == {} and devices[0].level == "WARN"
    applied = result["applied"]
    assert applied["enableSubpixel"] is False and applied["speckleFilter"] is True and applied["median"] == "KERNEL_7x7"
    assert result["executed_filters"] == ["MEDIAN", "SPECKLE"]
    assert applied["maxDisparity"] == result["predicted_max_disparity"] == pytest.approx(8192.0)
    assert result["changed_at_start"] == {}
    disparity = result["streams"]["disparity"]
    assert disparity["frames"] > 50 and len(disparity["samples"]) == stereo_probe.SAMPLES
    assert disparity["samples"][0]["type"] == "RAW16" and disparity["samples"][0]["max_bits"] == 13
    assert set(result["streams"]) == set(stereo_probe.STREAMS)
    encode = result["encode"]
    assert set(encode) == {"disparity", "rectifiedLeft", "rectifiedRight"}
    assert encode["disparity"]["direct"]["jpegs"] == 0
    flat = encode["disparity"]["manip"]
    assert flat["jpegs"] > 50 and flat["distinct_sizes"] == 1 and flat["identical_to_previous"] == flat["jpegs"] - 1
    ramp = encode["rectifiedLeft"]["manip"]
    assert ramp["jpegs"] > 50 and ramp["samples"] and ramp["samples"][0]["gray_std"] > 10
    log = result["device_log"]
    assert log["distinct"] == 1 and log["groups"][0]["node"] == "ImageManip(12)" and log["groups"][0]["level"] == "ERR"
    assert "disparity RAW16 max 6300" in result["summary"] and "disparity/direct 0 JPEG" in result["summary"]
    json.dumps(result)


def test_run_case_d_keeps_the_disparity_8_bit(dai):
    clock = FakeClock()
    result = stereo_probe.run_case(stereo_probe.CASES_BY_NAME["D"], SAVED, probe_args(), dai, FakeDevice,
                                   clock=clock, sleep=clock.sleep)
    assert result["executed_filters"] == ["MEDIAN"] and result["predicted_max_disparity"] == 95.0
    assert result["streams"]["disparity"]["samples"][0]["type"] == "RAW8"
    assert list(result["encode"]) == ["disparity"]
    assert result["encode"]["disparity"]["manip"]["samples"][0]["gray_std"] > 5
    assert result["device_log"]["lines"] == 0


def test_a_failing_case_is_recorded_and_the_next_one_runs(dai, tmp_path, capsys):
    attempts = []

    def open_device():
        attempts.append(1)
        if len(attempts) == 1:
            raise RuntimeError("no device")
        return FakeDevice()

    clock = FakeClock()
    output = tmp_path / "probe" / "out.json"
    code = stereo_probe.main(["--width", "64", "--height", "4", "--seconds", "2", "--cases", "A,F",
                              "--output", str(output), "--stereo-median", "5x5"],
                             dai=dai, open_device=open_device, clock=clock, sleep=clock.sleep, wall_clock=lambda: 1e9)
    assert code == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed == json.loads(output.read_text())
    assert printed["complete"] is True and printed["saved_stereo"]["stereo_median"] == "5x5"
    a, f = printed["cases"]
    assert a["name"] == "A" and a["error"] == "RuntimeError: no device" and "traceback" in a and "streams" not in a
    assert f["name"] == "F" and f["error"] is None and f["applied"]["median"] == "MEDIAN_OFF"
    assert f["settings"]["stereo_preset"] == "NONE" and f["executed_filters"] == []


def test_the_budget_skips_cases(dai, capsys):
    clock = FakeClock()
    clock.t = 0.0

    def slow_device():
        clock.t += 30.0
        return FakeDevice()

    stereo_probe.main(["--width", "64", "--height", "4", "--seconds", "2", "--budget", "70"],
                      dai=dai, open_device=slow_device, clock=clock, sleep=clock.sleep)
    printed = json.loads(capsys.readouterr().out)
    assert [c["name"] for c in printed["cases"]] == ["A", "B"]
    assert [s["name"] for s in printed["skipped"]] == ["C", "D", "E", "F"]
    assert "time budget" in printed["skipped"][0]["reason"]


def test_unknown_cases_exit_2(capsys):
    assert stereo_probe.main(["--cases", "Q"], dai=fake_dai(), open_device=FakeDevice) == 2
    assert capsys.readouterr().out == ""


# --- POST /api/stereo_probe -----------------------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    main.save_config({**main.DEFAULT_CONFIG, "fps": 12, "stereo_subpixel": False, "stereo_median": "5x5"})
    actions = []
    monkeypatch.setattr(main, "stop_vio", lambda: actions.append("stop"))
    monkeypatch.setattr(main, "start_vio", lambda: actions.append("start"))
    return main, fastapi_testclient.TestClient(main.app), actions


def output_of(cmd):
    return cmd[cmd.index("--output") + 1]


def complete_document(**extra):
    return {"probe": "stereo", "complete": True, "cases": [{"name": "A", "summary": "ok"}], **extra}


def test_probe_command_matches_the_vio_settings(client):
    main, _, _ = client
    config = main.load_config()
    cmd = main.stereo_probe_command(config, main.CONFIG_FILE.parent / "probe" / "x.json", ["A", "E"])
    assert cmd[:3] == ["python", "-u", "/app/stereo_probe.py"]
    args = stereo_probe.parse_args(cmd[3:])
    assert (args.fps, args.width, args.height, args.cases) == (12, config["resolution_width"],
                                                               config["resolution_height"], "A,E")
    assert args.quality == config["depth_stream_quality"]
    assert resolve_stereo_settings(args) == resolve_stereo_settings(config)


def test_a_successful_probe_restarts_the_vio_and_saves_the_result(client, monkeypatch, tmp_path):
    main, http, actions = client
    seen = {}

    def run(cmd, timeout):
        actions.append("probe")
        seen.update(cmd=cmd, timeout=timeout)
        with open(output_of(cmd), "w") as f:
            json.dump(complete_document(), f)
        return 0, json.dumps(complete_document()) + "\n", "opening device\n" * 2000

    monkeypatch.setattr(main, "run_stereo_probe_process", run)
    response = http.post("/api/stereo_probe?cases=a,e")
    assert response.status_code == 200
    assert actions == ["stop", "probe", "start"]
    body = response.json()
    assert body["probe"] == complete_document() and body["error"] is None and body["cases"] == ["A", "E"]
    assert body["vio_restarted"] is True and body["timed_out"] is False and body["returncode"] == 0
    assert len(body["stderr_tail"]) == main.STEREO_PROBE_STDERR_CHARS and "stdout_tail" not in body
    assert seen["timeout"] == main.STEREO_PROBE_TIMEOUT_S and "A,E" in seen["cmd"]
    saved = list((tmp_path / "probe").glob("stereo-*.json"))
    assert len(saved) == 1 and body["saved_to"] == str(saved[0]) and json.loads(saved[0].read_text()) == body


def test_the_probe_result_can_come_from_stdout(client, monkeypatch):
    main, http, actions = client
    monkeypatch.setattr(main, "run_stereo_probe_process",
                        lambda cmd, timeout: (0, "depthai noise\n" + json.dumps(complete_document()) + "\n", ""))
    response = http.post("/api/stereo_probe")
    assert response.status_code == 200 and response.json()["probe"] == complete_document()
    assert response.json()["cases"] == ["A", "B", "C", "D", "E", "F"]


@pytest.mark.parametrize("outcome, status", [("crash", 502), ("exit", 502), ("garbage", 502), ("timeout", 504)])
def test_the_vio_is_restarted_after_a_failed_probe(client, monkeypatch, outcome, status):
    main, http, actions = client

    def run(cmd, timeout):
        actions.append("probe")
        if outcome == "crash":
            raise FileNotFoundError("python")
        if outcome == "timeout":
            with open(output_of(cmd), "w") as f:
                json.dump({"probe": "stereo", "complete": False, "cases": [{"name": "A"}]}, f)
            return None, "", "case B hangs"
        if outcome == "garbage":
            return 0, "no json here", ""
        return -11, "", "Segmentation fault"

    monkeypatch.setattr(main, "run_stereo_probe_process", run)
    response = http.post("/api/stereo_probe")
    assert response.status_code == status
    assert actions == ["stop", "probe", "start"]
    body = response.json()
    assert body["error"] and body["vio_restarted"] is True and os.path.exists(body["saved_to"])
    assert body["timed_out"] is (outcome == "timeout")
    if outcome == "timeout":
        assert body["probe"]["complete"] is False and body["stderr_tail"] == "case B hangs"
        assert "killed" in body["error"]
    if outcome == "crash":
        assert "could not run" in body["error"] and body["probe"] is None
    if outcome == "garbage":
        assert body["stdout_tail"] == "no json here"
    if outcome == "exit":
        assert body["returncode"] == -11 and "code -11" in body["error"]


def test_the_vio_is_restarted_when_stopping_it_fails(client, monkeypatch):
    main, http, actions = client

    def stop():
        actions.append("stop")
        raise RuntimeError("stuck")

    monkeypatch.setattr(main, "stop_vio", stop)
    monkeypatch.setattr(main, "run_stereo_probe_process", lambda cmd, timeout: actions.append("probe"))
    response = http.post("/api/stereo_probe")
    assert response.status_code == 502 and actions == ["stop", "start"]
    assert "stuck" in response.json()["error"]


def test_a_failed_restart_is_reported(client, monkeypatch):
    main, http, actions = client

    def start():
        raise RuntimeError("no python")

    monkeypatch.setattr(main, "start_vio", start)
    monkeypatch.setattr(main, "run_stereo_probe_process", lambda cmd, timeout: (0, json.dumps(complete_document()), ""))
    body = http.post("/api/stereo_probe").json()
    assert body["vio_restarted"] is False and body["vio_restart_error"] == "no python"


def test_one_probe_at_a_time(client, monkeypatch):
    main, http, actions = client
    monkeypatch.setattr(main, "run_stereo_probe_process", lambda cmd, timeout: (0, json.dumps(complete_document()), ""))
    assert main.stereo_probe_lock.acquire(blocking=False)
    try:
        response = http.post("/api/stereo_probe")
    finally:
        main.stereo_probe_lock.release()
    assert response.status_code == 409 and actions == []
    assert http.post("/api/stereo_probe").status_code == 200


def test_a_concurrent_request_gets_409_while_the_probe_runs(client, monkeypatch):
    main, http, actions = client
    running, release = threading.Event(), threading.Event()

    def run(cmd, timeout):
        running.set()
        release.wait(5)
        return 0, json.dumps(complete_document()), ""

    monkeypatch.setattr(main, "run_stereo_probe_process", run)
    results = []
    thread = threading.Thread(target=lambda: results.append(http.post("/api/stereo_probe").status_code))
    thread.start()
    assert running.wait(5)
    assert http.post("/api/stereo_probe").status_code == 409
    release.set()
    thread.join(5)
    assert results == [200] and actions == ["stop", "start"]


def test_mock_mode_and_bad_cases_are_refused(client, monkeypatch):
    main, http, actions = client
    monkeypatch.setattr(main, "run_stereo_probe_process", lambda cmd, timeout: actions.append("probe"))
    assert http.post("/api/stereo_probe?cases=A,X").status_code == 400
    monkeypatch.setattr(main, "MOCK", True)
    response = http.post("/api/stereo_probe")
    assert response.status_code == 403 and response.json()["mock"] is True
    assert actions == []


def test_the_probe_is_post_only_and_hidden_from_the_schema(client):
    main, http, actions = client
    assert http.get("/api/stereo_probe").status_code == 405
    assert "/api/stereo_probe" not in http.get("/openapi.json").json()["paths"]
    assert actions == []
