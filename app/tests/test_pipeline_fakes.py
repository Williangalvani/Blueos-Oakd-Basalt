"""The strict fake pipeline (pipeline_fakes.py) makes the checks depthai v3.3.0 makes before the camera runs."""
import os
import sys
from unittest import mock

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import depth_points  # noqa: E402
from pipeline_fakes import StrictPipeline  # noqa: E402

dai = mock.MagicMock()


def test_a_script_input_and_output_with_the_same_name_fail_at_start():
    """test16 on cb60cc4, word for word: Script inputs and outputs are one key space (group "io")."""
    p = StrictPipeline()
    stereo, script = p.create(dai.node.StereoDepth), p.create(dai.node.Script)
    stereo.depth.link(script.inputs["depth"])
    script.outputs["depth"].createOutputQueue(maxSize=2, blocking=False)
    with pytest.raises(ValueError, match=r"""^'Script\.io\["depth"\]' redefined\. Inputs and outputs must have """
                                         r"unique names$"):
        p.start()
    assert not p.isBuilt()


def test_the_same_name_in_two_scripts_or_on_one_side_is_fine():
    p = StrictPipeline()
    stereo = p.create(dai.node.StereoDepth)
    for _ in range(2):
        script = p.create(dai.node.Script)
        stereo.depth.link(script.inputs["depth"])
        script.inputs["depth"].setBlocking(False)  # the same port again
        script.outputs["frames"].createOutputQueue(maxSize=2, blocking=False)
    p.start()
    assert p.isBuilt()


def test_links_must_match_datatypes():
    p = StrictPipeline()
    stereo, tracker = p.create(dai.node.StereoDepth), p.create(dai.node.FeatureTracker)
    slc, script = p.create(dai.node.SpatialLocationCalculator), p.create(dai.node.Script)
    with pytest.raises(RuntimeError, match="Cannot link 'FeatureTracker.outputFeatures' to "
                                           "'SpatialLocationCalculator.inputDepth'"):
        tracker.outputFeatures.link(slc.inputDepth)
    with pytest.raises(RuntimeError, match="Cannot link"):
        slc.out.link(tracker.inputImage)
    # A Script's ports take and give any message (Buffer, with descendants)
    stereo.depth.link(script.inputs["depth"])
    script.outputs["slc_depth"].link(slc.inputDepth)
    script.outputs["config"].link(slc.inputConfig)
    slc.out.link(script.inputs["spatial"])
    stereo.rectifiedLeft.link(tracker.inputImage)
    camera = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camera.requestOutput((640, 400)).link(stereo.left)
    encoder = p.create(dai.node.VideoEncoder)
    with pytest.raises(RuntimeError, match="Cannot link 'VideoEncoder.out' to 'StereoDepth.right'"):
        encoder.out.link(stereo.right)  # EncodedFrame is not an ImgFrame


def test_a_link_twice_and_a_link_into_an_output_fail():
    p = StrictPipeline()
    stereo, script = p.create(dai.node.StereoDepth), p.create(dai.node.Script)
    stereo.depth.link(script.inputs["depth"])
    with pytest.raises(RuntimeError, match="already linked"):
        stereo.depth.link(script.inputs["depth"])
    with pytest.raises(TypeError):
        stereo.depth.link(script.outputs["depth"])
    with pytest.raises(AttributeError):
        script.inputs["depth"].link(stereo.left)


def test_no_queue_after_start():
    p = StrictPipeline()
    script = p.create(dai.node.Script)
    script.outputs["points"].createOutputQueue(maxSize=2, blocking=False)
    p.start()
    with pytest.raises(RuntimeError, match="after pipeline is built"):
        script.outputs["status"].createOutputQueue(maxSize=2, blocking=False)


def test_a_script_may_only_use_the_ports_the_pipeline_declares():
    p = StrictPipeline()
    stereo, script = p.create(dai.node.StereoDepth), p.create(dai.node.Script)
    stereo.depth.link(script.inputs["depth"])
    script.inputs["control"].createInputQueue(maxSize=4, blocking=False)
    script.setScript(depth_points.gate_script())
    with pytest.raises(RuntimeError, match=f"node.outputs\\['{depth_points.GATE_OUT}'\\]"):
        p.start()
    script.outputs[depth_points.GATE_OUT].createOutputQueue(maxSize=2, blocking=False)
    p.start()


def test_a_link_from_a_port_the_fake_does_not_know_fails_at_start():
    p = StrictPipeline()
    stereo, script = p.create(dai.node.StereoDepth), p.create(dai.node.Script)
    stereo.someNewOutput.link(script.inputs["x"])
    with pytest.raises(AssertionError, match="StereoDepth.someNewOutput is not a known port"):
        p.start()
