"""
A strict fake of a depthai v3.3.0 pipeline for pnp_vio.build_pipeline, making the checks depthai makes on the host
before anything reaches the camera (depthai-core v3.3.0):

- link (Node.cpp Output::link): the output's datatype must match the input's (isDatatypeMatch: the same type, or
  the side declared "with descendants" is a parent of the other: Buffer is the parent of every message type),
  else "Cannot link"; the same output -> input link twice is "already linked". (The MSender -> MReceiver rule
  never applies here: every port of these nodes is the default MSender / SReceiver.)
- start (Pipeline.cpp getPipelineSchema): a node's inputs AND outputs share one (group, name) key space. A Script's
  inputs and outputs are both in group "io" (Script.hpp), so an input and an output with the same name raise
  ValueError "'Script.io["depth"]' redefined. Inputs and outputs must have unique names" (test16, cb60cc4).
- start (Pipeline.cpp build): the XLink stream names of the host queues ("__x_<id>_<name>" for an output queue,
  "__x_<id>_<group>_<name>" for an input queue) must be unique; no queue can be created once it is built.
- on the camera a Script only has the inputs and outputs the host declared: its literal node.inputs['x'] /
  node.outputs['x'] names are checked against them at start.

Nodes are MagicMocks (the tests assert their calls); their ports are MagicMocks that know their node, group,
name, direction and datatype. A link from a port not described here fails at start: add it to PORTS.
"""
import re
from unittest import mock

BUFFER = "Buffer"
IMG_FRAME = "ImgFrame"

# Node type -> attribute -> (direction, depthai name, datatype, descendants), from the node headers
PORTS = {
    "Camera": {"inputControl": ("in", "inputControl", "CameraControl", False),
               "raw": ("out", "raw", IMG_FRAME, False)},
    "StereoDepth": {
        "inputConfig": ("in", "inputConfig", "StereoDepthConfig", False),
        "left": ("in", "left", IMG_FRAME, True), "right": ("in", "right", IMG_FRAME, True),
        **{name: ("out", name, IMG_FRAME, False)
           for name in ("depth", "disparity", "syncedLeft", "syncedRight", "rectifiedLeft", "rectifiedRight",
                        "confidenceMap")},
        "outConfig": ("out", "outConfig", "StereoDepthConfig", False)},
    "FeatureTracker": {"inputConfig": ("in", "inputConfig", "FeatureTrackerConfig", False),
                       "inputImage": ("in", "inputImage", IMG_FRAME, False),
                       "outputFeatures": ("out", "outputFeatures", "TrackedFeatures", False),
                       "passthroughInputImage": ("out", "passthroughInputImage", IMG_FRAME, False)},
    "IMU": {"out": ("out", "out", "IMUData", False)},
    "SpatialLocationCalculator": {
        "inputConfig": ("in", "inputConfig", "SpatialLocationCalculatorConfig", False),
        "inputDepth": ("in", "inputDepth", IMG_FRAME, False),
        "out": ("out", "out", "SpatialLocationCalculatorData", False),
        "passthroughDepth": ("out", "passthroughDepth", IMG_FRAME, False)},
    "VideoEncoder": {"input": ("in", "in", IMG_FRAME, True), "bitstream": ("out", "bitstream", IMG_FRAME, False),
                     "out": ("out", "out", "EncodedFrame", False)},
    "ImageManip": {"inputConfig": ("in", "inputConfig", "ImageManipConfig", True),
                   "inputImage": ("in", "inputImage", IMG_FRAME, True), "out": ("out", "out", IMG_FRAME, True)},
    "Sync": {"out": ("out", "out", "MessageGroup", False)},
}
# Node type -> attribute -> (direction, group) of its port maps; their ports take any message (Buffer, descendants)
MAPS = {"Script": {"inputs": ("in", "io"), "outputs": ("out", "io")}, "Sync": {"inputs": ("in", "inputs")}}

SCRIPT_PORT = re.compile(r"""node\.(inputs|outputs)\[\s*(['"])(\w+)\2\s*\]""")


def subclass_of(parent, child):
    """DatatypeEnum.cpp isDatatypeSubclassOf for the types used here: Buffer is the parent of all the others."""
    return parent == BUFFER and child != BUFFER


def datatype_match(out, inp):
    """Node.cpp isDatatypeMatch."""
    return (out.datatype == inp.datatype or (out.descendants and subclass_of(out.datatype, inp.datatype))
            or (inp.descendants and subclass_of(inp.datatype, out.datatype)))


def is_port(obj):
    return isinstance(obj, mock.NonCallableMock) and vars(obj).get("fake_port") is True


def label(port):
    group = f'{port.group}["{port.name}"]' if port.group else port.name
    return f"{port.node_name}.{group}"


class PortMap(dict):
    """A Script's / Sync's inputs or outputs: operator[] declares the port (as depthai's InputMap / OutputMap)."""

    def __init__(self, pipeline, node, direction, group):
        super().__init__()
        self.pipeline, self.node, self.direction, self.group = pipeline, node, direction, group

    def __missing__(self, name):
        self[name] = self.pipeline.port(self.node, self.direction, self.group, name, BUFFER, True)
        return self[name]


class StrictPipeline:
    def __init__(self):
        self.created = []  # (kind, node), as p.create returned them
        self.ports = {}  # node id -> [port, ...] in declaration order
        self.links = set()
        self.streams = []  # XLink stream names of the host queues
        self.built = False
        self.next_id = 0

    def create(self, kind):
        name = getattr(kind, "_mock_name", None) or str(kind)
        node = mock.MagicMock(name=name)
        node.id = self.next_id
        self.next_id += 1
        node.getName.return_value = name
        node.fake_kind = name
        self.ports[node.id] = []
        for attribute, (direction, port_name, datatype, descendants) in PORTS.get(name, {}).items():
            setattr(node, attribute, self.port(node, direction, "", port_name, datatype, descendants))
        for attribute, (direction, group) in MAPS.get(name, {}).items():
            setattr(node, attribute, PortMap(self, node, direction, group))
        if name == "Camera":
            node.build.return_value = node
            node.requestOutput.side_effect = lambda *args, **kwargs: self.port(
                node, "out", "dynamicOutputs", str(node.requestOutput.call_count - 1), IMG_FRAME, False)
        self.created.append((kind, node))
        return node

    def port(self, node, direction, group, name, datatype, descendants):
        port = mock.MagicMock(name=f"{node.fake_kind}.{name}")
        port.fake_port = True
        port.node, port.node_name, port.direction = node, node.fake_kind, direction
        port.group, port.name, port.datatype, port.descendants = group, name, datatype, descendants
        if direction == "out":
            port.link.side_effect = lambda target: self.link(port, target)
            port.createOutputQueue.side_effect = lambda *args, **kwargs: self.queue(
                f"__x_{node.id}_{name}")
            port.createInputQueue.side_effect = AttributeError(f"{label(port)} is an output: no createInputQueue")
        else:
            port.link.side_effect = AttributeError(f"{label(port)} is an input: it has no link()")
            port.createInputQueue.side_effect = lambda *args, **kwargs: self.queue(
                f"__x_{node.id}_{group}_{name}")
            port.createOutputQueue.side_effect = AttributeError(f"{label(port)} is an input: no createOutputQueue")
        self.ports[node.id].append(port)
        return port

    def link(self, out, target):
        if not is_port(target) or target.direction != "in":
            raise TypeError(f"link(): incompatible function arguments: {label(out)} -> {target!r}")
        if not datatype_match(out, target):
            raise RuntimeError(f"Cannot link '{label(out)}' to '{label(target)}'")
        key = (id(out), id(target))
        if key in self.links:
            raise RuntimeError(f"'{label(out)}' already linked to '{label(target)}'")  # std::logic_error
        self.links.add(key)
        return mock.DEFAULT

    def queue(self, stream):
        if self.built:
            raise RuntimeError("Cannot create queue after pipeline is built")
        self.streams.append(stream)
        return mock.DEFAULT

    def start(self):
        for _, node in self.created:
            ports = self.ports[node.id]
            seen = set()
            for port in [p for p in ports if p.direction == "in"] + [p for p in ports if p.direction == "out"]:
                if (port.group, port.name) in seen:
                    raise ValueError(f"'{label(port)}' redefined. Inputs and outputs must have unique names")
                seen.add((port.group, port.name))
            self.check_untyped_links(node)
            if node.fake_kind == "Script" and node.setScript.called:
                self.check_script_ports(node, node.setScript.call_args[0][0])
        duplicates = {s for s in self.streams if self.streams.count(s) > 1}
        if duplicates:
            raise RuntimeError(f"Stream name '{sorted(duplicates)[0]}' is not unique")
        self.built = True

    def check_untyped_links(self, node):
        for attribute, child in node._mock_children.items():
            if is_port(child) or not isinstance(child, mock.NonCallableMock):
                continue
            link = child._mock_children.get("link")
            if link is not None and link.called:
                raise AssertionError(f"fake pipeline: {node.fake_kind}.{attribute} is not a known port; "
                                     "add it to pipeline_fakes.PORTS")

    def check_script_ports(self, node, code):
        declared = {(p.direction, p.name) for p in self.ports[node.id]}
        for kind, _, name in SCRIPT_PORT.findall(code):
            direction = "in" if kind == "inputs" else "out"
            if (direction, name) not in declared:
                raise RuntimeError(f"Script({node.id}): node.{kind}['{name}'] is used by its code but the "
                                   "pipeline does not declare it")

    def isBuilt(self):
        return self.built

    def isRunning(self):
        return self.built
