"""
static/features_overlay.js under node: where the browser draws each tracked feature (main page and Cockpit
widget). Points come from feature_stream.features_payload, so the test also checks that the server's grid
scaling and the browser's mapping agree. Skipped without node.
"""
import json
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import feature_stream  # noqa: E402

OVERLAY_JS = os.path.join(os.path.dirname(__file__), "..", "static", "features_overlay.js")
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

# Reads {"calls": [[function name, args...], ...]} on stdin and prints each result. drawFeatures gets a
# recording 2D context: the marker centres in draw order.
RUNNER = r"""
const overlay = require(process.argv[1]);
const input = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = input.calls.map(([name, ...args]) => {
  if (name !== 'drawFeatures') return overlay[name](...args);
  const drawn = [];
  const ctx = new Proxy({}, {
    get: (target, key) => (key in target ? target[key]
      : key === 'arc' || key === 'rect' ? (x, y) => drawn.push([key, x, y]) : () => {}),
    set: (target, key, value) => { target[key] = value; return true; },
  });
  overlay.drawFeatures(ctx, ...args);
  return drawn;
});
process.stdout.write(JSON.stringify(out));
"""


def run_overlay(*calls):
    result = subprocess.run([NODE, "-e", RUNNER, os.path.abspath(OVERLAY_JS)], input=json.dumps({"calls": calls}),
                            capture_output=True, text=True, timeout=30, check=True)
    return json.loads(result.stdout)


def test_the_states_are_the_server_s():
    (states,) = run_overlay(["featureContainRect", [1, 1], 1, 1])  # any call; the module loaded
    assert states == {"x": 0, "y": 0, "w": 1, "h": 1}
    source = open(OVERLAY_JS).read()
    assert "const FEATURE_STATES = " + json.dumps(list(feature_stream.STATES)).replace('"', "'") + ";" in source


def test_contain_rect_letterboxes_like_object_fit_contain():
    wide, tall, exact = run_overlay(["featureContainRect", [640, 400], 800, 800],
                                    ["featureContainRect", [640, 400], 1600, 500],
                                    ["featureContainRect", [640, 400], 320, 200])
    assert wide == {"x": 0, "y": 150, "w": 800, "h": 500}  # bars above and below
    assert tall == {"x": 400, "y": 0, "w": 800, "h": 500}  # bars left and right
    assert exact == {"x": 0, "y": 0, "w": 320, "h": 200}


def view_points(uv, image_size, frame_size, rect):
    """Where the browser draws features found at uv (rectified-left pixel-centre coordinates)."""
    f = feature_stream.features_payload(np.asarray(uv, float), np.full(len(uv), feature_stream.INLIER),
                                        image_size, frame_size)
    return run_overlay(*[["featureToView", point, f, rect] for point in f["points"]])


@pytest.mark.parametrize("frame_size", [(640, 400), (320, 200)], ids=["full", "decimated"])
def test_a_feature_lands_on_its_pixel_in_the_image_shown(frame_size):
    """
    A feature at rectified-left pixel (u, v) covers u..u+1 in edge coordinates, so on an image shown at
    scale s it is drawn at (u + 0.5) * s, whether the points went through the decimated depth grid or not.
    """
    uv = [(0, 0), (639, 399), (100, 50), (320.5, 200.25)]
    rect = {"x": 0, "y": 0, "w": 1280, "h": 800}  # shown at 2x
    drawn = np.array(view_points(uv, (640, 400), frame_size, rect))
    expected = (np.array(uv) + 0.5) * 2
    tolerance = 0.05 * 2 * 640 / frame_size[0]  # the payload rounds to 0.1 px on its grid
    np.testing.assert_allclose(drawn, expected, atol=tolerance)
    assert drawn[0].tolist() == [1, 1] and drawn[1].tolist() == [1279, 799]  # the corner pixels' centres


def test_letterboxing_offsets_and_scales_the_features():
    rect = {"x": 400, "y": 0, "w": 800, "h": 500}  # a 1600 x 500 view, 640 x 400 shown at 1.25x
    (corner, centre) = view_points([(0, 0), (319.5, 199.5)], (640, 400), (640, 400), rect)
    assert corner == [400 + 0.5 * 1.25, 0.5 * 1.25]
    assert centre == [800, 250]  # the image centre is the view centre


def test_without_image_size_the_points_grid_is_the_image():
    (point,) = run_overlay(["featureToView", [10, 20, 4], {"frame_size": [320, 200], "image_size": None},
                            {"x": 0, "y": 0, "w": 320, "h": 200}])
    assert point == [10.5, 20.5]


def test_draw_features_draws_every_point_best_state_on_top():
    points = [[1, 1, feature_stream.INLIER], [2, 2, feature_stream.NO_DEPTH], [3, 3, feature_stream.OUTLIER],
              [4, 4, feature_stream.DEPTH], [5, 5, feature_stream.MATCHED]]
    f = {"frame_size": [640, 400], "image_size": [640, 400], "points": points}
    (drawn,) = run_overlay(["drawFeatures", f, {"x": 0, "y": 0, "w": 640, "h": 400}])
    # One marker each, in state order: no depth first, inliers last; outliers are squares (rect at x - 3)
    assert [d[1] + (3 if d[0] == "rect" else 0) - 0.5 for d in drawn] == [2, 4, 5, 3, 1]
    assert [d[0] for d in drawn] == ["arc", "arc", "arc", "rect", "arc"]
    (nothing,) = run_overlay(["drawFeatures", None, {"x": 0, "y": 0, "w": 1, "h": 1}])
    assert nothing == []
