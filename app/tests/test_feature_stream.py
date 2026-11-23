import json
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import feature_stream  # noqa: E402
from feature_stream import DEPTH, INLIER, MATCHED, NO_DEPTH, OUTLIER, STATES  # noqa: E402
from pnp_odometry import Intrinsics, PnPOdometry  # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def test_feature_states_take_the_best_state_of_each_feature():
    # 6 features: 0, 1 with depth; 2..5 matched to the reference; RANSAC kept matched[1] and matched[3]
    states = feature_stream.feature_states(6, depth_valid=[True, True, False, True, False, True],
                                           matched_index=[2, 3, 4, 5], inlier_index=[1, 3])
    assert states.tolist() == [DEPTH, DEPTH, OUTLIER, INLIER, OUTLIER, INLIER]
    # No RANSAC result: matched, whatever the depth
    assert feature_stream.feature_states(3, [False, True, True], [0, 1]).tolist() == [MATCHED, MATCHED, DEPTH]
    assert feature_stream.feature_states(2).tolist() == [NO_DEPTH, NO_DEPTH]
    # Out-of-range indices and a mask of the wrong length are ignored instead of raising
    assert feature_stream.feature_states(2, [True], [0, 7], [0, 5]).tolist() == [INLIER, NO_DEPTH]
    assert feature_stream.feature_states(0, [], [], []).tolist() == []


def test_positions_are_scaled_to_the_depth_grid():
    uv = np.array([[320.0, 200.0], [639.0, 399.0]])
    np.testing.assert_array_equal(feature_stream.to_frame_grid(uv, (640, 400), (640, 400)), uv)
    # Decimated depth (e.g. stereo_decimation 2): half the grid
    np.testing.assert_allclose(feature_stream.to_frame_grid(uv, (640, 400), (320, 200)), uv / 2)


def test_payload_is_small_one_line_json():
    uv = np.array([[100.04, 50.06], [300.0, 200.0], [10.0, 20.0]])
    states = np.array([INLIER, OUTLIER, NO_DEPTH], np.uint8)
    payload = feature_stream.features_payload(uv, states, (640, 400), (320, 200), sequence=7, ok=False,
                                              reason="too_few_inliers", now=12.5)
    assert payload["frame_size"] == [320, 200] and payload["image_size"] == [640, 400]
    assert payload["points"] == [[50.0, 25.0, INLIER], [150.0, 100.0, OUTLIER], [5.0, 10.0, NO_DEPTH]]
    assert payload["counts"] == {"no_depth": 1, "depth": 0, "matched": 0, "outlier": 1, "inlier": 1}
    assert payload["states"] == list(STATES) and payload["sequence"] == 7 and payload["unix_time"] == 12.5
    assert payload["ok"] is False and payload["reason"] == "too_few_inliers" and payload["pid"] == os.getpid()
    encoded = json.dumps(payload)
    assert "\n" not in encoded
    # 300 features stay a few KB
    many = feature_stream.features_payload(np.random.default_rng(0).uniform(0, 640, (300, 2)),
                                           np.full(300, INLIER, np.uint8), (640, 400), (640, 400))
    assert len(json.dumps(many)) < 6000


def service(tmp_path, clock, **kwargs):
    return feature_stream.FeatureStreamService(tmp_path, (640, 400), clock=clock, log=lambda m: None, **kwargs)


def tick(svc, n=4, **kwargs):
    return svc.tick(np.array([[10.0 * k, 5.0 * k] for k in range(n)]), (400, 640), 3, True, None,
                    depth_valid=np.ones(n, bool), **kwargs)


def test_service_publishes_only_while_leased_and_at_most_every_interval(tmp_path):
    clock = Clock(time.time())
    (tmp_path / feature_stream.FEATURES_FILE).write_text("{}")  # from a previous run
    svc = service(tmp_path, clock)
    assert not (tmp_path / feature_stream.FEATURES_FILE).exists()
    assert not tick(svc) and not (tmp_path / feature_stream.FEATURES_FILE).exists()

    feature_stream.touch_lease(tmp_path)
    clock.t += 0.1
    assert not tick(svc)  # the lease is only looked at every poll_interval
    clock.t += 0.5
    assert tick(svc)
    data = json.loads((tmp_path / feature_stream.FEATURES_FILE).read_text())
    assert data["counts"]["depth"] == 4 and data["sequence"] == 3 and data["unix_time"] == clock.t
    clock.t += 0.1
    assert not tick(svc)  # within the interval
    clock.t += 0.15
    assert tick(svc) and svc.published == 2

    # The lease runs out: nothing more is written
    old = clock.t - feature_stream.LEASE_S - 1
    os.utime(tmp_path / feature_stream.LEASE_FILE, (old, old))
    clock.t += feature_stream.LEASE_S + 1
    assert not tick(svc) and svc.published == 2 and not svc.wanted


def test_service_idle_cost_is_one_comparison_per_frame(tmp_path, monkeypatch):
    clock = Clock(time.time())
    svc = service(tmp_path, clock)
    stats = []
    real_fresh = feature_stream._lease_fresh
    monkeypatch.setattr(feature_stream, "_lease_fresh", lambda *a: stats.append(1) or real_fresh(*a))
    uv = np.zeros((300, 2))
    for _ in range(150):  # 10 s at 15 fps
        clock.t += 1 / 15
        svc.tick(uv, (400, 640))
    assert 18 <= len(stats) <= 20  # one stat every 0.5 s
    start = time.perf_counter()
    for _ in range(10000):
        svc.tick(uv, (400, 640))
    assert (time.perf_counter() - start) / 10000 < 50e-6


def test_service_errors_never_reach_the_vio_loop(tmp_path):
    clock = Clock(time.time())
    logs = []
    svc = feature_stream.FeatureStreamService(tmp_path, (640, 400), clock=clock, log=logs.append)
    feature_stream.touch_lease(tmp_path)
    assert not svc.tick(np.zeros((3, 2)), "not a shape")
    assert svc.errors == 1 and any("Feature stream:" in m for m in logs)


def test_publishing_300_features_is_cheap(tmp_path):
    clock = Clock(time.time())
    svc = service(tmp_path, clock, interval=0.0)
    feature_stream.touch_lease(tmp_path)
    rng = np.random.default_rng(1)
    uv = rng.uniform(0, 640, (300, 2))
    valid = rng.random(300) > 0.3
    matched = np.flatnonzero(rng.random(300) > 0.2)
    inliers = np.flatnonzero(rng.random(len(matched)) > 0.3)
    start = time.perf_counter()
    for _ in range(50):
        clock.t += 0.01
        assert svc.tick(uv, (400, 640), 1, True, None, valid, matched, inliers)
    took = (time.perf_counter() - start) / 50
    print(f"features.json with 300 features: {took * 1000:.3f} ms per write")
    assert took < 5e-3


def test_latest_file_returns_each_new_file_once_and_only_if_fresh(tmp_path):
    clock = Clock(time.time())
    path = tmp_path / feature_stream.FEATURES_FILE
    reader = feature_stream.LatestFile(path, max_age_s=3.0, clock=clock)
    assert reader.read_new() is None
    path.write_bytes(b'{"a": 1}')
    assert reader.read_new() == b'{"a": 1}'
    assert reader.read_new() is None
    path.write_bytes(b'{"a": 22}')
    assert reader.read_new() == b'{"a": 22}'
    path.write_bytes(b'{"a": 333}')
    clock.t += 10
    assert reader.read_new() is None  # left over from a stopped VIO


def test_sse_event_framing():
    assert feature_stream.sse_event("features", b'{"a": 1}') == b'event: features\ndata: {"a": 1}\n\n'
    assert feature_stream.sse_event("status", '{"b":\n2}') == b'event: status\ndata: {"b":2}\n\n'


def test_odometry_reports_which_features_have_depth():
    depth = np.full((400, 640), 2000, np.uint16)
    depth[:, :100] = 0
    odometry = PnPOdometry(Intrinsics(300.0, 300.0, 320.0, 200.0), image_size=(640, 400))
    uv = np.array([[50.0, 100.0], [300.0, 100.0], [500.0, 300.0]])
    frame = odometry.process(np.array([1, 2, 3]), uv, depth, 1.0)
    assert frame.depth_valid.tolist() == [False, True, True] and frame.valid_depth == 2
    frame = odometry.process(np.array([1, 2, 3]), uv, depth, 1.1)
    assert frame.depth_valid.tolist() == [False, True, True]
    assert frame.matched_index.tolist() == [1, 2]
    states = feature_stream.feature_states(3, frame.depth_valid, frame.matched_index, frame.pose.inlier_index)
    assert states[0] == NO_DEPTH and set(states[1:].tolist()) <= {MATCHED, OUTLIER, INLIER}


@pytest.mark.parametrize("decimation", [1, 2])
def test_odometry_and_stream_agree_on_the_depth_grid(decimation):
    """The features are drawn where the odometry looked their depth up."""
    from pnp_odometry import scale_to_depth_grid
    uv = np.array([[123.0, 45.0], [600.0, 380.0]])
    shape = (400 // decimation, 640 // decimation)
    np.testing.assert_allclose(feature_stream.to_frame_grid(uv, (640, 400), (shape[1], shape[0])),
                               scale_to_depth_grid(uv, (640, 400), shape))
