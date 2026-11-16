import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pnp_odometry import (  # noqa: E402
    Intrinsics,
    PnPOdometry,
    PnPParams,
    back_project,
    camera_to_body_frd,
    confidence_from_inliers,
    integrate_gyro,
    lookup_depth,
    matrix_to_rotvec,
    rotation_angle,
    rotvec_to_matrix,
    solve_relative_pose,
)

WIDTH, HEIGHT = 640, 400
K = Intrinsics(fx=400.0, fy=400.0, cx=320.0, cy=200.0)


def project(points, intrinsics=K):
    return np.stack([
        intrinsics.fx * points[:, 0] / points[:, 2] + intrinsics.cx,
        intrinsics.fy * points[:, 1] / points[:, 2] + intrinsics.cy,
    ], axis=1)


def random_scene(rng, n=200):
    """Points in C0 that project inside the image, 1.5-6 m away."""
    uv = np.stack([rng.uniform(20, WIDTH - 20, n), rng.uniform(20, HEIGHT - 20, n)], axis=1)
    z = rng.uniform(1.5, 6.0, n)
    return back_project(uv, z, K)


def make_motion(rotvec_01, position_delta):
    """
    Camera motion as R_01 (current relative to previous) and displacement expressed in C1.
    Returns (R_10, t_10) for X1 = R_10 X0 + t_10.
    """
    R_01 = rotvec_to_matrix(rotvec_01)
    R_10 = R_01.T
    t_10 = -np.asarray(position_delta, dtype=np.float64)
    return R_01, R_10, t_10


def transform(points, R_10, t_10):
    return points @ R_10.T + t_10


def test_recovers_known_motion():
    rng = np.random.default_rng(0)
    P0 = random_scene(rng)
    rotvec = np.radians([1.0, -2.0, 1.5])
    delta = np.array([0.05, -0.02, 0.10])
    R_01, R_10, t_10 = make_motion(rotvec, delta)
    uv1 = project(transform(P0, R_10, t_10)) + rng.normal(0, 0.3, (len(P0), 2))

    result = solve_relative_pose(P0, uv1, K)

    assert result.ok, result.reason
    np.testing.assert_allclose(result.position_delta, delta, atol=5e-3)
    np.testing.assert_allclose(result.angle_delta, rotvec, atol=math.radians(0.1))
    assert result.inliers >= 0.95 * len(P0)
    assert result.confidence > 90


def test_recovers_motion_with_gyro_prior():
    rng = np.random.default_rng(1)
    P0 = random_scene(rng)
    rotvec = np.radians([0.0, 5.0, 0.0])
    delta = np.array([0.0, 0.0, 0.2])
    R_01, R_10, t_10 = make_motion(rotvec, delta)
    uv1 = project(transform(P0, R_10, t_10))

    prior = rotvec_to_matrix(rotvec + np.radians([0.3, -0.3, 0.2]))  # slightly biased gyro
    result = solve_relative_pose(P0, uv1, K, R_prior=prior)

    assert result.ok, result.reason
    np.testing.assert_allclose(result.position_delta, delta, atol=1e-3)
    assert result.gyro_disagreement_deg < 1.0


def test_handles_outliers():
    rng = np.random.default_rng(2)
    P0 = random_scene(rng, 300)
    rotvec = np.radians([-1.0, 0.5, 3.0])
    delta = np.array([-0.08, 0.03, 0.04])
    R_01, R_10, t_10 = make_motion(rotvec, delta)
    uv1 = project(transform(P0, R_10, t_10)) + rng.normal(0, 0.3, (len(P0), 2))

    # 30% gross outliers: wrong pixel matches and wrong depths
    bad = rng.choice(len(P0), size=90, replace=False)
    uv1[bad[:45]] += rng.uniform(-60, 60, (45, 2))
    P0 = P0.copy()
    P0[bad[45:]] *= rng.uniform(0.3, 3.0, (45, 1))

    result = solve_relative_pose(P0, uv1, K)

    assert result.ok, result.reason
    np.testing.assert_allclose(result.position_delta, delta, atol=1e-2)
    np.testing.assert_allclose(result.angle_delta, rotvec, atol=math.radians(0.2))
    assert 180 <= result.inliers <= 230
    assert result.confidence < 80


def test_too_few_points():
    rng = np.random.default_rng(3)
    P0 = random_scene(rng, 8)
    uv1 = project(P0)
    result = solve_relative_pose(P0, uv1, K, params=PnPParams(min_inliers=15))
    assert not result.ok
    assert result.reason == "too_few_matches"


def test_pure_noise_is_rejected():
    rng = np.random.default_rng(4)
    P0 = random_scene(rng, 100)
    uv1 = np.stack([rng.uniform(0, WIDTH, 100), rng.uniform(0, HEIGHT, 100)], axis=1)
    result = solve_relative_pose(P0, uv1, K)
    assert not result.ok
    assert result.reason in ("too_few_inliers", "pnp_failed", "implausible_translation")


def test_gyro_gate_rejects_disagreement():
    rng = np.random.default_rng(5)
    P0 = random_scene(rng)
    rotvec = np.radians([0.0, 2.0, 0.0])
    R_01, R_10, t_10 = make_motion(rotvec, [0.0, 0.0, 0.05])
    uv1 = project(transform(P0, R_10, t_10))

    wrong_prior = rotvec_to_matrix(np.radians([0.0, -10.0, 0.0]))
    result = solve_relative_pose(P0, uv1, K, R_prior=wrong_prior, params=PnPParams(gyro_gate_deg=5.0))
    assert not result.ok
    assert result.reason == "gyro_mismatch"


def test_lookup_depth_median_and_rejection():
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    depth[100:105, 100:105] = 0          # invalid hole
    depth[200, 200] = 9000               # single spike, median ignores it
    depth[300:305, 300:305] = 40000      # beyond max depth
    uv = np.array([[102, 102], [200, 200], [302, 302], [0, 0], [-50, -50]], dtype=np.float64)

    z = lookup_depth(depth, uv, PnPParams(depth_window=2, max_depth_m=15.0))

    assert math.isnan(z[0])
    assert z[1] == pytest.approx(2.0)
    assert math.isnan(z[2])
    assert z[3] == pytest.approx(2.0)   # corner: only the in-image part of the window is used
    assert math.isnan(z[4])             # fully outside the image


def test_integrate_gyro_constant_rate():
    omega = np.array([0.1, -0.2, 0.3])
    times = np.arange(0.0, 1.0, 0.005)
    omegas = np.tile(omega, (len(times), 1))
    R = integrate_gyro(times, omegas, 0.2, 0.3)
    np.testing.assert_allclose(matrix_to_rotvec(R), omega * 0.1, atol=1e-9)

    # IMU mounted with a 90 degree yaw relative to the camera
    R_cam_imu = rotvec_to_matrix([0.0, 0.0, math.pi / 2])
    R = integrate_gyro(times, omegas, 0.2, 0.3, R_cam_imu=R_cam_imu)
    np.testing.assert_allclose(matrix_to_rotvec(R), R_cam_imu @ omega * 0.1, atol=1e-9)

    assert integrate_gyro(times, omegas, 2.0, 2.1) is None
    assert integrate_gyro([], [], 0.0, 0.1) is None


def test_camera_to_body_frd():
    # Forward-looking camera: optical z (forward) -> FRD x, x (right) -> FRD y, y (down) -> FRD z
    np.testing.assert_allclose(camera_to_body_frd([0, 0, 1]), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(camera_to_body_frd([1, 0, 0]), [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(camera_to_body_frd([0, 1, 0]), [0, 0, 1], atol=1e-12)
    # Camera pitched 90 degrees down: optical forward is body down
    np.testing.assert_allclose(camera_to_body_frd([0, 0, 1], 90.0), [0, 0, 1], atol=1e-12)
    # Rotation vectors map the same way: a turn to the right (about optical +y) is a positive FRD yaw
    np.testing.assert_allclose(camera_to_body_frd([0, 0.1, 0]), [0, 0, 0.1], atol=1e-12)


def test_confidence():
    assert confidence_from_inliers(0, 0, 60) == 0.0
    assert confidence_from_inliers(120, 120, 60) == pytest.approx(100.0)
    assert confidence_from_inliers(30, 30, 60) == pytest.approx(50.0)
    assert confidence_from_inliers(60, 120, 60) == pytest.approx(50.0)


def plane_depth_image(n_c, d_c, intrinsics=K):
    """Depth (uint16 mm) of the plane n.X = d (camera frame) rendered at every pixel."""
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays = np.stack([(u - intrinsics.cx) / intrinsics.fx, (v - intrinsics.cy) / intrinsics.fy, np.ones_like(u, float)], axis=-1)
    z = d_c / (rays @ n_c)
    return np.clip(np.rint(z * 1000.0), 0, 65535).astype(np.uint16)


def test_odometry_end_to_end_with_depth_images():
    """Two frames looking at a slanted wall; features carry ids, depth comes from rendered images."""
    rng = np.random.default_rng(6)
    n0 = np.array([0.0, 0.2, 1.0]); n0 /= np.linalg.norm(n0)
    d0 = 3.0
    depth0 = plane_depth_image(n0, d0)

    uv0 = np.stack([rng.uniform(10, WIDTH - 10, 250), rng.uniform(10, HEIGHT - 10, 250)], axis=1)
    P0 = back_project(uv0, lookup_depth(depth0, uv0, PnPParams(depth_window=0)), K)
    ids = np.arange(1000, 1250)

    rotvec = np.radians([0.5, 1.0, -0.5])
    delta = np.array([0.03, 0.0, 0.06])
    R_01, R_10, t_10 = make_motion(rotvec, delta)

    P1 = transform(P0, R_10, t_10)
    uv1 = project(P1)
    keep = (uv1[:, 0] > 2) & (uv1[:, 0] < WIDTH - 2) & (uv1[:, 1] > 2) & (uv1[:, 1] < HEIGHT - 2)
    # Plane in C1: n1 = R_10 n0, d1 = d0 + n1.t_10
    n1 = R_10 @ n0
    depth1 = plane_depth_image(n1, d0 + n1 @ t_10)

    odom = PnPOdometry(K)
    first = odom.process(ids, uv0, depth0, timestamp=10.0)
    assert not first.pose.ok and first.pose.reason == "no_reference"
    assert first.valid_depth == 250

    # Drop a few tracks and add new ids, as the device tracker does
    gyro_prior = rotvec_to_matrix(rotvec)
    second = odom.process(np.append(ids[keep][5:], [5000, 5001]),
                          np.vstack([uv1[keep][5:], [[100, 100], [200, 200]]]),
                          depth1, timestamp=10.0667, R_prior=gyro_prior)
    assert second.pose.ok, second.pose.reason
    assert second.dt == pytest.approx(0.0667)
    np.testing.assert_allclose(second.pose.position_delta, delta, atol=5e-3)
    np.testing.assert_allclose(second.pose.angle_delta, rotvec, atol=math.radians(0.2))

    # Tracking loss: all ids new -> no matches; the next frame starts from this one
    lost = odom.process(np.arange(9000, 9010), uv1[:10], depth1, timestamp=10.133)
    assert not lost.pose.ok
    assert lost.pose.reason == "too_few_matches"
    still = odom.process(np.arange(9000, 9010), uv1[:10], depth1, timestamp=10.2)
    assert still.dt == pytest.approx(10.2 - 10.133)


def test_non_increasing_timestamp():
    odom = PnPOdometry(K)
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    uv = np.array([[100.0, 100.0]] * 20)
    odom.process(np.arange(20), uv, depth, timestamp=1.0)
    frame = odom.process(np.arange(20), uv, depth, timestamp=1.0)
    assert not frame.pose.ok
    assert frame.pose.reason == "non_increasing_timestamp"


def test_rotation_angle():
    assert rotation_angle(np.eye(3)) == pytest.approx(0.0)
    assert rotation_angle(rotvec_to_matrix([0, 0, 0.3])) == pytest.approx(0.3)
