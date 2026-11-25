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
    camera_to_body_matrix,
    camera_to_body_rotation,
    confidence_from_inliers,
    integrate_gyro,
    intrinsics_from_frame,
    lever_arm_correction,
    lookup_depth,
    matrix_to_rotvec,
    optical_roll_matrix,
    refracted_intrinsics,
    rotation_angle,
    rotvec_to_matrix,
    scale_to_depth_grid,
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


def test_integrate_gyro_over_a_long_interval_with_a_dropout():
    omega = np.array([0.0, 0.5, 0.0])
    times = np.arange(0.0, 3.0, 0.005)
    omegas = np.tile(omega, (len(times), 1))
    # A held reference spans many frames: still exact over a full second
    R = integrate_gyro(times, omegas, 0.5, 1.5)
    np.testing.assert_allclose(matrix_to_rotvec(R), omega * 1.0, atol=1e-9)
    # A hole in the middle would be bridged by holding one sample: no prior instead
    holed = (times < 0.9) | (times > 1.2)
    assert integrate_gyro(times[holed], omegas[holed], 0.5, 1.5) is None
    assert integrate_gyro(times[holed], omegas[holed], 0.2, 0.8) is not None


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


def test_intrinsics_from_frame():
    matrix = [[400.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]]
    assert intrinsics_from_frame(matrix, (WIDTH, HEIGHT), (WIDTH, HEIGHT)) == K
    # Decimated depth frame: matrix on the 320x200 grid, scaled back to the feature grid
    half = [[200.0, 0.0, 160.0], [0.0, 200.0, 100.0], [0.0, 0.0, 1.0]]
    assert intrinsics_from_frame(half, (320, 200), (WIDTH, HEIGHT)) == K
    assert intrinsics_from_frame(np.array(matrix), [WIDTH, HEIGHT], (WIDTH, HEIGHT)) == K
    # fx != fy survives the scaling
    anisotropic = [[81.55, 0.0, 164.7], [0.0, 118.55, 99.5], [0.0, 0.0, 1.0]]
    assert intrinsics_from_frame(anisotropic, (320, 200), (WIDTH, HEIGHT)) == Intrinsics(163.1, 237.1, 329.4, 199.0)


@pytest.mark.parametrize("matrix, size", [
    (None, (WIDTH, HEIGHT)),
    ([[1.0, 0.0], [0.0, 1.0]], (WIDTH, HEIGHT)),
    ([[400.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], (1280, 720)),  # different aspect
    ([[400.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], (0, 0)),
    ([[0.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], (WIDTH, HEIGHT)),
    ([[400.0, 0.0, 900.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], (WIDTH, HEIGHT)),  # cx outside
    ([[float("nan"), 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], (WIDTH, HEIGHT)),
    ([[400.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]], None),
])
def test_intrinsics_from_frame_rejects_implausible(matrix, size):
    assert intrinsics_from_frame(matrix, size, (WIDTH, HEIGHT)) is None


def test_scale_to_depth_grid():
    uv = np.array([[100.0, 50.0], [639.0, 399.0]])
    assert scale_to_depth_grid(uv, (WIDTH, HEIGHT), (HEIGHT, WIDTH)) is uv
    assert scale_to_depth_grid(uv, None, (200, 320)) is uv
    np.testing.assert_allclose(scale_to_depth_grid(uv, (WIDTH, HEIGHT), (200, 320)), uv / 2)


def test_decimated_depth_gives_the_same_pose():
    """Feature pixels stay on the full grid; depth lookups go to the decimated grid."""
    rng = np.random.default_rng(7)
    n0 = np.array([0.1, 0.2, 1.0]); n0 /= np.linalg.norm(n0)
    depth0 = plane_depth_image(n0, 3.0)
    uv0 = np.stack([rng.uniform(10, WIDTH - 10, 200), rng.uniform(10, HEIGHT - 10, 200)], axis=1)
    P0 = back_project(uv0, lookup_depth(depth0, uv0, PnPParams(depth_window=0)), K)
    rotvec, delta = np.radians([0.3, -0.8, 0.2]), np.array([0.02, 0.01, 0.05])
    _, R_10, t_10 = make_motion(rotvec, delta)
    uv1 = project(transform(P0, R_10, t_10))
    n1 = R_10 @ n0
    depth1 = plane_depth_image(n1, 3.0 + n1 @ t_10)

    full, decimated = PnPOdometry(K, image_size=(WIDTH, HEIGHT)), PnPOdometry(K, image_size=(WIDTH, HEIGHT))
    ids = np.arange(200)
    full.process(ids, uv0, depth0, 1.0)
    decimated.process(ids, uv0, depth0[::2, ::2], 1.0)
    a = full.process(ids, uv1, depth1, 1.1)
    b = decimated.process(ids, uv1, depth1[::2, ::2], 1.1)
    assert a.pose.ok and b.pose.ok
    assert b.valid_depth >= 0.95 * a.valid_depth
    np.testing.assert_allclose(b.pose.position_delta, delta, atol=1e-2)
    np.testing.assert_allclose(b.pose.angle_delta, a.pose.angle_delta, atol=math.radians(0.2))

    # Without image_size, lookups use full-grid pixels on the half-size depth: most fall outside it
    # and the rest read depth from the wrong place
    naive = PnPOdometry(K)
    naive.process(ids, uv0, depth0[::2, ::2], 1.0)
    c = naive.process(ids, uv1, depth1[::2, ::2], 1.1)
    assert c.valid_depth < 0.4 * a.valid_depth


FLAT_DEPTH = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
GRID_UV = np.array([(u, v) for u in np.linspace(50, 590, 6) for v in np.linspace(50, 350, 5)])
# Same ids at shuffled pixels: plenty of matches, no consistent pose
SHUFFLED_UV = GRID_UV[np.random.default_rng(42).permutation(len(GRID_UV))]


def test_reset_drops_the_reference():
    odom = PnPOdometry(K)
    odom.process(np.arange(30), GRID_UV, FLAT_DEPTH, 1.0)
    assert odom.process(np.arange(30), GRID_UV, FLAT_DEPTH, 1.1).pose.ok
    assert odom.process(np.arange(30), SHUFFLED_UV, FLAT_DEPTH, 1.2).gap_frames == 0
    assert odom.gap_frames == 1
    odom.reset()
    assert odom.prev_points is None and odom.prev_timestamp is None and odom.gap_frames == 0
    assert odom.process(np.arange(30), GRID_UV, FLAT_DEPTH, 1.3).pose.reason == "no_reference"
    frame = odom.process(np.arange(30), GRID_UV, FLAT_DEPTH, 1.4)
    assert frame.pose.ok and frame.dt == pytest.approx(0.1) and frame.gap_frames == 0


def test_non_increasing_timestamp_keeps_the_reference():
    odom = PnPOdometry(K)
    ids = np.arange(30)
    odom.process(ids, GRID_UV, FLAT_DEPTH, timestamp=1.0)
    reference = odom.prev_points
    frame = odom.process(ids, GRID_UV, FLAT_DEPTH, timestamp=1.0)
    assert not frame.pose.ok
    assert frame.pose.reason == "non_increasing_timestamp"
    # An earlier frame is never adopted, even with nothing left to match
    earlier = odom.process(ids + 100, GRID_UV, FLAT_DEPTH, timestamp=0.9)
    assert earlier.pose.reason == "non_increasing_timestamp" and not earlier.reference_reset
    assert odom.prev_timestamp == 1.0 and odom.prev_points is reference and odom.gap_frames == 0
    later = odom.process(ids, GRID_UV, FLAT_DEPTH, timestamp=1.1)
    assert later.pose.ok and later.dt == pytest.approx(0.1)


def two_step_plane_scene(seed, delta_a, delta_b):
    """
    Slanted wall seen from C0, C1, C2. Per-step motions (R_01, displacement in the later frame) are
    ([0.5, 1, -0.5] deg, delta_a) for C0 -> C1 and ([-0.3, 0.8, 0.2] deg, delta_b) for C1 -> C2.
    Returns ids, (uv, depth) for each frame, and R_20, t_20 (X2 = R_20 X0 + t_20).
    """
    rng = np.random.default_rng(seed)
    n0 = np.array([0.0, 0.2, 1.0]); n0 /= np.linalg.norm(n0)
    d0 = 3.0
    depth0 = plane_depth_image(n0, d0)
    uv0 = np.stack([rng.uniform(40, WIDTH - 40, 250), rng.uniform(40, HEIGHT - 40, 250)], axis=1)
    P0 = back_project(uv0, lookup_depth(depth0, uv0, PnPParams(depth_window=0)), K)

    _, R_10, t_10 = make_motion(np.radians([0.5, 1.0, -0.5]), delta_a)
    _, R_21, t_21 = make_motion(np.radians([-0.3, 0.8, 0.2]), delta_b)
    R_20, t_20 = R_21 @ R_10, R_21 @ t_10 + t_21

    views = [(uv0, depth0)]
    inside = np.ones(len(P0), dtype=bool)
    for R, t in ((R_10, t_10), (R_20, t_20)):
        uv = project(transform(P0, R, t))
        inside &= (uv[:, 0] > 2) & (uv[:, 0] < WIDTH - 2) & (uv[:, 1] > 2) & (uv[:, 1] < HEIGHT - 2)
        n = R @ n0
        views.append((uv, plane_depth_image(n, d0 + n @ t)))
    ids = np.flatnonzero(inside)
    return ids, [(uv[ids], depth) for uv, depth in views], R_20, t_20


def test_failed_frame_keeps_the_last_good_reference():
    """Frame 0 good, frame 1 fails (LK errors on most features), frame 2 is solved against frame 0."""
    delta_b = np.array([0.04, 0.01, 0.05])
    ids, ((uv0, depth0), (uv1, depth1), (uv2, depth2)), R_20, t_20 = two_step_plane_scene(
        9, np.array([0.03, 0.0, 0.06]), delta_b)
    rng = np.random.default_rng(10)
    bad = rng.random(len(ids)) < 0.8
    uv1_blurred = uv1 + bad[:, None] * rng.uniform(-80, 80, (len(ids), 2))

    odom = PnPOdometry(K)
    odom.process(ids, uv0, depth0, 10.0)
    failed = odom.process(ids, uv1_blurred, depth1, 10.0667)
    assert failed.pose.reason in ("too_few_inliers", "pnp_failed")
    assert failed.pose.matches == len(ids)
    assert failed.dt == pytest.approx(0.0667) and failed.reference_age_s == pytest.approx(0.0667)
    assert failed.gap_frames == 0 and not failed.reference_reset
    assert odom.prev_timestamp == 10.0 and odom.gap_frames == 1

    R_02 = R_20.T
    good = odom.process(ids, uv2, depth2, 10.1333, R_prior=R_02)  # gyro integrated over the whole gap
    assert good.pose.ok, good.pose.reason
    assert good.dt == pytest.approx(0.1333) and good.reference_age_s == pytest.approx(0.1333)
    assert good.gap_frames == 1 and not good.reference_reset
    # The delta covers 0 -> 2, the interval time_delta_us = dt * 1e6 reports, not just 1 -> 2
    np.testing.assert_allclose(good.pose.position_delta, -t_20, atol=5e-3)
    np.testing.assert_allclose(good.pose.angle_delta, matrix_to_rotvec(R_02), atol=math.radians(0.2))
    assert np.linalg.norm(good.pose.position_delta - delta_b) > 0.03
    assert good.pose.gyro_disagreement_deg < 0.5
    # A good frame becomes the reference
    assert odom.prev_timestamp == 10.1333 and odom.gap_frames == 0
    assert odom.process(ids, uv2, depth2, 10.2).pose.ok


def test_translation_gate_allows_one_step_per_bridged_frame():
    ids, ((uv0, depth0), (uv1, depth1), (uv2, depth2)), _, t_20 = two_step_plane_scene(
        11, np.array([0.0, 0.0, 0.07]), np.array([0.0, 0.0, 0.07]))
    params = PnPParams(max_translation_m=0.1)
    assert 0.1 < np.linalg.norm(t_20) < 0.2

    direct = PnPOdometry(K, params)
    direct.process(ids, uv0, depth0, 1.0)
    assert direct.process(ids, uv2, depth2, 1.2).pose.reason == "implausible_translation"

    bridged = PnPOdometry(K, params)
    bridged.process(ids, uv0, depth0, 1.0)
    shuffled = uv1[np.random.default_rng(12).permutation(len(ids))]
    assert not bridged.process(ids, shuffled, depth1, 1.1).pose.ok
    frame = bridged.process(ids, uv2, depth2, 1.2)
    assert frame.pose.ok, frame.pose.reason
    assert frame.gap_frames == 1
    np.testing.assert_allclose(frame.pose.position_delta, -t_20, atol=5e-3)


def test_stale_reference_is_replaced_after_max_age():
    ids = np.arange(30)
    odom = PnPOdometry(K, PnPParams(max_reference_age_s=0.25))
    odom.process(ids, GRID_UV, FLAT_DEPTH, 1.0)
    for n, t in enumerate((1.1, 1.2)):
        frame = odom.process(ids, SHUFFLED_UV, FLAT_DEPTH, t)
        assert not frame.pose.ok and frame.pose.matches == 30
        assert frame.gap_frames == n and not frame.reference_reset
        assert odom.prev_timestamp == 1.0
    stale = odom.process(ids, SHUFFLED_UV, FLAT_DEPTH, 1.3)
    assert not stale.pose.ok and stale.reference_reset
    assert stale.reference_age_s == pytest.approx(0.3) and stale.gap_frames == 2
    assert odom.prev_timestamp == 1.3 and odom.gap_frames == 0
    # The failed frame is the new reference: the next frame is solved against it
    frame = odom.process(ids, SHUFFLED_UV, FLAT_DEPTH, 1.4)
    assert frame.pose.ok and frame.dt == pytest.approx(0.1) and frame.gap_frames == 0


def test_zero_max_age_holds_the_reference():
    ids = np.arange(30)
    odom = PnPOdometry(K, PnPParams(max_reference_age_s=0.0))
    odom.process(ids, GRID_UV, FLAT_DEPTH, 1.0)
    for t in (2.0, 5.0, 30.0):
        assert not odom.process(ids, SHUFFLED_UV, FLAT_DEPTH, t).reference_reset
    frame = odom.process(ids, GRID_UV, FLAT_DEPTH, 31.0)
    assert frame.pose.ok and frame.dt == pytest.approx(30.0) and frame.gap_frames == 3


def test_dying_reference_is_replaced_by_the_current_frame():
    ids = np.arange(30)
    odom = PnPOdometry(K, PnPParams(min_inliers=15))
    odom.process(ids, GRID_UV, FLAT_DEPTH, 1.0)
    # 20 reference features still tracked: the failure keeps the reference
    frame = odom.process(ids[:20], SHUFFLED_UV[:20], FLAT_DEPTH, 1.1)
    assert not frame.pose.ok and frame.pose.matches == 20 and not frame.reference_reset
    assert odom.prev_timestamp == 1.0
    # Only 10 left (plus new ids): it can never solve again, so this frame takes over
    new_ids = np.concatenate([ids[:10], np.arange(100, 120)])
    dying = odom.process(new_ids, GRID_UV, FLAT_DEPTH, 1.2)
    assert dying.pose.reason == "too_few_matches" and dying.pose.matches == 10
    assert dying.reference_reset and dying.gap_frames == 1
    assert odom.prev_timestamp == 1.2 and sorted(odom.prev_points) == sorted(new_ids.tolist())
    frame = odom.process(new_ids, GRID_UV, FLAT_DEPTH, 1.3)
    assert frame.pose.ok and frame.pose.matches == 30 and frame.dt == pytest.approx(0.1)


# OAK-D-W-PoE CAM_B at 640x400: what the device rectifies to (alpha 0) and the raw calibration
K_RECTIFIED = Intrinsics(fx=163.1, fy=237.1, cx=329.4, cy=199.0)
K_RAW = Intrinsics(fx=285.11, fy=285.05, cx=323.2, cy=199.1)


def anisotropic_wall_motion(delta, seed=13, depth_correction=1.0):
    """
    A wall 2 m away seen through K_RECTIFIED, the camera translating by delta (no rotation). Returns
    the recovered position_delta for odometry run with K_RECTIFIED and with K_RAW, the pre-fix bug.
    """
    rng = np.random.default_rng(seed)
    n = np.array([0.0, 0.15, 1.0]); n /= np.linalg.norm(n)
    depth0 = plane_depth_image(n, 2.0, K_RECTIFIED)
    uv0 = np.stack([rng.uniform(15, WIDTH - 15, 300), rng.uniform(15, HEIGHT - 15, 300)], axis=1)
    P0 = back_project(uv0, lookup_depth(depth0, uv0, PnPParams(depth_window=0)), K_RECTIFIED)
    t_10 = -np.asarray(delta, dtype=np.float64)
    uv1 = project(P0 + t_10, K_RECTIFIED)
    depth1 = plane_depth_image(n, 2.0 + n @ t_10, K_RECTIFIED)
    keep = (uv1[:, 0] > 2) & (uv1[:, 0] < WIDTH - 2) & (uv1[:, 1] > 2) & (uv1[:, 1] < HEIGHT - 2)
    ids = np.arange(len(uv0))
    recovered = {}
    for name, intrinsics in (("rectified", K_RECTIFIED), ("raw", K_RAW)):
        odom = PnPOdometry(intrinsics, PnPParams(depth_correction=depth_correction), image_size=(WIDTH, HEIGHT))
        odom.process(ids, uv0, depth0, 1.0)
        frame = odom.process(ids[keep], uv1[keep], depth1, 1.1, R_prior=np.eye(3))
        assert frame.pose.ok, frame.pose.reason
        recovered[name] = frame.pose.position_delta
    return recovered


@pytest.mark.parametrize("axis, raw_scale", [(0, 163.1 / 285.11), (1, 237.1 / 285.05), (2, 1.0)])
def test_anisotropic_rectified_k_gives_true_scale_and_raw_k_reproduces_the_bug(axis, raw_scale):
    """Regression for the raw-K bug: lateral x0.57, vertical x0.83, forward x1.0 with the raw calibration."""
    delta = np.zeros(3)
    delta[axis] = 0.05
    recovered = anisotropic_wall_motion(delta)
    np.testing.assert_allclose(recovered["rectified"], delta, atol=1e-3)
    assert recovered["raw"][axis] / 0.05 == pytest.approx(raw_scale, abs=0.02)
    assert raw_scale > 0.95 or recovered["raw"][axis] / 0.05 < 0.9


def test_depth_correction_scales_translation_linearly():
    delta = np.array([0.03, 0.02, 0.05])
    recovered = anisotropic_wall_motion(delta, depth_correction=1.3)
    np.testing.assert_allclose(recovered["rectified"], 1.3 * delta, atol=1.5e-3)


def test_lookup_depth_applies_unit_and_correction_after_the_range_check():
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    depth[0:5, 0:5] = 100  # 0.1 m: below min depth whatever the correction
    uv = np.array([[320.0, 200.0], [2.0, 2.0]])
    z = lookup_depth(depth, uv, PnPParams(depth_correction=1.25, min_depth_m=0.2))
    assert z[0] == pytest.approx(2.5)
    assert math.isnan(z[1])
    assert lookup_depth(depth, uv[:1], PnPParams(depth_unit_scale=0.01, max_depth_m=30.0))[0] == pytest.approx(20.0)


N_WATER = 1.333


def test_refracted_intrinsics_scales_the_focal_lengths_only():
    water = refracted_intrinsics(K_RECTIFIED, N_WATER)
    assert water.fx == pytest.approx(163.1 * N_WATER) and water.fy == pytest.approx(237.1 * N_WATER)
    assert (water.cx, water.cy) == (K_RECTIFIED.cx, K_RECTIFIED.cy)
    assert refracted_intrinsics(K_RECTIFIED, 1.0) == K_RECTIFIED
    assert refracted_intrinsics(K_RECTIFIED) == K_RECTIFIED


def test_odometry_uses_the_refracted_matrix_and_keeps_the_calibration():
    odom = PnPOdometry(K_RECTIFIED, PnPParams(refractive_index=N_WATER))
    assert odom.calibration_intrinsics == K_RECTIFIED
    assert odom.intrinsics == refracted_intrinsics(K_RECTIFIED, N_WATER)
    assert PnPOdometry(K_RECTIFIED).intrinsics == K_RECTIFIED


def test_lookup_depth_multiplies_refractive_index_and_depth_scale():
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    depth[0:5, 0:5] = 150  # below min depth on the device value, whatever the factors
    uv = np.array([[320.0, 200.0], [2.0, 2.0]])
    assert lookup_depth(depth, uv[:1], PnPParams())[0] == pytest.approx(2.0)
    assert lookup_depth(depth, uv[:1], PnPParams(refractive_index=1.0))[0] == pytest.approx(2.0)
    z = lookup_depth(depth, uv, PnPParams(refractive_index=N_WATER, depth_correction=1.1, min_depth_m=0.2))
    assert z[0] == pytest.approx(2.0 * N_WATER * 1.1) and math.isnan(z[1])


def flat_port_motion(delta, n, seed=21):
    """
    A wall 2 m away seen from behind a flat port in water of index n, the camera translating by delta.
    Paraxially the in-air K_RECTIFIED images the scene like n*fx, n*fy, and the device's stereo depth is
    the true Z / n. Returns the position_delta PnP recovers with refractive_index n and with 1.0.
    """
    rng = np.random.default_rng(seed)
    K_water = refracted_intrinsics(K_RECTIFIED, n)
    normal = np.array([0.1, 0.15, 1.0]); normal /= np.linalg.norm(normal)

    def device_depth(d):
        return np.rint(plane_depth_image(normal, d, K_water) / n).astype(np.uint16)

    uv0 = np.stack([rng.uniform(15, WIDTH - 15, 300), rng.uniform(15, HEIGHT - 15, 300)], axis=1)
    depth0 = device_depth(2.0)
    P0 = back_project(uv0, n * lookup_depth(depth0, uv0, PnPParams(depth_window=0)), K_water)
    t_10 = -np.asarray(delta, dtype=np.float64)
    uv1 = project(P0 + t_10, K_water)
    depth1 = device_depth(2.0 + normal @ t_10)
    keep = (uv1[:, 0] > 2) & (uv1[:, 0] < WIDTH - 2) & (uv1[:, 1] > 2) & (uv1[:, 1] < HEIGHT - 2)
    ids = np.arange(len(uv0))
    recovered = {}
    for index in (n, 1.0):
        odom = PnPOdometry(K_RECTIFIED, PnPParams(refractive_index=index), image_size=(WIDTH, HEIGHT))
        odom.process(ids, uv0, depth0, 1.0)
        frame = odom.process(ids[keep], uv1[keep], depth1, 1.1, R_prior=np.eye(3))
        assert frame.pose.ok, frame.pose.reason
        recovered[index] = frame.pose.position_delta
    return recovered


def test_refractive_index_recovers_true_motion_behind_a_flat_port():
    delta = np.array([0.03, -0.02, 0.06])
    recovered = flat_port_motion(delta, N_WATER)
    water, air = recovered[N_WATER], recovered[1.0]
    np.testing.assert_allclose(water, delta, atol=1.5e-3)
    # Same pixel motion, larger translation with n > 1, in the same direction
    assert np.linalg.norm(water) > np.linalg.norm(air) * 1.1
    assert np.all(np.sign(water) == np.sign(air)) and np.dot(water, air) / (
        np.linalg.norm(water) * np.linalg.norm(air)) > 0.95
    # Paraxially the in-air model only shrinks motion along the optical axis: lateral X = (u - cx) Z / f is
    # the same with (n f, n Z) as with (f, Z)
    np.testing.assert_allclose(air[:2], delta[:2], atol=1.5e-3)
    assert air[2] == pytest.approx(delta[2] / N_WATER, abs=1.5e-3)


def test_refractive_index_one_is_the_in_air_model():
    delta = np.array([0.03, 0.02, 0.05])
    recovered = flat_port_motion(delta, 1.0)
    np.testing.assert_allclose(recovered[1.0], delta, atol=1.5e-3)


LEVER_ARM = np.array([0.1655, -0.0069, 0.180])


def lever_arm_body_delta(position_delta, R_01, lever_arm):
    """Body (IMU) displacement from the camera's, as pnp_vio logs it in lever_delta_position."""
    return np.asarray(position_delta, dtype=np.float64) - lever_arm_correction(R_01, lever_arm)


def yaw_frd(angle):
    """Rotation about body z (down): positive yaws the nose right."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def camera_observation(R_wb1, p_b1, camera_angle, lever_arm=LEVER_ARM, camera_roll=0.0):
    """
    The vehicle body (IMU) goes from the world origin (R_wb0 = I) to (R_wb1, p_b1); the camera sits at
    lever_arm in body FRD, pitched by camera_angle and rolled by camera_roll. Returns what PnP and the gyro
    report: the camera displacement expressed in its current optical frame, and R_01 in the optical frame.
    """
    M = camera_to_body_matrix(camera_angle, camera_roll)  # optical -> body
    R_wc0, R_wc1 = M, R_wb1 @ M
    p_c0, p_c1 = lever_arm, p_b1 + R_wb1 @ lever_arm
    return R_wc1.T @ (p_c1 - p_c0), R_wc0.T @ R_wc1


@pytest.mark.parametrize("roll", [0.0, 90.0, 180.0, -90.0, 45.0])
@pytest.mark.parametrize("angle", [0.0, 30.0, 90.0, -45.0])
def test_camera_to_body_matrix_matches_the_vector_mapping(angle, roll):
    M = camera_to_body_matrix(angle, roll)
    np.testing.assert_allclose(M @ M.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(M) == pytest.approx(1.0)
    v = np.array([0.3, -0.2, 0.9])
    np.testing.assert_allclose(M @ v, camera_to_body_frd(v, angle, roll), atol=1e-12)
    # A rotation mapped to body has the mapped rotation vector
    rotvec = np.radians([3.0, -1.0, 7.0])
    np.testing.assert_allclose(matrix_to_rotvec(camera_to_body_rotation(rotvec_to_matrix(rotvec), angle, roll)),
                               camera_to_body_frd(rotvec, angle, roll), atol=1e-12)
    # The roll is about the optical axis: it never moves where the camera looks
    np.testing.assert_allclose(M @ [0, 0, 1], camera_to_body_frd([0, 0, 1], angle), atol=1e-12)


def test_roll_defaults_keep_the_unrolled_mapping():
    np.testing.assert_allclose(optical_roll_matrix(), np.eye(3), atol=1e-15)
    for angle in (0.0, 90.0, 30.0):
        np.testing.assert_allclose(camera_to_body_matrix(angle, 0.0), camera_to_body_matrix(angle), atol=1e-15)
    np.testing.assert_allclose(optical_roll_matrix(360.0), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(optical_roll_matrix(180.0), optical_roll_matrix(-180.0), atol=1e-12)
    # +90: physical image right is unrolled down, physical image top (-y) is unrolled right
    np.testing.assert_allclose(optical_roll_matrix(90.0) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(optical_roll_matrix(90.0) @ [0, -1, 0], [1, 0, 0], atol=1e-12)


# (camera_angle, camera_roll) -> where each optical axis points in body FRD
OPT_X, OPT_Y, OPT_Z = np.eye(3)
MOUNT_AXES = {
    # Forward: image right is body right, image down is body down
    (0.0, 0.0): {"+x": [0, 1, 0], "+y": [0, 0, 1], "+z": [1, 0, 0]},
    # Downward: looks down, image top (-y) forward, image right right
    (90.0, 0.0): {"+x": [0, 1, 0], "+y": [-1, 0, 0], "+z": [0, 0, 1]},
    # Downward rolled 90: image top right, image left (-x) forward
    (90.0, 90.0): {"+x": [-1, 0, 0], "+y": [0, -1, 0], "+z": [0, 0, 1]},
    # Downward rolled 180: image top back, image bottom (+y) forward
    (90.0, 180.0): {"+x": [0, -1, 0], "+y": [1, 0, 0], "+z": [0, 0, 1]},
    # Downward rolled -90: image top left, image right forward
    (90.0, -90.0): {"+x": [1, 0, 0], "+y": [0, 1, 0], "+z": [0, 0, 1]},
    # Forward rolled 90: image top right, image right down
    (0.0, 90.0): {"+x": [0, 0, 1], "+y": [0, -1, 0], "+z": [1, 0, 0]},
}


@pytest.mark.parametrize("mount", list(MOUNT_AXES), ids=[f"angle{a:g}-roll{r:g}" for a, r in MOUNT_AXES])
def test_mount_maps_each_optical_axis_to_its_body_axis(mount):
    angle, roll = mount
    axes = MOUNT_AXES[mount]
    for name, optical in (("+x", OPT_X), ("+y", OPT_Y), ("+z", OPT_Z)):
        expected = np.asarray(axes[name], dtype=float)
        # Translations along +-x_opt, +-y_opt, +z_opt
        np.testing.assert_allclose(camera_to_body_frd(0.1 * optical, angle, roll), 0.1 * expected, atol=1e-12)
        np.testing.assert_allclose(camera_to_body_frd(-0.1 * optical, angle, roll), -0.1 * expected, atol=1e-12)
        # A rotation about an optical axis is the same rotation about its body axis
        R_body = camera_to_body_rotation(rotvec_to_matrix(0.05 * optical), angle, roll)
        np.testing.assert_allclose(matrix_to_rotvec(R_body), 0.05 * expected, atol=1e-12)


def test_downward_forward_motion_and_yaw_are_body_forward_and_yaw():
    d, psi = 0.07, math.radians(4.0)
    # Moving forward the scene below slides towards the image bottom: the camera moves towards the image top
    np.testing.assert_allclose(camera_to_body_frd([0, -d, 0], 90.0), [d, 0, 0], atol=1e-12)
    # Yawing right turns a downward camera about its optical axis
    np.testing.assert_allclose(camera_to_body_frd([0, 0, psi], 90.0), [0, 0, psi], atol=1e-12)
    np.testing.assert_allclose(camera_to_body_rotation(rotvec_to_matrix([0, 0, psi]), 90.0), yaw_frd(psi),
                               atol=1e-12)


BODY_MOTIONS = {
    "forward": (np.zeros(3), np.array([0.06, 0.0, 0.0])),
    "strafe_and_descend": (np.zeros(3), np.array([0.0, 0.04, 0.03])),
    "yaw_right": (np.array([0.0, 0.0, math.radians(3.0)]), np.array([0.02, 0.0, 0.0])),
    "pitch_and_roll": (np.radians([1.5, -2.0, 0.5]), np.array([0.03, -0.02, 0.01])),
}


def mounted_frames(rng, R_wb1, p_b1, angle, roll, lever_arm=LEVER_ARM):
    """
    A scene in front of the camera mounted at (angle, roll) and lever_arm, seen before and after the body moves
    from the origin to (R_wb1, p_b1): (points in C0, points in C1).
    """
    M = camera_to_body_matrix(angle, roll)
    P0 = random_scene(rng)
    world = P0 @ M.T + lever_arm  # body at t0 = world
    R_wc1, p_c1 = R_wb1 @ M, p_b1 + R_wb1 @ lever_arm
    return P0, (world - p_c1) @ R_wc1


@pytest.mark.parametrize("motion", list(BODY_MOTIONS))
@pytest.mark.parametrize("angle, roll", [(0.0, 0.0), (90.0, 0.0), (90.0, 90.0), (90.0, -90.0), (30.0, 180.0)])
def test_pnp_odometry_recovers_the_body_motion_for_each_mount(angle, roll, motion):
    rotvec, p_b1 = BODY_MOTIONS[motion]
    R_wb1 = rotvec_to_matrix(rotvec)
    rng = np.random.default_rng(11)
    P0, P1 = mounted_frames(rng, R_wb1, p_b1, angle, roll)
    assert np.all(P1[:, 2] > 0.5)
    ids = np.arange(len(P0))
    M = camera_to_body_matrix(angle, roll)
    odometry = PnPOdometry(K, PnPParams(refractive_index=1.0))
    odometry.process_depths(ids, project(P0), P0[:, 2], 1.0)
    # The camera's gyro (R_cam_imu = I) reports the body rotation in the optical frame
    prior = M.T @ R_wb1 @ M
    frame = odometry.process_depths(ids, project(P1) + rng.normal(0, 0.2, (len(P1), 2)), P1[:, 2], 1.1, prior)
    assert frame.pose.ok, frame.pose.reason
    R_body = camera_to_body_rotation(frame.pose.rotation, angle, roll)
    np.testing.assert_allclose(R_body, R_wb1, atol=math.radians(0.15))
    np.testing.assert_allclose(camera_to_body_frd(frame.pose.angle_delta, angle, roll), rotvec,
                               atol=math.radians(0.15))
    np.testing.assert_allclose(camera_to_body_rotation(prior, angle, roll), R_wb1, atol=1e-12)
    # The camera's delta in body FRD, corrected to the IMU, is the body's displacement in its current frame
    camera_delta = camera_to_body_frd(frame.pose.position_delta, angle, roll)
    np.testing.assert_allclose(lever_arm_body_delta(camera_delta, R_wb1, LEVER_ARM), R_wb1.T @ p_b1, atol=4e-3)


@pytest.mark.parametrize("camera_roll", [0.0, 90.0, 180.0, -90.0])
@pytest.mark.parametrize("camera_angle", [0.0, 90.0])
def test_lever_arm_pure_yaw_about_the_imu_gives_no_body_translation(camera_angle, camera_roll):
    psi = math.radians(8.0)
    delta_optical, R_01 = camera_observation(yaw_frd(psi), np.zeros(3), camera_angle, camera_roll=camera_roll)
    camera_delta = camera_to_body_frd(delta_optical, camera_angle, camera_roll)
    # The camera itself moved: it swings on the lever arm
    assert np.linalg.norm(camera_delta) > 0.02
    R_body = camera_to_body_rotation(R_01, camera_angle, camera_roll)
    np.testing.assert_allclose(R_body, yaw_frd(psi), atol=1e-12)
    np.testing.assert_allclose(lever_arm_correction(R_body, LEVER_ARM), camera_delta, atol=1e-12)
    np.testing.assert_allclose(lever_arm_body_delta(camera_delta, R_body, LEVER_ARM), 0.0, atol=1e-12)


@pytest.mark.parametrize("camera_roll", [0.0, 90.0, 180.0, -90.0])
def test_lever_arm_recovers_the_imu_motion_with_rotation_and_translation(camera_roll):
    R_wb1 = rotvec_to_matrix([0.05, -0.03, 0.2])
    p_b1 = np.array([0.04, -0.02, 0.01])
    delta_optical, R_01 = camera_observation(R_wb1, p_b1, 90.0, camera_roll=camera_roll)
    camera_delta = camera_to_body_frd(delta_optical, 90.0, camera_roll)
    body = lever_arm_body_delta(camera_delta, camera_to_body_rotation(R_01, 90.0, camera_roll), LEVER_ARM)
    # Body displacement expressed in the current body frame, like position_delta
    np.testing.assert_allclose(body, R_wb1.T @ p_b1, atol=1e-12)
    assert np.linalg.norm(camera_delta - body) > 0.02


def test_lever_arm_correction_is_omega_cross_r_to_first_order():
    omega, dt = np.array([0.1, -0.2, 0.5]), 0.01
    correction = lever_arm_correction(rotvec_to_matrix(omega * dt), LEVER_ARM)
    np.testing.assert_allclose(correction / dt, np.cross(omega, LEVER_ARM), rtol=1e-2)
    np.testing.assert_allclose(lever_arm_correction(np.eye(3), LEVER_ARM), 0.0)


def test_rotation_angle():
    assert rotation_angle(np.eye(3)) == pytest.approx(0.0)
    assert rotation_angle(rotvec_to_matrix([0, 0, 0.3])) == pytest.approx(0.3)


def test_a_frame_with_missing_depth_carries_the_reference_so_no_delta_is_lost():
    """
    process_depths(depth_missing=True): frame 1 has no depth (points mode: its points never came) but solves
    against frame 0; frame 0's inliers, moved into frame 1's camera, are the reference frame 2 solves against.
    """
    rng = np.random.default_rng(7)
    P0 = random_scene(rng)
    ids = np.arange(len(P0))
    steps = [(np.radians([0.5, -1.0, 0.8]), np.array([0.04, -0.01, 0.06])),
             (np.radians([-0.4, 0.6, -0.5]), np.array([-0.02, 0.03, 0.05]))]
    odometry = PnPOdometry(K, PnPParams(refractive_index=1.0))
    noise = lambda: rng.normal(0, 0.2, (len(P0), 2))  # noqa: E731
    first = odometry.process_depths(ids, project(P0), P0[:, 2], 1.0)
    assert first.pose.reason == "no_reference"
    points = P0
    results = []
    for k, (rotvec, delta) in enumerate(steps):
        _, R_10, t_10 = make_motion(rotvec, delta)
        points = transform(points, R_10, t_10)
        missing = k == 0
        z = np.full(len(P0), np.nan) if missing else points[:, 2]
        results.append(odometry.process_depths(ids, project(points) + noise(), z, 1.1 + 0.1 * k,
                                               depth_missing=missing))
        if missing:
            carried = np.array([odometry.prev_points[i] for i in sorted(odometry.prev_points)])
            truth = points[sorted(odometry.prev_points)]
            assert len(carried) >= 0.9 * len(P0)
            np.testing.assert_allclose(carried, truth, atol=0.03)
    (one, two) = results
    assert one.pose.ok and one.reference_carried and one.valid_depth == 0 and not one.reference_reset
    assert two.pose.ok and not two.reference_carried and two.dt == pytest.approx(0.1)
    for result, (rotvec, delta) in zip(results, steps):
        np.testing.assert_allclose(result.pose.position_delta, delta, atol=6e-3)
        np.testing.assert_allclose(result.pose.angle_delta, rotvec, atol=math.radians(0.15))


def test_without_depth_missing_an_all_invalid_frame_still_becomes_the_reference():
    """full_frame's behaviour, unchanged: a solved frame with no valid depth is the (empty) next reference."""
    rng = np.random.default_rng(8)
    P0 = random_scene(rng)
    ids = np.arange(len(P0))
    odometry = PnPOdometry(K, PnPParams(refractive_index=1.0))
    odometry.process_depths(ids, project(P0), P0[:, 2], 1.0)
    _, R_10, t_10 = make_motion(np.radians([0.2, 0.1, 0.0]), [0.02, 0.0, 0.03])
    P1 = transform(P0, R_10, t_10)
    result = odometry.process_depths(ids, project(P1), np.full(len(P0), np.nan), 1.1)
    assert result.pose.ok and not result.reference_carried and odometry.prev_points == {}
