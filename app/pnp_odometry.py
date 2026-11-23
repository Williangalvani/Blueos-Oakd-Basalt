"""
Frame-to-frame visual odometry from tracked features + depth, solved with PnP.

Pure geometry: no depthai, no MAVLink. Everything the device-side script needs to turn
(feature ids, feature pixels, depth image, gyro samples) into a relative pose lives here so it
can be unit tested on a desktop.

Conventions
-----------
- Camera frame is the OpenCV optical frame of the rectified left camera:
  x right, y down, z forward. Units are metres and seconds.
- Depth images are uint16 millimetres (depthai StereoDepth.depth); 0 means invalid.
- For a reference frame C0 (the last good frame, usually the previous one) and the current frame
  C1, PnP solves the transform that maps points from C0 into C1: X1 = R_10 @ X0 + t_10.
- The reported motion follows MAVLink VISION_POSITION_DELTA, over the interval since the reference:
  * angle_delta: rotation vector of the current frame relative to the reference
    (Rodrigues of R_01 = R_10.T).
  * position_delta: displacement of the camera from C0 to C1, expressed in the current frame C1
    (that is R_10 @ (-R_10.T @ t_10) = -t_10).
"""

import math
import warnings
from dataclasses import dataclass, field
from typing import Dict, Optional

import cv2
import numpy as np


# Optical (x right, y down, z forward) -> FLU (x forward, y left, z up)
OPTICAL_TO_FLU = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])

# FLU -> FRD, same as MavlinkNode.flu_to_frd_frame
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_matrix(cls, matrix):
        m = np.asarray(matrix, dtype=np.float64)
        return cls(fx=m[0, 0], fy=m[1, 1], cx=m[0, 2], cy=m[1, 2])

    def matrix(self):
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ])


def refracted_intrinsics(intrinsics, refractive_index=1.0):
    """
    Effective pinhole matrix behind a flat port in a medium of refractive_index, from the in-air
    calibration (paraxial): a ray at angle a in the water leaves the port at n*tan(a), so fx and fy
    become n*fx, n*fy; cx, cy are unchanged. The device's stereo depth is low by the same factor
    (Z = n*f*B/d), which PnPParams.refractive_index puts back.
    """
    n = float(refractive_index)
    return Intrinsics(fx=intrinsics.fx * n, fy=intrinsics.fy * n, cx=intrinsics.cx, cy=intrinsics.cy)


def intrinsics_from_frame(matrix, frame_size, image_size):
    """
    Intrinsics on the image_size (width, height) grid from an ImgFrame's transformation. On the
    OAK-D-W-PoE the depth frame carries the raw CAM_B calibration here, not the rectified matrix
    (see rectified_intrinsics.py), so callers check it against the raw calibration.
    A frame that is a uniform downscale of image_size (decimated depth) is scaled back up.
    None if the matrix is missing or not a plausible pinhole matrix for that grid.
    """
    try:
        m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        frame_w, frame_h = (int(v) for v in frame_size)
    except (TypeError, ValueError):
        return None
    width, height = image_size
    if frame_w <= 0 or frame_h <= 0 or width * frame_h != height * frame_w or not np.all(np.isfinite(m)):
        return None
    m = np.diag([width / frame_w, height / frame_h, 1.0]) @ m
    intrinsics = Intrinsics.from_matrix(m)
    if intrinsics.fx <= 0 or intrinsics.fy <= 0:
        return None
    if not (0 < intrinsics.cx < width and 0 < intrinsics.cy < height):
        return None
    return intrinsics


def scale_to_depth_grid(uv, image_size, depth_shape):
    """Pixel coordinates on the image_size (width, height) grid -> the depth image's grid (decimation)."""
    if image_size is None:
        return uv
    width, height = image_size
    depth_h, depth_w = depth_shape[:2]
    if (depth_w, depth_h) == (width, height):
        return uv
    return uv * np.array([depth_w / width, depth_h / height])


@dataclass
class PnPParams:
    min_inliers: int = 15
    min_inlier_ratio: float = 0.3
    reprojection_error_px: float = 2.0
    ransac_iterations: int = 100
    ransac_confidence: float = 0.99
    # Max disagreement between PnP rotation and gyro-integrated rotation before the frame is rejected
    gyro_gate_deg: float = 5.0
    # Reject solutions that move further than this per frame (m); a solve that bridges failed frames
    # gets this much for each frame interval it spans
    max_translation_m: float = 0.5
    # After a failed frame, keep solving against the last good frame for at most this long (s); 0 = no cap
    max_reference_age_s: float = 1.0
    # Inlier count at which confidence saturates
    good_inliers: int = 60
    # Depth lookup
    depth_window: int = 2
    min_depth_m: float = 0.2
    max_depth_m: float = 15.0
    # Device depth unit (mm) -> m
    depth_unit_scale: float = 0.001
    # User correction (--depth-scale) multiplied into the looked-up depth, so translation scales with it;
    # min/max_depth_m apply to the uncorrected device depth
    depth_correction: float = 1.0
    # Medium in front of a flat port (--refractive-index, 1.0 = air): multiplies the looked-up depth on top
    # of depth_correction, and PnPOdometry multiplies fx, fy of its camera matrix by it (refracted_intrinsics)
    refractive_index: float = 1.0
    min_valid_depth_fraction: float = 0.3


@dataclass
class PoseResult:
    ok: bool
    reason: str = ""
    # Rotation of the current frame relative to the previous one (R_01) and the matching rotation vector
    rotation: Optional[np.ndarray] = None
    angle_delta: Optional[np.ndarray] = None
    # Camera displacement C0 -> C1 expressed in C1 (m)
    position_delta: Optional[np.ndarray] = None
    matches: int = 0
    inliers: int = 0
    confidence: float = 0.0
    reprojection_rms_px: float = float("nan")
    gyro_disagreement_deg: float = float("nan")
    # RANSAC inliers as indices into the matched points (None if RANSAC did not run)
    inlier_index: Optional[np.ndarray] = None


@dataclass
class FrameResult:
    pose: PoseResult
    timestamp: float
    # Time since the reference: the interval pose covers (None without a reference)
    dt: Optional[float] = None
    features: int = 0
    valid_depth: int = 0
    # Age of the reference this frame was solved against (s); same as dt, kept for rejected frames' logs
    reference_age_s: Optional[float] = None
    # Failed frames between the reference and this one, this one excluded
    gap_frames: int = 0
    # This frame failed and replaced a reference that was too old or had too few matches left
    reference_reset: bool = False
    # Indices into this frame's ids/uv of the features matched to the reference (pose.inlier_index refers to these)
    matched_index: Optional[np.ndarray] = None
    # Per feature of this frame's ids/uv: True where the depth lookup gave a valid depth
    depth_valid: Optional[np.ndarray] = None
    extra: Dict = field(default_factory=dict)


def lookup_depth(depth, uv, params=PnPParams()):
    """
    Depth in metres at each pixel in uv (N x 2), as the median of the valid pixels in a
    (2w+1)^2 window, times params.depth_correction and params.refractive_index. Zero and out-of-range
    depths are ignored; returns NaN where too few window pixels are valid.
    """
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if len(uv) == 0:
        return np.zeros(0)
    h, w = depth.shape[:2]
    r = params.depth_window
    offsets = np.arange(-r, r + 1)
    du, dv = np.meshgrid(offsets, offsets)
    cols = np.rint(uv[:, 0])[:, None].astype(np.int64) + du.ravel()[None, :]
    rows = np.rint(uv[:, 1])[:, None].astype(np.int64) + dv.ravel()[None, :]
    inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    samples = depth[np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)].astype(np.float64) * params.depth_unit_scale
    valid = inside & (samples >= params.min_depth_m) & (samples <= params.max_depth_m)
    samples[~valid] = np.nan

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        z = np.nanmedian(samples, axis=1)
    enough = valid.sum(axis=1) >= max(1, math.ceil(params.min_valid_depth_fraction * samples.shape[1]))
    z[~enough] = np.nan
    return z * (params.depth_correction * params.refractive_index)


def back_project(uv, z, intrinsics):
    """Pixels (N x 2) with depth z (N,) -> 3D points (N x 3) in the optical camera frame."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    z = np.asarray(z, dtype=np.float64)
    x = (uv[:, 0] - intrinsics.cx) * z / intrinsics.fx
    y = (uv[:, 1] - intrinsics.cy) * z / intrinsics.fy
    return np.stack([x, y, z], axis=1)


def rotation_angle(R):
    """Rotation angle (rad) of a rotation matrix."""
    return float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def rotvec_to_matrix(rotvec):
    return cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64).reshape(3, 1))[0]


def matrix_to_rotvec(R):
    return cv2.Rodrigues(np.asarray(R, dtype=np.float64))[0].ravel()


def integrate_gyro(times, omegas, t0, t1, R_cam_imu=None, max_gap=0.1):
    """
    Integrate gyro samples between t0 and t1 into R_01, the rotation of the camera at t1
    relative to t0 (same convention as PoseResult.rotation).

    times: (N,) sample timestamps (s), sorted. omegas: (N x 3) angular rate (rad/s) in the IMU frame.
    R_cam_imu: rotation taking IMU-frame vectors to the camera frame (identity if None).
    Each sample is held until the next one (zero-order hold). Returns None when the samples don't
    cover the interval: no data, nearest sample further than max_gap from an end, or a hole longer
    than max_gap inside it (likely over the long intervals a held reference spans).
    """
    times = np.asarray(times, dtype=np.float64)
    omegas = np.asarray(omegas, dtype=np.float64).reshape(-1, 3)
    if len(times) == 0 or t1 <= t0:
        return None
    if times[0] > t0 + max_gap or times[-1] < t1 - max_gap:
        return None
    inner = times[(times > t0) & (times < t1)]
    bounds = np.concatenate([[t0], inner, [t1]])
    if np.max(np.diff(bounds)) > max_gap:
        return None
    if R_cam_imu is not None:
        omegas = omegas @ np.asarray(R_cam_imu, dtype=np.float64).T

    R = np.eye(3)
    for a, b in zip(bounds[:-1], bounds[1:]):
        idx = max(int(np.searchsorted(times, a, side="right")) - 1, 0)
        R = R @ rotvec_to_matrix(omegas[idx] * (b - a))
    return R


def camera_rotation_matrix(camera_angle_deg):
    """Same pitch rotation MavlinkNode.create_camera_rotation_matrix applies in the FLU frame."""
    a = math.radians(camera_angle_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ])


def camera_to_body_matrix(camera_angle_deg=0.0):
    """Rotation taking optical camera-frame vectors to body FRD (camera_to_body_frd as a matrix)."""
    return FLU_TO_FRD @ camera_rotation_matrix(camera_angle_deg) @ OPTICAL_TO_FLU


def camera_to_body_frd(vector, camera_angle_deg=0.0):
    """
    Optical camera-frame vector -> vehicle body FRD, matching MavlinkNode: convert to FLU, apply the
    camera_angle pitch rotation, then flip FLU -> FRD. Works for translations and rotation vectors.
    """
    return camera_to_body_matrix(camera_angle_deg) @ np.asarray(vector, dtype=np.float64)


def camera_to_body_rotation(R, camera_angle_deg=0.0):
    """A rotation matrix in the optical camera frame (e.g. R_01) -> the same rotation in body FRD."""
    M = camera_to_body_matrix(camera_angle_deg)
    return M @ np.asarray(R, dtype=np.float64) @ M.T


def lever_arm_correction(R_01, lever_arm):
    """
    r - R_01^T r: the displacement of a point at lever_arm r (body FRD, m, relative to the IMU) caused by
    the rotation alone, expressed in the current body frame. R_01 is the rotation of the current body
    frame relative to the reference (body FRD). First order, over dt: omega x r * dt.
    """
    r = np.asarray(lever_arm, dtype=np.float64)
    return r - np.asarray(R_01, dtype=np.float64).T @ r


def lever_arm_body_delta(position_delta, R_01, lever_arm):
    """
    Body (IMU) displacement from the camera's: camera delta - (r - R_01^T r), all body FRD in the current
    frame. Diagnostics only: ArduPilot's EKF3 applies VISO_POS_X/Y/Z itself for VISION_POSITION_DELTA, so
    the deltas sent stay the camera's.
    """
    return np.asarray(position_delta, dtype=np.float64) - lever_arm_correction(R_01, lever_arm)


def confidence_from_inliers(inliers, matches, good_inliers):
    """0..100: inlier ratio scaled down when the absolute inlier count is low."""
    if matches <= 0:
        return 0.0
    ratio = inliers / matches
    return float(100.0 * ratio * min(1.0, inliers / float(good_inliers)))


def solve_relative_pose(points_prev, uv_curr, intrinsics, R_prior=None, params=PnPParams(), max_translation_m=None):
    """
    Relative pose from 3D points in the reference frame and their 2D observations in the current one.

    R_prior: optional gyro rotation R_01 used as the initial guess and as a sanity gate.
    max_translation_m: translation gate for this solve (params.max_translation_m if None).
    """
    if max_translation_m is None:
        max_translation_m = params.max_translation_m
    points_prev = np.asarray(points_prev, dtype=np.float64).reshape(-1, 3)
    uv_curr = np.asarray(uv_curr, dtype=np.float64).reshape(-1, 2)
    n = len(points_prev)
    if n < max(6, params.min_inliers):
        return PoseResult(ok=False, reason="too_few_matches", matches=n)

    K = intrinsics.matrix()
    if R_prior is not None:
        rvec = matrix_to_rotvec(np.asarray(R_prior).T).reshape(3, 1)
        use_guess = True
    else:
        rvec = np.zeros((3, 1))
        use_guess = False
    tvec = np.zeros((3, 1))

    try:
        success, rvec, tvec, inlier_idx = cv2.solvePnPRansac(
            points_prev, uv_curr, K, None,
            rvec=rvec, tvec=tvec, useExtrinsicGuess=use_guess,
            iterationsCount=params.ransac_iterations,
            reprojectionError=params.reprojection_error_px,
            confidence=params.ransac_confidence,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error as e:
        return PoseResult(ok=False, reason=f"pnp_error: {e}", matches=n)

    if not success or inlier_idx is None:
        return PoseResult(ok=False, reason="pnp_failed", matches=n)

    inlier_idx = inlier_idx.ravel()
    n_in = len(inlier_idx)
    result = PoseResult(ok=False, matches=n, inliers=n_in, inlier_index=inlier_idx)
    if n_in < params.min_inliers or n_in / n < params.min_inlier_ratio:
        result.reason = "too_few_inliers"
        return result

    projected, _ = cv2.projectPoints(points_prev[inlier_idx], rvec, tvec, K, None)
    err = projected.reshape(-1, 2) - uv_curr[inlier_idx]
    result.reprojection_rms_px = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))

    R_10 = rotvec_to_matrix(rvec)
    R_01 = R_10.T
    t_10 = tvec.ravel()

    if R_prior is not None:
        result.gyro_disagreement_deg = math.degrees(rotation_angle(R_01 @ np.asarray(R_prior).T))
        if result.gyro_disagreement_deg > params.gyro_gate_deg:
            result.reason = "gyro_mismatch"
            return result

    if np.linalg.norm(t_10) > max_translation_m:
        result.reason = "implausible_translation"
        return result

    result.ok = True
    result.rotation = R_01
    result.angle_delta = matrix_to_rotvec(R_01)
    result.position_delta = -t_10
    result.confidence = confidence_from_inliers(n_in, n, params.good_inliers)
    return result


class PnPOdometry:
    """
    Odometry over persistent feature ids, each frame solved against a reference frame.

    A good frame's features with valid depth become the 3D reference for the next frame. When a
    frame fails, its delta is dropped but the last good frame stays the reference (points and
    timestamp), so the next good frame's delta and dt cover the whole gap and no motion is lost.
    The device tracker keeps ids, so features still tracked keep matching the reference.

    The failed frame's own 3D points are not merged into the reference: they are in that frame's
    camera, and without its pose they can't be expressed in the reference camera. The reference
    therefore only ever loses matches, as features drop out of tracking.

    A failed frame replaces the reference instead (the gap is dropped, frame.reference_reset) when
    the reference is older than params.max_reference_age_s, or fewer than params.min_inliers of
    its points are still matched, so it could never solve again.

    intrinsics is the in-air camera matrix (calibration_intrinsics); the matrix used for back-projection
    and PnP (intrinsics) has fx, fy multiplied by params.refractive_index.
    """

    def __init__(self, intrinsics, params=None, image_size=None):
        self.params = params or PnPParams()
        self.calibration_intrinsics = intrinsics
        self.intrinsics = refracted_intrinsics(intrinsics, self.params.refractive_index)
        # (width, height) of the feature pixel grid; depth may be coarser (decimation)
        self.image_size = image_size
        self.reset()

    def reset(self):
        self.prev_points = None  # reference: dict id -> 3D point (optical, m)
        self.prev_timestamp = None  # reference timestamp
        self.gap_frames = 0  # failed frames since the reference

    def _set_reference(self, points, timestamp):
        self.prev_points = points
        self.prev_timestamp = timestamp
        self.gap_frames = 0

    def _reference_exhausted(self, age, matches):
        max_age = self.params.max_reference_age_s
        return (max_age > 0 and age > max_age) or matches < self.params.min_inliers

    def process(self, ids, uv, depth, timestamp, R_prior=None):
        """
        ids: (N,) feature ids, uv: (N x 2) pixel positions in the rectified left image,
        depth: depth image (uint16 mm) aligned to the same image, timestamp: seconds.
        R_prior: gyro rotation R_01 between the reference (prev_timestamp) and this frame (or None).
        """
        ids = np.asarray(ids, dtype=np.int64).ravel()
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)

        z = lookup_depth(depth, scale_to_depth_grid(uv, self.image_size, depth.shape), self.params)
        valid = np.isfinite(z)
        points = back_project(uv[valid], z[valid], self.intrinsics)
        current = dict(zip(ids[valid].tolist(), points))

        frame = FrameResult(pose=PoseResult(ok=False, reason="no_reference"), timestamp=timestamp,
                            features=len(ids), valid_depth=int(valid.sum()), depth_valid=valid)
        if self.prev_points is None:
            self._set_reference(current, timestamp)
            return frame

        frame.dt = frame.reference_age_s = timestamp - self.prev_timestamp
        frame.gap_frames = self.gap_frames
        id_list = ids.tolist()
        matched_index = [k for k, i in enumerate(id_list) if i in self.prev_points]
        frame.matched_index = np.asarray(matched_index, dtype=np.int64)
        matched = [(self.prev_points[id_list[k]], uv[k]) for k in matched_index]
        if frame.dt <= 0:
            # Repeated or out of order: leave the reference alone, never adopt an older frame
            frame.pose = PoseResult(ok=False, reason="non_increasing_timestamp", matches=len(matched))
            return frame
        if matched:
            P = np.array([m[0] for m in matched])
            q = np.array([m[1] for m in matched])
            frame.pose = solve_relative_pose(P, q, self.intrinsics, R_prior, self.params,
                                             self.params.max_translation_m * (1 + self.gap_frames))
        else:
            frame.pose = PoseResult(ok=False, reason="too_few_matches", matches=0)

        if frame.pose.ok:
            self._set_reference(current, timestamp)
        elif self._reference_exhausted(frame.dt, len(matched)):
            frame.reference_reset = True
            self._set_reference(current, timestamp)
        else:
            self.gap_frames += 1
        return frame
