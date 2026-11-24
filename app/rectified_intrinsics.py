"""
Camera matrix of the rectified-left grid that the PnP features and depth are on, how it is chosen,
and a check of it against the device's own depth. numpy/OpenCV only, no depthai.

What the device does (depthai-core sources; the README's Camera matrix section has the file:line references):
- enableDistortionCorrection(True) selects mesh rectification, generated on the device like
  cv::initUndistortRectifyMap from the first 8 distortion coefficients of the calibration.
- setAlphaScaling(alpha) is OpenCV's getOptimalNewCameraMatrix free scaling parameter.
- The mesh is built by the closed firmware. depthai-core's own host-side model of the rectified
  stereo output (CalibrationHandler::getRTABMapCameraModel) is
  getOptimalNewCameraMatrix(K_camera, D[:8], size, alpha), using that camera's own K, without the
  stereo rectification rotation. That model matches the measurements on the OAK-D-W-PoE:
  f = Z * d / B ~ 167 px from the device depth, and fy / fx ~ 1.4 from RGB <-> rect_left matches.
- The depth frame's ImgTransformation reports the raw calibration K, so it can't be used.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import cv2
import numpy as np

from pnp_odometry import Intrinsics
from stereo_config import parse_intrinsics_override

# The device's mesh generator only uses k1, k2, p1, p2, k3, k4, k5, k6 (StereoDepth.hpp)
MESH_DISTORTION_COEFFS = 8
# Relative tolerance for "the frame matrix is this matrix"
SAME_MATRIX_RTOL = 1e-3


@dataclass
class StereoCalibration:
    """What pnp_vio reads from the device calibration, on the configured width x height grid."""
    raw: Intrinsics  # stereo left camera (CAM_B), unrectified
    distortion: List[float] = field(default_factory=list)  # CAM_B coefficients (depthai returns 14)
    sockets: Dict[str, Intrinsics] = field(default_factory=dict)  # raw K by --intrinsics-from socket name
    baseline_m: Optional[float] = None  # from the calibration
    baseline_spec_m: Optional[float] = None  # board design value
    R_cam_imu: Optional[np.ndarray] = None


@dataclass
class IntrinsicsChoice:
    intrinsics: Intrinsics
    source: str
    warnings: List[str] = field(default_factory=list)
    # What the depth frame reported and which matrix it equals: "rectified", "raw", "neither" or None
    frame: Optional[Intrinsics] = None
    frame_matches: Optional[str] = None


def rectified_intrinsics(K, dist, size, alpha, distortion_correction=True):
    """
    Camera matrix of the device's rectified-left image: getOptimalNewCameraMatrix(K, dist[:8], size, alpha).

    K: raw 3x3 camera matrix on the size (width, height) grid; dist: calibration distortion
    coefficients (only the first 8 are used, as on the device). Returns None when the device's
    matrix is not known: alpha None (device default) or distortion correction off (3x3 homography
    rectification).
    """
    if alpha is None or not distortion_correction:
        return None
    width, height = (int(v) for v in size)
    coeffs = np.zeros(MESH_DISTORTION_COEFFS)
    given = np.asarray(dist if dist is not None else [], dtype=np.float64).ravel()[:MESH_DISTORTION_COEFFS]
    coeffs[:len(given)] = given
    new_K, _ = cv2.getOptimalNewCameraMatrix(np.asarray(K, dtype=np.float64).reshape(3, 3), coeffs,
                                             (width, height), float(alpha))
    return Intrinsics.from_matrix(new_K)


def rectified_for_settings(calibration, stereo_settings, size):
    """(rectified Intrinsics or None, why it is None) for the configured stereo settings."""
    alpha = stereo_settings.get("stereo_alpha")
    correction = stereo_settings.get("stereo_distortion_correction", True)
    if not correction:
        return None, "distortion correction is off (homography rectification)"
    if alpha is None:
        return None, "alpha scaling is not set (device default)"
    if not calibration.distortion:
        return None, "no distortion coefficients in the calibration"
    return rectified_intrinsics(calibration.raw.matrix(), calibration.distortion, size, alpha, correction), None


def intrinsics_override(text):
    """'fx,fy,cx,cy' -> Intrinsics, None if empty; ValueError if malformed."""
    values = parse_intrinsics_override(text)
    return None if values is None else Intrinsics(*values)


def as_list(intrinsics):
    return None if intrinsics is None else [float(intrinsics.fx), float(intrinsics.fy),
                                            float(intrinsics.cx), float(intrinsics.cy)]


def as_dict(intrinsics):
    return None if intrinsics is None else dict(zip(("fx", "fy", "cx", "cy"), as_list(intrinsics)))


def same_intrinsics(a, b, rtol=SAME_MATRIX_RTOL):
    if a is None or b is None:
        return False
    return bool(np.allclose(as_list(a), as_list(b), rtol=rtol, atol=0.0))


def frame_matches(frame, raw, rectified):
    if frame is None:
        return None
    if same_intrinsics(frame, rectified):
        return "rectified"
    if same_intrinsics(frame, raw):
        return "raw"
    return "neither"


def choose_intrinsics(requested, raw, rectified=None, frame=None, frame_problem=None, override=None,
                      sockets=None, rectified_problem=None):
    """
    Pick the camera matrix for PnP and say where it came from.

    requested: --intrinsics-from (rectified, frame, cam_b or cam_c). raw: CAM_B calibration K;
    rectified: computed K (None when unknown, rectified_problem says why); frame: the depth frame's
    matrix (None when unusable, frame_problem says why); override: --intrinsics-override, which wins.
    """
    sockets = sockets or {}
    warnings = []
    matches = frame_matches(frame, raw, rectified)

    def choice(intrinsics, source):
        return IntrinsicsChoice(intrinsics, source, warnings, frame, matches)

    if override is not None:
        return choice(override, "override")
    if requested in sockets:
        return choice(sockets[requested], requested)
    if requested == "rectified":
        if rectified is not None:
            return choice(rectified, "rectified")
        warnings.append(f"rectified camera matrix unknown: {rectified_problem}; using the depth frame matrix "
                        "instead, which is unverified")
    if frame is not None and matches == "raw" and rectified is not None:
        warnings.append("the depth frame camera matrix is the raw CAM_B calibration although distortion "
                        "correction and alpha scaling are set, so it is the unrectified K; ignored, using the "
                        "computed rectified K")
        return choice(rectified, "rectified (frame matrix is raw)")
    if frame is not None:
        return choice(frame, "frame" if requested == "frame" else "frame (fallback)")
    warnings.append(frame_problem or "no depth frame camera matrix")
    if rectified is not None:
        return choice(rectified, "rectified (fallback)")
    warnings.append("using the raw CAM_B calibration, which is wrong if alpha scaling or distortion correction "
                    "is in effect")
    return choice(raw, "cam_b (fallback)")


# --- Self-check against the device depth --------------------------------------------------------

def focal_from_depth_and_disparity(depth_mm, disparity_px, baseline_m):
    """
    Focal length (px) the device used to turn disparity into depth: Z = f * B / d, so f = Z * d / B.
    On the rectified grid that is fx of the rectified camera matrix.
    """
    return np.asarray(depth_mm, dtype=np.float64) / 1000.0 * np.asarray(disparity_px, dtype=np.float64) / baseline_m


def _gray(image):
    image = np.asarray(image)
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image.astype(np.uint8)


def stereo_row_matches(rect_left, rect_right, max_dy=2.0, min_dx=0.5, ratio=0.7, max_features=4000):
    """
    Feature matches between the rectified images that are on the same row (|dy| < max_dy) with a
    positive disparity (dx > min_dx). Returns (left pixel positions N x 2, disparities N, candidates).
    SIFT on CLAHE-enhanced images, ORB if this OpenCV has no SIFT.
    """
    clahe = cv2.createCLAHE(4.0, (8, 8))
    left, right = clahe.apply(_gray(rect_left)), clahe.apply(_gray(rect_right))
    if hasattr(cv2, "SIFT_create"):
        detector, norm = cv2.SIFT_create(max_features), cv2.NORM_L2
    else:
        detector, norm = cv2.ORB_create(max_features), cv2.NORM_HAMMING
    k1, d1 = detector.detectAndCompute(left, None)
    k2, d2 = detector.detectAndCompute(right, None)
    if d1 is None or d2 is None or len(k1) < 2 or len(k2) < 2:
        return np.zeros((0, 2)), np.zeros(0), 0
    pairs = cv2.BFMatcher(norm).knnMatch(d1, d2, k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]
    if not good:
        return np.zeros((0, 2)), np.zeros(0), 0
    p1 = np.array([k1[m.queryIdx].pt for m in good])
    p2 = np.array([k2[m.trainIdx].pt for m in good])
    dx, dy = p1[:, 0] - p2[:, 0], p1[:, 1] - p2[:, 1]
    keep = (np.abs(dy) < max_dy) & (dx > min_dx)
    return p1[keep], dx[keep], len(good)


def depth_at(depth_mm, uv, depth_range_mm=(1, 65535), window=1, min_valid=3):
    """Median valid depth (mm) in a (2*window+1)^2 box at each pixel; NaN with fewer than min_valid valid pixels."""
    depth = np.asarray(depth_mm)
    h, w = depth.shape[:2]
    out = np.full(len(uv), np.nan)
    for k, (u, v) in enumerate(np.asarray(uv, dtype=np.float64).reshape(-1, 2)):
        col, row = int(round(u)), int(round(v))
        box = depth[max(row - window, 0):row + window + 1, max(col - window, 0):col + window + 1].astype(np.float64)
        box = box[(box >= max(depth_range_mm[0], 1)) & (box <= depth_range_mm[1])]
        if len(box) >= min_valid:
            out[k] = np.median(box)
    return out


def measure_rectified_focal(rect_left, rect_right, depth_raw, baseline_m, depth_range_mm=(300, 4000)):
    """
    Median focal length implied by the device depth at row-consistent left/right feature matches.
    depth_raw: uint16 mm, aligned to rect_left (scaled to its grid if decimated).
    """
    uv, disparity, candidates = stereo_row_matches(rect_left, rect_right)
    depth = np.asarray(depth_raw)
    scale = np.array([depth.shape[1] / np.asarray(rect_left).shape[1], depth.shape[0] / np.asarray(rect_left).shape[0]])
    z = depth_at(depth, uv * scale, depth_range_mm)
    valid = np.isfinite(z)
    f = focal_from_depth_and_disparity(z[valid], disparity[valid], baseline_m)
    result = {"candidates": int(candidates), "row_consistent": int(len(uv)), "n_matches": int(valid.sum()),
              "f_from_depth": None, "f_iqr": None}
    if valid.any():
        result["f_from_depth"] = float(np.median(f))
        result["f_iqr"] = [float(v) for v in np.percentile(f, [25, 75])]
    return result


def focal_check(fx_K, measurement, tolerance=0.15, min_matches=10, fx_raw=None):
    """Compare the fx in use with the focal length measured from the device depth."""
    f = measurement.get("f_from_depth")
    n = measurement.get("n_matches", 0)
    result = {"fx_K": fx_K, "f_from_depth": f, "ratio": None, "n_matches": n, "tolerance": tolerance,
              "ok": False, "f_iqr": measurement.get("f_iqr"), "fx_raw": fx_raw}
    if f is None or n < min_matches:
        result["message"] = (f"only {n} usable matches (need {min_matches}): point the camera at a textured scene "
                             "0.5-3 m away and keep it still")
        return result
    ratio = f / fx_K
    result["ratio"] = ratio
    result["ok"] = abs(ratio - 1.0) <= tolerance
    if result["ok"]:
        result["message"] = f"device depth implies f = {f:.1f} px, the camera matrix has fx = {fx_K:.1f} px: consistent"
    else:
        result["message"] = (f"device depth implies f = {f:.1f} px but the camera matrix has fx = {fx_K:.1f} px "
                             f"(ratio {ratio:.2f}, tolerance {tolerance:.0%})")
        if fx_raw and abs(fx_K / fx_raw - 1.0) <= 0.01:
            result["message"] += "; fx is the raw calibration, use --intrinsics-from rectified"
    return result
