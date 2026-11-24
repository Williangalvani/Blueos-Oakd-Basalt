"""
How much depth is lost when the camera sends its disparity as MJPEG: the host side of stereo_probe.py's
mjpeg_loss mode (numpy/OpenCV only, no depthai).

The probe captures on the camera (stereo_probe.run_mjpeg_loss):
- run A: StereoDepth with subpixel off and the median last (speckle on, the saved settings otherwise), so
  stereo.disparity is RAW8 (0..95 px, 0 = invalid). Every disparity frame goes raw to the host and into one MJPEG
  VideoEncoder per quality (the same frames); the VIO's FeatureTracker runs on rectifiedLeft; every n-th
  stereo.depth frame is kept too (the camera's own disparity -> depth conversion);
- run B, right after: the VIO's saved stereo settings (subpixel on): every n-th RAW16 depth frame and the features.

Here:
- depth = fx * B / disparity (depth_times_disparity_mm: the rectified fx and the baseline the VIO uses for its
  disparity stream), 0 = invalid, sampled at the tracked features with the VIO's rules (RULES): "points" (the
  default transport: the camera SpatialLocationCalculator's AVERAGE of the pixels inside the PnP depth range in
  the 5x5 window, depth_from_points' validity), "full_frame" (lookup_depth: the 5x5 median) and "centre" (the
  feature's pixel);
- JPEG vs raw on the same frame (paired by sequence number, checked by device timestamp): per-point error in mm
  and %, valid/invalid flips, error by distance, gray-level error, JPEG bytes and Mbps;
- subpixel loss: A's integer-disparity depth vs B's subpixel depth. The frames differ (two pipeline runs) and the
  FeatureTracker ids restart with each run, so points cannot be matched by id: both runs are reduced to per-pixel
  temporal medians of a still scene and compared at B's feature positions (and each B frame against A's median);
- the quantisation step: dz = z^2 / (fx * B) per disparity pixel, / 2^bits with subpixel.
"""

import math
import warnings

import numpy as np

import depth_points
import depth_stream
from pnp_odometry import PnPParams, depth_from_points, lookup_depth, scale_to_depth_grid

DEFAULT_QUALITIES = (70, 85, 95, 100)
MAX_QUALITIES = 6
# Distance bins (m) of the reference depth: <0.5, 0.5-1, 1-2, 2-4, >=4
DISTANCE_EDGES_M = (0.5, 1.0, 2.0, 4.0)
RULES = ("points", "full_frame", "centre")
PRIMARY_RULE = "points"  # depth_points.DEFAULT_TRANSPORT
# Decoded JPEG gray below this is invalid: "zero" is the raw frame's rule (only 0), "viewer" the depth stream
# page's (JPEG rings around 0)
DECODE_RULES = {"zero": 1, "viewer": depth_stream.DISPARITY_INVALID_BELOW}
PRIMARY_DECODE = "zero"
QUANTISATION_DISTANCES_M = (0.25, 0.5, 1.0, 2.0, 3.0, 4.0)
# Gray-level error histogram: |decoded - raw| of 0, 1, 2 and >= GRAY_LEVELS
GRAY_LEVELS = 3
# A pixel of a temporal median map is valid when at least this share of the frames had a depth there
MEDIAN_MIN_VALID = 0.5


def parse_qualities(text):
    """'95, 70' -> [95, 70]; None or empty -> DEFAULT_QUALITIES. ValueError for anything outside 1-100."""
    if text is None or not str(text).strip():
        return list(DEFAULT_QUALITIES)
    values = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        try:
            quality = int(part)
        except ValueError:
            raise ValueError(f"quality {part!r} is not a whole number") from None
        if not 1 <= quality <= 100:
            raise ValueError(f"quality {quality} is outside 1-100")
        values.append(quality)
    values = list(dict.fromkeys(values))
    if not values:
        raise ValueError("no quality given")
    if len(values) > MAX_QUALITIES:
        raise ValueError(f"at most {MAX_QUALITIES} qualities, got {len(values)}")
    return values


def depth_times_disparity_mm(fx, baseline_m):
    """fx * B in mm * px (depth_mm = this / disparity_px), or None without a usable fx or baseline."""
    try:
        k = float(fx) * float(baseline_m) * 1000.0
    except (TypeError, ValueError):
        return None
    return k if math.isfinite(k) and k > 0 else None


def disparity_depth_mm(disparity, k_mm, invalid_below=1):
    """
    Depth in mm (0 = invalid) of a disparity image in pixels; values below invalid_below are invalid. float32 for
    an 8-bit image (a lookup table: the probe converts several 640x400 frames per frame on the Pi), else float64.
    """
    disparity = np.asarray(disparity)
    if disparity.dtype == np.uint8:
        levels = np.arange(256, dtype=np.float64)
        lut = np.where(levels >= invalid_below, k_mm / np.maximum(levels, 1.0), 0.0).astype(np.float32)
        return lut[disparity]
    d = disparity.astype(np.float64)
    with np.errstate(divide="ignore"):
        return np.where(d >= invalid_below, k_mm / np.maximum(d, 1e-12), 0.0)


def decode_jpeg(data):
    """A JPEG decoded to a uint8 gray image, or None if it does not decode."""
    import cv2
    image = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_GRAYSCALE)
    return image if image is not None and image.size else None


def centre_pixels(uv, image_size, depth_size):
    """(x, y) int arrays: each feature's pixel on the depth grid, as depth_points.feature_windows picks it."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    width, height = image_size
    depth_w, depth_h = depth_size
    x = np.clip(np.floor(uv[:, 0] * (depth_w / float(width)) + 0.5), 0, depth_w - 1).astype(np.int64)
    y = np.clip(np.floor(uv[:, 1] * (depth_h / float(height)) + 0.5), 0, depth_h - 1).astype(np.int64)
    return x, y


def window_average(depth_mm, windows, thresholds):
    """
    The camera SpatialLocationCalculator's AVERAGE per window (x, y, w, h; step 1): (mean, count) of the pixels
    with lower < depth < upper (exclusive thresholds); mean 0 where none.
    """
    depth = np.asarray(depth_mm)
    windows = np.asarray(windows, dtype=np.int64).reshape(-1, 4)
    lower, upper = thresholds
    offsets = np.arange(2 * depth_points.WINDOW_RADIUS + 1)
    dx, dy = (a.ravel() for a in np.meshgrid(offsets, offsets))
    x0, y0, w, h = windows.T
    inside = (dx[None, :] < w[:, None]) & (dy[None, :] < h[:, None])
    cols = np.minimum(x0[:, None] + dx[None, :], depth.shape[1] - 1)
    rows = np.minimum(y0[:, None] + dy[None, :], depth.shape[0] - 1)
    pixels = depth[rows, cols].astype(np.float64)
    take = inside & (pixels > lower) & (pixels < upper)
    count = take.sum(axis=1)
    total = np.where(take, pixels, 0.0).sum(axis=1)
    return np.where(count > 0, total / np.maximum(count, 1), 0.0), count


def sample_rules(depth_mm, uv, image_size, params=PnPParams()):
    """{rule: depth in mm at each feature, NaN where the rule finds none} with the VIO's rules (RULES)."""
    depth = np.asarray(depth_mm)
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if not len(uv):
        return {rule: np.zeros(0) for rule in RULES}
    depth_size = (depth.shape[1], depth.shape[0])
    windows = depth_points.feature_windows(uv, image_size, depth_size)
    mean, count = window_average(depth, windows, depth_points.thresholds_mm(params.min_depth_m, params.max_depth_m))
    points = depth_from_points(mean, count, np.full(len(uv), depth_points.SAMPLES_PER_WINDOW), params) * 1000.0
    full = lookup_depth(depth, scale_to_depth_grid(uv, image_size, depth.shape), params) * 1000.0
    x, y = centre_pixels(uv, image_size, depth_size)
    centre = depth[y, x].astype(np.float64)
    in_range = (centre > 0) & (centre >= params.min_depth_m * 1000.0) & (centre <= params.max_depth_m * 1000.0)
    return {"points": points, "full_frame": full, "centre": np.where(in_range, centre, np.nan)}


def _round(value, digits=1):
    return None if value is None or not math.isfinite(value) else round(float(value), digits)


def error_stats(values, digits=1):
    values = np.asarray(values, dtype=np.float64)
    if not values.size:
        return {"median": None, "p95": None, "max": None, "mean": None}
    return {"median": _round(np.median(values), digits), "p95": _round(np.percentile(values, 95), digits),
            "max": _round(values.max(), digits), "mean": _round(values.mean(), digits)}


def _pct(part, whole):
    return round(100.0 * part / whole, 2) if whole else None


def distance_labels(edges=DISTANCE_EDGES_M):
    edges = list(edges)
    return ([f"<{edges[0]:g}"] + [f"{a:g}-{b:g}" for a, b in zip(edges, edges[1:])] + [f">={edges[-1]:g}"])


def distance_bins(depth_mm, edges=DISTANCE_EDGES_M):
    """Bin index (into distance_labels) of each depth in mm."""
    return np.searchsorted(np.asarray(edges, dtype=np.float64) * 1000.0, np.asarray(depth_mm, dtype=np.float64),
                           side="right")


def compare(reference, test, edges=DISTANCE_EDGES_M):
    """
    Per-point comparison of two depth samples (mm, NaN = invalid) of the same points: the error where both are
    valid (|test - reference| in mm and in % of the reference; bias = mean signed error), the valid -> invalid and
    invalid -> valid flips, and both by the reference's distance.
    """
    ref = np.asarray(reference, dtype=np.float64).ravel()
    tst = np.asarray(test, dtype=np.float64).ravel()
    ref_ok, tst_ok = np.isfinite(ref), np.isfinite(tst)
    both = ref_ok & tst_ok
    signed = tst[both] - ref[both]
    abs_mm = np.abs(signed)
    pct = 100.0 * abs_mm / ref[both]
    lost, gained = int((ref_ok & ~tst_ok).sum()), int((~ref_ok & tst_ok).sum())
    result = {"points": int(ref.size), "reference_valid": int(ref_ok.sum()), "both_valid": int(both.sum()),
              "valid_to_invalid": lost, "valid_to_invalid_pct": _pct(lost, int(ref_ok.sum())),
              "invalid_to_valid": gained, "invalid_to_valid_pct": _pct(gained, int((~ref_ok).sum())),
              "error_mm": error_stats(abs_mm), "error_pct": error_stats(pct, 2),
              "bias_mm": _round(signed.mean()) if signed.size else None,
              "exact_pct": _pct(int((abs_mm < 0.5).sum()), int(both.sum()))}
    bins = distance_bins(np.where(ref_ok, ref, 0.0), edges)
    by_distance = {}
    for index, label in enumerate(distance_labels(edges)):
        in_bin = ref_ok & (bins == index)
        sel = both & (bins == index)
        err = np.abs(tst[sel] - ref[sel])
        bin_lost = int((in_bin & ~tst_ok).sum())
        by_distance[label] = {"reference_valid": int(in_bin.sum()), "both_valid": int(sel.sum()),
                              "valid_to_invalid_pct": _pct(bin_lost, int(in_bin.sum())),
                              "error_mm": error_stats(err), "error_pct": error_stats(100.0 * err / ref[sel], 2)}
    result["by_distance"] = by_distance
    return result


class GrayErrors:
    """|decoded gray - raw disparity| over pixels: where the raw is valid, and what invalid (0) pixels decode to."""

    def __init__(self):
        self.levels = np.zeros(GRAY_LEVELS + 1, np.int64)
        self.valid = self.invalid = self.invalid_nonzero = self.invalid_viewer_valid = 0
        self.max = 0

    def add(self, raw, decoded):
        raw, decoded = np.asarray(raw, dtype=np.uint8), np.asarray(decoded, dtype=np.uint8)
        ok = raw > 0
        diff = np.maximum(raw, decoded) - np.minimum(raw, decoded)
        # One pass in uint8: 0..GRAY_LEVELS for valid pixels, then invalid ones decoded as 0 / below the viewer's
        # rule / at or above it
        invalid = ((decoded > 0).view(np.uint8) + (decoded >= DECODE_RULES["viewer"]).view(np.uint8)
                   + np.uint8(GRAY_LEVELS + 1))
        code = np.where(ok, np.minimum(diff, np.uint8(GRAY_LEVELS)), invalid)
        counts = np.bincount(code.ravel(), minlength=GRAY_LEVELS + 4)
        self.levels += counts[:GRAY_LEVELS + 1]
        self.valid += int(counts[:GRAY_LEVELS + 1].sum())
        self.invalid += int(counts[GRAY_LEVELS + 1:].sum())
        self.invalid_nonzero += int(counts[GRAY_LEVELS + 2:].sum())
        self.invalid_viewer_valid += int(counts[GRAY_LEVELS + 3])
        if self.valid:
            self.max = max(self.max, int(np.where(ok, diff, 0).max()))

    def result(self):
        labels = [str(level) for level in range(GRAY_LEVELS)] + [f">={GRAY_LEVELS}"]
        return {"valid_pixels": self.valid,
                "levels_pct": {label: _pct(int(n), self.valid) for label, n in zip(labels, self.levels)},
                "max_levels": self.max, "invalid_pixels": self.invalid,
                "invalid_decoded_nonzero_pct": _pct(self.invalid_nonzero, self.invalid),
                f"invalid_decoded_ge{DECODE_RULES['viewer']}_pct": _pct(self.invalid_viewer_valid, self.invalid)}


def jpeg_rate(sizes, fps):
    """JPEG bytes per frame and the Mbps at fps frames per second."""
    sizes = np.asarray(list(sizes), dtype=np.float64)
    if not sizes.size:
        return {"jpegs": 0, "bytes_mean": None, "bytes_median": None, "bytes_min": None, "bytes_max": None,
                "mbps_at_fps": None}
    return {"jpegs": int(sizes.size), "bytes_mean": round(float(sizes.mean())),
            "bytes_median": round(float(np.median(sizes))), "bytes_min": int(sizes.min()),
            "bytes_max": int(sizes.max()), "mbps_at_fps": round(float(sizes.mean()) * 8.0 * float(fps) / 1e6, 3)}


def raw_mbps(width, height, bytes_per_pixel, fps):
    return round(width * height * bytes_per_pixel * 8.0 * float(fps) / 1e6, 2)


def quantisation_table(k_mm, bits, distances_m=QUANTISATION_DISTANCES_M):
    """Theory: disparity d = fB / z; one disparity step moves z by about z^2 / (fB), 2^bits times less with subpixel."""
    rows = []
    for z_m in distances_m:
        z = z_m * 1000.0
        step = z * z / k_mm
        rows.append({"z_m": z_m, "disparity_px": round(k_mm / z, 2), "step_mm": round(step, 1),
                     "step_pct": round(100.0 * step / z, 2), "step_subpixel_mm": round(step / 2 ** bits, 2),
                     "step_subpixel_pct": round(100.0 * step / z / 2 ** bits, 3)})
    return rows


def measured_steps(disparities, k_mm, edges=DISTANCE_EDGES_M):
    """
    The integer-disparity depth step at the features: z(d) - z(d + 1) = fB / d - fB / (d + 1) at each valid raw
    disparity d, by distance, next to the z^2 / (fB) approximation at the bin's median depth.
    """
    d = np.asarray(disparities, dtype=np.float64).ravel()
    d = d[d > 0]
    z = k_mm / d if d.size else np.zeros(0)
    step = k_mm / d - k_mm / (d + 1.0) if d.size else np.zeros(0)
    bins = distance_bins(z, edges)
    result = {"points": int(d.size), "distinct_disparities": int(np.unique(d).size),
              "disparity_range": [int(d.min()), int(d.max())] if d.size else None, "by_distance": {}}
    for index, label in enumerate(distance_labels(edges)):
        sel = bins == index
        if not sel.any():
            result["by_distance"][label] = {"points": 0, "median_z_mm": None, "step_mm": None, "theory_mm": None}
            continue
        median_z = float(np.median(z[sel]))
        result["by_distance"][label] = {"points": int(sel.sum()), "median_z_mm": round(median_z),
                                        "step_mm": round(float(np.median(step[sel])), 1),
                                        "theory_mm": round(median_z * median_z / k_mm, 1)}
    return result


def temporal_median(frames_mm, min_valid=MEDIAN_MIN_VALID):
    """Per-pixel median of the valid (> 0) depths over frames (float32 mm), 0 where fewer than min_valid had one."""
    stack = np.stack([np.asarray(frame, dtype=np.float32) for frame in frames_mm])
    stack[~(stack > 0)] = np.nan
    valid = np.isfinite(stack).sum(axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nanmedian(stack, axis=0)
    median[(valid < math.ceil(min_valid * len(stack))) | ~np.isfinite(median)] = 0.0
    return median


def temporal_noise(frames_mm, uv, image_size):
    """Standard deviation over frames (mm) of the depth at each feature's pixel, where at least 2 frames had one."""
    stack = np.stack([np.asarray(frame, dtype=np.float64) for frame in frames_mm])
    x, y = centre_pixels(uv, image_size, (stack.shape[2], stack.shape[1]))
    samples = stack[:, y, x]
    samples[~(samples > 0)] = np.nan
    enough = np.isfinite(samples).sum(axis=0) >= 2
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        std = np.nanstd(samples[:, enough], axis=0)
    z = np.nanmedian(samples[:, enough], axis=0) if enough.any() else np.zeros(0)
    return {"points": int(enough.sum()), "std_mm": error_stats(std), "std_pct": error_stats(100.0 * std / z, 2)}


def pair(reference, others, period_s, min_share=0.5):
    """
    Pairs the records ({"seq", "ts"}) of another stream with the reference stream's: by sequence number, kept
    where the device timestamps agree (within half a frame period); by nearest timestamp when the sequence numbers
    are not the reference's (fewer than min_share of the records match one, or of those checked fewer than
    min_share agree in time). Returns ({index in others: index in reference}, info).
    """
    tolerance = 0.5 * period_s
    by_seq = {}
    for i, record in enumerate(reference):
        by_seq.setdefault(record["seq"], i)
    seq_pairs = {j: by_seq[o["seq"]] for j, o in enumerate(others) if o["seq"] in by_seq}

    def agrees(j, i):
        a, b = others[j].get("ts"), reference[i].get("ts")
        return None if a is None or b is None else abs(a - b) <= tolerance

    checks = {j: agrees(j, i) for j, i in seq_pairs.items()}
    checked = [ok for ok in checks.values() if ok is not None]
    info = {"records": len(others), "by_sequence": len(seq_pairs), "timestamps_checked": len(checked),
            "timestamps_agree": int(sum(checked))}
    if len(seq_pairs) >= min_share * len(others) and (not checked or sum(checked) >= min_share * len(checked)):
        pairs = {j: i for j, i in seq_pairs.items() if checks[j] is not False}
        info["method"] = "sequence"
    else:
        times = [(r["ts"], i) for i, r in enumerate(reference) if r.get("ts") is not None]
        pairs, used = {}, set()
        if times:
            ts = np.array([t for t, _ in times])
            for j, other in enumerate(others):
                if other.get("ts") is None:
                    continue
                k = int(np.argmin(np.abs(ts - other["ts"])))
                if abs(ts[k] - other["ts"]) <= tolerance and times[k][1] not in used:
                    pairs[j] = times[k][1]
                    used.add(times[k][1])
        info["method"] = "timestamp"
    info["paired"] = len(pairs)
    info["unpaired"] = len(others) - len(pairs)
    return pairs, info


class QualityResult:
    """Accumulates one quality's samples over the frames compared."""

    def __init__(self, quality):
        self.quality = quality
        self.reference = {rule: [] for rule in RULES}
        self.test = {decode: {rule: [] for rule in RULES} for decode in DECODE_RULES}
        self.features_gray, self.frame_gray = GrayErrors(), GrayErrors()
        self.frames = self.decode_failed = self.size_mismatch = 0

    def add(self, reference, raw_disparity, gray, uv, image_size, k_mm, params):
        if gray is None:
            self.decode_failed += 1
            return
        if gray.shape != raw_disparity.shape:
            self.size_mismatch += 1
            return
        self.frames += 1
        for rule in RULES:
            self.reference[rule].append(reference[rule])
        for decode, below in DECODE_RULES.items():
            sampled = sample_rules(disparity_depth_mm(gray, k_mm, below), uv, image_size, params)
            for rule in RULES:
                self.test[decode][rule].append(sampled[rule])
        self.frame_gray.add(raw_disparity, gray)
        x, y = centre_pixels(uv, image_size, (raw_disparity.shape[1], raw_disparity.shape[0]))
        self.features_gray.add(raw_disparity[y, x], gray[y, x])

    def result(self):
        out = {"frames_compared": self.frames, "decode_failed": self.decode_failed,
               "size_mismatch": self.size_mismatch, "gray_error_features": self.features_gray.result(),
               "gray_error_frame": self.frame_gray.result(), "decode": {}}
        for decode in DECODE_RULES:
            out["decode"][decode] = {
                rule: compare(np.concatenate(self.reference[rule]) if self.reference[rule] else np.zeros(0),
                              np.concatenate(self.test[decode][rule]) if self.test[decode][rule] else np.zeros(0))
                for rule in RULES}
        return out


def analyse_mjpeg(records, qualities, k_mm, image_size, params, fps):
    """
    JPEG vs raw RAW8 disparity on the same frames of one run: records {"disparity": [{"seq", "ts", "type",
    "image"}], "features": [{"seq", "ts", "uv"}], "jpeg": {quality: [{"seq", "ts", "data"}]}}. Returns
    {"frames", "pairing", "raw", "qualities": {quality: ...}}.
    """
    period = 1.0 / max(float(fps), 1.0)
    disparity = records["disparity"]
    feature_pairs, feature_info = pair(disparity, records["features"], period)
    features_of = {i: records["features"][j] for j, i in feature_pairs.items()}
    jpeg_of, pairing = {}, {"features": feature_info}
    for quality in qualities:
        pairs, info = pair(disparity, records["jpeg"].get(quality, []), period)
        jpeg_of[quality] = {i: records["jpeg"][quality][j] for j, i in pairs.items()}
        pairing[f"q{quality}"] = info
    results = {quality: QualityResult(quality) for quality in qualities}
    raw_valid = {rule: [] for rule in RULES}
    centre_disparities, feature_counts, types, compared = [], [], set(), 0
    for i, frame in enumerate(disparity):
        types.add(frame.get("type"))
        image, features = frame.get("image"), features_of.get(i)
        if image is None or features is None or image.dtype != np.uint8:
            continue
        uv = features["uv"]
        compared += 1
        feature_counts.append(len(uv))
        reference = sample_rules(disparity_depth_mm(image, k_mm), uv, image_size, params)
        for rule in RULES:
            raw_valid[rule].append(np.isfinite(reference[rule]))
        x, y = centre_pixels(uv, image_size, (image.shape[1], image.shape[0]))
        centre_disparities.append(image[y, x])
        for quality in qualities:
            jpeg = jpeg_of[quality].get(i)
            if jpeg is not None:
                results[quality].add(reference, image, decode_jpeg(jpeg["data"]), uv, image_size, k_mm, params)
    all_valid = {rule: np.concatenate(v) if v else np.zeros(0, bool) for rule, v in raw_valid.items()}
    return {
        "frames": {"disparity": len(disparity), "features": len(records["features"]),
                   "disparity_with_features": compared, "disparity_types": sorted(t for t in types if t)},
        "pairing": pairing,
        "raw": {"features_per_frame": error_stats(feature_counts),
                "valid_pct": {rule: _pct(int(v.sum()), int(v.size)) for rule, v in all_valid.items()},
                "steps": measured_steps(np.concatenate(centre_disparities) if centre_disparities else [], k_mm)},
        "qualities": {quality: {**results[quality].result(),
                                **{"rate": jpeg_rate([len(r["data"]) for r in records["jpeg"].get(quality, [])], fps)}}
                      for quality in qualities},
    }


def conversion_check(disparity_frames, depth_frames, k_mm, period_s):
    """
    The camera's own depth vs fx * B / disparity on the same frames (run A, integer disparity): the depth x
    disparity the camera used (median over the pixels valid in both) against the host's constant.
    """
    pairs, info = pair(disparity_frames, depth_frames, period_s)
    products = []
    for j, i in pairs.items():
        disparity, depth = disparity_frames[i].get("image"), depth_frames[j].get("image")
        if disparity is None or depth is None or disparity.shape != depth.shape:
            continue
        ok = (disparity > 0) & (depth > 0)
        products.append(depth[ok].astype(np.float64) * disparity[ok].astype(np.float64))
    values = np.concatenate(products) if products else np.zeros(0)
    result = {"frames": len(products), "pixels": int(values.size), "pairing": info,
              "host_depth_times_disparity_mm": round(k_mm, 1), "camera_depth_times_disparity_mm": None,
              "ratio": None, "p5_p95": None}
    if values.size:
        median = float(np.median(values))
        result.update(camera_depth_times_disparity_mm=round(median, 1), ratio=round(median / k_mm, 4),
                      p5_p95=[round(float(v), 1) for v in np.percentile(values, [5, 95])])
    return result


def subpixel_loss(a_disparity, a_depth, b_depth, b_features, k_mm, image_size, params, period_s):
    """
    Integer-disparity depth (run A) vs subpixel depth (run B, the VIO's) with the camera at rest: per-pixel
    temporal medians of each run, compared at the features of B's kept frames (reference = B). "median_maps"
    uses A's host conversion fx * B / d, "median_maps_camera_depth" A's own depth output; "per_frame" compares
    each B frame with A's median map (B's frame-to-frame noise included). Also the temporal noise of each run.
    """
    a_frames = [disparity_depth_mm(r["image"], k_mm) for r in a_disparity if r.get("image") is not None]
    a_camera = [r["image"] for r in a_depth if r.get("image") is not None]
    b_frames = [r for r in b_depth if r.get("image") is not None]
    pairs, info = pair(b_frames, b_features, period_s)
    uv_of = {i: b_features[j]["uv"] for j, i in pairs.items()}
    result = {"frames": {"a": len(a_frames), "a_camera_depth": len(a_camera), "b": len(b_frames),
                         "b_with_features": len(uv_of)}, "pairing_b": info}
    if not a_frames or not uv_of:
        result["error"] = "no frames to compare (run A's disparity or run B's depth with features is missing)"
        return result
    za = temporal_median(a_frames)
    zb = temporal_median([r["image"] for r in b_frames])
    za_camera = temporal_median(a_camera) if a_camera else None
    uv_all = np.concatenate([uv_of[i] for i in sorted(uv_of)])
    ref = sample_rules(zb, uv_all, image_size, params)
    test = sample_rules(za, uv_all, image_size, params)
    result["median_maps"] = {rule: compare(ref[rule], test[rule]) for rule in RULES}
    if za_camera is not None:
        camera = sample_rules(za_camera, uv_all, image_size, params)
        result["median_maps_camera_depth"] = {rule: compare(ref[rule], camera[rule]) for rule in RULES}
    per_ref, per_test = {rule: [] for rule in RULES}, {rule: [] for rule in RULES}
    for i in sorted(uv_of):
        frame_ref = sample_rules(b_frames[i]["image"], uv_of[i], image_size, params)
        frame_test = sample_rules(za, uv_of[i], image_size, params)
        for rule in RULES:
            per_ref[rule].append(frame_ref[rule])
            per_test[rule].append(frame_test[rule])
    result["per_frame"] = {rule: compare(np.concatenate(per_ref[rule]), np.concatenate(per_test[rule]))
                           for rule in RULES}
    result["noise"] = {"a": temporal_noise(a_frames, uv_all, image_size),
                       "b": temporal_noise([r["image"] for r in b_frames], uv_all, image_size)}
    return result


# --- summary --------------------------------------------------------------------------------------------

def _fmt(value, digits=0):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def summary_lines(document):
    """A compact text table of the probe's result (also in the JSON as summary_table)."""
    lines = []
    camera, cal = document.get("camera", {}), document.get("calibration") or {}
    fps = camera.get("fps")
    lines.append(f"MJPEG loss of the RAW8 disparity, {camera.get('width')}x{camera.get('height')} @ {fps} fps, "
                 f"fx {_fmt(cal.get('fx'), 1)} px ({cal.get('fx_source')}), baseline "
                 f"{_fmt((cal.get('baseline_m') or 0) * 1000, 1)} mm, fx*B {_fmt(cal.get('depth_times_disparity_mm'))}"
                 f" mm*px; rule {PRIMARY_RULE}, decode {PRIMARY_DECODE} (gray 0 invalid)")
    mjpeg = document.get("mjpeg") or {}
    if mjpeg:
        lines.append("q    kB/frame   Mbps  gray 0/1/2/>=3 %     |err| mm med/p95/max   |err| % med/p95/max   "
                     "v->i %  i->v %  i->v % (viewer)  frames  run")
        for quality, entry in mjpeg.items():
            rate = entry.get("rate", {})
            gray = entry.get("gray_error_features", {}).get("levels_pct", {})
            primary = entry.get("decode", {}).get(PRIMARY_DECODE, {}).get(PRIMARY_RULE, {})
            viewer = entry.get("decode", {}).get("viewer", {}).get(PRIMARY_RULE, {})
            mm, pct = primary.get("error_mm", {}), primary.get("error_pct", {})
            gray_text = "/".join(_fmt(v) for v in gray.values())
            mm_text = "/".join(_fmt(mm.get(key)) for key in ("median", "p95", "max"))
            pct_text = "/".join(_fmt(pct.get(key), 1) for key in ("median", "p95", "max"))
            kb, mbps = _fmt((rate.get("bytes_mean") or 0) / 1000, 1), _fmt(rate.get("mbps_at_fps"), 2)
            lost, gained = _fmt(primary.get("valid_to_invalid_pct"), 2), _fmt(primary.get("invalid_to_valid_pct"), 2)
            viewer_gained = _fmt(viewer.get("invalid_to_valid_pct"), 2)
            lines.append(
                f"{quality:<4} {kb:>8}  {mbps:>5}  {gray_text:<19}  {mm_text:<21}  {pct_text:<20}  {lost:>6}  "
                f"{gained:>6}  {viewer_gained:>15}  {entry.get('frames_compared', 0):>6}  {entry.get('run')}")
        raw = document.get("raw_rates_mbps") or {}
        lines.append(f"raw  RAW8 disparity {_fmt(raw.get('disparity_raw8'), 1)} Mbps, RAW16 depth "
                     f"{_fmt(raw.get('depth_raw16'), 1)} Mbps at {fps} fps")
        labels = distance_labels()
        lines.append("|err| p95 mm by distance (m): q    " + "  ".join(f"{label:>8}" for label in labels))
        for quality, entry in mjpeg.items():
            bins = entry.get("decode", {}).get(PRIMARY_DECODE, {}).get(PRIMARY_RULE, {}).get("by_distance", {})
            lines.append(f"                              {quality:<4} " + "  ".join(
                f"{_fmt(bins.get(label, {}).get('error_mm', {}).get('p95')):>8}" for label in labels))
    encoders = document.get("encoders") or {}
    if encoders:
        lines.append(f"encoders: {encoders.get('note')}")
    check = document.get("conversion_check") or {}
    if check.get("ratio") is not None:
        lines.append(f"camera depth x disparity / host fx*B = {check['ratio']} ({check['pixels']} px, "
                     f"{check['frames']} frames)")
    sub = document.get("subpixel") or {}
    maps = (sub.get("median_maps") or {}).get(PRIMARY_RULE)
    if maps:
        labels = distance_labels()
        lines.append(f"subpixel loss (A integer vs B subpixel, static scene, temporal medians, {PRIMARY_RULE}): "
                     f"|err| mm med/p95 {_fmt(maps['error_mm']['median'])}/{_fmt(maps['error_mm']['p95'])}, % "
                     f"{_fmt(maps['error_pct']['median'], 1)}/{_fmt(maps['error_pct']['p95'], 1)}, bias "
                     f"{_fmt(maps.get('bias_mm'))} mm, v->i {_fmt(maps.get('valid_to_invalid_pct'), 1)} %, "
                     f"{maps['both_valid']} points")
        lines.append("  p95 mm by distance (m): " + "  ".join(
            f"{label} {_fmt(maps['by_distance'][label]['error_mm']['p95'])}" for label in labels))
    elif sub.get("error"):
        lines.append(f"subpixel loss: {sub['error']}")
    quant = document.get("quantisation") or {}
    if quant.get("theory"):
        lines.append(f"quantisation step (theory, {quant.get('subpixel_bits')} subpixel bits): " + ", ".join(
            f"{row['z_m']:g} m {row['step_mm']:g} / {row['step_subpixel_mm']:g} mm" for row in quant["theory"]))
    measured = (quant.get("measured_integer") or {}).get("by_distance") or {}
    if measured:
        lines.append("  integer step at the features (measured / theory, mm): " + ", ".join(
            f"{label} {_fmt(v.get('step_mm'))}/{_fmt(v.get('theory_mm'))}" for label, v in measured.items()
            if v.get("points")))
    return lines
