#!/usr/bin/env python3
"""
Focal length implied by OAK-D depth, from snapshot PNGs (the same check as /api/focal_check).

    python app/tools/check_rectified_focal.py rect_left.png rect_right.png depth_raw.png 7.496 [--fx 163.1]

depth_raw.png is the 16-bit snapshot (/api/snapshot?stream=depth&raw=1); the baseline is in cm
(calib.getBaselineDistance(CAM_C, CAM_B, False), logged by pnp_vio at startup). With --fx, the
result is compared with that camera matrix fx. Prints JSON.
"""

import argparse
import json
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from rectified_intrinsics import focal_check, measure_rectified_focal  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("rect_left")
    parser.add_argument("rect_right")
    parser.add_argument("depth_raw")
    parser.add_argument("baseline_cm", type=float)
    parser.add_argument("--fx", type=float, help="fx of the camera matrix to check (px)")
    parser.add_argument("--fx-raw", type=float, help="raw calibration fx, to say so when --fx is the raw one")
    parser.add_argument("--tolerance", type=float, default=0.15)
    parser.add_argument("--depth-range-mm", type=int, nargs=2, default=(300, 4000))
    args = parser.parse_args(argv)

    images = []
    for path in (args.rect_left, args.rect_right, args.depth_raw):
        image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if image is None:
            parser.error(f"can't read {path}")
        images.append(image)
    if images[2].dtype != "uint16":
        parser.error(f"{args.depth_raw} is not 16-bit depth (use the raw=1 depth snapshot)")

    measurement = measure_rectified_focal(*images, args.baseline_cm / 100.0, tuple(args.depth_range_mm))
    result = dict(measurement)
    if args.fx:
        result.update(focal_check(args.fx, measurement, args.tolerance, fx_raw=args.fx_raw))
    print(json.dumps(result, indent=2))
    return 0 if not args.fx or result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
