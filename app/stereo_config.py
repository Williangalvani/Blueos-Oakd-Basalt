"""
StereoDepth settings for the PnP VIO, shared by main.py (config, web form, command line) and
pnp_vio.py (argparse, device configuration), plus the validators of the camera-model settings
(intrinsics override, depth scale, refractive index, lever arm) both sides check.

The defaults favour sparse but trustworthy depth at the tracked features over dense depth: a preset
first, then LR check, a confidence threshold, speckle and range filters, no spatial/temporal
smoothing and no decimation, so depth stays on the full-resolution rectified-left grid the feature
tracker runs on. depthai is passed in by the caller so this module can be unit tested without it.
"""

import argparse
import math
import re
from dataclasses import dataclass

PRESETS = ("ROBOTICS", "DEFAULT", "DENSITY", "ACCURACY", "HIGH_DETAIL", "FACE", "NONE")
MEDIAN_FILTERS = {"off": "MEDIAN_OFF", "3x3": "KERNEL_3x3", "5x5": "KERNEL_5x5", "7x7": "KERNEL_7x7"}


@dataclass(frozen=True)
class Setting:
    key: str  # config.json / VIOConfig key and argparse dest
    kind: type  # bool, int, float or str
    default: object  # None: leave the preset/device value
    label: str
    hint: str = ""
    choices: tuple = ()
    nullable: bool = False  # None (empty in the form, "none" on the command line) allowed
    minimum: object = None  # inclusive bounds checked by the web config (config_schema.py)
    maximum: object = None

    @property
    def flag(self):
        return "--" + self.key.replace("_", "-")

    @property
    def optional(self):
        return self.nullable or self.default is None

    def parse(self, text):
        if self.optional and text.strip().lower() in ("", "none", "auto"):
            return None
        return self.kind(text)


STEREO_SETTINGS = (
    Setting("stereo_preset", str, "ROBOTICS", "Stereo preset",
            "Applied first; the settings below override it. NONE keeps the plain node defaults", PRESETS),
    Setting("stereo_lr_check", bool, True, "Left-right check"),
    Setting("stereo_lr_check_threshold", int, 5, "LR check threshold", "Max L-R disparity difference (px)",
            minimum=0, maximum=128),
    Setting("stereo_confidence_threshold", int, 50, "Confidence threshold", "0-255, lower keeps fewer, more confident pixels",
            minimum=0, maximum=255),
    Setting("stereo_brightness_min", int, None, "Min brightness",
            "Pixels at or below are invalidated. Empty: 1 when the edge fill color is 0 (drops the black border), else 0",
            minimum=0, maximum=255),
    Setting("stereo_brightness_max", int, 255, "Max brightness", "Saturated pixels are invalidated; 256 disables",
            minimum=0, maximum=256),
    Setting("stereo_speckle_filter", bool, True, "Speckle filter"),
    Setting("stereo_speckle_range", int, 50, "Speckle range", "Speckle search range", minimum=0, maximum=65535),
    Setting("stereo_depth_min_mm", int, 300, "Min depth (mm)", "Threshold filter", minimum=0, maximum=65535),
    Setting("stereo_depth_max_mm", int, 4000, "Max depth (mm)", "Threshold filter", minimum=1, maximum=65535),
    Setting("stereo_spatial_filter", bool, False, "Spatial filter", "Fills holes by smoothing; off keeps depth honest at corners"),
    Setting("stereo_temporal_filter", bool, False, "Temporal filter", "Off: it lags behind motion"),
    Setting("stereo_decimation", int, 1, "Decimation", "1 keeps depth on the tracker's pixel grid", (1, 2, 3, 4)),
    Setting("stereo_subpixel", bool, True, "Subpixel"),
    Setting("stereo_subpixel_bits", int, 3, "Subpixel bits", "The median filter needs 3", (3, 4, 5)),
    Setting("stereo_median", str, "7x7", "Median filter", "", tuple(MEDIAN_FILTERS)),
    Setting("stereo_distortion_correction", bool, True, "Distortion correction"),
    Setting("stereo_alpha", float, 0.0, "Alpha scaling",
            "0 crops to valid pixels only, 1 keeps all source pixels. Empty: device default", nullable=True,
            minimum=0.0, maximum=1.0),
    Setting("stereo_edge_fill_color", int, 0, "Edge fill color", "0-255, or -1 to replicate edge pixels",
            minimum=-1, maximum=255),
    Setting("stereo_p2", int, None, "Horizontal P2 penalty", "Cost aggregation; empty keeps the preset value",
            minimum=0, maximum=65535),
)

SETTINGS_BY_KEY = {setting.key: setting for setting in STEREO_SETTINGS}
STEREO_DEFAULTS = {setting.key: setting.default for setting in STEREO_SETTINGS}

INTRINSICS_SOURCES = ("rectified", "frame", "cam_b", "cam_c")
# Multiplicative correction of the device depth; outside this range it is almost certainly a typo
DEPTH_SCALE_RANGE = (0.2, 5.0)
# Medium in front of the flat port: 1.0 is air, 1.333 fresh water (sea water is about 1.34)
DEFAULT_REFRACTIVE_INDEX = 1.333
REFRACTIVE_INDEX_RANGE = (1.0, 2.0)
# Camera relative to the IMU in body FRD (m), from the CAD; ArduPilot's VISO_POS_X/Y/Z range is +-5 m
DEFAULT_LEVER_ARM_M = (0.1655, -0.0069, 0.180)
LEVER_ARM_LIMIT_M = 5.0


def parse_depth_scale(value):
    scale = float(value)
    low, high = DEPTH_SCALE_RANGE
    if not (math.isfinite(scale) and low <= scale <= high):
        raise ValueError(f"depth scale must be between {low} and {high}, got {value!r}")
    return scale


def parse_refractive_index(value):
    n = float(value)
    low, high = REFRACTIVE_INDEX_RANGE
    if not (math.isfinite(n) and low <= n <= high):
        raise ValueError(f"refractive index must be between {low} and {high}, got {value!r}")
    return n


def clamp_refractive_index(value):
    """A saved refractive index forced into range (the default if it isn't a number), so a hand-edited
    config.json can't make the VIO process exit on its arguments and restart forever."""
    try:
        n = float(value)
    except (TypeError, ValueError):
        return DEFAULT_REFRACTIVE_INDEX
    if not math.isfinite(n):
        return DEFAULT_REFRACTIVE_INDEX
    low, high = REFRACTIVE_INDEX_RANGE
    return min(max(n, low), high)


def parse_lever_arm(value):
    """'x,y,z' or a sequence of three numbers (m, body FRD) -> (x, y, z); ValueError if malformed."""
    if isinstance(value, str):
        parts = [part for part in re.split(r"[,\s]+", value.strip()) if part]
    else:
        try:
            parts = list(value)
        except TypeError:
            raise ValueError(f"lever arm needs three numbers x,y,z, got {value!r}")
    if len(parts) != 3:
        raise ValueError(f"lever arm needs three numbers x,y,z, got {value!r}")
    values = tuple(float(part) for part in parts)
    if not all(math.isfinite(v) and abs(v) <= LEVER_ARM_LIMIT_M for v in values):
        raise ValueError(f"lever arm values must be within +-{LEVER_ARM_LIMIT_M:g} m, got {value!r}")
    return values


def format_lever_arm(values):
    """(x, y, z) -> 'x,y,z', as the form and the command line take it."""
    return ",".join(f"{float(v):g}" for v in values)


def parse_intrinsics_override(text):
    """'fx,fy,cx,cy' (pixels on the configured rectified-left grid) -> (fx, fy, cx, cy); None if empty."""
    if text is None or not str(text).strip():
        return None
    parts = [part for part in re.split(r"[,\s]+", str(text).strip()) if part]
    if len(parts) != 4:
        raise ValueError(f"intrinsics override needs four numbers fx,fy,cx,cy, got {text!r}")
    values = tuple(float(part) for part in parts)
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError(f"intrinsics override values must be positive, got {text!r}")
    return values


def add_stereo_arguments(parser):
    group = parser.add_argument_group("stereo depth")
    for setting in STEREO_SETTINGS:
        help_text = f"{setting.label}. {setting.hint}".strip(". ")
        if setting.kind is bool:
            group.add_argument(setting.flag, dest=setting.key, action=argparse.BooleanOptionalAction,
                               default=setting.default, help=help_text)
        else:
            group.add_argument(setting.flag, dest=setting.key, type=setting.parse, default=setting.default,
                               choices=setting.choices or None, help=help_text)


def resolve_stereo_settings(values):
    """Stereo settings from a config dict or argparse namespace, defaults filled and 'auto' values resolved."""
    if not isinstance(values, dict):
        values = vars(values)
    settings = {}
    for setting in STEREO_SETTINGS:
        value = values.get(setting.key, setting.default)
        settings[setting.key] = value if value is None else setting.kind(value)
    if settings["stereo_brightness_min"] is None:
        settings["stereo_brightness_min"] = 1 if settings["stereo_edge_fill_color"] == 0 else 0
    return settings


def stereo_command_args(config):
    """pnp_vio.py command-line arguments for the stereo settings in a saved configuration."""
    args = []
    for setting in STEREO_SETTINGS:
        value = config.get(setting.key, setting.default)
        if setting.kind is bool:
            args.append(setting.flag if value else "--no-" + setting.flag[2:])
        elif value is not None and value != "":
            args += [setting.flag, str(setting.kind(value))]
        elif setting.default is not None:
            args += [setting.flag, "none"]
    return args


def stereo_warnings(settings, intrinsics_from="rectified"):
    warnings = []
    if settings["stereo_median"] != "off" and settings["stereo_subpixel"] and settings["stereo_subpixel_bits"] != 3:
        warnings.append(f"median filter {settings['stereo_median']} needs 3 subpixel bits, "
                        f"not {settings['stereo_subpixel_bits']}; median filter disabled")
    if settings["stereo_decimation"] != 1:
        warnings.append(f"decimation {settings['stereo_decimation']}: depth is coarser than the feature grid, "
                        "feature depth lookups are scaled down to it")
    if settings["stereo_alpha"] is not None and intrinsics_from in ("cam_b", "cam_c"):
        warnings.append(f"alpha scaling {settings['stereo_alpha']} changes the rectified camera matrix, but "
                        f"intrinsics come from the raw {intrinsics_from} calibration; use --intrinsics-from rectified")
    return warnings


def effective_median(settings):
    if settings["stereo_subpixel"] and settings["stereo_subpixel_bits"] != 3:
        return "off"
    return settings["stereo_median"]


def apply_stereo_config(stereo, settings, dai):
    """Configure a dai.node.StereoDepth: preset first, then every override in settings."""
    if settings["stereo_preset"] != "NONE":
        stereo.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, settings["stereo_preset"]))

    config = stereo.initialConfig
    stereo.setExtendedDisparity(False)  # incompatible with subpixel
    stereo.setLeftRightCheck(settings["stereo_lr_check"])
    config.setLeftRightCheckThreshold(settings["stereo_lr_check_threshold"])
    config.setConfidenceThreshold(settings["stereo_confidence_threshold"])
    stereo.setSubpixel(settings["stereo_subpixel"])
    if settings["stereo_subpixel"]:
        stereo.setSubpixelFractionalBits(settings["stereo_subpixel_bits"])
    config.setMedianFilter(getattr(dai.MedianFilter, MEDIAN_FILTERS[effective_median(settings)]))

    post = config.postProcessing
    post.brightnessFilter.minBrightness = settings["stereo_brightness_min"]
    post.brightnessFilter.maxBrightness = settings["stereo_brightness_max"]
    post.speckleFilter.enable = settings["stereo_speckle_filter"]
    post.speckleFilter.speckleRange = settings["stereo_speckle_range"]
    post.thresholdFilter.minRange = settings["stereo_depth_min_mm"]
    post.thresholdFilter.maxRange = settings["stereo_depth_max_mm"]
    post.spatialFilter.enable = settings["stereo_spatial_filter"]
    post.temporalFilter.enable = settings["stereo_temporal_filter"]
    post.decimationFilter.decimationFactor = settings["stereo_decimation"]
    if settings["stereo_p2"] is not None:
        config.costAggregation.horizontalPenaltyCostP2 = settings["stereo_p2"]

    stereo.enableDistortionCorrection(settings["stereo_distortion_correction"])
    if settings["stereo_alpha"] is not None:
        stereo.setAlphaScaling(settings["stereo_alpha"])
    stereo.setRectifyEdgeFillColor(settings["stereo_edge_fill_color"])
    # Depth on the same pixel grid as stereo.rectifiedLeft, which the feature tracker runs on
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)


def effective_stereo_config(stereo, settings) -> dict:
    """What the StereoDepth node will run with, read back from its config after the preset and overrides."""
    config = stereo.initialConfig
    post = config.postProcessing
    readers = {
        "lr_check": config.getLeftRightCheck,
        "lr_check_threshold": config.getLeftRightCheckThreshold,
        "confidence_threshold": config.getConfidenceThreshold,
        "subpixel": config.getSubpixel,
        "subpixel_bits": config.getSubpixelFractionalBits,
        "extended_disparity": config.getExtendedDisparity,
        "median": lambda: str(config.getMedianFilter()),
        "brightness_min": lambda: post.brightnessFilter.minBrightness,
        "brightness_max": lambda: post.brightnessFilter.maxBrightness,
        "speckle_filter": lambda: post.speckleFilter.enable,
        "speckle_range": lambda: post.speckleFilter.speckleRange,
        "depth_min_mm": lambda: post.thresholdFilter.minRange,
        "depth_max_mm": lambda: post.thresholdFilter.maxRange,
        "spatial_filter": lambda: post.spatialFilter.enable,
        "temporal_filter": lambda: post.temporalFilter.enable,
        "decimation": lambda: post.decimationFilter.decimationFactor,
        "p1": lambda: config.costAggregation.horizontalPenaltyCostP1,
        "p2": lambda: config.costAggregation.horizontalPenaltyCostP2,
    }
    effective = {"preset": settings["stereo_preset"]}
    for name, read in readers.items():
        try:
            value = read()
        except Exception as e:
            value = f"unavailable ({e})"
        effective[name] = value if isinstance(value, (bool, int, float, str)) else str(value)
    # Node properties have no getters; these are what was set
    effective["distortion_correction"] = settings["stereo_distortion_correction"]
    effective["alpha"] = settings["stereo_alpha"]
    effective["edge_fill_color"] = settings["stereo_edge_fill_color"]
    effective["depth_align"] = "RECTIFIED_LEFT"
    return effective
