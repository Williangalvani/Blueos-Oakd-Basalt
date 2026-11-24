"""
Schema of the saved configuration (config.json): one entry per field with its type, default, bounds,
label and help text. main.py builds DEFAULT_CONFIG, the pydantic model behind POST /api/config and the
settings form from it, and serves it as GET /api/config/schema so the page validates with the same rules.
The stereo fields come from stereo_config.STEREO_SETTINGS, which also drives pnp_vio.py's arguments.
"""

import math
import re
from dataclasses import dataclass
from html import escape
from typing import List, Literal, Optional

from pydantic import BaseModel, Field, ValidationError, create_model, field_validator

from stereo_config import (DEFAULT_LEVER_ARM_M, DEFAULT_REFRACTIVE_INDEX, DEPTH_SCALE_RANGE, INTRINSICS_SOURCES,
                           LEVER_ARM_LIMIT_M, REFRACTIVE_INDEX_RANGE, STEREO_SETTINGS, format_lever_arm,
                           parse_intrinsics_override, parse_lever_arm)

DVL_SOURCES = ("auto", "off")
# depth_stream.VARIANTS (kept here so the schema does not import the stream code)
DEPTH_STREAM_ENCODINGS = ("auto", "mono_features", "disparity", "host_jpeg")
# pose_stream.ATTITUDE_MODES
POSE_ATTITUDE_SOURCES = ("autopilot", "gyro")
# depth_points.TRANSPORTS
DEPTH_TRANSPORTS = ("points", "full_frame")
# snapshots.ENCODINGS
SNAPSHOT_ENCODINGS = ("camera_jpeg", "raw")
VEHICLE_IP_RE = re.compile(r"^[A-Za-z0-9.\-:\[\]]+$")

GROUPS = (
    ("pnp", "PnP (option D)"),
    ("camera", "Camera"),
    ("stereo", "Stereo depth (PnP)"),
    ("output", "MAVLink output"),
)


@dataclass(frozen=True)
class ConfigField:
    key: str
    kind: str  # int, float, bool, choice, text, lever_arm, intrinsics
    default: object
    label: str
    help: str = ""
    group: str = "pnp"
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    choices: tuple = ()
    nullable: bool = False
    step: Optional[float] = None
    placeholder: str = ""

    def as_dict(self):
        default = list(self.default) if isinstance(self.default, tuple) else self.default
        return {"name": self.key, "type": self.kind, "default": default, "label": self.label, "help": self.help,
                "group": self.group, "min": self.minimum, "max": self.maximum, "choices": list(self.choices),
                "nullable": self.nullable, "step": self.step, "placeholder": self.placeholder}


def _stereo_field(setting):
    if setting.choices:
        kind = "choice"
    else:
        kind = {bool: "bool", int: "int", float: "float"}.get(setting.kind, "text")
    return ConfigField(setting.key, kind, setting.default, setting.label, setting.hint, "stereo",
                       setting.minimum, setting.maximum, setting.choices, setting.optional,
                       step=0.01 if setting.kind is float else (1 if setting.kind is int else None),
                       placeholder="auto" if setting.optional else "")


CONFIG_FIELDS = (
    ConfigField("refractive_index", "float", DEFAULT_REFRACTIVE_INDEX, "PnP Refractive Index (water)",
                "PnP only: water in front of the flat port. The calibration is in air, so fx, fy and the depth are "
                "multiplied by it. 1.333 fresh water, ~1.34 sea water, 1.0 = in air (bench tests)",
                minimum=REFRACTIVE_INDEX_RANGE[0], maximum=REFRACTIVE_INDEX_RANGE[1], step=0.001),
    ConfigField("depth_scale", "float", 1.0, "PnP Depth Scale",
                "PnP only: extra trim on the camera depth on top of the refractive index (e.g. a residual measured "
                "with the depth probe); 1 = none", minimum=DEPTH_SCALE_RANGE[0], maximum=DEPTH_SCALE_RANGE[1],
                step=0.001),
    ConfigField("min_inliers", "int", 15, "PnP Min Inliers",
                "PnP only: RANSAC inliers a frame needs to be accepted (and sent)", minimum=6, maximum=500, step=1),
    ConfigField("max_reference_age", "float", 1.0, "PnP Max Reference Age (s)",
                "PnP only: how long failed frames keep the last good frame as reference, so the next delta covers "
                "the gap; 0 = no limit", minimum=0.0, maximum=10.0, step=0.1),
    ConfigField("num_features", "int", 300, "PnP Features", "PnP only: features tracked on the camera",
                minimum=50, maximum=1000, step=1),
    ConfigField("max_features", "int", 0, "PnP Max Features",
                "PnP only: hard cap on the features the camera detects per frame (FeatureTracker numMaxFeatures); "
                "0 = depthai's automatic maximum. Each feature costs about 64 B on the link at every frame (the "
                "message has no smaller form) and one ROI in points mode", minimum=0, maximum=1000, step=1),
    ConfigField("tracking_recovery_frames", "int", 20, "PnP Recovery After (frames)",
                "PnP only: consecutive failed frames before the tracker is re-initialised; 0 disables",
                minimum=0, maximum=1000, step=1),
    ConfigField("lever_arm_enabled", "bool", False, "Lever arm diagnostics (log only)",
                "PnP only. Does NOT change what is sent: the autopilot applies the camera offset itself from "
                "VISO_POS_X/Y/Z. When on, the data log also gets each delta corrected to the IMU (lever_delta_* "
                "fields), for comparison only"),
    ConfigField("lever_arm_m", "lever_arm", DEFAULT_LEVER_ARM_M, "Lever Arm x,y,z (m)",
                "Camera relative to the IMU, body FRD (forward, right, down), as VISO_POS_X/Y/Z. Only used by the "
                "diagnostics above", minimum=-LEVER_ARM_LIMIT_M, maximum=LEVER_ARM_LIMIT_M,
                placeholder="0.1655,-0.0069,0.18"),
    ConfigField("intrinsics_from", "choice", "rectified", "PnP Intrinsics Source",
                "PnP only: camera matrix of the rectified-left image. rectified is the matrix the camera rectifies "
                "to; frame reads the depth frame matrix; cam_b/cam_c are the raw calibration",
                choices=INTRINSICS_SOURCES),
    ConfigField("intrinsics_override", "intrinsics", None, "PnP Intrinsics Override",
                "PnP only: wins over the source; pixels on the width x height grid. Empty = off", nullable=True,
                placeholder="fx,fy,cx,cy"),
    ConfigField("depth_transport", "choice", "points", "PnP Depth Transport",
                "PnP only: points = the camera computes the depth at the tracked features (a 5x5 average per "
                "feature) and sends only those (~2 Mbps instead of ~40 at 640x400 10 fps); full depth frames cross "
                "the link only while the depth probe runs or someone watches a host_jpeg stream. full_frame = every "
                "depth frame goes to the Pi (the previous behaviour, for comparison)", choices=DEPTH_TRANSPORTS),
    ConfigField("snapshots", "bool", True, "Snapshots",
                "PnP only: on-demand camera snapshots (live views); no bandwidth until requested"),
    ConfigField("snapshot_encoding", "choice", "camera_jpeg", "Snapshot Encoding",
                "PnP only: camera_jpeg = rect_left, rect_right and rgb snapshots are JPEG-encoded on the camera "
                "(quality 95, instead of 256 kB raw at 640x400); raw = sent raw and PNG-encoded on the Pi (the fallback, "
                "also used automatically if the camera's encoder returns nothing). The depth snapshot is always "
                "raw: 16-bit millimetres do not fit a JPEG", choices=SNAPSHOT_ENCODINGS),
    ConfigField("depth_stream", "bool", True, "Live depth stream",
                "PnP only: the camera image with the tracked features (or the depth), MJPEG-encoded on the "
                "camera for the Live panel; sent only while the panel is open"),
    ConfigField("depth_stream_fps", "float", 5.0, "Depth Stream FPS",
                "PnP only: the camera sends every round(FPS / this)-th depth frame", minimum=0.5, maximum=15.0,
                step=0.5),
    ConfigField("depth_stream_quality", "int", 70, "Depth Stream JPEG Quality",
                "PnP only: MJPEG quality on the camera (lower = less USB bandwidth)", minimum=10, maximum=100,
                step=1),
    ConfigField("depth_stream_encoding", "choice", "auto", "Depth Stream Encoding",
                "PnP only: auto = mono_features, then host_jpeg if the camera sends no frames or blank ones; "
                "mono_features = the rectified-left camera image the features are tracked on, encoded on the "
                "camera, with the tracked features drawn by the browser (Live camera); disparity = the camera's "
                "disparity (pixels) straight to the camera encoder, needs an 8-bit disparity (Subpixel and "
                "Extended off, Speckle off or Median filter last), else it runs as host_jpeg; host_jpeg = the "
                "Pi encodes the depth (uses Pi CPU, capped at 25 % of a core)",
                choices=DEPTH_STREAM_ENCODINGS),
    ConfigField("depth_stream_debug", "bool", False, "Depth stream camera debug log",
                "PnP only: while the depth stream runs, the camera logs at DEBUG level and the stream nodes' log "
                "lines appear under Live depth (and /api/status); back to normal when the stream stops"),
    ConfigField("pose_attitude_source", "choice", "autopilot", "3D Pose Attitude",
                "PnP only, viewer only: autopilot = the 3D pose's orientation is ArduPilot's ATTITUDE (read from "
                "mavlink2rest, NED with the autopilot heading), gyro-integrated (drifting) while it is missing or "
                "more than 3 s old; gyro = gyro-integrated only. The VIO output does not use it",
                choices=POSE_ATTITUDE_SOURCES),

    ConfigField("fps", "int", 30, "FPS", "Camera frame rate; lower = less camera and Pi CPU", "camera", 1, 60, step=1),
    ConfigField("camera_angle", "float", 0.0, "Camera Angle (°)", "0 = forward, 90 = down", "camera", -180.0, 180.0,
                step=0.1),
    ConfigField("resolution_width", "int", 640, "Width", "Rectified-left image width (px)", "camera", 160, 1280,
                step=1),
    ConfigField("resolution_height", "int", 400, "Height", "Rectified-left image height (px)", "camera", 100, 800,
                step=1),

    *(_stereo_field(setting) for setting in STEREO_SETTINGS),

    ConfigField("vehicle_ip", "text", "127.0.0.1", "Vehicle IP",
                "Host (and optional :port) of BlueOS's mavlink2rest", "output", placeholder="127.0.0.1"),
    ConfigField("quality_rate", "float", 1.0, "Quality Rate (Hz)", "NAMED_VALUE_FLOAT quality messages per second",
                "output", 0.1, 10.0, step=0.1),
    ConfigField("log_only", "bool", False, "Log only: do not send anything to the autopilot",
                "No MAVLink output and no autopilot parameter changes; poses only go to the data log", "output"),
    ConfigField("dvl_source", "choice", "auto", "DVL overlay source",
                "auto: plot the VISION_POSITION_DELTA velocity of another MAVLink component (e.g. the Water Linked "
                "DVL extension) dashed over the VIO velocity, read from mavlink2rest at <= 2 Hz while the page is "
                "open; off: never poll", "output", choices=DVL_SOURCES),
)

FIELDS_BY_KEY = {field.key: field for field in CONFIG_FIELDS}


def default_config() -> dict:
    return {field.key: list(field.default) if isinstance(field.default, tuple) else field.default
            for field in CONFIG_FIELDS}


def migrate_saved(saved: dict) -> dict:
    """
    A config.json from any earlier version as a full config: keys of removed settings (e.g. algorithm, the Basalt
    tuning) are dropped, missing ones get their default, and a choice that no longer exists gets the default.
    """
    config = default_config()
    for key, value in saved.items():
        field = FIELDS_BY_KEY.get(key)
        if field is None:
            continue
        removed_choice = field.kind == "choice" and value not in field.choices and not (value is None and field.nullable)
        config[key] = field.default if removed_choice else value
    return config


def schema() -> dict:
    """What GET /api/config/schema returns."""
    return {"groups": [{"name": name, "label": label} for name, label in GROUPS],
            "fields": [field.as_dict() for field in CONFIG_FIELDS]}


# --- server-side validation -----------------------------------------------------------------------

def _annotation(field):
    if field.kind == "bool":
        kind = bool
    elif field.kind == "int":
        kind = int
    elif field.kind == "float":
        kind = float
    elif field.kind == "choice":
        kind = Literal[field.choices]
    elif field.kind == "lever_arm":
        kind = List[float]
    else:
        kind = str
    return Optional[kind] if field.nullable else kind


def _pydantic_field(field):
    if field.kind == "lever_arm":
        return Field(default_factory=lambda: list(DEFAULT_LEVER_ARM_M))
    bounds = {}
    if field.kind in ("int", "float"):
        if field.minimum is not None:
            bounds["ge"] = field.minimum
        if field.maximum is not None:
            bounds["le"] = field.maximum
    return Field(field.default, description=field.help, **bounds)


class _ConfigBase(BaseModel):
    @field_validator("lever_arm_m", mode="before", check_fields=False)
    @classmethod
    def _check_lever_arm(cls, value):
        return list(parse_lever_arm(value))

    @field_validator("intrinsics_override", check_fields=False)
    @classmethod
    def _check_intrinsics_override(cls, value):
        if value is None or not value.strip():
            return None
        parse_intrinsics_override(value)
        return value.strip()

    @field_validator("vehicle_ip", check_fields=False)
    @classmethod
    def _check_vehicle_ip(cls, value):
        value = value.strip()
        if not VEHICLE_IP_RE.match(value):
            raise ValueError("must be a host name or IP address, optionally with :port")
        return value


ConfigModel = create_model("VIOConfig", __base__=_ConfigBase,
                           **{field.key: (_annotation(field), _pydantic_field(field)) for field in CONFIG_FIELDS})


def _number(value):
    return f"{value:g}" if isinstance(value, float) else str(value)


def _friendly(error):
    loc = [part for part in error.get("loc", ()) if part != "body"]
    key = str(loc[0]) if loc else ""
    field = FIELDS_BY_KEY.get(key)
    label = f"{field.label} ({key})" if field else (key or "config")
    kind, ctx, given = error.get("type", ""), error.get("ctx") or {}, error.get("input")
    if field is not None and kind in ("less_than_equal", "greater_than_equal") and \
            field.minimum is not None and field.maximum is not None:
        message = f"must be between {_number(field.minimum)} and {_number(field.maximum)} (got {given!r})"
    elif kind == "literal_error" and field is not None:
        message = f"must be one of {', '.join(str(c) for c in field.choices)} (got {given!r})"
    elif kind == "value_error":
        message = str(ctx.get("error", error.get("msg", "invalid value")))
    elif kind in ("int_parsing", "int_from_float", "int_type"):
        message = f"must be a whole number (got {given!r})"
    elif kind in ("float_parsing", "float_type", "finite_number"):
        message = f"must be a number (got {given!r})"
    else:
        message = error.get("msg", "invalid value")
    return {"loc": ["body", *loc], "field": key, "msg": f"{label}: {message}", "type": kind}


def _cross_field_errors(config):
    errors = []
    low, high = config.get("stereo_depth_min_mm"), config.get("stereo_depth_max_mm")
    if isinstance(low, int) and isinstance(high, int) and low >= high:
        errors.append({"loc": ["body", "stereo_depth_max_mm"], "field": "stereo_depth_max_mm", "type": "value_error",
                       "msg": f"Max depth (mm) (stereo_depth_max_mm): must be above Min depth (mm) ({low})"})
    for key in ("refractive_index", "depth_scale", "max_reference_age", "camera_angle", "quality_rate",
                "depth_stream_fps"):
        value = config.get(key)
        if isinstance(value, float) and not math.isfinite(value):
            errors.append({"loc": ["body", key], "field": key, "type": "finite_number",
                           "msg": f"{FIELDS_BY_KEY[key].label} ({key}): must be a finite number"})
    return errors


def validate_config(data) -> tuple:
    """(clean config dict or None, list of errors). Each error has loc/msg/type like FastAPI's 422 detail, plus field."""
    if not isinstance(data, dict):
        return None, [{"loc": ["body"], "field": "", "msg": "config must be a JSON object", "type": "dict_type"}]
    try:
        config = ConfigModel(**data).model_dump()
    except ValidationError as e:
        return None, [_friendly(error) for error in e.errors()]
    errors = _cross_field_errors(config)
    return (None, errors) if errors else (config, [])


# --- settings form --------------------------------------------------------------------------------

def form_value(field, value):
    if field.kind == "lever_arm":
        try:
            return format_lever_arm(parse_lever_arm(value))
        except (TypeError, ValueError):
            return format_lever_arm(DEFAULT_LEVER_ARM_M)
    return "" if value is None else str(value)


def render_field(field, value) -> str:
    """One settings row; input markup as the page has always rendered it (name before value)."""
    name = escape(field.key)
    hint = f'<div class="hint" title="{escape(field.help)}">{escape(field.help)}</div>' if field.help else ""
    if field.kind == "bool":
        control = (f'<label class="checkbox"><input type="checkbox" name="{name}" {"checked" if value else ""}> '
                   f'{escape(field.label)}</label>')
    else:
        label = f'<label>{escape(field.label)}</label>'
        if field.kind == "choice":
            options = "".join(
                f'<option value="{escape(str(choice))}"{" selected" if str(choice) == str(value) else ""}>'
                f'{escape(str(choice))}</option>' for choice in field.choices)
            control = f'{label}<select name="{name}" data-value="{escape(form_value(field, value))}">{options}</select>'
        elif field.kind in ("int", "float"):
            bounds = "".join(f' {attr}="{_number(bound)}"' for attr, bound in (("min", field.minimum),
                                                                             ("max", field.maximum))
                             if bound is not None)
            step = "any" if field.kind == "float" else "1"
            control = (f'{label}<input type="number" name="{name}" value="{escape(form_value(field, value))}" '
                       f'step="{step}"{bounds}>')
        else:
            placeholder = f' placeholder="{escape(field.placeholder)}"' if field.placeholder else ""
            control = (f'{label}<input type="text" name="{name}" value="{escape(form_value(field, value))}"'
                       f'{placeholder}>')
    return f'<div class="field" data-field="{name}">{control}{hint}</div>'


def render_group(group, config) -> str:
    return "\n".join(render_field(field, config.get(field.key, field.default))
                     for field in CONFIG_FIELDS if field.group == group)
