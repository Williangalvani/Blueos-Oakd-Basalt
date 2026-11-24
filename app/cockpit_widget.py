"""
BlueOS service metadata and the Cockpit external widget (live depth + tracked features).

Discovery:
1. BlueOS's helper scans the listening ports and reads GET /register_service (service_metadata()). Its
   name gives the sanitized name "oakdvio". works_in_relative_paths makes BlueOS proxy the extension at
   /extensionv2/oakdvio/ (nginx strips the prefix, passes WebSocket upgrades, and honours
   X-Accel-Buffering: no for the streams).
2. Cockpit lists the services from /helper/v1.0/web_services. For each one with extras.cockpit it fetches
   <base> + extras.cockpit, with <base> = http://<vehicle>/extensionv2/oakdvio, and parses it with
   ExtrasJsonSchema (src/libs/blueos/schemas.ts). The keys must be camelCase: current Cockpit doesn't
   convert them, and older versions converted snake_case to camelCase.
3. With useExtensionPathAsBaseUrl, Cockpit prefixes iframeUrl and iconUrl with /extensionv2/oakdvio and
   composes the iframe source from the vehicle address, so the widget works from a remote Cockpit too.

Every URL in the widget page is relative to the extension's base, so it works both directly on port 8095
and behind the proxy.
"""

SERVICE_NAME = "OAK-D VIO"
VERSION = "0.0.2"  # the dockerfile's version LABEL
EXTRAS_PATH = "/cockpit_extras.json"
WIDGET_PATH = "/widget/depth"
WIDGET_ICON_PATH = "/static/widget_icon.svg"
WIDGET_NAME = "OAK-D VIO depth"
COCKPIT_API_VERSION = "1.0.0"


def service_metadata():
    """GET /register_service: helper.ServiceMetadata (name, description, icon, company, version, webpage, api)."""
    return {
        "name": SERVICE_NAME,
        "description": "PnP visual odometry from an OAK-D camera for ArduPilot (VISION_POSITION_DELTA), "
                       "with a live depth and tracked-features widget for Cockpit",
        "icon": "mdi-camera-iris",
        "company": "Blue Robotics",
        "version": VERSION,
        "webpage": "https://github.com/williangalvani/Blueos-Oakd-Basalt",
        "api": "/docs",
        "works_in_relative_paths": True,
        "extras": {"cockpit": EXTRAS_PATH},
    }


def cockpit_extras():
    """GET /cockpit_extras.json: Cockpit's ExtrasJsonSchema with one ExternalWidgetSetupInfo."""
    return {
        "targetSystem": "Cockpit",
        "targetCockpitApiVersion": COCKPIT_API_VERSION,
        "widgets": [{
            "name": WIDGET_NAME,
            "iframeUrl": WIDGET_PATH,
            "iconUrl": WIDGET_ICON_PATH,
            "version": VERSION,
            "useExtensionPathAsBaseUrl": True,
            # The page fills its iframe and scales its own drawing; Cockpit's zoom only affects the text
            "contentZoom": 1,
            "scaleContentWithWidget": True,
        }],
        "actions": [],
    }


def widget_problem(config, process, telemetry):
    """Why the widget can't show live features right now, or None."""
    if config.get("algorithm") != "pnp":
        return f"needs the PnP algorithm (current: {config.get('algorithm')})"
    if not process.get("running"):
        return "VIO stopped"
    if telemetry is None or telemetry.get("stale"):
        return "waiting for the VIO process…"
    if telemetry.get("phase") != "running":
        return f"VIO {str(telemetry.get('phase')).replace('_', ' ')}…"
    return None


def widget_status(problem, telemetry, depth, log_only, now, mock=False):
    """The status line of the widget: one small JSON object, sent once a second. mock: mock mode (no camera)."""
    telemetry = telemetry if telemetry and not telemetry.get("stale") else {}
    recent = telemetry.get("recent") or {}
    last = telemetry.get("last_frame") or {}
    output = telemetry.get("output") or {}
    failure = telemetry.get("last_failure")
    depth = depth or {}
    return {
        "server_time": now,
        "problem": problem,
        "fps": recent.get("fps"),
        "fps_configured": telemetry.get("fps_configured"),
        "ok_percent": recent.get("ok_percent"),
        "inliers_mean": recent.get("inliers_mean"),
        "min_inliers": telemetry.get("min_inliers"),
        "last_frame": {key: last.get(key) for key in ("ok", "reason", "features", "valid_depth", "matches",
                                                      "inliers")} if last else None,
        "last_failure": None if not failure else {
            "reason": failure.get("reason"),
            "age_s": now - failure["unix_time"] if failure.get("unix_time") else None},
        "sent_total": output.get("messages_sent"),
        "send_rate_hz": output.get("rate_hz"),
        "output_mode": output.get("mode"),
        "log_only": bool(log_only),
        "mock": bool(mock),
        "depth": {key: depth.get(key) for key in (
            "enabled", "streaming", "fps", "fps_target", "image_size", "depth_range_mm", "gray_far", "gray_near",
            "gray_invalid_below", "errors", "last_error", "encoder_stalled")},
    }
