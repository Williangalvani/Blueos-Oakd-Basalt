import json
import os
import re
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cockpit_widget  # noqa: E402
import feature_stream  # noqa: E402
import vio_telemetry  # noqa: E402
from test_web_ui import client, external_urls, live_telemetry, running, save_config  # noqa: E402,F401

REPO = Path(__file__).resolve().parents[2]
COCKPIT_SCHEMAS = Path(os.environ.get("COCKPIT_SRC", Path.home() / "git/blueos/cockpit")) / "src/libs/blueos/schemas.ts"

# Cockpit src/libs/blueos/schemas.ts: ExtrasJsonSchema and ExternalWidgetSetupInfoSchema (field: (type, required))
EXTRAS_FIELDS = {"targetCockpitApiVersion": ("string", True), "targetSystem": ("string", True),
                 "widgets": ("array", False), "actions": ("array", False), "joystickSuggestions": ("array", False)}
WIDGET_FIELDS = {"name": ("string", True), "iframeUrl": ("string", True), "iconUrl": ("string", False),
                 "iframeIcon": ("string", False), "collapsibleContainerName": ("string", False),
                 "version": ("string", False), "startCollapsed": ("boolean", False),
                 "useExtensionPathAsBaseUrl": ("boolean", False), "contentZoom": ("number", False),
                 "scaleContentWithWidget": ("boolean", False)}
PY_TYPES = {"string": str, "boolean": bool, "array": list, "number": (int, float)}
# BlueOS core/services/helper/main.py ServiceMetadata: required str fields
SERVICE_REQUIRED = ("name", "description", "icon", "company", "version", "webpage", "api")
SERVICE_OPTIONAL = {"route": str, "new_page": bool, "extra_query": str, "avoid_iframes": bool,
                    "sanitized_name": str, "works_in_relative_paths": bool, "extras": dict}


def check_fields(obj, fields):
    """Like Cockpit's zod parse, but also refuses keys zod would silently drop (snake_case, typos)."""
    assert set(obj) <= set(fields), f"keys Cockpit ignores: {set(obj) - set(fields)}"
    for key, (kind, required) in fields.items():
        if required:
            assert key in obj, key
        if key in obj:
            value = obj[key]
            assert isinstance(value, PY_TYPES[kind]) and not (kind == "number" and isinstance(value, bool)), key


def sanitized_name(name):
    """BlueOS helper: re.sub(r"[^a-z0-9]", "", name.lower())"""
    return re.sub(r"[^a-z0-9]", "", name.lower())


# --- extras JSON and service metadata ---------------------------------------------------------------

def test_extras_json_matches_the_cockpit_schema(client):
    main, http = client
    response = http.get("/cockpit_extras.json")
    assert response.status_code == 200 and response.headers["content-type"] == "application/json"
    assert response.headers["access-control-allow-origin"] == "*"
    extras = response.json()
    assert extras == cockpit_widget.cockpit_extras()
    check_fields(extras, EXTRAS_FIELDS)
    assert extras["targetSystem"] == "Cockpit" and re.fullmatch(r"\d+\.\d+\.\d+", extras["targetCockpitApiVersion"])
    depth, pose = extras["widgets"]
    for widget in (depth, pose):
        check_fields(widget, WIDGET_FIELDS)
        # .refine(): either iconUrl or iframeIcon
        assert "iconUrl" in widget or "iframeIcon" in widget
        # Relative to the extension: Cockpit prefixes /extensionv2/<name>, so both must start with "/"
        assert widget["useExtensionPathAsBaseUrl"] is True and widget["iconUrl"].startswith("/")
        assert widget["contentZoom"] == 1 and widget["scaleContentWithWidget"] is True
    assert depth["name"] == "OAK-D VIO depth" and depth["iframeUrl"] == "/widget/depth"
    assert pose["name"] == "OAK-D VIO 3D pose" and pose["iframeUrl"] == "/widget/pose"
    assert pose["iconUrl"] == "/static/pose_icon.svg" and len({depth["name"], pose["name"]}) == 2
    assert extras["actions"] == []


@pytest.mark.skipif(not COCKPIT_SCHEMAS.exists(), reason="Cockpit source not found (set COCKPIT_SRC)")
def test_the_schema_mirror_matches_the_cockpit_source():
    source = COCKPIT_SCHEMAS.read_text()

    def fields(start):
        block = source[source.index(start):]
        block = block[:block.index("})")]
        return {name: (kind, not re.search(r"\.(optional|default|nullish)\(", rest)) for name, kind, rest in
                re.findall(r"^\s+(\w+): z\.(\w+)\((.*)$", block, re.M)}

    widget = fields("const ExternalWidgetSetupInfoSchema")
    assert widget == WIDGET_FIELDS
    extras = fields("export const ExtrasJsonSchema")
    assert {name: kind for name, (kind, _) in extras.items()} == {name: kind for name, (kind, _) in EXTRAS_FIELDS.items()}
    assert {name for name, (_, required) in extras.items() if required} == {"targetCockpitApiVersion", "targetSystem"}
    # Not wrapped in preprocessCamelCase any more: the keys have to be camelCase already
    assert "ExtrasJsonSchema = z.object" in source


def test_register_service_matches_the_blueos_helper_metadata(client):
    main, http = client
    response = http.get("/register_service")
    assert response.status_code == 200 and response.headers["access-control-allow-origin"] == "*"
    metadata = response.json()
    for key in SERVICE_REQUIRED:
        assert isinstance(metadata[key], str) and metadata[key], key
    for key, kind in SERVICE_OPTIONAL.items():
        if key in metadata:
            assert isinstance(metadata[key], kind), key
    assert set(metadata) <= set(SERVICE_REQUIRED) | set(SERVICE_OPTIONAL)
    assert metadata["works_in_relative_paths"] is True
    assert metadata["extras"] == {"cockpit": "/cockpit_extras.json"}
    assert all(isinstance(v, str) for v in metadata["extras"].values())  # Dict[str, str]
    assert sanitized_name(metadata["name"]) == "oakdvio"
    label = re.search(r'LABEL version="([^"]+)"', (REPO / "dockerfile").read_text()).group(1)
    assert metadata["version"] == label


def test_cockpit_discovery_through_the_blueos_proxy(client):
    """Cockpit's getWidgetsFromBlueOS + IFrame URL composition, with BlueOS's nginx route stripping the prefix."""
    main, http = client
    vehicle = "192.168.2.2"
    metadata = http.get("/register_service").json()
    name = sanitized_name(metadata["name"])
    base_url = f"http://{vehicle}/extensionv2/{name}"  # blueOsServiceUrl with worksInRelativePaths
    extras_url = base_url + metadata["extras"]["cockpit"]

    def through_proxy(url):
        path = urlparse(url).path
        prefix = f"/extensionv2/{name}/"
        assert path.startswith(prefix)
        return http.get("/" + path[len(prefix):])

    extras = through_proxy(extras_url).json()
    extension_path = urlparse(base_url).path
    expected = {"/widget/depth": "/static/widget_icon.svg", "/widget/pose": "/static/pose_icon.svg"}
    for widget in extras["widgets"]:
        source = extension_path + widget["iframeUrl"] if widget["useExtensionPathAsBaseUrl"] else widget["iframeUrl"]
        icon = base_url + widget["iconUrl"]
        iframe = f"http://{vehicle}{source}"  # useVehicleAddressAsBase
        assert iframe == "http://192.168.2.2/extensionv2/oakdvio" + widget["iframeUrl"]
        assert icon == "http://192.168.2.2/extensionv2/oakdvio" + expected[widget["iframeUrl"]]
        page = through_proxy(iframe)
        assert page.status_code == 200 and "text/html" in page.headers["content-type"]
        svg = through_proxy(icon)
        assert svg.status_code == 200 and "svg" in svg.headers["content-type"] and "<svg" in svg.text
    assert set(expected) == {widget["iframeUrl"] for widget in extras["widgets"]}

    # The pose widget's module and its vendored three.js, resolved the way the page's script resolves them
    page = through_proxy(f"http://{vehicle}/extensionv2/{name}/widget/pose").text
    base = re.search(r"location\.pathname\.replace\((/.+?/), ''\)", page).group(1)
    assert base == r"/widget\/pose\/?$/"
    page_base = f"http://{vehicle}/extensionv2/{name}/"
    module = through_proxy(page_base + "static/pose3d.js")
    assert module.status_code == 200 and "javascript" in module.headers["content-type"]
    for path in re.findall(r"from '\./(vendor/three/[\w.]+)'", module.text):
        vendored = through_proxy(page_base + "static/" + path)
        assert vendored.status_code == 200 and "javascript" in vendored.headers["content-type"], path


# --- widget page ----------------------------------------------------------------------------------

def test_widget_page_is_self_contained_and_relative(client):
    main, http = client
    response = http.get("/widget/depth")  # VIO stopped and Basalt selected: still 200 for Cockpit's check
    assert response.status_code == 200 and response.headers["content-type"].startswith("text/html")
    assert response.headers["cache-control"] == "no-cache"
    page = response.text
    assert "<!--WIDGET_" not in page and "<style>" in page and "<script>" in page
    assert not re.search(r"<script[^>]+src=|<link[^>]+href=|@import|url\(", page)
    assert external_urls(page) == set()
    for element in ("view", "depth-img", "depth-canvas", "overlay", "status-line", "depth-legend", "message",
                    "feature-keys", "n-inlier", "n-outlier", "n-matched", "n-depth", "n-no_depth"):
        assert f'id="{element}"' in page
    # Relative to the extension's base (port 8095 or /extensionv2/oakdvio/)
    assert "location.pathname.replace(/widget\\/depth\\/?$/, '')" in page
    assert "BASE + 'api/stream/depth'" in page and "BASE + 'api/stream/features'" in page
    assert not re.search(r"""['"`]/api""", page)
    # Fills the iframe without scrollbars; stops the streams while hidden
    assert "overflow: hidden" in page and "visibilityState" in page and "EventSource" in page
    # The same state order as the server, from the overlay shared with the main page (inlined before widget.js)
    assert "const FEATURE_STATES = " + json.dumps(list(feature_stream.STATES)).replace('"', "'") + ";" in page
    assert page.index("function featureToView") < page.index("function drawOverlay")
    for element in ("depth-legend-box", "mono-label", "mono-rate"):
        assert f'id="{element}"' in page
    # The camera link line comes from the formatting shared with the main page
    assert page.index("function linkWidgetText(") < page.index("function renderStatus(")
    # So does the parameter check's hint
    assert page.index("function paramWidgetText(") < page.index("function renderStatus(")
    assert "paramWidgetText(s.autopilot_params)" in page


def test_full_page_links_the_widget_and_works_behind_the_proxy(client):
    main, http = client
    page = http.get("/").text
    assert 'id="widget-link" href="widget/depth"' in page
    js = http.get("/static/ui.js").text
    # Every API call is relative, so BlueOS can show the page at /extensionv2/oakdvio/
    assert "'api/status'" in js and not re.search(r"""['"`]/api""", js)


def test_widget_icon_is_served(client):
    main, http = client
    response = http.get(cockpit_widget.WIDGET_ICON_PATH)
    assert response.status_code == 200 and response.text.lstrip().startswith("<svg")


# --- widget status --------------------------------------------------------------------------------

def test_widget_problem():
    pnp, up = {"algorithm": "pnp"}, {"running": True}
    fresh = {"stale": False, "phase": "running"}
    assert "PnP" in cockpit_widget.widget_problem({"algorithm": "basalt"}, up, fresh)
    assert cockpit_widget.widget_problem(pnp, {"running": False}, fresh) == "VIO stopped"
    assert "waiting" in cockpit_widget.widget_problem(pnp, up, None)
    assert "waiting" in cockpit_widget.widget_problem(pnp, up, {"stale": True, "phase": "running"})
    assert cockpit_widget.widget_problem(pnp, up, {"stale": False, "phase": "opening_device"}) == "VIO opening device…"
    assert cockpit_widget.widget_problem(pnp, up, fresh) is None


def test_widget_status_from_live_telemetry():
    now = time.time()
    telemetry = live_telemetry(fps=10, seconds=3.0, now=now, mode="log-only")
    status = telemetry.status(now)
    status["stale"] = False
    s = cockpit_widget.widget_status(None, status, {"enabled": True, "streaming": True, "fps": 4.9,
                                                    "depth_range_mm": [300, 4000], "gray_far": 16,
                                                    "encoder_stalled": True},
                                     log_only=True, now=now + 1)
    assert s["problem"] is None and s["log_only"] is True and s["output_mode"] == "log-only"
    assert s["fps"] == pytest.approx(10, rel=0.1) and s["ok_percent"] == pytest.approx(80, abs=5)
    assert s["last_frame"]["features"] == 150 and s["last_frame"]["valid_depth"] == 120
    assert s["sent_total"] == status["sent_total"] > 0
    assert s["last_failure"]["reason"] == "too_few_matches" and 0 < s["last_failure"]["age_s"] < 5
    assert s["depth"]["fps"] == 4.9 and s["depth"]["depth_range_mm"] == [300, 4000]
    assert s["depth"]["encoder_stalled"] is True
    assert s["pose"] is None  # the VIO's telemetry has no pose status without a pose stream
    assert s["link"] is None  # nor a link block without a link monitor
    json.dumps(s, allow_nan=False)
    # The camera link's headline numbers reach the widget
    status["link"] = {"available": True, "mock": False, "down_mbps_10s": 61.9, "peak_down_mbps_60s": 64.0,
                      "share_of_100mbps": 0.619, "reference_mbps": 100.0, "streams": [{"name": "depth"}]}
    s = cockpit_widget.widget_status(None, status, {}, False, now + 1)
    assert s["link"] == {"available": True, "mock": False, "down_mbps_10s": 61.9, "peak_down_mbps_60s": 64.0,
                         "share_of_100mbps": 0.619, "reference_mbps": 100.0}
    del status["link"]
    # The depth encoding, blank frames and the pose's attitude source reach the widget
    status["pose"] = {"frame": "NED", "frame_text": "NED", "attitude_source": "gyro",
                      "attitude_text": "gyro-integrated, drifting", "attitude_age_s": 7.5, "x": 1.0}
    blank = {"variant": "manip", "text": "blank frames from manip: ..."}
    s = cockpit_widget.widget_status(None, status, {"encoding": "gray8-disparity", "blank": blank,
                                                    "disparity": {"nearest_mm": 347}}, False, now + 1)
    assert s["pose"] == {"frame": "NED", "frame_text": "NED", "attitude_source": "gyro",
                         "attitude_text": "gyro-integrated, drifting", "attitude_age_s": 7.5}
    assert s["depth"]["encoding"] == "gray8-disparity" and s["depth"]["blank"] == blank
    assert s["depth"]["disparity"] == {"nearest_mm": 347}
    # mono_features: the widget shows the camera image, not depth, and says where it comes from
    mono = {"encoding": "gray8-mono-rectified-left", "variant": "mono_features", "requested_variant": "auto",
            "encoded_on": "camera", "source": "stereo.rectifiedLeft", "variant_reason": None}
    s = cockpit_widget.widget_status(None, status, mono, False, now + 1)
    assert {key: s["depth"][key] for key in mono} == mono
    # Stale telemetry shows nothing old
    status["stale"] = True
    stale = cockpit_widget.widget_status("VIO stopped", status, {}, False, now)
    assert stale["fps"] is None and stale["last_frame"] is None and stale["last_failure"] is None
    # The parameter check, without the values (the widget only hints; the main page has the table)
    assert s["autopilot_params"] is None
    check = {"checked_at": now, "age_s": 3.0, "reachable": True, "armed": False, "ok": False,
             "mismatches": [{"name": "EK3_SRC1_POSXY", "current": 6.0, "recommended": 0, "needs_reboot": False}],
             "unreadable": [], "pending_reboot": ["VISO_TYPE"], "error": None, "checking": False,
             "interval_s": 30.0, "mavlink2rest": "http://x/mavlink2rest"}
    s = cockpit_widget.widget_status(None, status, {}, False, now, autopilot_params=check)
    assert s["autopilot_params"] == {"checked_at": now, "reachable": True, "ok": False,
                                     "mismatches": [{"name": "EK3_SRC1_POSXY"}], "unreadable": [],
                                     "pending_reboot": ["VISO_TYPE"]}


# --- feature endpoints ----------------------------------------------------------------------------

def sse_events(body):
    events = []
    for block in body.decode().split("\n\n"):
        fields = dict(line.split(": ", 1) for line in block.splitlines() if ": " in line)
        if "event" in fields:
            events.append((fields["event"], json.loads(fields["data"])))
    return events


def test_feature_stream_without_the_vio_explains_why(client, tmp_path):
    main, http = client
    response = http.get("/api/stream/features?events=1")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/event-stream")
    assert response.headers["cache-control"] == "no-store" and response.headers["x-accel-buffering"] == "no"
    assert response.content.startswith(b"retry: 3000\n\n")
    ((kind, status),) = sse_events(response.content)
    assert kind == "status" and "PnP" in status["problem"]
    save_config(tmp_path, algorithm="pnp")
    ((_, status),) = sse_events(http.get("/api/stream/features?events=1").content)
    assert status["problem"] == "VIO stopped"
    # Asking still takes the leases, so a VIO that starts publishes right away
    assert (vio_telemetry.DEFAULT_DIR / feature_stream.LEASE_FILE).exists()
    assert (vio_telemetry.DEFAULT_DIR / vio_telemetry.STATUS_LEASE).exists()


@pytest.fixture
def publishing_vio():
    """A VIO stand-in: real Telemetry and FeatureStreamService, 20 frames per second, 0.05 s publish interval."""
    directory = vio_telemetry.DEFAULT_DIR
    telemetry = live_telemetry(fps=20, seconds=2.0)
    service = feature_stream.FeatureStreamService(directory, (640, 400), interval=0.05, poll_interval=0.01,
                                                  log=lambda m: None)
    stop = threading.Event()
    uv = np.array([[100.0, 100.0], [320.0, 200.0], [600.0, 380.0], [10.0, 10.0]])

    def loop():
        n = 0
        while not stop.is_set():
            t = time.time()
            telemetry.record_frame(t, True, "ok", 4, 3, 2, 1, 5.0, dt=0.05, position_delta=(0, 0, 0), sent=False)
            service.tick(uv, (200, 320), n, True, None, depth_valid=[True, True, True, False],
                         matched_index=[0, 1, 2], inlier_index=[0, 2])
            telemetry.publish_due()
            n += 1
            time.sleep(0.02)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield service
    stop.set()
    thread.join()


def test_feature_stream_forwards_the_vio_features(client, tmp_path, publishing_vio):
    main, http = client
    save_config(tmp_path, algorithm="pnp", log_only=True)
    running(http)
    response = http.get("/api/stream/features?events=4")
    events = sse_events(response.content)
    assert [kind for kind, _ in events].count("status") >= 1 and events[0][0] == "status"
    features = [data for kind, data in events if kind == "features"]
    assert features, events
    f = features[-1]
    # Points on the (decimated) depth grid, states by priority
    assert f["frame_size"] == [320, 200] and f["image_size"] == [640, 400]
    assert f["points"] == [[50.0, 50.0, feature_stream.INLIER], [160.0, 100.0, feature_stream.OUTLIER],
                           [300.0, 190.0, feature_stream.INLIER], [5.0, 5.0, feature_stream.NO_DEPTH]]
    assert f["counts"]["inlier"] == 2 and f["states"] == list(feature_stream.STATES)
    # The bytes the VIO wrote, unchanged
    raw = response.content.decode().split("event: features\ndata: ")[1].split("\n\n")[0]
    assert json.loads(raw) == features[0]
    assert publishing_vio.published >= 1

    # Status: the VIO's numbers once its status.json is published (the endpoint takes the status lease)
    deadline = time.time() + 3
    status = {}
    while time.time() < deadline:
        status = sse_events(http.get("/api/stream/features?events=1").content)[0][1]
        if status["problem"] is None:
            break
        time.sleep(0.1)
    assert status["problem"] is None and status["log_only"] is True and status["last_frame"]["inliers"] == 1
    assert status["last_failure"]["reason"] == "too_few_matches" and isinstance(status["fps"], float)

    # Nobody watching: once the linger ends the lease is removed and the VIO stops writing
    lease = vio_telemetry.DEFAULT_DIR / feature_stream.LEASE_FILE
    assert lease.exists()
    main.viewer_registry.linger_s = 0.0
    main.viewer_registry.tick()
    assert not lease.exists()
    deadline = time.time() + 3
    while publishing_vio.wanted and time.time() < deadline:
        time.sleep(0.02)
    assert not publishing_vio.wanted
    count = publishing_vio.published
    time.sleep(0.2)
    assert publishing_vio.published == count


def test_features_one_shot(client, tmp_path):
    main, http = client
    response = http.get("/api/features")
    assert response.status_code == 503 and "PnP" in response.json()["detail"]
    save_config(tmp_path, algorithm="pnp")
    running(http)
    response = http.get("/api/features")
    assert response.status_code == 404 and "retry" in response.json()["detail"]
    assert (vio_telemetry.DEFAULT_DIR / feature_stream.LEASE_FILE).exists()
    service = feature_stream.FeatureStreamService(vio_telemetry.DEFAULT_DIR, (640, 400), log=lambda m: None)
    assert service.tick(np.array([[64.0, 40.0]]), (400, 640), 9, False, "too_few_matches", [True])
    response = http.get("/api/features")
    assert response.status_code == 200 and response.headers["cache-control"] == "no-store"
    data = response.json()
    assert data["points"] == [[64.0, 40.0, feature_stream.DEPTH]] and data["reason"] == "too_few_matches"
    assert data["sequence"] == 9 and data["ok"] is False
