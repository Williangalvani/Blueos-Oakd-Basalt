import hashlib
import json
import math
import os
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mock_mode  # noqa: E402
import pose_stream  # noqa: E402
import vio_telemetry  # noqa: E402
from test_cockpit_widget import sse_events  # noqa: E402
from test_web_ui import client, external_urls, running, save_config  # noqa: E402,F401

STATIC = Path(__file__).resolve().parents[1] / "static"


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


# --- the integrated pose --------------------------------------------------------------------------

def test_integrator_chains_body_deltas_in_start_frd():
    integrator = pose_stream.PoseIntegrator()
    integrator.step([1.0, 0.0, 0.0])                                   # 1 m forward
    pose = integrator.step([1.0, 0.0, 0.0], rotation_delta=[0.0, 0.0, math.pi / 2])  # turn right, 1 m forward
    assert (pose["x"], pose["y"], pose["z"]) == pytest.approx((1.0, 1.0, 0.0))
    assert pose["yaw"] == pytest.approx(math.pi / 2) and pose["roll"] == pytest.approx(0.0)
    pose = integrator.step([0.0, 0.0, 0.5])                            # down is +z
    assert pose["z"] == pytest.approx(0.5)
    pose = integrator.step(None, yaw_delta=-math.pi / 2)               # yaw only, no displacement
    assert pose["yaw"] == pytest.approx(0.0) and pose["x"] == pytest.approx(1.0)
    pose = integrator.step([0.0, 0.0, 0.0], rotation_delta=[0.1, 0.0, 0.0])
    assert pose["roll"] == pytest.approx(0.1) and pose["pitch"] == pytest.approx(0.0, abs=1e-12)
    for bad in ([float("nan")] * 3, None):
        before = dict(integrator.pose())
        integrator.step(bad, rotation_delta=bad)
        assert integrator.pose() == before


def test_quaternion_and_euler_describe_the_same_rotation():
    rng = np.random.default_rng(1)
    for _ in range(50):
        rotvec = rng.normal(0, 1.2, 3)
        R = pose_stream.rotvec_to_matrix(rotvec)
        assert np.allclose(R @ R.T, np.eye(3)) and np.linalg.det(R) == pytest.approx(1.0)
        w, x, y, z = pose_stream.quaternion(R)
        assert w >= 0 and w * w + x * x + y * y + z * z == pytest.approx(1.0)
        from_q = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                           [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                           [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        assert np.allclose(from_q, R, atol=1e-9)
        roll, pitch, yaw = pose_stream.euler_zyx(R)
        assert np.allclose(pose_stream.rotvec_to_matrix([0, 0, yaw]) @ pose_stream.rotvec_to_matrix([0, pitch, 0])
                           @ pose_stream.rotvec_to_matrix([roll, 0, 0]), R, atol=1e-9)


def test_the_mock_survey_is_a_closed_lawnmower():
    """mock_mode.motion and mock_vio's rotation deltas through the viewer's integration: lanes, turns, depth steps."""
    import mock_vio
    integrator, dt = pose_stream.PoseIntegrator(), 0.05
    xs, ys, zs, yaws, rolls, pitches = [], [], [], [], [], []
    assert mock_mode.heading(mock_mode.SURVEY_PERIOD_S - 1e-9) == pytest.approx(0.0, abs=1e-6)
    for n in range(1, int(round(mock_mode.SURVEY_PERIOD_S / dt)) + 1):
        t = n * dt
        vx, vy, vz, yaw_rate = mock_mode.motion(t)
        yaw_delta = mock_mode.heading(t) - mock_mode.heading(t - dt)
        assert abs(yaw_delta) <= math.pi / 2 / mock_mode.TURN_S * dt + 1e-9
        pose = integrator.step([vx * dt, vy * dt, vz * dt], mock_vio.body_rotation_delta(t - dt, t, yaw_delta))
        for values, key in ((xs, "x"), (ys, "y"), (zs, "z"), (yaws, "yaw"), (rolls, "roll"), (pitches, "pitch")):
            values.append(pose[key])
    assert max(xs) - min(xs) > 7.0 and max(ys) - min(ys) > 3.0  # 8 m lanes, stepping sideways
    assert max(zs) == pytest.approx(2 * mock_mode.STEP_DEPTH_M, abs=0.05) and min(zs) > -0.05
    assert {round(math.degrees(y)) % 360 for y in yaws} >= {0, 90, 180, 270}  # all four headings
    # The roll and pitch the viewer shows are the mock's bounded oscillation
    assert max(map(abs, rolls)) == pytest.approx(mock_mode.ROLL_WOBBLE[0], abs=0.005)
    assert max(map(abs, pitches)) == pytest.approx(mock_mode.PITCH_WOBBLE[0], abs=0.005)
    # Closed: back where it started (the wobble only leaves a small residue)
    assert math.hypot(xs[-1], ys[-1]) < 0.3 and abs(zs[-1]) < 0.05 and abs(yaws[-1]) < math.radians(1)


# --- VIO process side -----------------------------------------------------------------------------

def test_service_publishes_only_while_leased_at_its_rate(tmp_path):
    (tmp_path / pose_stream.POSE_FILE).write_text("old")
    clock, logs = Clock(), []
    service = pose_stream.PoseStreamService(tmp_path, camera_angle_deg=30.0, clock=clock, log=logs.append,
                                            poll_interval=0.0)
    assert not (tmp_path / pose_stream.POSE_FILE).exists()  # never a previous run's pose
    assert not service.record(clock.t, True, [0.1, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert not (tmp_path / pose_stream.POSE_FILE).exists() and service.frames == 1

    pose_stream.touch_lease(tmp_path)
    os.utime(tmp_path / pose_stream.LEASE_FILE, (clock.t, clock.t))
    written = 0
    for _ in range(20):  # 20 frames over one second: at most 10 writes
        clock.t += 0.05
        written += service.record(clock.t, True, [0.05, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert written == 10 and logs == ["Pose stream started: a viewer is open"]
    pose = json.loads((tmp_path / pose_stream.POSE_FILE).read_text())
    assert pose["frame"] == "start-FRD" and pose["camera_angle_deg"] == 30.0 and pose["rate_hz"] == 10.0
    # The last write was the 20th frame's (the unleased first one, then 19 of 0.05 m)
    assert pose["frames"] == 20 and pose["ok_frames"] == 20 and pose["ok"] is True
    assert pose["x"] == pytest.approx(0.1 + 19 * 0.05, abs=1e-3) and pose["q"] == [1.0, 0.0, 0.0, 0.0]
    trail = json.loads((tmp_path / pose_stream.TRAIL_FILE).read_text())
    assert trail["epoch"] == pose["epoch"] and trail["frame"] == "start-FRD"
    # Written with the first pose, then at most once a second (the viewer extends it from the poses)
    assert trail["points"][0][1:] == [0.0, 0.0, 0.0] and trail["unix_time"] == pytest.approx(1000.05)

    # A rejected frame keeps the pose; the lease runs out: nothing more is written
    clock.t += 0.2
    service.record(clock.t, False, None, None)
    assert json.loads((tmp_path / pose_stream.POSE_FILE).read_text())["ok"] is False
    clock.t += pose_stream.LEASE_S + 1
    assert not service.record(clock.t, True, [1.0, 0.0, 0.0])
    assert logs[-1] == "Pose stream stopped: no viewer"


def test_trail_is_bounded_and_keeps_the_newest_point(tmp_path):
    clock = Clock()
    service = pose_stream.PoseStreamService(tmp_path, clock=clock, trail_points=8, trail_step_m=0.5,
                                            log=lambda m: None)
    for n in range(100):
        clock.t += 0.1
        service.record(clock.t, True, [0.1, 0.0, 0.0])  # 10 m in 0.1 m steps
    assert len(service.trail) <= 8 and service.trail[0][1:] == [0.0, 0.0, 0.0]
    assert service.trail[-1][1] == pytest.approx(service.integrator.position[0], abs=0.5)
    xs = [p[1] for p in service.trail]
    assert xs == sorted(xs)
    # A vehicle holding still adds a point only every TRAIL_STEP_S, and only if it moved at all
    still = pose_stream.PoseStreamService(tmp_path, clock=clock, log=lambda m: None)
    for _ in range(50):
        clock.t += 0.1
        still.record(clock.t, True, [0.0, 0.0, 0.0])
    assert len(still.trail) == 1


def test_trail_is_rewritten_for_the_server_s_freshness_check(tmp_path):
    clock = Clock()
    service = pose_stream.PoseStreamService(tmp_path, clock=clock, poll_interval=0.0, log=lambda m: None)
    pose_stream.touch_lease(tmp_path)
    writes = []
    for _ in range(60):  # 6 s holding still
        clock.t += 0.1
        os.utime(tmp_path / pose_stream.LEASE_FILE, (clock.t, clock.t))
        before = (tmp_path / pose_stream.TRAIL_FILE).stat().st_mtime_ns if (
            tmp_path / pose_stream.TRAIL_FILE).exists() else None
        service.record(clock.t, True, [0.0, 0.0, 0.0])
        if (tmp_path / pose_stream.TRAIL_FILE).stat().st_mtime_ns != before:
            writes.append(clock.t)
    gaps = np.diff(writes)
    assert len(writes) >= 3 and gaps.max() <= pose_stream.TRAIL_REFRESH_S + 0.15


def test_service_survives_an_unwritable_directory(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    logs = []
    service = pose_stream.PoseStreamService(blocker / "sub", clock=Clock(), log=logs.append, poll_interval=0.0)
    service.wanted = True
    service._last_poll = float("inf")
    assert service.record(1001.0, True, [1.0, 0.0, 0.0]) is False
    assert service.errors == 1 and any("Pose stream" in line for line in logs)


# --- main.py side ---------------------------------------------------------------------------------

@pytest.fixture
def publishing_pose():
    """A VIO stand-in: a real PoseStreamService recording 20 frames a second."""
    service = pose_stream.PoseStreamService(vio_telemetry.DEFAULT_DIR, poll_interval=0.01, log=lambda m: None)
    telemetry = vio_telemetry.Telemetry(vio_telemetry.DEFAULT_DIR, fps=20, log=lambda m: None)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            t = time.time()
            telemetry.record_frame(t, True, "ok", 4, 3, 2, 1, 5.0, dt=0.05, position_delta=(0.01, 0, 0), sent=False)
            service.record(t, True, [0.01, 0.0, 0.002], [0.0, 0.0, 0.01])
            telemetry.publish_due()
            time.sleep(0.05)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    yield service
    stop.set()
    thread.join()


def test_pose_stream_without_the_vio_explains_why(client, tmp_path):
    main, http = client
    response = http.get("/api/stream/pose?events=1")
    assert response.status_code == 200 and response.headers["content-type"].startswith("text/event-stream")
    assert response.headers["cache-control"] == "no-store" and response.headers["x-accel-buffering"] == "no"
    ((kind, status),) = sse_events(response.content)
    assert kind == "status" and "PnP" in status["problem"]
    assert (vio_telemetry.DEFAULT_DIR / pose_stream.LEASE_FILE).exists()
    assert http.get("/api/pose").status_code == 503


def test_pose_stream_forwards_the_vio_pose(client, tmp_path, publishing_pose):
    main, http = client
    save_config(tmp_path, algorithm="pnp", log_only=True)
    running(http)
    events = sse_events(http.get("/api/stream/pose?events=6").content)
    kinds = [kind for kind, _ in events]
    assert kinds[0] == "status" and "pose" in kinds and kinds.count("trail") <= 1
    trail = [data for kind, data in events if kind == "trail"]
    pose = [data for kind, data in events if kind == "pose"][-1]
    assert pose["frame"] == "start-FRD" and set(pose) >= {"x", "y", "z", "roll", "pitch", "yaw", "q", "epoch"}
    assert pose["x"] > 0 and pose["z"] > 0 and pose["yaw"] > 0
    if trail:
        assert trail[0]["epoch"] == pose["epoch"] and trail[0]["points"][0][1:] == [0.0, 0.0, 0.0]
    response = http.get("/api/pose")
    assert response.status_code == 200 and response.headers["cache-control"] == "no-store"
    assert response.json()["epoch"] == pose["epoch"]
    assert publishing_pose.published >= 1


def test_pose_endpoints_stay_out_of_the_openapi_schema(client):
    main, http = client
    paths = http.get("/openapi.json").json()["paths"]
    assert "/api/stream/pose" not in paths and "/api/pose" not in paths


# --- the viewer pages -----------------------------------------------------------------------------

def test_pose_widget_page_is_relative_and_self_contained(client):
    main, http = client
    response = http.get("/widget/pose")  # VIO stopped and Basalt selected: still 200 for Cockpit's check
    assert response.status_code == 200 and response.headers["content-type"].startswith("text/html")
    assert response.headers["cache-control"] == "no-cache"
    page = response.text
    assert external_urls(page) == set() and not re.search(r"""['"`]/(api|static)""", page)
    assert not re.search(r"<script[^>]+src=|<link[^>]+href=", page)
    assert "location.pathname.replace(/widget\\/pose\\/?$/, '')" in page
    assert "import(BASE + 'static/pose3d.js')" in page and "mountPoseViewer(root, { base: BASE, fill: true })" in page
    assert 'id="pose-root"' in page and "overflow: hidden" in page


def test_pose_viewer_module_is_relative_and_vendored(client):
    main, http = client
    module = http.get("/static/pose3d.js")
    assert module.status_code == 200 and "javascript" in module.headers["content-type"]
    js = module.text
    assert external_urls(js) == set() and not re.search(r"""['"`]/(api|static)""", js)
    imports = re.findall(r"""^import .* from '([^']+)';""", js, re.M)
    assert imports == ["./vendor/three/three.module.min.js", "./vendor/three/OrbitControls.js"]
    assert "base + 'api/stream/pose?client='" in js and "EventSource" in js and "visibilityState" in js
    for control in ("Reset view", "Clear trail", "Follow"):
        assert control in js
    for field in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert f'data-v="{field}"' in js
    assert "export function mountPoseViewer" in js and "gridLines(" in js and "OrbitControls" in js
    for path in imports:
        vendored = http.get("/static/" + path[2:])
        assert vendored.status_code == 200 and "javascript" in vendored.headers["content-type"]
        # Nothing fetched from elsewhere (three.js only mentions a namespace URI and a forum post in strings)
        assert not re.search(r"""(\bimport\s*\(?\s*|\bfrom\s*)['"]https?:""", vendored.text)


def test_vendored_three_is_pinned_and_licensed():
    vendor = STATIC / "vendor" / "three"
    three = (vendor / "three.module.min.js").read_bytes()
    assert b'"156"' in three  # three.REVISION
    assert (vendor / "LICENSE").read_text().startswith("The MIT License")
    readme = (vendor / "README").read_text()
    assert "r156" in readme and "MIT" in readme and hashlib.sha256(three).hexdigest()[:8] in readme
    orbit = (vendor / "OrbitControls.js").read_text()
    assert "from './three.module.min.js';" in orbit and "from 'three'" not in orbit
    assert not re.search(r"""from ['"]https?:""", orbit)


def test_main_page_has_the_pose_panel_and_loads_it_relatively(client):
    main, http = client
    page = http.get("/").text
    assert 'id="panel-pose"' in page and 'id="pose-root"' in page
    assert 'id="pose-widget-link" href="widget/pose"' in page
    assert "new URL('static/pose3d.js', poseBase)" in page and "IntersectionObserver" in page
    assert external_urls(page) == set()
