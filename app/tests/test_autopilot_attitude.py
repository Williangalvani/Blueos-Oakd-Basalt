import datetime
import json
import math
import os
import sys
import urllib.error

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import autopilot_attitude  # noqa: E402
import mock_mode  # noqa: E402
import pose_stream  # noqa: E402
import vio_telemetry  # noqa: E402
from test_pose_stream import Clock  # noqa: E402
from test_tracking_recovery import pnp_vio  # noqa: E402,F401
from test_web_ui import client, save_config  # noqa: E402,F401

DEG = math.pi / 180.0


def iso(t):
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat().replace("+00:00", "Z")


def entry(roll=0.0, pitch=0.0, yaw=0.0, t=1000.0, counter=1, stamp=True):
    """mavlink2rest's GET .../messages/ATTITUDE answer."""
    status = {"time": {"counter": counter, "first_update": iso(900.0), "frequency": 10.0}}
    if stamp:
        status["time"]["last_update"] = iso(t)
    return {"message": {"type": "ATTITUDE", "time_boot_ms": 123456, "roll": roll, "pitch": pitch, "yaw": yaw,
                        "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}, "status": status}


def sample(roll=0.0, pitch=0.0, yaw=0.0, t=1000.0, counter=1):
    return autopilot_attitude.parse_attitude(entry(roll, pitch, yaw, t, counter), t)


def zyx(roll, pitch, yaw):
    return (pose_stream.rotvec_to_matrix([0, 0, yaw]) @ pose_stream.rotvec_to_matrix([0, pitch, 0])
            @ pose_stream.rotvec_to_matrix([roll, 0, 0]))


# --- parsing --------------------------------------------------------------------------------------

def test_parse_attitude_reads_radians_and_the_mavlink2rest_time():
    s = autopilot_attitude.parse_attitude(entry(0.1, -0.2, 3.0, t=1234.5, counter=7), received_at=99.0)
    assert (s.roll, s.pitch, s.yaw) == pytest.approx((0.1, -0.2, 3.0))
    assert s.unix_time == pytest.approx(1234.5) and s.counter == 7 and s.time_boot_ms == 123456
    # Nanosecond fractions, as mavlink2rest writes them
    nanos = entry()
    nanos["status"]["time"]["last_update"] = "2026-11-23T10:00:00.123456789Z"
    assert autopilot_attitude.parse_attitude(nanos, 0.0).unix_time == pytest.approx(
        datetime.datetime(2026, 11, 23, 10, 0, 0, 123456, datetime.timezone.utc).timestamp())
    # Without last_update: the time it was received
    assert autopilot_attitude.parse_attitude(entry(stamp=False), received_at=55.0).unix_time == 55.0
    # ArduPilot's yaw is -pi..pi; anything a little outside is wrapped
    assert autopilot_attitude.parse_attitude(entry(yaw=1.5 * math.pi), 0.0).yaw == pytest.approx(-0.5 * math.pi)


def test_the_sample_matrix_is_body_frd_to_ned_zyx():
    rng = np.random.default_rng(3)
    for _ in range(20):
        roll, pitch, yaw = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        R = sample(roll, pitch, yaw).matrix()
        assert np.allclose(R, zyx(roll, pitch, yaw), atol=1e-12)
        assert pose_stream.euler_zyx(R) == pytest.approx((roll, pitch, yaw), abs=1e-9)
    # Heading east (yaw +90 deg): body forward is +y (east) in NED
    assert sample(yaw=math.pi / 2).matrix() @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-12)


@pytest.mark.parametrize("bad", [
    None, [], {}, {"message": {"roll": 0.0, "pitch": 0.0}}, {"message": {"type": "ATTITUDE", "roll": "x", "pitch": 0,
                                                                         "yaw": 0}},
    {"message": {"type": "HEARTBEAT", "roll": 0, "pitch": 0, "yaw": 0}},
    entry(roll=float("nan")), entry(yaw=float("inf")), entry(pitch=2.0), entry(roll=4.0)])
def test_parse_attitude_rejects_what_is_not_a_usable_attitude(bad):
    with pytest.raises(ValueError):
        autopilot_attitude.parse_attitude(bad, 0.0)


# --- the poller -----------------------------------------------------------------------------------

class Fetch:
    def __init__(self, *answers):
        self.answers = list(answers)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        answer = self.answers.pop(0) if len(self.answers) > 1 else self.answers[0]
        if isinstance(answer, Exception):
            raise answer
        return answer


def test_source_reports_fresh_then_stale_samples():
    clock, logs = Clock(1000.0), []
    source = autopilot_attitude.AttitudeSource(Fetch(entry(yaw=0.5, t=999.8, counter=3)), clock=clock,
                                               log=logs.append)
    assert source.latest() == (None, None) and source.state()["fresh"] is False
    assert source.poll_once() is True
    s, age = source.latest()
    assert s.yaw == 0.5 and age == pytest.approx(0.2)
    # The same message again (mavlink2rest keeps the last one): no new sample, and it ages
    assert source.poll_once() is False and source.samples == 1
    clock.t += autopilot_attitude.STALE_S
    assert source.latest()[0] is None and source.latest()[1] == pytest.approx(3.2)
    state = source.state()
    assert state["fresh"] is False and state["age_s"] == pytest.approx(3.2) and state["stale_after_s"] == 3.0
    assert logs == ["Autopilot attitude: reading ArduPilot ATTITUDE (mavlink2rest)"]


def test_source_without_a_time_waits_for_the_counter_to_move():
    clock = Clock(1000.0)
    fetch = Fetch(entry(stamp=False, counter=5), entry(stamp=False, counter=5), entry(yaw=0.3, stamp=False, counter=6))
    source = autopilot_attitude.AttitudeSource(fetch, clock=clock, log=lambda m: None)
    assert source.poll_once() is False and source.poll_once() is False  # maybe an old message
    clock.t += 0.5
    assert source.poll_once() is True and source.latest() == (source.sample, 0.0) and source.sample.yaw == 0.3


def test_source_counts_errors_logs_once_and_recovers():
    clock, logs = Clock(1000.0), []
    fetch = Fetch(urllib.error.URLError("refused"), {"message": {}}, urllib.error.URLError("refused"),
                  entry(t=1000.0))
    source = autopilot_attitude.AttitudeSource(fetch, clock=clock, log=logs.append)
    assert [source.poll_once() for _ in range(4)] == [False, False, False, True]
    assert source.errors == 3 and source.last_error is None and source.reads == 4
    assert len(logs) == 2 and "not available" in logs[0] and "gyro-integrated" in logs[0] and "reading" in logs[1]


def test_source_thread_polls_and_stops():
    fetch = Fetch(entry(t=1000.0))
    source = autopilot_attitude.AttitudeSource(fetch, clock=Clock(1000.0), poll_interval=0.01, log=lambda m: None)
    source.start()
    import time
    deadline = time.monotonic() + 2.0
    while fetch.calls < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    source.close()
    assert fetch.calls >= 3 and not source._thread.is_alive()


def test_from_mavlink2rest_reads_the_autopilot_component_read_only(monkeypatch):
    import autopilot_params
    seen = []

    def get(self, path):
        seen.append((self.base_url, path, self.timeout))
        return entry(t=1000.0)

    monkeypatch.setattr(autopilot_params.AutopilotParams, "_get", get)
    monkeypatch.setattr(autopilot_params.AutopilotParams, "_post", lambda self, m: pytest.fail("must not write"))
    source = autopilot_attitude.from_mavlink2rest("192.168.2.2", log=lambda m: None, poll_interval=10.0)
    source.close()
    assert seen[0] == ("http://192.168.2.2/mavlink2rest", "/mavlink/vehicles/1/components/1/messages/ATTITUDE",
                       autopilot_attitude.TIMEOUT_S)
    assert "192.168.2.2" in source.description


# --- the integrator with the autopilot attitude ---------------------------------------------------

def test_position_is_rotated_by_the_autopilot_attitude():
    integrator = pose_stream.PoseIntegrator()
    east = sample(yaw=math.pi / 2, t=10.0)
    pose = integrator.step([1.0, 0.0, 0.0], t=10.0, attitude=east)  # 1 m forward, heading east
    assert integrator.frame == "NED" and integrator.source == "autopilot"
    assert (pose["x"], pose["y"], pose["z"]) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert pose["yaw_deg"] == pytest.approx(90.0) and pose["yaw"] == pytest.approx(math.pi / 2)
    # Pitched 30 deg nose down (pitch -30): forward goes down too
    pose = integrator.step([2.0, 0.0, 0.0], t=10.1, attitude=sample(pitch=-30 * DEG, t=10.1, counter=2))
    assert (pose["x"], pose["y"], pose["z"]) == pytest.approx((2 * math.cos(30 * DEG), 1.0, 2 * math.sin(30 * DEG)))
    assert pose["pitch_deg"] == pytest.approx(-30.0) and pose["roll_deg"] == pytest.approx(0.0, abs=1e-9)
    # The gyro delta of the same frame does not override the autopilot (no bias in the view)
    pose = integrator.step([0.0, 0.0, 0.0], rotation_delta=[0.0, 0.0, 0.2], t=10.2,
                           attitude=sample(yaw=1.0, t=10.2, counter=3))
    assert pose["yaw"] == pytest.approx(1.0)


def test_between_samples_the_gyro_carries_the_attitude_from_the_sample_s_time():
    integrator = pose_stream.PoseIntegrator()
    yaw_rate, dt = 0.5, 0.1
    for n in range(1, 11):
        integrator.step([0.0, 0.0, 0.0], rotation_delta=[0.0, 0.0, yaw_rate * dt], t=100.0 + n * dt)
    # A sample taken at t=100.5 (yaw 0.25 since the autopilot's yaw 0 at 100.0) read at t=101.0
    s = sample(yaw=0.25 + 1.0, t=100.5)  # the autopilot's heading is 1 rad off the start
    pose = integrator.step([0.0, 0.0, 0.0], rotation_delta=[0.0, 0.0, yaw_rate * dt], t=101.1, attitude=s)
    assert pose["yaw"] == pytest.approx(1.0 + 0.55, abs=1e-9)  # the sample plus the 0.3 rad turned since
    pose = integrator.step([1.0, 0.0, 0.0], rotation_delta=[0.0, 0.0, yaw_rate * dt], t=101.2, attitude=s)
    assert pose["yaw"] == pytest.approx(1.0 + 0.6, abs=1e-9)
    assert (pose["x"], pose["y"]) == pytest.approx((math.cos(1.6), math.sin(1.6)))
    # A sample older than the kept history uses the oldest kept rotation (no crash, bounded error)
    old = sample(yaw=0.0, t=50.0, counter=9)
    integrator.step(None, t=101.3, attitude=old)
    assert len(integrator.history) <= pose_stream.CHAIN_HISTORY_S / dt + 2


def test_the_first_autopilot_sample_turns_the_start_frd_pose_and_trail_into_ned(tmp_path):
    clock, logs = Clock(1000.0), []
    current = {"s": None}

    class Source:
        def latest(self, now):
            s = current["s"]
            return (s, now - s.unix_time) if s else (None, None)

        def state(self, now):
            return {"fresh": current["s"] is not None}

        def close(self):
            pass

    service = pose_stream.PoseStreamService(tmp_path, clock=clock, log=logs.append, attitude=Source(),
                                            trail_step_m=0.1)
    epoch = service.epoch
    for _ in range(10):  # 2 m forward without an autopilot yet: start-FRD, gyro
        clock.t += 0.1
        service.record(clock.t, True, [0.2, 0.0, 0.0], [0.0, 0.0, 0.0])
    payload = service.payload(clock.t)
    assert payload["frame"] == "start-FRD" and payload["attitude_source"] == "gyro"
    assert payload["attitude_text"] == "gyro-integrated, drifting" and payload["x"] == pytest.approx(2.0)
    # The autopilot appears: heading 90 deg (east)
    current["s"] = sample(yaw=math.pi / 2, t=clock.t)
    clock.t += 0.1
    service.record(clock.t, True, [0.2, 0.0, 0.0], [0.0, 0.0, 0.0])
    payload = service.payload(clock.t)
    assert payload["frame"] == "NED" and payload["attitude_source"] == "autopilot"
    assert (payload["x"], payload["y"]) == pytest.approx((0.0, 2.2), abs=1e-6)  # the 2 m went east too
    assert all(abs(p[1]) < 1e-3 and p[2] >= 0 for p in service.trail) and service.trail[-1][2] == pytest.approx(2.2)
    assert service.epoch != epoch and service.trail_payload(clock.t)["frame"] == "NED"
    # The 11 points so far were turned; the frame's own point came after
    assert service.realigned == {"unix_time": clock.t, "yaw_deg": 90.0, "trail_points": len(service.trail) - 1}
    assert any("now in NED" in line and "turned by 90.0 deg" in line for line in logs)
    assert service.switches == 1 and service.last_switch["from"] == "gyro"


def test_an_autopilot_from_the_start_keeps_the_trail_epoch(tmp_path):
    clock, logs = Clock(1000.0), []
    source = autopilot_attitude.AttitudeSource(lambda: entry(yaw=1.0, t=999.9), clock=clock, log=lambda m: None)
    source.poll_once()
    service = pose_stream.PoseStreamService(tmp_path, clock=clock, log=logs.append, attitude=source)
    epoch = service.epoch
    service.record(clock.t, False, None, None)  # the first frame has no reference yet
    assert service.integrator.frame == "NED" and service.epoch == epoch and service.realigned["trail_points"] == 0
    assert service.trail == [[1000.0, 0.0, 0.0, 0.0]] and logs[-1].endswith("from the VIO start")


def test_stale_attitude_falls_back_to_the_gyro_without_a_jump_and_back(tmp_path):
    clock, logs = Clock(1000.0), []
    fetch = {"entry": entry(yaw=0.0, t=1000.0, counter=1)}
    source = autopilot_attitude.AttitudeSource(lambda: fetch["entry"], clock=clock, log=logs.append)
    service = pose_stream.PoseStreamService(tmp_path, clock=clock, log=logs.append, attitude=source)
    epoch = service.epoch
    bias = 0.65 * DEG  # rad/s, like the OAK-D-W's gyro at rest
    counter = 1
    sources, yaws, positions = [], [], []
    for n in range(1, 151):  # 15 s at 10 fps, heading north at 1 m/s; ATTITUDE stops from 5 s to 10 s
        clock.t = 1000.0 + n * 0.1
        if not 5.0 <= n * 0.1 < 10.0 and n % 5 == 0:
            counter += 1
            fetch["entry"] = entry(yaw=0.0, t=clock.t, counter=counter)
        source.poll_once()
        service.record(clock.t, True, [0.1, 0.0, 0.0], [0.0, 0.0, bias * 0.1])
        sources.append(service.integrator.source)
        yaws.append(service.integrator.pose()["yaw"])
        positions.append(service.integrator.position.copy())
    # Autopilot until the last sample (4.5 s) is more than 3 s old, gyro until the next sample (10 s)
    assert sources[:75] == ["autopilot"] * 75 and set(sources[76:99]) == {"gyro"} and sources[100:] == ["autopilot"] * 50
    # The autopilot's 0 heading, carried by the gyro since each sample: at most 0.5 s of bias while it comes,
    # up to STALE_S of it while it ages
    assert max(abs(y) for y in yaws[:50]) <= bias * 0.5 + 1e-9 and max(yaws[50:75]) <= bias * 3.0 + 1e-9
    assert max(abs(y) for y in yaws[110:]) <= bias * 0.5 + 1e-9
    gyro = [y for y, s in zip(yaws, sources) if s == "gyro"]
    # Continues from the last (carried) autopilot attitude, drifting at the bias
    assert gyro[0] == pytest.approx(yaws[sources.index("gyro") - 1] + bias * 0.1)
    assert np.diff(gyro) == pytest.approx([bias * 0.1] * (len(gyro) - 1))
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    assert steps.max() == pytest.approx(0.1, abs=1e-9)  # no jump at either switch
    assert service.integrator.frame == "NED" and service.epoch == epoch  # the trail is never redrawn
    status = service.status(clock.t)
    assert status["switches"] == 3 and status["last_switch"]["to"] == "autopilot"
    assert any("gyro-integrated from its last value" in line for line in logs)
    assert any("ATTITUDE back" in line for line in logs)


def test_payload_units_labels_and_status(tmp_path):
    clock = Clock(1000.0)
    source = autopilot_attitude.AttitudeSource(lambda: entry(roll=0.1, pitch=0.2, yaw=-0.3, t=999.5), clock=clock,
                                               log=lambda m: None)
    source.poll_once()
    service = pose_stream.PoseStreamService(tmp_path, clock=clock, log=lambda m: None, attitude=source)
    service.record(clock.t, True, [0.0, 0.0, 0.0], None)
    p = service.payload(clock.t)
    assert (p["roll"], p["pitch"], p["yaw"]) == pytest.approx((0.1, 0.2, -0.3), abs=1e-4)
    assert (p["roll_deg"], p["pitch_deg"], p["yaw_deg"]) == pytest.approx((5.7296, 11.4592, -17.1887), abs=1e-3)
    assert p["units"] == {"x, y, z": "m", "roll, pitch, yaw": "rad", "roll_deg, pitch_deg, yaw_deg": "deg",
                          "attitude_age_s": "s"}
    assert p["attitude_source"] == "autopilot" and p["attitude_age_s"] == 0.5 and p["attitude_mode"] == "autopilot"
    assert p["frame"] == "NED" and "x north, y east, z down" in p["frame_text"]
    assert p["autopilot"]["samples"] == 1 and p["autopilot"]["message"] == "ATTITUDE"
    json.dumps(p)
    # No autopilot configured (pose_attitude_source gyro): labelled as drifting, start-FRD
    gyro = pose_stream.PoseStreamService(tmp_path / "g", clock=clock, log=lambda m: None)
    gyro.record(clock.t, True, [0.0, 0.0, 0.0], [0.0, 0.0, 0.1])
    g = gyro.payload(clock.t)
    assert (g["attitude_mode"], g["attitude_source"], g["attitude_age_s"], g["autopilot"]) == ("gyro", "gyro", None, None)
    assert g["attitude_text"] == "gyro-integrated, drifting" and g["frame"] == "start-FRD"
    assert g["yaw_deg"] == pytest.approx(math.degrees(0.1), abs=1e-3)
    # Lost after NED: the frame text says the heading drifts
    assert "drifts" in pose_stream.frame_text("NED", "gyro") and "drifts" not in pose_stream.frame_text("NED", "autopilot")


def test_telemetry_status_carries_the_pose_attitude(tmp_path):
    telemetry = vio_telemetry.Telemetry(tmp_path, log=lambda m: None)
    assert telemetry.status()["pose"] is None
    telemetry.pose_status = lambda: {"attitude_source": "gyro"}
    assert telemetry.status()["pose"] == {"attitude_source": "gyro"}
    telemetry.pose_status = lambda: 1 / 0
    assert "division" in telemetry.status()["pose"]["error"]


# --- mock mode ------------------------------------------------------------------------------------

def test_mock_attitude_follows_the_survey_with_gaps():
    clock = Clock(5000.0)
    started = clock.t
    fetch = mock_mode.attitude_fetch(started, "gaps", clock=clock)
    clock.t = started + 40.0
    s = autopilot_attitude.parse_attitude(fetch(), clock.t)
    heading = mock_mode.ATTITUDE_HEADING_OFFSET + mock_mode.heading(clock.t) - mock_mode.heading(started)
    assert s.yaw == pytest.approx(math.remainder(heading, math.tau), abs=1e-9)
    assert (s.roll, s.pitch) == pytest.approx(mock_mode.attitude(40.0))
    assert clock.t - s.unix_time < 1.0 / mock_mode.ATTITUDE_RATE_HZ + 1e-9
    # The last 8 s of every 120 s: frozen, so stale after 3 s
    period, gap = mock_mode.ATTITUDE_GAP
    clock.t = started + period - gap + 5.0
    frozen = autopilot_attitude.parse_attitude(fetch(), clock.t)
    assert clock.t - frozen.unix_time == pytest.approx(5.0) and clock.t - frozen.unix_time > autopilot_attitude.STALE_S
    clock.t = started + period + 0.05
    assert clock.t - autopilot_attitude.parse_attitude(fetch(), clock.t).unix_time < 0.2
    # "on" never freezes, "off" has none
    clock.t = started + period - 1.0
    assert clock.t - autopilot_attitude.parse_attitude(
        mock_mode.attitude_fetch(started, "on", clock=clock)(), clock.t).unix_time < 0.2
    with pytest.raises(urllib.error.URLError):
        mock_mode.attitude_fetch(started, "off", clock=clock)()
    assert mock_mode.attitude_mode({}) == "gaps" and mock_mode.attitude_mode({"OAKD_MOCK_ATTITUDE": "Off"}) == "off"
    assert mock_mode.attitude_mode({"OAKD_MOCK_ATTITUDE": "bogus"}) == "gaps"


def test_mock_attitude_matches_the_mock_gyro_deltas():
    """The mock VIO's rotation deltas (without its bias) chained from the start = the mock ATTITUDE minus its offset."""
    import mock_vio
    started, dt = 7000.0, 0.1
    clock = Clock(started)
    fetch = mock_mode.attitude_fetch(started, "on", clock=clock)
    R = np.eye(3)
    for n in range(1, 400):
        t = started + n * dt
        yaw_delta = math.remainder(mock_mode.heading(t) - mock_mode.heading(t - dt), math.tau)
        R = R @ pose_stream.rotvec_to_matrix(mock_vio.body_rotation_delta(t - dt - started, t - started, yaw_delta))
        if n % 50 == 0:
            clock.t = t
            s = autopilot_attitude.parse_attitude(fetch(), t)
            expected = pose_stream.rotvec_to_matrix([0, 0, mock_mode.ATTITUDE_HEADING_OFFSET]) @ R
            assert np.allclose(s.matrix(), expected, atol=1e-6)


# --- configuration --------------------------------------------------------------------------------

def test_pose_attitude_source_reaches_the_vio_command(client, tmp_path):
    import config_schema
    main, http = client
    assert config_schema.POSE_ATTITUDE_SOURCES == pose_stream.ATTITUDE_MODES
    pnp = {**main.DEFAULT_CONFIG}
    assert pnp["pose_attitude_source"] == "autopilot"
    command = main.build_vio_command(pnp)
    assert command[command.index("--pose-attitude-source") + 1] == "autopilot"
    command = main.build_vio_command({**pnp, "pose_attitude_source": "gyro"})
    assert command[command.index("--pose-attitude-source") + 1] == "gyro"
    command = main.build_vio_command({**pnp, "pose_attitude_source": "compass"})  # hand-edited
    assert command[command.index("--pose-attitude-source") + 1] == "autopilot"
    assert http.post("/api/config", json={**pnp, "pose_attitude_source": "compass"}).status_code == 422
    assert http.post("/api/config", json={**pnp, "pose_attitude_source": "gyro"}).status_code == 200
    assert http.get("/api/status").json()["pose"] == {"configured_source": "gyro"}
    fields = {f["name"]: f for f in http.get("/api/config/schema").json()["fields"]}
    assert fields["pose_attitude_source"]["choices"] == ["autopilot", "gyro"]
    import mock_vio
    assert mock_vio.parse_args([]).pose_attitude_source == "autopilot"
    assert mock_vio.parse_args(["--pose-attitude-source", "gyro"]).pose_attitude_source == "gyro"


def test_the_vio_reads_attitude_only_when_configured(pnp_vio, monkeypatch, tmp_path):
    from test_tracking_recovery import run_fake, scenario
    assert pnp_vio.parse_args([]).pose_attitude_source == "autopilot"
    assert pnp_vio.parse_args(["--pose-attitude-source", "gyro"]).pose_attitude_source == "gyro"
    with pytest.raises(SystemExit):
        pnp_vio.parse_args(["--pose-attitude-source", "compass"])
    captured = []
    real = pose_stream.PoseStreamService

    def service(*args, **kwargs):
        captured.append(real(*args, **kwargs))
        return captured[-1]

    monkeypatch.setattr(pnp_vio.pose_stream, "PoseStreamService", service)
    run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_lost=0), ["--vehicle-ip", "10.0.0.9"])
    (source,) = pnp_vio.attitude_sources
    assert captured[-1].attitude is source and source._stop.is_set()  # closed with the pipeline
    status = captured[-1].status()
    assert status["attitude_mode"] == "autopilot" and status["attitude_source"] == "gyro"
    assert status["frame"] == "start-FRD" and status["autopilot"]["fresh"] is False
    for path in (tmp_path / "logs").iterdir():
        path.unlink()
    run_fake(pnp_vio, monkeypatch, tmp_path, scenario(n_lost=0), ["--pose-attitude-source", "gyro"])
    assert len(pnp_vio.attitude_sources) == 1 and captured[-1].attitude is None
