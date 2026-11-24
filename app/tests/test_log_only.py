import importlib
import json
import os
import signal
import sys
from unittest import mock

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import vio_log  # noqa: E402
from vio_supervisor import VIOSupervisor  # noqa: E402


def read_lines(path):
    with open(path) as f:
        return [json.loads(line) for line in f]


# --- log-only mode selection ---------------------------------------------------------------

@pytest.mark.parametrize("value, forced", [
    ("1", True), ("true", True), ("YES", True), (" on ", True),
    ("0", False), ("", False), ("false", False), ("no", False),
])
def test_env_var_forces_log_only(value, forced):
    assert vio_log.env_forces_log_only({vio_log.LOG_ONLY_ENV: value}) is forced
    assert vio_log.resolve_log_only(False, {vio_log.LOG_ONLY_ENV: value}) is forced
    assert vio_log.resolve_log_only(True, {vio_log.LOG_ONLY_ENV: value}) is True


def test_env_var_unset_and_no_flag_is_not_log_only():
    assert vio_log.resolve_log_only(False, {}) is False


@pytest.fixture
def no_param_access(monkeypatch):
    """Every MAVLink request through AutopilotParams fails the test: no parameter reads or writes at VIO start."""
    import autopilot_params

    def refuse(*args, **kwargs):
        pytest.fail("the VIO start must not touch autopilot parameters")
    for name in ("_post", "read", "write"):
        monkeypatch.setattr(autopilot_params.AutopilotParams, name, refuse)


def test_log_only_constructs_no_sender_and_touches_no_params(no_param_access):
    sender, messages = mock.Mock(), []
    result = vio_log.setup_autopilot_output(True, "192.168.2.2", sender, log=messages.append)
    assert result is None
    sender.assert_not_called()
    assert messages == [vio_log.LOG_ONLY_BANNER]


def test_normal_mode_connects_the_sender_and_writes_no_params(no_param_access):
    sender = mock.Mock()
    result = vio_log.setup_autopilot_output(False, "192.168.2.2", sender, log=lambda m: None)
    sender.assert_called_once_with("ws://192.168.2.2/mavlink2rest/ws/mavlink?filter=SEND_ONLY")
    sender.return_value.connect.assert_called_once_with()
    assert result is sender.return_value


@pytest.fixture
def pnp_vio(monkeypatch, tmp_path):
    """pnp_vio imported with a fake depthai, logging into tmp_path, and a device that never shows up."""
    monkeypatch.setitem(sys.modules, "depthai", mock.MagicMock())
    monkeypatch.delenv(vio_log.LOG_ONLY_ENV, raising=False)
    sys.modules.pop("pnp_vio", None)
    module = importlib.import_module("pnp_vio")
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path)
    monkeypatch.setattr(module, "open_vio_log", lambda: vio_log.open_vio_log(tmp_path))
    monkeypatch.setattr(module, "MavlinkSender", mock.Mock())

    def no_device(timeout, log):
        log("🔎 Attempt 1: no device discovered")
        raise module.DeviceOpenTimeout("No usable OAK device")
    monkeypatch.setattr(module, "open_oak_device", no_device)
    previous = signal.getsignal(signal.SIGTERM)
    yield module
    signal.signal(signal.SIGTERM, previous)
    sys.modules.pop("pnp_vio", None)


def run_pnp_main(module, argv):
    with pytest.raises(SystemExit) as exit_info:
        module.main(argv)
    assert exit_info.value.code == 2
    (log_file,) = [p for p in vio_log.LOG_DIR.iterdir() if p.name.startswith("vio-")]
    return read_lines(log_file)


@pytest.mark.parametrize("argv, env", [(["--log-only"], None), ([], "1")])
def test_pnp_vio_log_only_never_builds_sender_or_params(pnp_vio, no_param_access, monkeypatch, argv, env):
    if env is not None:
        monkeypatch.setenv(vio_log.LOG_ONLY_ENV, env)
    lines = run_pnp_main(pnp_vio, argv)
    pnp_vio.MavlinkSender.assert_not_called()

    events = [line["event"] for line in lines]
    assert events == ["start", "warning", "device_open", "error", "exit"]
    assert lines[0]["log_only"] is True
    assert lines[0]["log_only_forced_by_env"] is (env is not None)
    assert lines[1]["message"] == vio_log.LOG_ONLY_BANNER
    assert lines[-1]["exit_code"] == 2


def test_pnp_vio_normal_mode_sends_but_never_configures_the_autopilot(pnp_vio, no_param_access):
    lines = run_pnp_main(pnp_vio, ["--vehicle-ip", "10.0.0.1"])
    assert not hasattr(pnp_vio, "AutopilotParams")
    pnp_vio.MavlinkSender.assert_called_once_with("ws://10.0.0.1/mavlink2rest/ws/mavlink?filter=SEND_ONLY")
    pnp_vio.MavlinkSender.return_value.close.assert_called_once_with()
    assert lines[0]["log_only"] is False


# --- JSONL writer ------------------------------------------------------------------------------

def test_jsonl_lines_are_flushed_and_json_safe(tmp_path):
    path = vio_log.create_log_file(tmp_path, now=0, pid=42)
    log = vio_log.JsonlLog(path, clock=lambda: 1700000000.5, monotonic=lambda: 12.25)
    log.step(position_delta=np.array([0.1, -0.2, 0.3]), inliers=np.int64(40),
             reprojection_rms_px=float("nan"), reason=None)
    # Readable before close(): every line is flushed as it is written
    (line,) = read_lines(path)
    assert line == {
        "type": "step",
        "wall_time": "2023-11-14T22:13:20.500000+00:00",
        "unix_time": 1700000000.5,
        "monotonic": 12.25,
        "position_delta": [0.1, -0.2, 0.3],
        "inliers": 40,
        "reprojection_rms_px": None,
        "reason": None,
    }
    log.event("exit", exit_code=0)
    log.close()
    log.event("ignored_after_close")
    lines = read_lines(path)
    assert [entry["type"] for entry in lines] == ["step", "event"]
    assert lines[1]["event"] == "exit"


def test_log_file_names_do_not_collide(tmp_path):
    first = vio_log.create_log_file(tmp_path / "logs", now=0, pid=7)
    second = vio_log.create_log_file(tmp_path / "logs", now=0, pid=7)
    assert first.name.startswith("vio-") and first.name.endswith("-7.jsonl")
    assert second != first and second.name.endswith("-7-1.jsonl")
    assert first.exists() and second.exists()


# --- supervisor events --------------------------------------------------------------------------

class FakeProcess:
    pid = 1234

    def __init__(self):
        self.returncode = None

    def poll(self):
        return self.returncode

    def terminate(self):
        self.returncode = -signal.SIGTERM

    def wait(self, timeout=None):
        return self.returncode


def test_supervisor_events_file(tmp_path):
    path = tmp_path / "logs" / "supervisor.jsonl"
    now = [0.0]
    processes = []

    def launch():
        processes.append(FakeProcess())
        return processes[-1]

    supervisor = VIOSupervisor(launch, clock=lambda: now[0], log=lambda m: None,
                               on_event=lambda event, **fields: vio_log.append_event(path, event, **fields))
    supervisor._ensure_thread = lambda: None
    supervisor.start()
    now[0] = 5.0
    processes[0].returncode = -signal.SIGSEGV
    supervisor.poll()
    now[0] = 7.0
    supervisor.poll()
    supervisor.stop()

    lines = read_lines(path)
    assert [line["event"] for line in lines] == ["launch", "exit", "restart", "launch", "stop"]
    exit_line = lines[1]
    assert exit_line["exit_code"] == -signal.SIGSEGV
    assert exit_line["description"] == "signal SIGSEGV"
    assert exit_line["run_time"] == 5.0
    assert exit_line["restart_count"] == 0
    assert lines[2]["restart_count"] == 1
    assert lines[4]["description"] == "signal SIGTERM"


def test_supervisor_event_errors_do_not_break_supervision():
    messages = []

    def broken(event, **fields):
        raise OSError("disk full")

    supervisor = VIOSupervisor(FakeProcess, clock=lambda: 0.0, log=messages.append, on_event=broken)
    supervisor._ensure_thread = lambda: None
    supervisor.start()
    assert supervisor.status()["running"] is True
    assert any("disk full" in m for m in messages)


# --- web API ------------------------------------------------------------------------------------

@pytest.fixture
def client(monkeypatch, tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    import main
    monkeypatch.setattr(vio_log, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(main, "CONFIG_FILE", tmp_path / "config.json")
    monkeypatch.delenv(vio_log.LOG_ONLY_ENV, raising=False)
    return main, fastapi_testclient.TestClient(main.app)


def test_api_logs_lists_and_downloads(client, tmp_path):
    main, http = client
    assert http.get("/api/logs").json() == []
    path = vio_log.create_log_file(vio_log.LOG_DIR, now=0, pid=1)
    path.write_text('{"type": "event"}\n')
    (vio_log.LOG_DIR / "notes.txt").write_text("not a log")

    (entry,) = http.get("/api/logs").json()
    assert entry["name"] == path.name
    assert entry["size"] == path.stat().st_size
    assert entry["mtime"] == pytest.approx(path.stat().st_mtime)

    response = http.get(f"/api/logs/{path.name}")
    assert response.status_code == 200
    assert response.text == '{"type": "event"}\n'


@pytest.mark.parametrize("name", [
    "..%2Fconfig.json", "..%2F..%2Fetc%2Fpasswd", "%2Fetc%2Fpasswd", "..", ".hidden.jsonl",
    "notes.txt", "missing.jsonl", "a%5Cb.jsonl",
])
def test_api_logs_rejects_traversal_and_unknown_names(client, tmp_path, name):
    main, http = client
    vio_log.LOG_DIR.mkdir(parents=True)
    (tmp_path / "config.json").write_text("{}")
    (vio_log.LOG_DIR / ".hidden.jsonl").write_text("{}")
    (vio_log.LOG_DIR / "notes.txt").write_text("x")
    assert http.get(f"/api/logs/{name}").status_code == 404


def test_resolve_log_file_rejects_symlink_out_of_dir(tmp_path):
    outside = tmp_path / "secret.jsonl"
    outside.write_text("{}")
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "link.jsonl").symlink_to(outside)
    assert vio_log.resolve_log_file("link.jsonl", logs) is None
    assert vio_log.resolve_log_file("../secret.jsonl", logs) is None


def test_log_only_config_and_env_reach_command_and_status(client, monkeypatch):
    main, http = client
    config = main.load_config()
    assert config["log_only"] is False
    assert "--log-only" not in main.build_vio_command(config)
    status = http.get("/api/status").json()
    assert status["log_only"] is False and status["log_only_forced_by_env"] is False

    main.save_config({**config, "log_only": True, "algorithm": "pnp"})
    assert "--log-only" in main.build_vio_command(main.load_config())
    assert http.get("/api/status").json()["log_only"] is True

    # The env var wins over a saved log_only=False, for both algorithms
    monkeypatch.setenv(vio_log.LOG_ONLY_ENV, "1")
    for algorithm in ("pnp", "basalt"):
        config = {**main.DEFAULT_CONFIG, "algorithm": algorithm, "log_only": False}
        assert main.build_vio_command(config)[-1] == "--log-only"
    main.save_config({**main.DEFAULT_CONFIG})
    status = http.get("/api/status").json()
    assert status["log_only"] is True and status["log_only_forced_by_env"] is True


def test_config_api_accepts_log_only(client, monkeypatch):
    main, http = client
    monkeypatch.setattr(main, "start_vio", lambda: None)
    response = http.post("/api/config", json={**main.DEFAULT_CONFIG, "log_only": True})
    assert response.status_code == 200
    assert main.load_config()["log_only"] is True
    page = http.get("/").text
    assert "Log only: do not send anything to the autopilot" in page
    assert 'name="log_only" checked' in page
    assert "'api/logs/'" in page
