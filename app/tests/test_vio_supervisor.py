import os
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vio_supervisor import RestartBackoff, VIOSupervisor, describe_exit  # noqa: E402


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeProcess:
    next_pid = 100

    def __init__(self):
        FakeProcess.next_pid += 1
        self.pid = FakeProcess.next_pid
        self.returncode = None
        self.terminated = False

    def poll(self):
        return self.returncode

    def exit(self, code):
        self.returncode = code

    def terminate(self):
        self.terminated = True
        self.returncode = -signal.SIGTERM

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -signal.SIGKILL


def make_supervisor(**kwargs):
    clock = FakeClock()
    launched = []

    def launch():
        process = FakeProcess()
        launched.append(process)
        return process

    supervisor = VIOSupervisor(launch, clock=clock, log=lambda msg: None, **kwargs)
    # Drive poll() by hand; no watch thread in these tests
    supervisor._ensure_thread = lambda: None
    return supervisor, clock, launched


def test_backoff_doubles_caps_and_resets_after_stable_run():
    backoff = RestartBackoff(initial=2.0, maximum=30.0, stable_after=60.0)
    assert [backoff.next_delay(1.0) for _ in range(6)] == [2.0, 4.0, 8.0, 16.0, 30.0, 30.0]
    assert backoff.next_delay(60.0) == 2.0
    assert backoff.next_delay(5.0) == 4.0


def test_restarts_after_crash_with_backoff():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    assert len(launched) == 1

    clock.now = 1.0
    launched[0].exit(-signal.SIGSEGV)
    supervisor.poll()
    status = supervisor.status()
    assert status["running"] is False
    assert status["last_exit_code"] == -signal.SIGSEGV
    assert status["restart_in"] == pytest.approx(2.0)

    clock.now = 2.9
    supervisor.poll()
    assert len(launched) == 1

    clock.now = 3.0
    supervisor.poll()
    assert len(launched) == 2
    status = supervisor.status()
    assert status == {"running": True, "pid": launched[1].pid, "restart_count": 1,
                      "last_exit_code": -signal.SIGSEGV, "restart_in": None}

    # Second quick crash: delay doubles
    clock.now = 4.0
    launched[1].exit(1)
    supervisor.poll()
    assert supervisor.status()["restart_in"] == pytest.approx(4.0)
    clock.now = 8.0
    supervisor.poll()
    assert len(launched) == 3
    assert supervisor.restart_count == 2
    assert supervisor.last_exit_code == 1


def test_backoff_resets_after_stable_run():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    for i in range(3):
        launched[-1].exit(1)
        supervisor.poll()
        clock.now = supervisor.restart_at
        supervisor.poll()
    assert supervisor.backoff.current == 16.0

    clock.now += 61.0
    launched[-1].exit(-signal.SIGSEGV)
    supervisor.poll()
    assert supervisor.status()["restart_in"] == pytest.approx(2.0)


def test_clean_exit_is_also_restarted():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    launched[0].exit(0)
    supervisor.poll()
    clock.now = 2.0
    supervisor.poll()
    assert len(launched) == 2


def test_no_restart_after_requested_stop():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    supervisor.stop()
    assert launched[0].terminated
    for t in (1.0, 10.0, 100.0):
        clock.now = t
        supervisor.poll()
    assert len(launched) == 1
    assert supervisor.status()["running"] is False
    assert supervisor.status()["restart_in"] is None
    assert supervisor.restart_count == 0
    assert supervisor.last_exit_code is None


def test_stop_cancels_pending_restart():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    launched[0].exit(-signal.SIGSEGV)
    supervisor.poll()
    supervisor.stop()
    clock.now = 100.0
    supervisor.poll()
    assert len(launched) == 1


def test_config_restart_does_not_count_or_loop():
    supervisor, clock, launched = make_supervisor()
    supervisor.start()
    supervisor.start()  # config change: terminate and relaunch
    assert launched[0].terminated
    assert len(launched) == 2
    clock.now = 100.0
    supervisor.poll()
    assert len(launched) == 2
    assert supervisor.restart_count == 0
    assert supervisor.status()["pid"] == launched[1].pid


def test_failed_launch_is_retried():
    clock = FakeClock()
    attempts = []

    def launch():
        attempts.append(clock.now)
        if len(attempts) == 1:
            raise OSError("no python")
        return FakeProcess()

    supervisor = VIOSupervisor(launch, clock=clock, log=lambda msg: None)
    supervisor._ensure_thread = lambda: None
    supervisor.start()
    assert supervisor.status()["running"] is False
    clock.now = 2.0
    supervisor.poll()
    assert attempts == [0.0, 2.0]
    assert supervisor.status()["running"] is True


def test_describe_exit():
    assert describe_exit(-signal.SIGSEGV) == "signal SIGSEGV"
    assert describe_exit(2) == "exit code 2"


def wait_for(condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


@pytest.mark.parametrize("script, expected", [
    ("import os, signal; os.kill(os.getpid(), signal.SIGSEGV)", -signal.SIGSEGV),
    ("import sys; sys.exit(3)", 3),
])
def test_real_subprocess_crash_is_restarted(script, expected):
    launched = []

    def launch():
        process = subprocess.Popen([sys.executable, "-c", script])
        launched.append(process)
        return process

    supervisor = VIOSupervisor(launch, backoff=RestartBackoff(initial=0.05, maximum=0.2),
                               poll_interval=0.02, log=lambda msg: None)
    try:
        supervisor.start()
        assert wait_for(lambda: supervisor.restart_count >= 2)
        assert supervisor.last_exit_code == expected
    finally:
        supervisor.shutdown()
    assert not supervisor._thread.is_alive()
    count = len(launched)
    time.sleep(0.3)
    assert len(launched) == count
    assert all(p.poll() is not None for p in launched)


def test_real_subprocess_stop_is_not_restarted():
    launched = []

    def launch():
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        launched.append(process)
        return process

    supervisor = VIOSupervisor(launch, backoff=RestartBackoff(initial=0.05, maximum=0.2),
                               poll_interval=0.02, log=lambda msg: None)
    try:
        supervisor.start()
        assert wait_for(lambda: supervisor.status()["running"])
        supervisor.stop()
        time.sleep(0.3)
        assert len(launched) == 1
        assert launched[0].poll() == -signal.SIGTERM
        assert supervisor.restart_count == 0
        assert supervisor._thread.is_alive()  # still watching, just idle
    finally:
        supervisor.shutdown()
    assert not supervisor._thread.is_alive()
