"""
Keeps the VIO subprocess alive: if it exits without stop() being called (error, crash or
segfault inside depthai), it is relaunched after an exponential backoff.
"""

import signal
import subprocess
import threading
import time


class RestartBackoff:
    """Doubling restart delay, reset once a run has lasted stable_after seconds."""

    def __init__(self, initial=2.0, maximum=30.0, stable_after=60.0):
        self.initial = initial
        self.maximum = maximum
        self.stable_after = stable_after
        self.current = initial

    def next_delay(self, run_time):
        if run_time >= self.stable_after:
            self.current = self.initial
        delay = self.current
        self.current = min(self.current * 2, self.maximum)
        return delay

    def reset(self):
        self.current = self.initial


def describe_exit(returncode):
    if returncode is not None and returncode < 0:
        try:
            return f"signal {signal.Signals(-returncode).name}"
        except ValueError:
            return f"signal {-returncode}"
    return f"exit code {returncode}"


class VIOSupervisor:
    """
    launch() must return a started subprocess.Popen-like object (poll/terminate/wait/kill/pid).
    The watch thread calls poll() periodically; tests can call it directly with a fake clock.
    on_event(event, **fields), if given, receives launches, exits, restarts and stops (e.g. to
    write them to a log file); its errors are logged and otherwise ignored.
    """

    def __init__(self, launch, backoff=None, poll_interval=0.5, stop_timeout=5.0,
                 clock=time.monotonic, log=print, on_event=None):
        self.launch = launch
        self.backoff = backoff or RestartBackoff()
        self.poll_interval = poll_interval
        self.stop_timeout = stop_timeout
        self.clock = clock
        self.log = log
        self.on_event = on_event

        self.process = None
        self.wanted = False
        self.started_at = None
        self.restart_at = None
        self.restart_count = 0
        self.last_exit_code = None

        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread = None

    def start(self):
        """(Re)start the process on request; clears any pending automatic restart."""
        with self._lock:
            self._stop_process()
            self.backoff.reset()
            self.wanted = True
            self._launch()
        self._ensure_thread()

    def stop(self):
        with self._lock:
            self.wanted = False
            self.restart_at = None
            self._stop_process()

    def shutdown(self):
        self._shutdown.set()
        self.stop()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=self.poll_interval * 4 + 1)

    def status(self):
        with self._lock:
            running = self.process is not None and self.process.poll() is None
            restart_in = None
            if self.restart_at is not None:
                restart_in = max(0.0, self.restart_at - self.clock())
            return {
                "running": running,
                "pid": self.process.pid if running else None,
                "restart_count": self.restart_count,
                "last_exit_code": self.last_exit_code,
                "restart_in": restart_in,
            }

    def poll(self):
        """Detect unexpected exits and perform due restarts. Called by the watch thread."""
        with self._lock:
            if not self.wanted:
                return
            now = self.clock()
            if self.process is not None:
                returncode = self.process.poll()
                if returncode is None:
                    return
                run_time = now - self.started_at
                self.process = None
                self.last_exit_code = returncode
                delay = self.backoff.next_delay(run_time)
                self.restart_at = now + delay
                self.log(f"⚠️ VIO process exited unexpectedly ({describe_exit(returncode)}) after "
                         f"{run_time:.1f}s; restarting in {delay:.1f}s")
                self._emit("exit", exit_code=returncode, description=describe_exit(returncode),
                           run_time=run_time, restart_in=delay, restart_count=self.restart_count)
                return
            if self.restart_at is not None and now >= self.restart_at:
                self.restart_count += 1
                self.log(f"🔁 Restarting VIO (restart #{self.restart_count})")
                self._emit("restart", restart_count=self.restart_count)
                self._launch()

    def _launch(self):
        self.restart_at = None
        try:
            self.process = self.launch()
        except Exception as e:
            # Treat a failed launch like an immediate crash so it is retried with backoff
            self.process = None
            delay = self.backoff.next_delay(0.0)
            self.restart_at = self.clock() + delay
            self.log(f"❌ Failed to launch VIO ({e}); retrying in {delay:.1f}s")
            self._emit("launch_failed", error=str(e), restart_in=delay, restart_count=self.restart_count)
            return
        self.started_at = self.clock()
        self._emit("launch", pid=self.process.pid, restart_count=self.restart_count)

    def _stop_process(self):
        process, self.process = self.process, None
        self.restart_at = None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=self.stop_timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        self.log("Stopped VIO process")
        self._emit("stop", pid=process.pid, exit_code=process.poll(),
                   description=describe_exit(process.poll()), restart_count=self.restart_count)

    def _emit(self, event, **fields):
        if self.on_event is None:
            return
        try:
            self.on_event(event, **fields)
        except Exception as e:
            self.log(f"❌ Could not record supervisor event {event}: {e}")

    def _ensure_thread(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._shutdown.clear()
        self._thread = threading.Thread(target=self._run, name="vio-supervisor", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._shutdown.wait(self.poll_interval):
            try:
                self.poll()
            except Exception as e:
                self.log(f"❌ VIO supervisor error: {e}")
