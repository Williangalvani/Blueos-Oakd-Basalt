"""
The autopilot parameter check behind the main page's warning banner: the vehicle against the recommended set
(autopilot_params.SUGGESTED_OPTICAL_HOLD_PARAMS), read every CHECK_INTERVAL_S in the background, on demand (the
ArduPilot panel's read) and right after a write. summary() is the compact result for /api/status.

Checks only send PARAM_REQUEST_READ. The one PARAM_SET path is write(): the user-confirmed POST, refused unless
a live HEARTBEAT says disarmed (autopilot_params.write_suggested_params). One lock serialises every autopilot
conversation, so the background check, the panel's read and a write never overlap; the background check skips
its turn while either runs, and backs off (up to BACKOFF_MAX_S) while nothing can be read.

Parameters that need a reboot stay in pending_reboot after a write until the autopilot is seen to have booted
after it (from SYSTEM_TIME, autopilot_params.AutopilotParams.boot_time) or this server restarts. Nothing here
reboots the autopilot.
"""

import threading
import time

import autopilot_params

CHECK_INTERVAL_S = 30.0
BACKOFF_MAX_S = 300.0
# The first background check, after the server starts
FIRST_CHECK_DELAY_S = 2.0


class ParamChecker:
    def __init__(self, make_autopilot, attempts=lambda: 2, interval_s=CHECK_INTERVAL_S, backoff_max_s=BACKOFF_MAX_S,
                 clock=time.time, on_heartbeat=None, log=print):
        self.make_autopilot = make_autopilot
        self.attempts = attempts
        self.interval_s = interval_s
        self.backoff_max_s = backoff_max_s
        self.clock = clock
        self.on_heartbeat = on_heartbeat
        self.log = log
        self._io = threading.Lock()
        self._state = threading.Lock()
        self._result = None
        self._error = "not checked yet"
        self._checked_at = None
        self._mavlink2rest = None
        self._pending = {}  # name -> unix time of the write that needs a reboot
        self._failures = 0
        self._busy = None  # "check" or "write" while one holds the lock
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    # --- autopilot conversations (one at a time) ---

    def read(self):
        """A full read (read_suggested_params' result), waiting for a running check or write; cached."""
        with self._io:
            return self._read_locked(self.make_autopilot())

    def check_now(self):
        """The background check: skipped (False) while a read or a write holds the lock."""
        if not self._io.acquire(blocking=False):
            return False
        try:
            self._read_locked(self.make_autopilot())
        finally:
            self._io.release()
        return True

    def write(self, values, heartbeat_wait):
        """write_suggested_params (raises autopilot_params.WriteRefused), then a fresh check; the write's result."""
        with self._io:
            self._busy = "write"
            try:
                autopilot = self.make_autopilot()
                try:
                    result = autopilot_params.write_suggested_params(autopilot, values, heartbeat_wait=heartbeat_wait,
                                                                     attempts=self.attempts())
                except autopilot_params.WriteRefused as e:
                    self._heartbeat(e.heartbeat)
                    raise
                written_at = self.clock()
                with self._state:
                    for name in result["reboot_required"]:
                        self._pending[name] = written_at
                self._heartbeat(result["heartbeat"])
                self._read_locked(autopilot)
                return result
            finally:
                self._busy = None

    def _read_locked(self, autopilot):
        busy, self._busy = self._busy, self._busy or "check"
        try:
            try:
                result, error = autopilot_params.read_suggested_params(autopilot, attempts=self.attempts()), None
            except Exception as e:
                result, error = None, f"check failed: {e}"
            boot = None
            if result is not None and result["heartbeat"]["reachable"]:
                try:
                    boot = autopilot.boot_time()
                except Exception:
                    boot = None
            with self._state:
                self._result, self._error, self._checked_at = result, error, self.clock()
                self._mavlink2rest = getattr(autopilot, "base_url", None)
                if boot is not None:
                    self._pending = {name: at for name, at in self._pending.items() if boot <= at}
                self._failures = 0 if _readable(result) else self._failures + 1
            if result is not None:
                self._heartbeat(result["heartbeat"])
            return result
        finally:
            self._busy = busy

    def _heartbeat(self, heartbeat):
        if self.on_heartbeat is not None and heartbeat is not None:
            self.on_heartbeat(heartbeat)

    # --- background thread ---

    def next_delay(self):
        """CHECK_INTERVAL_S while the vehicle answers; doubled per failed check up to BACKOFF_MAX_S."""
        with self._state:
            failures = self._failures
        return min(self.interval_s * 2 ** min(failures, 16), max(self.backoff_max_s, self.interval_s))

    def start(self, first_delay=FIRST_CHECK_DELAY_S):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(first_delay,), name="param-checker", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _run(self, delay):
        while not self._stop.is_set():
            self._wake.wait(delay)
            self._wake.clear()
            if self._stop.is_set():
                return
            try:
                self.check_now()
            except Exception as e:
                self.log(f"Parameter check failed: {e}")
            delay = self.next_delay()

    # --- the result ---

    def summary(self, now=None) -> dict:
        """
        checked_at, age_s, reachable (mavlink2rest answered and has an autopilot HEARTBEAT; None before the first
        check), armed, ok (everything read and as recommended, no reboot pending), mismatches [{name, current,
        recommended, needs_reboot}], unreadable [names], pending_reboot [names], error, checking, next_check_s.
        Keep-current entries (no recommended value) never count.
        """
        now = self.clock() if now is None else now
        with self._state:
            result, error, checked_at = self._result, self._error, self._checked_at
            pending = sorted(self._pending, key=lambda name: self._pending[name])
            mavlink2rest = self._mavlink2rest
        heartbeat = (result or {}).get("heartbeat") or {}
        reachable = None if checked_at is None else _readable_heartbeat(heartbeat)
        mismatches, unreadable = [], []
        for param in (result or {}).get("params", ()):
            if param["keep_current"]:
                continue
            if param["status"] == "differs":
                mismatches.append({"name": param["name"], "current": param["current"],
                                   "recommended": param["suggested"], "needs_reboot": param["needs_reboot"]})
            elif param["status"] == "unreadable":
                unreadable.append(param["name"])
        if error is None and reachable is False:
            error = heartbeat.get("detail") or "autopilot not reachable"
        return {
            "checked_at": checked_at, "age_s": None if checked_at is None else max(0.0, now - checked_at),
            "reachable": reachable, "armed": heartbeat.get("armed"),
            "ok": bool(reachable and result is not None and not mismatches and not unreadable and not pending),
            "mismatches": mismatches, "unreadable": unreadable, "pending_reboot": pending, "error": error,
            "checking": self._busy is not None, "interval_s": self.interval_s, "mavlink2rest": mavlink2rest,
        }


def _readable_heartbeat(heartbeat):
    return bool(heartbeat.get("reachable") and heartbeat.get("counter") is not None)


def _readable(result):
    """Whether a check reached the autopilot: a HEARTBEAT and at least one parameter read."""
    if result is None or not _readable_heartbeat(result["heartbeat"]):
        return False
    return any(param["status"] != "unreadable" for param in result["params"])
