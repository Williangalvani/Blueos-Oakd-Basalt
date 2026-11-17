"""
Log-only mode and the JSONL data log.

Log-only mode runs VIO as a passive logger: no autopilot parameter reads/writes and no MAVLink
sender at all, so nothing reaches mavlink2rest. It is on if any of the --log-only flag, the saved
log_only setting (passed as --log-only by main.py) or OAKD_LOG_ONLY=1 in the environment ask for it.

Every VIO process start writes one JSONL file under LOG_DIR; each line is flushed as it is written
so a crash (even a segfault inside depthai) loses at most the line being written.
This module is depthai-free so it can be unit tested.
"""

import datetime
import json
import math
import os
import re
import time
from pathlib import Path

LOG_DIR = Path("/app/data/logs")
LOG_ONLY_ENV = "OAKD_LOG_ONLY"
LOG_ONLY_BANNER = "LOG-ONLY: no MAVLink output, autopilot params untouched"

TRUE_VALUES = {"1", "true", "yes", "on"}
LOG_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*\.jsonl$")


def env_forces_log_only(environ=None):
    environ = os.environ if environ is None else environ
    return environ.get(LOG_ONLY_ENV, "").strip().lower() in TRUE_VALUES


def resolve_log_only(cli_flag=False, environ=None):
    return bool(cli_flag) or env_forces_log_only(environ)


def setup_autopilot_output(log_only, vehicle_ip, autopilot_params, mavlink_sender, log=print):
    """
    Configure the autopilot and return a connected-or-retrying MAVLink sender, or None in
    log-only mode, where neither factory is called.
    """
    if log_only:
        log(LOG_ONLY_BANNER)
        return None
    autopilot_params(vehicle_ip).configure_visual_odometry()
    sender = mavlink_sender(f"ws://{vehicle_ip}/mavlink2rest/ws/mavlink?filter=SEND_ONLY")
    if not sender.connect():
        log("MAVLink websocket not connected yet; will keep retrying")
    return sender


def _clean(value):
    """Make numpy values and non-finite floats JSON-safe (NaN/inf become null)."""
    if hasattr(value, "tolist"):  # numpy arrays and scalars
        value = value.tolist()
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if value is None or isinstance(value, (bool, int, str)):
        return value
    return str(value)


class JsonlLog:
    """Append-only JSONL writer; every line gets wall (ISO + unix) and monotonic times."""

    def __init__(self, path, clock=time.time, monotonic=time.monotonic):
        self.path = Path(path)
        self.clock = clock
        self.monotonic = monotonic
        self._file = open(self.path, "a", encoding="utf-8")

    def write(self, kind, **fields):
        if self._file is None:
            return
        now = self.clock()
        record = {
            "type": kind,
            "wall_time": datetime.datetime.fromtimestamp(now, datetime.timezone.utc).isoformat(),
            "unix_time": now,
            "monotonic": self.monotonic(),
            **fields,
        }
        self._file.write(json.dumps(_clean(record)) + "\n")
        self._file.flush()

    def event(self, event, **fields):
        self.write("event", event=event, **fields)

    def step(self, **fields):
        self.write("step", **fields)

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None


def create_log_file(log_dir=LOG_DIR, prefix="vio", now=None, pid=None):
    """Create a new, empty vio-YYYYmmdd-HHMMSS-pid.jsonl (suffixed if that name is taken)."""
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    base = f"{prefix}-{stamp}-{os.getpid() if pid is None else pid}"
    for n in range(1000):
        path = log_dir / (f"{base}.jsonl" if n == 0 else f"{base}-{n}.jsonl")
        try:
            with open(path, "x"):
                return path
        except FileExistsError:
            continue
    raise RuntimeError(f"Could not create a unique log file for {base} in {log_dir}")


def open_vio_log(log_dir=LOG_DIR, prefix="vio"):
    return JsonlLog(create_log_file(log_dir, prefix))


def append_event(path, event, **fields):
    """Append one event line to a long-lived log such as supervisor.jsonl."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    log = JsonlLog(path)
    try:
        log.event(event, **fields)
    finally:
        log.close()


def list_logs(log_dir=LOG_DIR):
    log_dir = Path(log_dir)
    if not log_dir.is_dir():
        return []
    logs = []
    for path in log_dir.iterdir():
        if path.is_file() and LOG_NAME_RE.match(path.name):
            stat = path.stat()
            logs.append({"name": path.name, "size": stat.st_size, "mtime": stat.st_mtime})
    return sorted(logs, key=lambda entry: entry["mtime"], reverse=True)


def resolve_log_file(name, log_dir=LOG_DIR):
    """Path of an existing log called name directly inside log_dir, or None (rejects traversal)."""
    if not LOG_NAME_RE.match(name):
        return None
    log_dir = Path(log_dir).resolve()
    path = (log_dir / name).resolve()
    if path.parent != log_dir or not path.is_file():
        return None
    return path
