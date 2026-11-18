"""
Host CPU load for the web page, from /proc/stat deltas (no psutil in the image).
"""

import threading
import time


def read_cpu_times(path="/proc/stat"):
    """(busy, total) jiffies of all CPUs, or None if unavailable (not Linux)."""
    try:
        with open(path) as f:
            fields = f.readline().split()
    except OSError:
        return None
    if not fields or fields[0] != "cpu":
        return None
    values = [int(v) for v in fields[1:]]
    # user nice system idle iowait irq softirq steal (guest time is already in user/nice)
    values = values[:8]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    total = sum(values)
    return total - idle, total


class CpuSampler:
    """percent() of all CPUs busy since the previous sample at least min_interval ago (None until two exist)."""

    def __init__(self, path="/proc/stat", min_interval=0.5, clock=time.monotonic):
        self.path = path
        self.min_interval = min_interval
        self.clock = clock
        self._previous = None
        self._previous_at = None
        self._percent = None
        self._lock = threading.Lock()

    def percent(self):
        with self._lock:
            now = self.clock()
            if self._previous_at is not None and now - self._previous_at < self.min_interval:
                return self._percent
            sample = read_cpu_times(self.path)
            if sample is None:
                return None
            if self._previous is not None:
                busy, total = sample[0] - self._previous[0], sample[1] - self._previous[1]
                if total > 0:
                    self._percent = 100.0 * busy / total
            self._previous, self._previous_at = sample, now
            return self._percent
