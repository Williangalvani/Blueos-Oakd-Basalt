"""
Wait for the OAK-D to become available, then open it, retrying with exponential backoff.

A PoE OAK-D that was just released by a previous pipeline can take many seconds to be
discoverable again, and opening it too early has been seen to segfault inside depthai.
The backoff logic and DeviceRestartWatch are depthai-free so they can be unit tested with fakes.
"""

import threading
import time

DEFAULT_TIMEOUT = 30.0
INITIAL_DELAY = 0.5
MAX_DELAY = 5.0
SETTLE_DELAY = 1.0
# DeviceRestartWatch: a frame this many sequence numbers, or this many seconds of device clock, behind the
# previous one means the camera started over
SEQUENCE_RESTART_MARGIN = 30
CLOCK_RESTART_S = 2.0


class DeviceOpenTimeout(RuntimeError):
    pass


def wait_and_open(discover, open_device, timeout=DEFAULT_TIMEOUT, initial_delay=INITIAL_DELAY,
                  max_delay=MAX_DELAY, settle_delay=SETTLE_DELAY,
                  clock=time.monotonic, sleep=time.sleep, log=print):
    """
    Poll discover() until it returns a non-empty list, then return open_device(devices).
    Both discovery misses and open errors are retried with a doubling delay (capped at
    max_delay) until timeout seconds have passed, then DeviceOpenTimeout is raised.
    """
    deadline = clock() + timeout
    delay = initial_delay
    attempt = 0
    last_error = ""

    while True:
        attempt += 1
        try:
            devices = discover()
            last_error = "no device discovered"
        except Exception as e:
            devices = []
            last_error = f"discovery failed: {e}"
        if devices:
            names = ", ".join(str(getattr(d, "name", d)) for d in devices)
            log(f"🔎 Attempt {attempt}: found {len(devices)} device(s): {names}")
            if settle_delay > 0:
                sleep(min(settle_delay, max(0.0, deadline - clock())))
            try:
                device = open_device(devices)
                log(f"✅ Device opened on attempt {attempt}")
                return device
            except Exception as e:
                last_error = f"open failed: {e}"
                log(f"⚠️ Attempt {attempt}: {last_error}")
        else:
            log(f"🔎 Attempt {attempt}: {last_error}")

        remaining = deadline - clock()
        if remaining <= 0:
            raise DeviceOpenTimeout(
                f"No usable OAK device after {attempt} attempts in {timeout:.0f}s ({last_error})")
        wait = min(delay, remaining)
        log(f"⏳ Retrying in {wait:.1f}s ({remaining:.1f}s left)")
        sleep(wait)
        delay = min(delay * 2, max_delay)


class DeviceRestartWatch:
    """
    Notices the OAK restarting under a running pipeline. On an XLink error depthai reconnects by itself
    (it reboots the device with the same pipeline and logs "Reconnection successful"): frames come back,
    but host -> device queues (snapshot requests, depth stream control, feature tracker config) can stay
    stuck, as depthai's XLinkOutHost only waits for the new connection after one of its writes failed. The
    caller restarts the pipeline when this triggers.

    Signals: depthai's reconnection callback (on_reconnection, called on its monitor thread) reporting
    RECONNECTED, or on_frame seeing the frame sequence number or the device clock start over.
    """

    def __init__(self, log=print, sequence_margin=SEQUENCE_RESTART_MARGIN, clock_margin_s=CLOCK_RESTART_S):
        self.log = log
        self.sequence_margin = sequence_margin
        self.clock_margin_s = clock_margin_s
        self._event = threading.Event()
        self.reason = None
        self._last = None

    @property
    def triggered(self):
        return self._event.is_set()

    def trigger(self, reason):
        if not self._event.is_set():
            self.reason = reason
            self._event.set()

    def on_reconnection(self, status):
        name = str(getattr(status, "name", status)).rsplit(".", 1)[-1]
        self.log(f"depthai reconnection: {name}")
        if name == "RECONNECTED":
            self.trigger("depthai reconnected to the camera after a link error")

    def on_frame(self, sequence, device_time_s):
        """True if the camera restarted (this frame is from the new session)."""
        last, self._last = self._last, (sequence, device_time_s)
        if last is not None and (sequence + self.sequence_margin < last[0]
                                 or device_time_s + self.clock_margin_s < last[1]):
            self.trigger(f"the camera restarted: frame sequence {last[0]} -> {sequence}, "
                         f"device clock {last[1]:.1f} -> {device_time_s:.1f} s")
        return self.triggered


def open_oak_device(timeout=DEFAULT_TIMEOUT, log=print):
    """Open the first available OAK device with depthai, waiting up to timeout seconds."""
    import depthai as dai

    return wait_and_open(
        discover=dai.Device.getAllAvailableDevices,
        open_device=lambda devices: dai.Device(devices[0]),
        timeout=timeout,
        log=log,
    )
