"""
Wait for the OAK-D to become available, then open it, retrying with exponential backoff.

A PoE OAK-D that was just released by a previous pipeline can take many seconds to be
discoverable again, and opening it too early has been seen to segfault inside depthai.
The backoff logic is depthai-free so it can be unit tested with fake callables.
"""

import time

DEFAULT_TIMEOUT = 30.0
INITIAL_DELAY = 0.5
MAX_DELAY = 5.0
SETTLE_DELAY = 1.0


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


def open_oak_device(timeout=DEFAULT_TIMEOUT, log=print):
    """Open the first available OAK device with depthai, waiting up to timeout seconds."""
    import depthai as dai

    return wait_and_open(
        discover=dai.Device.getAllAvailableDevices,
        open_device=lambda devices: dai.Device(devices[0]),
        timeout=timeout,
        log=log,
    )
