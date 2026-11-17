import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from device_open import DeviceOpenTimeout, wait_and_open  # noqa: E402


class FakeTime:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


def run(discover, open_device, fake, **kwargs):
    kwargs.setdefault("settle_delay", 0.0)
    return wait_and_open(discover, open_device, clock=fake.clock, sleep=fake.sleep,
                         log=lambda msg: None, **kwargs)


def test_opens_immediately_when_device_present():
    fake = FakeTime()
    assert run(lambda: ["dev"], lambda devices: ("opened", devices[0]), fake) == ("opened", "dev")
    assert fake.sleeps == []


def test_waits_for_discovery_with_doubling_backoff():
    fake = FakeTime()
    results = iter([[], [], [], [], ["dev"]])
    device = run(lambda: next(results), lambda devices: devices[0], fake)
    assert device == "dev"
    assert fake.sleeps == [0.5, 1.0, 2.0, 4.0]


def test_backoff_is_capped():
    fake = FakeTime()
    results = iter([[]] * 6 + [["dev"]])
    run(lambda: next(results), lambda devices: devices[0], fake, timeout=60.0)
    assert fake.sleeps == [0.5, 1.0, 2.0, 4.0, 5.0, 5.0]


def test_retries_open_errors_and_discovery_exceptions():
    fake = FakeTime()
    discovered = iter([RuntimeError("usb"), ["dev"], ["dev"], ["dev"]])
    opens = {"n": 0}

    def discover():
        value = next(discovered)
        if isinstance(value, Exception):
            raise value
        return value

    def open_device(devices):
        opens["n"] += 1
        if opens["n"] < 3:
            raise RuntimeError("No available devices")
        return devices[0]

    assert run(discover, open_device, fake) == "dev"
    assert opens["n"] == 3
    assert fake.sleeps == [0.5, 1.0, 2.0]


def test_settle_delay_before_open():
    fake = FakeTime()
    run(lambda: ["dev"], lambda devices: devices[0], fake, settle_delay=1.0)
    assert fake.sleeps == [1.0]


def test_gives_up_after_budget():
    fake = FakeTime()
    calls = {"n": 0}

    def discover():
        calls["n"] += 1
        return []

    with pytest.raises(DeviceOpenTimeout, match="no device discovered"):
        run(discover, lambda devices: devices[0], fake, timeout=30.0)
    # 0.5 + 1 + 2 + 4 + 5*4 = 27.5, then a final 2.5 s wait to reach the deadline exactly
    assert fake.sleeps == [0.5, 1.0, 2.0, 4.0, 5.0, 5.0, 5.0, 5.0, 2.5]
    assert fake.now == pytest.approx(30.0)
    assert calls["n"] == 10


def test_gives_up_reports_last_open_error():
    fake = FakeTime()

    def open_device(devices):
        raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")

    with pytest.raises(DeviceOpenTimeout, match="X_LINK_DEVICE_NOT_FOUND"):
        run(lambda: ["dev"], open_device, fake, timeout=5.0)
    assert fake.now == pytest.approx(5.0)
