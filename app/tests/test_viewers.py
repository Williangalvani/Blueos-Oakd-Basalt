"""viewers.ViewerRegistry: the reference count of stream clients, the linger, and the leases it keeps."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import viewers  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def registry(linger_s=4.0):
    clock = Clock()
    calls = []
    leases = {kind: (lambda k=kind: calls.append(("touch", k)), lambda k=kind: calls.append(("remove", k)))
              for kind in ("video", "features", "pose")}
    return viewers.ViewerRegistry(leases, linger_s=linger_s, refresh_s=1.0, clock=clock, thread=False), clock, calls


def test_a_viewer_takes_the_lease_at_once_and_renews_it_every_refresh():
    reg, clock, calls = registry()
    token = reg.open("video", "widget")
    assert calls == [("touch", "video")] and reg.count("video") == 1
    clock.t += 0.5
    reg.tick()
    assert calls == [("touch", "video")]
    clock.t += 0.6
    reg.tick()
    assert calls == [("touch", "video")] * 2
    reg.close(token)
    assert reg.count() == 0


def test_the_lease_outlives_the_last_viewer_by_the_linger_then_is_removed():
    reg, clock, calls = registry(linger_s=4.0)
    a = reg.open("video")
    b = reg.open("video", "widget")
    reg.close(a)
    assert reg.status()["video"] == 1
    reg.close(b)
    status = reg.status()
    assert status["video"] == 0 and status["video_idle"] is False and status["lingering_s"] == {"video": 4.0}
    assert status["text"] == "video stopping (no viewer)"
    for _ in range(3):
        clock.t += 1.0
        reg.tick()
    # One touch per open, then one per refresh while lingering
    assert ("remove", "video") not in calls and calls.count(("touch", "video")) == 2 + 3
    clock.t += 1.1
    reg.tick()
    assert calls[-1] == ("remove", "video")
    assert reg.status()["video_idle"] is True and reg.status()["text"] == "video idle"
    # Nothing more once idle
    count = len(calls)
    clock.t += 10
    reg.tick()
    assert len(calls) == count


def test_a_reconnect_within_the_linger_keeps_the_stream_going():
    reg, clock, calls = registry()
    reg.close(reg.open("video"))
    clock.t += 2.0
    token = reg.open("video")
    clock.t += 3.0
    reg.tick()
    assert ("remove", "video") not in calls and reg.status()["lingering_s"] == {}
    reg.close(token)


def test_kinds_are_counted_separately_and_only_video_counts_as_video():
    reg, clock, calls = registry(linger_s=0.0)
    features = reg.open("features", "widget")
    pose = reg.open("pose", "page")
    status = reg.status()
    assert status["total"] == 2 and status["video"] == 0 and status["video_idle"] is True
    assert status["by_kind"] == {"video": 0, "features": 1, "pose": 1}
    assert status["by_client"] == {"widget": 1, "page": 1}
    assert status["text"] == "video idle (2 viewers without video)"
    video = reg.open("video", "something-else")
    status = reg.status()
    assert status["text"] == "1 watching video, 2 other" and status["by_client"]["other"] == 1
    for token in (features, pose, video):
        reg.close(token)
    assert [c for c in calls if c[0] == "remove"] == [("remove", "features"), ("remove", "pose"), ("remove", "video")]
    assert reg.status()["opened_total"] == {"features": 1, "pose": 1, "video": 1}


def test_unknown_kinds_and_double_closes():
    reg, clock, calls = registry()
    with pytest.raises(ValueError):
        reg.open("audio")
    token = reg.open("pose")
    reg.close(token)
    reg.close(token)
    assert reg.count() == 0


def test_a_failing_lease_is_logged_not_raised():
    logs = []

    def broken():
        raise OSError("read-only")

    reg = viewers.ViewerRegistry({"video": (broken, None)}, clock=Clock(), thread=False, log=logs.append)
    reg.open("video")
    assert logs and "read-only" in logs[0]
