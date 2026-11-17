"""
Tracking-loss recovery for the PnP VIO, without restarting the depthai pipeline.

After `frames` consecutive failed frames the caller resets the host PnP reference and re-initialises
the on-device FeatureTracker by sending it a fresh FeatureTrackerConfig, first with the feature
maintainer off for one frame (drops every tracked feature and forces a full re-detection), then the
normal config. If tracking still fails, the next recovery waits twice as long, up to
`max_backoff` x `frames`; any successful frame resets both the count and the backoff.
depthai-free so it can be unit tested.
"""


class TrackingRecovery:
    def __init__(self, frames=20, max_backoff=16):
        self.frames = frames
        self.max_backoff = max_backoff
        self.failures = 0
        self.backoff = 1
        self.count = 0

    @property
    def enabled(self):
        return self.frames > 0

    @property
    def threshold(self):
        """Consecutive failures that trigger the next recovery."""
        return self.frames * self.backoff

    def update(self, ok):
        """Feed one frame's result; returns True when the caller should recover now."""
        if ok:
            self.failures = 0
            self.backoff = 1
            return False
        if not self.enabled:
            return False
        self.failures += 1
        if self.failures < self.threshold:
            return False
        self.failures = 0
        self.count += 1
        self.backoff = min(self.backoff * 2, self.max_backoff)
        return True


class FeatureTrackerReinit:
    """
    Two-step FeatureTracker re-initialisation. send(maintainer) must send a full FeatureTrackerConfig
    with the feature maintainer on or off. Call step() once per frame before start(), so the normal
    config goes out one frame after the maintainer-off one (the device may take one config per frame).
    """

    def __init__(self, send):
        self.send = send
        self.pending_restore = False

    def start(self):
        self.send(False)
        self.pending_restore = True

    def step(self):
        if self.pending_restore:
            self.pending_restore = False
            self.send(True)


def feature_id_counts(previous_ids, ids):
    """(new, tracked): ids not in the previous frame, and ids carried over from it."""
    tracked = sum(1 for i in ids if i in previous_ids)
    return len(ids) - tracked, tracked
