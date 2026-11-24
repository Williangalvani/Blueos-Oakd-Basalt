"""
Who is watching (main.py side): a reference count of the live clients per kind, with a linger.

Each MJPEG or SSE client (/api/stream/depth, /api/stream/features, /api/stream/pose; the page's panels and the
Cockpit widgets alike) opens a viewer while it is connected and closes it when it goes. While a kind has a
viewer, or for LINGER_S after its last one left (a page reload or a widget re-render reconnects within that),
its lease files are touched every REFRESH_S; that is what the VIO process polls (depth_stream.LEASE_FILE,
feature_stream.LEASE_FILE, pose_stream.LEASE_FILE, each fresh for 4 s) to send the camera images, the
features and the pose. When the linger ends, the kind's own lease is removed, so the VIO process stops
within its next poll (0.5 s) instead of waiting for the lease to go stale; if main.py dies the leases go
stale by themselves.

A daemon thread runs tick() every TICK_S once the first viewer opened, so the linger ends with nobody
connected. status() is what /api/status and the widget status show (viewers, "video idle").
"""

import itertools
import threading
import time
from collections import Counter

LINGER_S = 4.0
REFRESH_S = 1.0
TICK_S = 0.5
# Kinds that make the camera send images; the others (features, pose) are small and image-free
VIDEO_KINDS = frozenset({"video"})
CLIENTS = ("page", "widget", "other")


class ViewerRegistry:
    """
    leases: {kind: (touch(), remove() or None)}. touch() renews the lease the VIO process reads; remove() drops
    the kind's own lease when the linger ends (None: leave it to go stale, e.g. a lease other requests share).
    """

    def __init__(self, leases, linger_s=LINGER_S, refresh_s=REFRESH_S, clock=time.monotonic, tick_s=TICK_S,
                 thread=True, log=None):
        self.leases = dict(leases)
        self.linger_s = float(linger_s)
        self.refresh_s = float(refresh_s)
        self.clock = clock
        self.tick_s = tick_s
        self.thread = thread
        self.log = log
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._open = {}  # token -> (kind, client, opened at)
        self._left = {}  # kind -> when its last viewer left
        self._touched = {}  # kind -> last touch
        self._active = set()
        self._opened_total = Counter()
        self._thread = None

    def open(self, kind, client="page"):
        if kind not in self.leases:
            raise ValueError(f"unknown viewer kind {kind!r}")
        client = client if client in CLIENTS else "other"
        now = self.clock()
        with self._lock:
            token = next(self._ids)
            self._open[token] = (kind, client, now)
            self._left.pop(kind, None)
            self._opened_total[kind] += 1
            start = self.thread and self._thread is None
            if start:
                self._thread = threading.Thread(target=self._run, name="viewers", daemon=True)
        if start:
            self._thread.start()
        self.tick(now, force=kind)
        return token

    def close(self, token):
        now = self.clock()
        with self._lock:
            entry = self._open.pop(token, None)
            if entry is not None and not any(kind == entry[0] for kind, _, _ in self._open.values()):
                self._left[entry[0]] = now
        self.tick(now)

    def count(self, kind=None):
        with self._lock:
            return sum(1 for k, _, _ in self._open.values() if kind is None or k == kind)

    def _is_active(self, kind, now):
        if any(k == kind for k, _, _ in self._open.values()):
            return True
        left = self._left.get(kind)
        return left is not None and now - left < self.linger_s

    def tick(self, now=None, force=None):
        """Renew the leases of the active kinds (at most every refresh_s; `force` at once), remove ended ones."""
        now = self.clock() if now is None else now
        touch, remove = [], []
        with self._lock:
            for kind in self.leases:
                active = self._is_active(kind, now)
                if active:
                    if kind == force or now - self._touched.get(kind, float("-inf")) >= self.refresh_s:
                        self._touched[kind] = now
                        touch.append(kind)
                    self._active.add(kind)
                elif kind in self._active:
                    self._active.discard(kind)
                    self._left.pop(kind, None)
                    self._touched.pop(kind, None)
                    remove.append(kind)
        for kind in touch:
            self._call(self.leases[kind][0], kind)
        for kind in remove:
            if self.leases[kind][1] is not None:
                self._call(self.leases[kind][1], kind)

    def _call(self, function, kind):
        try:
            function()
        except Exception as e:
            if self.log is not None:
                self.log(f"Viewer lease for {kind} failed: {e}")

    def _run(self):
        while True:
            time.sleep(self.tick_s)
            self.tick()

    def status(self, now=None):
        now = self.clock() if now is None else now
        with self._lock:
            by_kind = Counter(kind for kind, _, _ in self._open.values())
            by_client = Counter(client for _, client, _ in self._open.values())
            lingering = {kind: round(self.linger_s - (now - left), 1) for kind, left in self._left.items()
                         if now - left < self.linger_s and not by_kind.get(kind)}
            opened = dict(self._opened_total)
        video = sum(by_kind.get(kind, 0) for kind in VIDEO_KINDS)
        video_lingering = any(kind in lingering for kind in VIDEO_KINDS)
        total = sum(by_kind.values())
        if video:
            text = f"{video} watching video" + (f", {total - video} other" if total > video else "")
        elif video_lingering:
            text = "video stopping (no viewer)"
        else:
            text = "video idle" + (f" ({total} viewer{'s' if total != 1 else ''} without video)" if total else "")
        return {"total": total, "by_kind": {kind: by_kind.get(kind, 0) for kind in self.leases},
                "by_client": dict(by_client), "video": video, "video_idle": not video and not video_lingering,
                "lingering_s": lingering, "linger_s": self.linger_s, "opened_total": opened, "text": text}
