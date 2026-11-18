"""
Optional DVL velocity for the web page's vx/vy plot (dashed overlay), read from mavlink2rest by main.py;
the VIO process is never involved.

Which message: VISION_POSITION_DELTA. The Water Linked DVL BlueOS extension feeds the DVL's motion to
ArduSub as VISION_POSITION_DELTA, the same message this extension sends: position_delta (m, body FRD) over
time_delta_usec, so velocity = position_delta / time_delta in the same body frame as our vx/vy, and the
overlay compares exactly the two inputs the EKF would fuse. Our own VISION_POSITION_DELTA comes from
system 1, component 195 (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, MavlinkSender) and is excluded.

Which component: unknown in advance (it depends on the DVL extension's MAVLink ids), so "auto" reads the
whole mavlink2rest message tree (GET <base>/mavlink/vehicles) at most every DISCOVERY_INTERVAL_S and
takes the most recently updated VISION_POSITION_DELTA of any other (system, component). After that only
that one message is polled (GET <base>/mavlink/vehicles/<s>/components/<c>/messages/VISION_POSITION_DELTA,
a few hundred bytes) at most every POLL_INTERVAL_S, with a short timeout, and only while the page asked
for history in the last LEASE_S. A new sample is taken when mavlink2rest's message counter advances.
If the source stops answering it is dropped and discovery starts again.

Unverified on a vehicle: mavlink2rest lists the messages it receives from the MAVLink router, so this
only works if the DVL's messages reach mavlink2rest's endpoint. When nothing is found the page shows
"DVL not found" and plots nothing. Nothing here raises.
"""

import json
import math
import threading
import time
import urllib.request
from collections import deque

MESSAGE = "VISION_POSITION_DELTA"
OWN_COMPONENTS = ((1, 195),)
POLL_INTERVAL_S = 0.5
DISCOVERY_INTERVAL_S = 10.0
TIMEOUT_S = 0.4
LEASE_S = 10.0
# Consecutive failed polls (about 3 s) before the source is dropped and searched for again
MAX_FAILURES = 6
# No new sample for this long: the DVL counts as not updating
STALE_S = 5.0
MAX_SAMPLES = 1200  # 10 min at 2 Hz


def fetch_json(url, timeout=TIMEOUT_S):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read())


def velocity_from_message(content):
    """(vx, vy, vz) in m/s from a VISION_POSITION_DELTA, or None if it carries no usable interval."""
    try:
        dt = float(content["time_delta_usec"]) * 1e-6
        delta = [float(v) for v in content["position_delta"]]
    except (KeyError, TypeError, ValueError):
        return None
    if len(delta) != 3 or not dt > 0 or not all(math.isfinite(v) for v in delta):
        return None
    return delta[0] / dt, delta[1] / dt, delta[2] / dt


def find_sources(tree, exclude=OWN_COMPONENTS):
    """[(system, component, last_update, counter)] with a VISION_POSITION_DELTA in a mavlink2rest vehicles tree."""
    found = []
    if not isinstance(tree, dict):
        return found
    vehicles = tree.get("vehicles", tree)
    for system, vehicle in (vehicles.items() if isinstance(vehicles, dict) else ()):
        components = vehicle.get("components", {}) if isinstance(vehicle, dict) else {}
        for component, data in (components.items() if isinstance(components, dict) else ()):
            try:
                ids = (int(system), int(component))
                entry = data["messages"][MESSAGE]
            except (KeyError, TypeError, ValueError):
                continue
            if ids in exclude:
                continue
            status = entry.get("status", {}).get("time", {}) if isinstance(entry, dict) else {}
            found.append((ids[0], ids[1], str(status.get("last_update", "")), status.get("counter", 0)))
    return sorted(found, key=lambda item: item[2], reverse=True)


class DvlPoller:
    def __init__(self, base_url=None, mode="auto", exclude=OWN_COMPONENTS, fetch=fetch_json, clock=time.time,
                 poll_interval=POLL_INTERVAL_S, discovery_interval=DISCOVERY_INTERVAL_S, lease_s=LEASE_S):
        self.base_url = base_url
        self.mode = mode
        self.exclude = tuple(exclude)
        self.fetch = fetch
        self.clock = clock
        self.poll_interval = poll_interval
        self.discovery_interval = discovery_interval
        self.lease_s = lease_s
        self.source = None
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.last_counter = None
        self.last_discovery = float("-inf")
        self.failures = 0
        self.last_error = None
        self.version = 0  # bumped on every change the page would see (for ETags)
        self._lease_until = float("-inf")
        self._lock = threading.Lock()
        self._thread = None

    def configure(self, base_url, mode):
        with self._lock:
            if (base_url, mode) != (self.base_url, self.mode):
                self.base_url, self.mode = base_url, mode
                self.source, self.last_counter, self.failures = None, None, 0
                self.last_discovery = float("-inf")
                self.samples.clear()
                self.version += 1

    def touch(self, start_thread=True):
        """Keep polling for LEASE_S; starts the polling thread on first use."""
        self._lease_until = self.clock() + self.lease_s
        if start_thread and self.mode != "off" and (self._thread is None or not self._thread.is_alive()):
            self._thread = threading.Thread(target=self._run, name="dvl-poller", daemon=True)
            self._thread.start()

    def _run(self):
        while self.clock() < self._lease_until and self.mode != "off":
            self.poll_once()
            time.sleep(self.poll_interval)

    def poll_once(self, now=None):
        now = self.clock() if now is None else now
        if self.mode == "off" or not self.base_url:
            return
        try:
            if self.source is None:
                if now - self.last_discovery < self.discovery_interval:
                    return
                self.last_discovery = now
                self._discover()
                if self.source is None:
                    return
            self._poll_source(now)
        except Exception as e:  # nothing about the DVL may break the page
            self.last_error = str(e)

    def _discover(self):
        try:
            tree = self.fetch(f"{self.base_url}/mavlink/vehicles")
        except Exception as e:
            self.last_error = f"mavlink2rest not reachable: {e}"
            return
        sources = find_sources(tree, self.exclude)
        if sources:
            system, component, _, _ = sources[0]
            with self._lock:
                self.source = (system, component)
                self.last_counter, self.failures, self.last_error = None, 0, None
                self.version += 1
        else:
            self.last_error = f"no {MESSAGE} from another MAVLink component"

    def _poll_source(self, now):
        system, component = self.source
        try:
            entry = self.fetch(f"{self.base_url}/mavlink/vehicles/{system}/components/{component}/messages/{MESSAGE}")
            counter = entry["status"]["time"]["counter"]
            velocity = velocity_from_message(entry["message"])
        except Exception as e:
            self.failures += 1
            self.last_error = f"{MESSAGE} from {system}:{component} unreadable: {e}"
            if self.failures >= MAX_FAILURES:
                with self._lock:
                    self.source = None
                    self.version += 1
            return
        self.failures = 0
        if counter == self.last_counter:
            return
        first = self.last_counter is None
        self.last_counter = counter
        if velocity is None or first:
            return  # the first read may be an old message; wait for a new one
        with self._lock:
            self.samples.append((now, *velocity))
            self.version += 1

    def snapshot(self, since=None, now=None):
        """What /api/history returns under "dvl"."""
        now = self.clock() if now is None else now
        with self._lock:
            samples = [s for s in self.samples if since is None or s[0] >= since]
            source = self.source
            last = self.samples[-1][0] if self.samples else None
        base = {"mode": self.mode, "message": MESSAGE,
                "source": None if source is None else {"system_id": source[0], "component_id": source[1]},
                "last_sample_age_s": None if last is None else now - last}
        if self.mode == "off":
            return {**base, "available": False, "hint": "DVL overlay off (settings: dvl_source)",
                    "t": [], "vx": [], "vy": [], "vz": []}
        fresh = last is not None and now - last <= STALE_S
        if source is None:
            hint = "DVL not found" + (f" ({self.last_error})" if self.last_error else "")
        elif not fresh:
            hint = f"DVL {source[0]}:{source[1]} not updating"
        else:
            hint = None
        return {**base, "available": bool(source is not None and samples), "hint": hint,
                "t": [s[0] for s in samples], "vx": [s[1] for s in samples], "vy": [s[2] for s in samples],
                "vz": [s[3] for s in samples]}
