#!/usr/bin/env python3
"""
A fake mavlink2rest for tests: an autopilot with parameters, a HEARTBEAT (armed
or not) and a SYSTEM_TIME (reboot() restarts its time_boot_ms), our own VISION_POSITION_DELTA (system 1, component 195) and optionally a DVL's
VISION_POSITION_DELTA, in mavlink2rest's REST shapes:

  GET  /mavlink2rest/mavlink/vehicles                                    the message tree
  GET  /mavlink2rest/mavlink/vehicles/<s>/components/<c>/messages/<NAME> one message (404 if unknown)
  POST /mavlink2rest/mavlink                                             PARAM_REQUEST_READ / PARAM_SET

Every posted message is kept in FakeVehicle.posted. Two ways to use it, neither touches a real host:
  - in-process: patch urllib.request.urlopen with vehicle.urlopen (only http://<vehicle.host>/... answers)
  - HTTP on 127.0.0.1: serve(vehicle); `python fake_mavlink2rest.py --port 8088`
"""

import argparse
import datetime
import io
import json
import math
import threading
import time
import urllib.error
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PREFIX = "/mavlink2rest"
AUTOPILOT = (1, 1)
OWN_VIO = (1, 195)
DVL = (1, 197)
ARMED_FLAG = 128

DEFAULT_PARAMS = {
    "VISO_TYPE": 0, "VISO_POS_X": 0.0, "VISO_POS_Y": 0.0, "VISO_POS_Z": 0.0, "VISO_ORIENT": 0,
    "VISO_DELAY_MS": 10, "EK3_SRC1_VELXY": 0, "EK3_SRC1_POSXY": 6, "EK3_SRC1_POSZ": 1, "EK3_SRC1_VELZ": 0,
    "EK3_SRC1_YAW": 1, "AHRS_EKF_TYPE": 3, "EK3_ENABLE": 1,
}
FLOAT_PARAMS = {"VISO_POS_X", "VISO_POS_Y", "VISO_POS_Z"}


def iso(t):
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat().replace("+00:00", "Z")


def float32(value):
    # Parameters travel as float32, like on the autopilot: 0.2 reads back as 0.20000000298023224
    import struct
    return struct.unpack("f", struct.pack("f", float(value)))[0]


class FakeResponse(io.BytesIO):
    status = 200

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class FakeVehicle:
    """
    armed: the HEARTBEAT's armed bit (change it any time). heartbeat=False: no HEARTBEAT at all.
    heartbeat_hz: how fast its counter advances (in clock time). dvl_velocity(t) -> (vx, vy, vz) or None for
    no DVL. reachable=False: every request fails like an unreachable host. A HEARTBEAT is refreshed when read.
    """

    def __init__(self, host="fake-vehicle", params=None, armed=False, heartbeat=True, heartbeat_hz=1.0,
                 dvl_velocity=None, dvl_hz=5.0, dvl_ids=DVL, clock=time.time, reachable=True, ignore_param_set=()):
        self.host = host
        self.params = {name: float32(value) for name, value in (DEFAULT_PARAMS if params is None else params).items()}
        self.armed = armed
        self.heartbeat = heartbeat
        self.heartbeat_hz = heartbeat_hz
        self.dvl_velocity = dvl_velocity
        self.dvl_hz = dvl_hz
        self.dvl_ids = tuple(dvl_ids)
        self.clock = clock
        self.reachable = reachable
        self.ignore_param_set = set(ignore_param_set)
        self.started = clock()
        self.booted = self.started - 600.0  # the autopilot's boot (SYSTEM_TIME.time_boot_ms); reboot() resets it
        self.posted = []
        self.requests = []
        self.param_value = None
        self.param_value_counter = 0
        self.param_value_time = None
        self.lock = threading.Lock()

    # --- messages ---

    def _entry(self, message, counter, last_update, hz):
        first = self.started
        return {"message": message, "status": {"time": {"counter": counter, "first_update": iso(first),
                                                       "frequency": hz, "last_update": iso(last_update)}}}

    def _ticks(self, hz):
        now = self.clock()
        count = int((now - self.started) * hz) + 1
        return count, self.started + (count - 1) / hz

    def heartbeat_entry(self):
        if not self.heartbeat:
            return None
        counter, last = self._ticks(self.heartbeat_hz)
        base_mode = 81 | (ARMED_FLAG if self.armed else 0)
        return self._entry({"type": "HEARTBEAT", "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                            "base_mode": {"bits": base_mode}, "custom_mode": 19, "mavlink_version": 3,
                            "mavtype": {"type": "MAV_TYPE_SUBMARINE"},
                            "system_status": {"type": "MAV_STATE_ACTIVE" if self.armed else "MAV_STATE_STANDBY"}},
                           counter, last, self.heartbeat_hz)

    def system_time_entry(self):
        if not self.heartbeat:
            return None
        counter, last = self._ticks(self.heartbeat_hz)
        return self._entry({"type": "SYSTEM_TIME", "time_unix_usec": int(last * 1e6),
                            "time_boot_ms": int((last - self.booted) * 1000)}, counter, last, self.heartbeat_hz)

    def reboot(self):
        """The autopilot restarts now: SYSTEM_TIME.time_boot_ms starts again from 0."""
        with self.lock:
            self.booted = self.clock()

    def dvl_entry(self):
        if self.dvl_velocity is None:
            return None
        counter, last = self._ticks(self.dvl_hz)
        velocity = self.dvl_velocity(last)
        if velocity is None:
            return None
        dt = 1.0 / self.dvl_hz
        return self._entry({"type": "VISION_POSITION_DELTA", "time_usec": int(last * 1e6),
                            "time_delta_usec": int(dt * 1e6), "angle_delta": [0.0, 0.0, 0.0],
                            "position_delta": [v * dt for v in velocity], "confidence": 90.0},
                           counter, last, self.dvl_hz)

    def own_vio_entry(self):
        counter, last = self._ticks(10.0)
        return self._entry({"type": "VISION_POSITION_DELTA", "time_usec": int(last * 1e6), "time_delta_usec": 100000,
                            "angle_delta": [0.0, 0.0, 0.0], "position_delta": [0.5, 0.5, 0.5], "confidence": 80.0},
                           counter, last, 10.0)

    def message(self, system, component, name):
        with self.lock:
            if (system, component) == AUTOPILOT:
                if name == "HEARTBEAT":
                    return self.heartbeat_entry()
                if name == "SYSTEM_TIME":
                    return self.system_time_entry()
                if name == "PARAM_VALUE" and self.param_value is not None:
                    return self._entry(self.param_value, self.param_value_counter, self.param_value_time, 0.0)
            if (system, component) == OWN_VIO and name == "VISION_POSITION_DELTA":
                return self.own_vio_entry()
            if (system, component) == self.dvl_ids and name == "VISION_POSITION_DELTA":
                return self.dvl_entry()
        return None

    def tree(self):
        vehicles = {}
        names = [(AUTOPILOT, "HEARTBEAT"), (AUTOPILOT, "PARAM_VALUE"), (OWN_VIO, "VISION_POSITION_DELTA"),
                 (self.dvl_ids, "VISION_POSITION_DELTA")]
        for (system, component), name in names:
            entry = self.message(system, component, name)
            if entry is None:
                continue
            components = vehicles.setdefault(str(system), {"components": {}})["components"]
            components.setdefault(str(component), {"messages": {}})["messages"][name] = entry
        return vehicles

    # --- posted messages ---

    @staticmethod
    def _param_name(param_id):
        if isinstance(param_id, list):
            param_id = "".join(param_id)
        return str(param_id).rstrip("\u0000")

    def post(self, payload):
        message = payload.get("message", {})
        with self.lock:
            self.posted.append(message)
            kind = message.get("type")
            if kind not in ("PARAM_REQUEST_READ", "PARAM_SET"):
                return
            name = self._param_name(message.get("param_id", ""))
            if name not in self.params:
                return  # the autopilot ignores unknown parameters
            if kind == "PARAM_SET" and name not in self.ignore_param_set:
                self.params[name] = float32(message["param_value"])
            self.param_value_counter += 1
            self.param_value_time = self.clock()
            self.param_value = {
                "type": "PARAM_VALUE", "param_id": list(name.ljust(16, "\u0000")),
                "param_value": self.params[name], "param_count": len(self.params), "param_index": 0,
                "param_type": {"type": "MAV_PARAM_TYPE_REAL32" if name in FLOAT_PARAMS else "MAV_PARAM_TYPE_INT8"}}

    def param_sets(self):
        return [m for m in self.posted if m.get("type") == "PARAM_SET"]

    # --- HTTP-ish handling shared by urlopen() and serve() ---

    def handle(self, method, path, body=None):
        """(status, JSON-able body)."""
        path = urllib.parse.urlparse(path).path
        self.requests.append((method, path))
        if not path.startswith(PREFIX + "/"):
            return 404, {"error": "not found"}
        parts = path[len(PREFIX):].strip("/").split("/")
        if method == "POST" and parts == ["mavlink"]:
            self.post(body or {})
            return 200, {"result": "ok"}
        if method == "GET" and parts == ["mavlink", "vehicles"]:
            return 200, self.tree()
        if (method == "GET" and len(parts) == 7 and parts[:2] == ["mavlink", "vehicles"] and parts[3] == "components"
                and parts[5] == "messages"):
            try:
                entry = self.message(int(parts[2]), int(parts[4]), parts[6])
            except ValueError:
                entry = None
            if entry is None:
                return 404, {"error": f"{parts[6]} not found"}
            return 200, entry
        return 404, {"error": "not found"}

    def urlopen(self, request, timeout=None, **kwargs):
        """Drop-in for urllib.request.urlopen that answers only http://<host>/mavlink2rest/..."""
        url = request if isinstance(request, str) else request.full_url
        parsed = urllib.parse.urlparse(url)
        if parsed.netloc != self.host or not self.reachable:
            raise urllib.error.URLError(f"fake network: no route to {parsed.netloc}")
        if isinstance(request, str) or request.data is None:
            status, body = self.handle("GET", parsed.path)
        else:
            status, body = self.handle("POST", parsed.path, json.loads(request.data))
        if status != 200:
            raise urllib.error.HTTPError(url, status, "Not Found", {}, io.BytesIO(json.dumps(body).encode()))
        return FakeResponse(json.dumps(body).encode())


def serve(vehicle, host="127.0.0.1", port=0):
    """Serve the vehicle over HTTP on localhost on a daemon thread; returns (server, "127.0.0.1:<port>")."""
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("the fake mavlink2rest only listens on localhost")

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._reply(*vehicle.handle("GET", self.path))

        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                return self._reply(400, {"error": "bad json"})
            self._reply(*vehicle.handle("POST", self.path, body))

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, name="fake-mavlink2rest", daemon=True).start()
    return server, f"127.0.0.1:{server.server_address[1]}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--armed", action="store_true")
    parser.add_argument("--no-dvl", action="store_true")
    args = parser.parse_args()
    dvl = None if args.no_dvl else (lambda t: (0.2 * math.sin(t / 5.0), 0.1 * math.cos(t / 7.0), 0.0))
    vehicle = FakeVehicle(host=f"127.0.0.1:{args.port}", armed=args.armed, dvl_velocity=dvl)
    server, address = serve(vehicle, port=args.port)
    print(f"fake mavlink2rest on http://{address}{PREFIX}  (armed={args.armed})", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    main()
