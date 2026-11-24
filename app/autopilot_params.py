"""
The recommended autopilot parameters for fusing this extension's VISION_POSITION_DELTA (ArduSub, EKF3), and
reading/writing them for the web page (param_checker.py, the ArduPilot panel). Nothing writes them
automatically: writes happen only on an explicit, confirmed request while a live HEARTBEAT says disarmed.
"""

import datetime
import json
import math
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional

from stereo_config import DEFAULT_LEVER_ARM_M


@dataclass(frozen=True)
class SuggestedParam:
    name: str
    value: Optional[float]  # None: keep the vehicle's value (listed for review only)
    needs_reboot: bool
    rationale: str


# Optical position hold with VISION_POSITION_DELTA (ArduSub, EKF3): the one recommended set. The web page checks
# the vehicle against it and writes it only when the user confirms. ArduPilot references: master 6f5d8560db.
SUGGESTED_OPTICAL_HOLD_PARAMS = (
    SuggestedParam("AHRS_EKF_TYPE", 3, True,
                   "EKF3 (ArduPilot's default, AP_AHRS.cpp:56): only EKF3 takes VISION_POSITION_DELTA "
                   "(AP_AHRS.cpp:1457-1461 writeBodyFrameOdom goes to EKF3 only)."),
    SuggestedParam("EK3_ENABLE", 1, True,
                   "EKF3 enabled (default 1, AP_NavEKF3.cpp:165-171); its parameters (EK3_*) exist only while it is "
                   "enabled."),
    SuggestedParam("VISO_TYPE", 1, True,
                   "MAVLink visual odometry: accept VISION_POSITION_DELTA from this extension. The other VISO_* "
                   "parameters only appear after this is set and the autopilot rebooted."),
    SuggestedParam("VISO_POS_X", DEFAULT_LEVER_ARM_M[0], False,
                   "Camera forward of the IMU (m, FRD, from the CAD). EKF3 adds (gyro rate) x VISO_POS to the "
                   "predicted camera velocity, so the extension sends uncorrected camera deltas. Relative to the "
                   "vehicle origin: subtract INS_POS_X if that is not 0."),
    SuggestedParam("VISO_POS_Y", DEFAULT_LEVER_ARM_M[1], False, "Camera right of the IMU (m, FRD, from the CAD)."),
    SuggestedParam("VISO_POS_Z", DEFAULT_LEVER_ARM_M[2], False, "Camera below the IMU (m, FRD, from the CAD)."),
    SuggestedParam("VISO_ORIENT", 0, False,
                   "Forward: the extension already rotates the deltas into body FRD with its Camera Angle, so "
                   "any other orientation would rotate them twice."),
    SuggestedParam("VISO_DELAY_MS", None, False,
                   "Keep the current value (default 10 ms); unverified. The EKF stamps each delta on arrival minus "
                   "this delay. A delta averages its interval, so its true delay is about half a frame interval "
                   "plus the pipeline latency; tune it from logs (range 0-250 ms)."),
    SuggestedParam("EK3_SRC1_VELXY", 6, False,
                   "ExternalNav: the EKF fuses the VISION_POSITION_DELTA body-frame velocity (all three axes) only "
                   "when VELXY is ExternalNav (AP_NavEKF3_PosVelFusion.cpp:2059-2063, Control.cpp:572-576)."),
    SuggestedParam("EK3_SRC1_POSXY", 0, False,
                   "0 None: VISION_POSITION_DELTA carries no absolute position. The EKF dead-reckons a relative "
                   "position from the body velocity (AID_RELATIVE, AP_NavEKF3_Control.cpp:297-305, 791, 812) and "
                   "ArduSub POSHOLD accepts a relative position when visual odometry is enabled (ArduSub "
                   "system.cpp:196-204, 234-266). 6 ExternalNav waits for an absolute position "
                   "(VISION_POSITION_ESTIMATE, Control.cpp:619-626) this extension never sends, so it only "
                   "misleads; 3 GPS needs a GPS fix (Control.cpp:595-601), lost when submerged."),
    SuggestedParam("EK3_SRC1_POSZ", 1, False, "Baro: depth from the pressure sensor."),
    SuggestedParam("EK3_SRC1_VELZ", 0, False,
                   "None (ArduSub's default, ArduSub/Parameters.h:442): the body-frame deltas already carry the "
                   "vertical velocity (FuseBodyVel fuses x, y and z); 6 would only fuse VISION_SPEED_ESTIMATE / "
                   "ODOMETRY velocity (PosVelFusion.cpp:604, 966), which this extension does not send."),
    SuggestedParam("EK3_SRC1_YAW", 1, False,
                   "Compass: the body-frame deltas carry no heading, so heading for an earth-frame hold comes from "
                   "the compass."),
)


# Values the web page may write, per suggested parameter: (min, max, allowed values or None, whole numbers only).
# Enumerations from the ArduPilot parameter docs; VISO_POS_* and VISO_DELAY_MS are ArduPilot's own ranges.
PARAM_LIMITS = {
    "AHRS_EKF_TYPE": (0, 11, (0, 2, 3, 10, 11), True),  # DCM, EKF2, EKF3, Sim, ExternalAHRS
    "EK3_ENABLE": (0, 1, (0, 1), True),
    "VISO_TYPE": (0, 3, (0, 1, 2, 3), True),  # None, MAVLink, IntelT265, VOXL
    "VISO_POS_X": (-5.0, 5.0, None, False),
    "VISO_POS_Y": (-5.0, 5.0, None, False),
    "VISO_POS_Z": (-5.0, 5.0, None, False),
    "VISO_ORIENT": (0, 25, (0, 2, 4, 6, 24, 25), True),  # Forward, Right, Back, Left, Up, Down
    "VISO_DELAY_MS": (0, 250, None, True),
    "EK3_SRC1_VELXY": (0, 7, (0, 3, 4, 5, 6, 7), True),  # None, GPS, Beacon, OpticalFlow, ExternalNav, WheelEncoder
    "EK3_SRC1_POSXY": (0, 6, (0, 3, 4, 6), True),  # None, GPS, Beacon, ExternalNav
    "EK3_SRC1_POSZ": (0, 6, (0, 1, 2, 3, 4, 6), True),  # None, Baro, RangeFinder, GPS, Beacon, ExternalNav
    "EK3_SRC1_VELZ": (0, 6, (0, 3, 6), True),  # None, GPS, ExternalNav
    "EK3_SRC1_YAW": (0, 8, (0, 1, 2, 3, 6, 8), True),  # None, Compass, GPS, GPS+Compass, ExternalNav, GSF
}
SUGGESTED_BY_NAME = {param.name: param for param in SUGGESTED_OPTICAL_HOLD_PARAMS}
# A HEARTBEAT older than this does not tell whether the vehicle is armed now
HEARTBEAT_MAX_AGE_S = 5.0


def _same_value(current, wanted):
    # Autopilot parameters are float32: 0.1655 comes back as 0.16550000011920929
    return math.isclose(float(current), float(wanted), rel_tol=1e-6, abs_tol=1e-6)


def message_time(message):
    """Unix time of mavlink2rest's status.time.last_update of a message, or None if it has none we can parse."""
    try:
        text = message["status"]["time"]["last_update"]
        stamp = datetime.datetime.fromisoformat(str(text).replace("Z", "+00:00"))
    except (KeyError, TypeError, ValueError):
        return None
    if stamp.tzinfo is None:
        return None
    return stamp.timestamp()


def _message_age(message, now):
    """Seconds since mavlink2rest's status.time.last_update of a message, or None if it has none we can parse."""
    stamp = message_time(message)
    return None if stamp is None else max(0.0, now - stamp)


def validate_param_values(values) -> list:
    """Problems with a {name: value} write request against PARAM_LIMITS (empty list if it is acceptable)."""
    if not isinstance(values, dict) or not values:
        return ["params must be a non-empty object of name: value"]
    problems = []
    for name, value in values.items():
        if name not in PARAM_LIMITS:
            problems.append(f"{name}: not one of the suggested parameters this page may write "
                            f"({', '.join(PARAM_LIMITS)})")
            continue
        low, high, allowed, whole = PARAM_LIMITS[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            problems.append(f"{name}: value must be a finite number, got {value!r}")
        elif not low <= value <= high:
            problems.append(f"{name}: {value:g} is outside {low:g}..{high:g}")
        elif whole and value != int(value):
            problems.append(f"{name}: must be a whole number, got {value:g}")
        elif allowed is not None and int(value) not in allowed:
            problems.append(f"{name}: {value:g} is not one of {', '.join(str(v) for v in allowed)}")
    return problems


def read_suggested_params(autopilot, attempts=2) -> dict:
    """
    Current value of every suggested parameter next to the suggestion, plus the HEARTBEAT armed state.
    Only sends PARAM_REQUEST_READ (never PARAM_SET). Status per parameter: ok, differs or unreadable.
    Without a reachable mavlink2rest or a fresh autopilot HEARTBEAT nothing is requested.
    """
    heartbeat = autopilot.heartbeat_status()
    can_read = heartbeat["reachable"] and heartbeat["counter"] is not None
    params = []
    for param in SUGGESTED_OPTICAL_HOLD_PARAMS:
        current, error = None, None
        if not can_read:
            error = heartbeat["detail"]
        else:
            try:
                reply = autopilot.read(param.name, attempts)
                if reply is None:
                    error = "no PARAM_VALUE reply"
                else:
                    current = float(reply[0])
            except Exception as e:
                error = str(e)
        suggested = param.value if param.value is not None else current
        if current is None:
            status = "unreadable"
        else:
            status = "ok" if _same_value(current, suggested) else "differs"
        low, high, allowed, whole = PARAM_LIMITS[param.name]
        params.append({"name": param.name, "current": current, "suggested": suggested,
                       "keep_current": param.value is None, "status": status, "error": error,
                       "needs_reboot": param.needs_reboot, "rationale": param.rationale,
                       "min": low, "max": high, "allowed": list(allowed) if allowed else None, "integer": whole})
    return {"heartbeat": heartbeat, "armed": heartbeat["armed"], "params": params}


class WriteRefused(Exception):
    """The vehicle is armed or its armed state is unknown (HTTP 409)."""

    def __init__(self, message, heartbeat):
        super().__init__(message)
        self.heartbeat = heartbeat


def write_suggested_params(autopilot, values, heartbeat_wait=2.5, attempts=2) -> dict:
    """
    Write {name: value} (already checked with validate_param_values) only while a live HEARTBEAT says
    disarmed; the armed bit is checked again before every PARAM_SET. Each written parameter is read back
    with a fresh PARAM_REQUEST_READ. Raises WriteRefused before writing anything if armed or unknown.
    """
    heartbeat = autopilot.live_armed_state(heartbeat_wait)
    if heartbeat["armed"] is not False:
        reason = "the vehicle is ARMED" if heartbeat["armed"] else f"the armed state is unknown ({heartbeat['detail']})"
        raise WriteRefused(f"Not writing: {reason}", heartbeat)
    changed, failed, unchanged, written = [], [], [], []
    names = list(values)
    for index, name in enumerate(names):
        new = float(values[name])
        try:
            reply = autopilot.read(name, attempts)
        except Exception as e:
            reply, error = None, str(e)
        else:
            error = "could not read the current value (needed for its type)"
        if reply is None:
            failed.append({"name": name, "old": None, "new": new, "error": error})
            continue
        old, param_type = float(reply[0]), reply[1]
        if _same_value(old, new):
            unchanged.append({"name": name, "value": old})
            continue
        if autopilot.is_armed() is not False:
            for skipped in names[index:]:
                failed.append({"name": skipped, "old": old if skipped == name else None, "new": float(values[skipped]),
                               "error": "not written: the vehicle armed or its HEARTBEAT was lost"})
            break
        try:
            reply = autopilot.write(name, new, param_type, attempts)
        except Exception as e:
            failed.append({"name": name, "old": old, "new": new, "error": f"PARAM_SET failed: {e}"})
            continue
        if reply is None:
            failed.append({"name": name, "old": old, "new": new, "error": "no PARAM_VALUE reply to PARAM_SET"})
            continue
        written.append((name, old, new))
    for name, old, new in written:
        try:
            reply = autopilot.read(name, attempts)
        except Exception:
            reply = None
        readback = None if reply is None else float(reply[0])
        if readback is not None and _same_value(readback, new):
            changed.append({"name": name, "old": old, "new": new, "readback": readback})
        else:
            failed.append({"name": name, "old": old, "new": new, "readback": readback,
                           "error": "read back does not match" if readback is not None else "no read back"})
    reboot = [entry["name"] for entry in changed
              if entry["name"] in SUGGESTED_BY_NAME and SUGGESTED_BY_NAME[entry["name"]].needs_reboot]
    return {"heartbeat": heartbeat, "changed": changed, "failed": failed, "unchanged": unchanged,
            "reboot_required": reboot}


MAV_MODE_FLAG_SAFETY_ARMED = 128


class AutopilotParams:
    def __init__(self, vehicle_ip, system_id=1, component_id=1, timeout=3.0):
        self.base_url = f"http://{vehicle_ip}/mavlink2rest"
        self.system_id = system_id
        self.component_id = component_id
        self.timeout = timeout

    def _get(self, path):
        with urllib.request.urlopen(f"{self.base_url}{path}", timeout=self.timeout) as response:
            return json.loads(response.read())

    def _post(self, message):
        payload = {
            "header": {"system_id": 255, "component_id": 0, "sequence": 0},
            "message": message,
        }
        request = urllib.request.Request(
            f"{self.base_url}/mavlink",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            response.read()

    def vehicle_message(self, name):
        """The autopilot's latest `name` message as mavlink2rest keeps it ({"message", "status"}); raises on errors."""
        return self._get(f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/{name}")

    def _vehicle_message(self, name):
        try:
            return self.vehicle_message(name)
        except Exception:
            return None

    @staticmethod
    def _encode_id(name):
        return list(name.ljust(16, "\u0000"))

    @staticmethod
    def _decode_id(param_id):
        if isinstance(param_id, list):
            param_id = "".join(param_id)
        return param_id.rstrip("\u0000")

    def _message_counter(self, name):
        message = self._vehicle_message(name)
        return message["status"]["time"]["counter"] if message else 0

    def _wait_param_value(self, name, previous_counter):
        """Waits for a PARAM_VALUE for `name` newer than `previous_counter`. Returns (value, type) or None."""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            message = self._vehicle_message("PARAM_VALUE")
            if message and message["status"]["time"]["counter"] > previous_counter:
                content = message["message"]
                if self._decode_id(content["param_id"]) == name:
                    return content["param_value"], content["param_type"]["type"]
            time.sleep(0.05)
        return None

    def _request_param(self, name, message, attempts):
        # mavlink2rest only keeps the latest PARAM_VALUE, so our reply can be overwritten by another one
        for _ in range(attempts):
            counter = self._message_counter("PARAM_VALUE")
            self._post(message)
            result = self._wait_param_value(name, counter)
            if result is not None:
                return result
        return None

    def read(self, name, attempts=3):
        return self._request_param(name, {
            "type": "PARAM_REQUEST_READ",
            "param_index": -1,
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
        }, attempts)

    def write(self, name, value, param_type, attempts=3):
        return self._request_param(name, {
            "type": "PARAM_SET",
            "param_value": float(value),
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
            "param_type": {"type": param_type},
        }, attempts)

    def is_armed(self):
        heartbeat = self._vehicle_message("HEARTBEAT")
        if heartbeat is None:
            return None
        return bool(heartbeat["message"]["base_mode"]["bits"] & MAV_MODE_FLAG_SAFETY_ARMED)

    def heartbeat_status(self, now=None):
        """
        The autopilot's last HEARTBEAT as mavlink2rest has it: reachable (mavlink2rest answered), armed
        (None if unknown: no HEARTBEAT, older than HEARTBEAT_MAX_AGE_S or malformed), counter, age_s, detail.
        """
        status = {"reachable": False, "armed": None, "counter": None, "age_s": None, "detail": None}
        try:
            heartbeat = self._get(
                f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/HEARTBEAT")
        except urllib.error.HTTPError as e:
            status.update(reachable=True, detail=f"no autopilot HEARTBEAT in mavlink2rest (HTTP {e.code})")
            return status
        except Exception as e:
            status["detail"] = f"mavlink2rest not reachable at {self.base_url}: {e}"
            return status
        status["reachable"] = True
        try:
            bits = heartbeat["message"]["base_mode"]["bits"]
            status["counter"] = heartbeat.get("status", {}).get("time", {}).get("counter")
        except (KeyError, TypeError):
            status["detail"] = "malformed HEARTBEAT"
            return status
        age = _message_age(heartbeat, time.time() if now is None else now)
        status["age_s"] = age
        if age is not None and age > HEARTBEAT_MAX_AGE_S:
            status["detail"] = f"last HEARTBEAT is {age:.0f} s old"
            return status
        status["armed"] = bool(bits & MAV_MODE_FLAG_SAFETY_ARMED)
        status["detail"] = "armed" if status["armed"] else "disarmed"
        return status

    def live_armed_state(self, wait=2.5, poll=0.1):
        """
        heartbeat_status(), but armed is only known if a new HEARTBEAT arrives within `wait` seconds, so a
        stale message kept by mavlink2rest after the autopilot went away never passes for "disarmed".
        """
        first = self.heartbeat_status()
        if first["armed"] is None:
            return first
        if first["counter"] is None:
            first.update(armed=None, detail="HEARTBEAT without a message counter: cannot tell that it is live")
            return first
        deadline = time.time() + wait
        while True:
            current = self.heartbeat_status()
            if current["armed"] is None:
                return current
            if current["counter"] is not None and current["counter"] > first["counter"]:
                return current
            if time.time() >= deadline:
                current.update(armed=None, detail=f"no new HEARTBEAT within {wait:g} s")
                return current
            time.sleep(poll)

    def boot_time(self, now=None):
        """
        Unix time the autopilot booted (now - SYSTEM_TIME.time_boot_ms), from the SYSTEM_TIME mavlink2rest keeps;
        None if there is none, it is older than HEARTBEAT_MAX_AGE_S or malformed. A GET only: sends nothing.
        """
        now = time.time() if now is None else now
        message = self._vehicle_message("SYSTEM_TIME")
        if message is None:
            return None
        stamp = message_time(message)
        try:
            boot_ms = float(message["message"]["time_boot_ms"])
        except (KeyError, TypeError, ValueError):
            return None
        if stamp is None or now - stamp > HEARTBEAT_MAX_AGE_S:
            return None
        return stamp - boot_ms / 1000.0
