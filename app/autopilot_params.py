"""
Configures the autopilot's EKF to fuse the VISION_POSITION_DELTA messages sent by this extension.
"""

import json
import math
import time
import urllib.request
from dataclasses import dataclass
from typing import Optional

from loguru import logger

from stereo_config import DEFAULT_LEVER_ARM_M

# (name, value, takes effect only after an autopilot reboot)
VISUAL_ODOMETRY_PARAMS = [
    ("AHRS_EKF_TYPE", 3, True),   # EKF3
    ("EK3_ENABLE", 1, True),
    ("VISO_TYPE", 1, True),       # MAVLink visual odometry
    ("EK3_SRC1_POSXY", 6, False),  # ExternalNav
    ("EK3_SRC1_VELXY", 6, False),  # ExternalNav
    ("EK3_SRC1_POSZ", 1, False),   # Baro
]


@dataclass(frozen=True)
class SuggestedParam:
    name: str
    value: Optional[float]  # None: keep the vehicle's value (listed for review only)
    needs_reboot: bool
    rationale: str


# Optical position hold with VISION_POSITION_DELTA (ArduSub, EKF3). Suggestions for a later UI; nothing
# here is written to the autopilot (configure_visual_odometry still uses VISUAL_ODOMETRY_PARAMS).
SUGGESTED_OPTICAL_HOLD_PARAMS = (
    SuggestedParam("VISO_TYPE", 1, True, "MAVLink visual odometry: accept VISION_POSITION_DELTA from this extension."),
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
                   "ExternalNav: fuse the VISION_POSITION_DELTA body velocity for horizontal velocity."),
    SuggestedParam("EK3_SRC1_POSXY", 0, False,
                   "0 None: VISION_POSITION_DELTA carries no absolute position, so the EKF dead-reckons a relative "
                   "position from the visual velocity; ArduSub accepts that for position hold when visual odometry "
                   "is enabled. 3 GPS only helps with a GPS fix at the surface (lost when submerged). The current "
                   "auto-configuration sets 6 ExternalNav, an absolute position this extension never sends."),
    SuggestedParam("EK3_SRC1_POSZ", 1, False, "Baro: depth from the pressure sensor."),
    SuggestedParam("EK3_SRC1_YAW", 1, False,
                   "Compass: the body-frame deltas carry no heading, so heading for an earth-frame hold comes from "
                   "the compass."),
)


def _same_value(current, wanted):
    # Autopilot parameters are float32: 0.1655 comes back as 0.16550000011920929
    return math.isclose(float(current), float(wanted), rel_tol=1e-6, abs_tol=1e-6)


def diff_params(current: dict) -> list:
    """
    Suggested optical-hold parameters that differ from `current` (name -> value, e.g. read from the
    autopilot), in SUGGESTED_OPTICAL_HOLD_PARAMS order. Each entry: name, current (None if missing),
    suggested, needs_reboot, rationale and status ("change" or "missing"). Keep-current entries are skipped.
    Pure: reads nothing from and writes nothing to the autopilot.
    """
    diff = []
    for param in SUGGESTED_OPTICAL_HOLD_PARAMS:
        if param.value is None:
            continue
        value = current.get(param.name)
        if value is not None:
            try:
                if _same_value(value, param.value):
                    continue
            except (TypeError, ValueError):
                pass
        diff.append({"name": param.name, "current": value, "suggested": param.value,
                     "needs_reboot": param.needs_reboot, "rationale": param.rationale,
                     "status": "missing" if value is None else "change"})
    return diff

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

    def _vehicle_message(self, name):
        try:
            return self._get(f"/mavlink/vehicles/{self.system_id}/components/{self.component_id}/messages/{name}")
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

    def _request_param(self, name, message, attempts=3):
        # mavlink2rest only keeps the latest PARAM_VALUE, so our reply can be overwritten by another one
        for _ in range(attempts):
            counter = self._message_counter("PARAM_VALUE")
            self._post(message)
            result = self._wait_param_value(name, counter)
            if result is not None:
                return result
        return None

    def read(self, name):
        return self._request_param(name, {
            "type": "PARAM_REQUEST_READ",
            "param_index": -1,
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
        })

    def write(self, name, value, param_type):
        return self._request_param(name, {
            "type": "PARAM_SET",
            "param_value": float(value),
            "target_system": self.system_id,
            "target_component": self.component_id,
            "param_id": self._encode_id(name),
            "param_type": {"type": param_type},
        })

    def is_armed(self):
        heartbeat = self._vehicle_message("HEARTBEAT")
        if heartbeat is None:
            return None
        return bool(heartbeat["message"]["base_mode"]["bits"] & MAV_MODE_FLAG_SAFETY_ARMED)

    def configure_visual_odometry(self):
        """Sets VISUAL_ODOMETRY_PARAMS on the autopilot. Never raises; failures are logged."""
        try:
            armed = self.is_armed()
        except Exception as e:
            logger.warning(f"Could not reach mavlink2rest at {self.base_url}, skipping autopilot setup: {e}")
            return
        if armed is None:
            logger.warning("No autopilot HEARTBEAT seen, skipping autopilot parameter setup")
            return
        if armed:
            logger.warning("Vehicle is armed, not changing autopilot parameters")
            return

        needs_reboot = []
        for name, wanted, reboot_required in VISUAL_ODOMETRY_PARAMS:
            try:
                current = self.read(name)
                if current is None:
                    logger.warning(f"{name}: not reported by the autopilot, skipping")
                    continue
                value, param_type = current
                if value == wanted:
                    logger.info(f"{name} = {wanted:g} (already set)")
                    continue
                result = self.write(name, wanted, param_type)
                if result is None or result[0] != wanted:
                    logger.error(f"{name}: failed to set to {wanted:g} (was {value:g})")
                    continue
                logger.info(f"{name}: {value:g} -> {wanted:g}")
                if reboot_required:
                    needs_reboot.append(name)
            except Exception as e:
                logger.error(f"{name}: {e}")

        if needs_reboot:
            logger.warning(f"Reboot the autopilot for {', '.join(needs_reboot)} to take effect")
