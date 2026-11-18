"""
Small MAVLink sender over mavlink2rest's websocket, for scripts that compute odometry on the host
(pnp_vio.py) and don't go through a depthai host node like MavlinkNode.
"""

import json
import threading
import time

import websocket
from loguru import logger

NAMED_VALUE_FLOAT_NAME_LEN = 10


class MavlinkSender:
    def __init__(self, mavlink_host, vehicle_id=1, component_id=195, reconnect_interval=5.0):
        self.mavlink_host = mavlink_host
        self.vehicle_id = vehicle_id
        self.component_id = component_id  # 195 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, as MavlinkNode
        self.reconnect_interval = reconnect_interval
        self.ws = None
        self.connected = False
        self._last_connect_attempt = 0.0
        self._start = time.monotonic()

    def connect(self, timeout=5.0):
        """Open the websocket; waits up to timeout for it to come up."""
        self._last_connect_attempt = time.time()
        logger.info(f"Connecting to WebSocket at {self.mavlink_host}")

        def on_open(ws):
            logger.info("WebSocket connection opened")
            self.connected = True

        def on_close(ws, close_status_code, close_msg):
            logger.warning("WebSocket connection closed")
            self.connected = False

        def on_error(ws, error):
            logger.error(f"WebSocket error: {error}")
            self.connected = False

        self.ws = websocket.WebSocketApp(self.mavlink_host, on_open=on_open, on_close=on_close, on_error=on_error)
        threading.Thread(target=self.ws.run_forever, daemon=True).start()

        deadline = time.time() + timeout
        while not self.connected and time.time() < deadline:
            time.sleep(0.05)
        return self.connected

    def send(self, message_type, message_data):
        if not self.connected:
            # Never block the odometry loop on a reconnect; retry at most every reconnect_interval
            if time.time() - self._last_connect_attempt > self.reconnect_interval:
                self.connect(timeout=0.0)
            return False
        message = {
            "header": {"system_id": self.vehicle_id, "component_id": self.component_id, "sequence": 0},
            "message": {"type": message_type, **message_data},
        }
        try:
            self.ws.send(json.dumps(message))
            return True
        except Exception as e:
            logger.error(f"Error sending {message_type} via WebSocket: {e}")
            self.connected = False
            return False

    def send_vision_position_delta(self, angle_delta, position_delta, time_delta_us, confidence, timestamp_us=None):
        """
        angle_delta (rad) and position_delta (m) in body FRD; confidence 0..100. ArduPilot ignores
        time_usec (it stamps the message on arrival, minus VISO_DELAY_MS) but divides by time_delta_usec.
        """
        return self.send("VISION_POSITION_DELTA", {
            "time_usec": int(timestamp_us if timestamp_us is not None else time.time() * 1e6),
            # Rounded: differences of float seconds land just below whole microseconds
            "time_delta_usec": int(round(time_delta_us)),
            "angle_delta": [float(a) for a in angle_delta],
            "position_delta": [float(p) for p in position_delta],
            "confidence": float(confidence),
        })

    def send_named_value_float(self, name, value):
        # NAMED_VALUE_FLOAT.name is char[10]; mavlink2rest takes char arrays as lists, like
        # AutopilotParams._encode_id does for PARAM_SET.param_id
        if len(name) > NAMED_VALUE_FLOAT_NAME_LEN:
            logger.warning(f"NAMED_VALUE_FLOAT name {name!r} truncated to {NAMED_VALUE_FLOAT_NAME_LEN} chars")
        padded = name[:NAMED_VALUE_FLOAT_NAME_LEN].ljust(NAMED_VALUE_FLOAT_NAME_LEN, "\u0000")
        return self.send("NAMED_VALUE_FLOAT", {
            # uint32: milliseconds since this sender started (epoch milliseconds would overflow)
            "time_boot_ms": int((time.monotonic() - self._start) * 1000) & 0xFFFFFFFF,
            "name": list(padded),
            "value": float(value),
        })

    def close(self):
        if self.ws:
            try:
                self.ws.close()
            except Exception as e:
                logger.error(f"Error closing WebSocket: {e}")
        self.connected = False
