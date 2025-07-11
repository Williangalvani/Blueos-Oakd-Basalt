import depthai as dai
import threading
import time
import json
import websocket
import argparse
import math
from loguru import logger


class MavlinkNode(dai.node.ThreadedHostNode):
    def __init__(self, mavlink_host="ws://127.0.0.1:6040/mavlink2rest/ws/mavlink?filter=SEND_ONLY", vehicle_id=1, component_id=195, camera_angle=0.0, quality_rate=1.0):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputTrans = dai.Node.Input(self)
        self.inputQuality = dai.Node.Input(self)
        # self.inputImg = dai.Node.Input(self)
        
        self.mavlink_host = mavlink_host
        self.vehicle_id = vehicle_id
        self.component_id = component_id  # Using component ID 195 (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        self.camera_angle = camera_angle  # Camera angle in degrees (0 = forward, 45 = 45° down)
        self.quality_rate = quality_rate  # Rate limit for quality messages (Hz)
        
        # Quality message rate limiting
        self.last_quality_time = 0.0
        self.quality_interval = 1.0 / quality_rate if quality_rate > 0 else float('inf')
        
        # Create camera rotation transformation if needed
        self.camera_rotation_matrix = None
        if self.camera_angle != 0.0:
            self.camera_rotation_matrix = self.create_camera_rotation_matrix()
            logger.info(f"Applied camera rotation of {self.camera_angle} degrees")
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # Velocity estimation tracking
        self.prev_translation = None
        self.prev_timestamp = None
        self.velocity_smoothing_alpha = 0.3  # Smoothing factor for velocity calculation
        self.smoothed_world_velocity = None  # Velocity in world frame for VISION_SPEED_ESTIMATE
        
        # Track if we're running
        self._running = False
        
    def create_websocket_connection(self):
        """
        Create and manage websocket connection to mavlink2rest
        """
        try:
            logger.info(f"Connecting to WebSocket at {self.mavlink_host}")
            
            def on_open(ws):
                logger.info("WebSocket connection opened")
                self.ws_connected = True
            
            def on_close(ws, close_status_code, close_msg):
                logger.warning("WebSocket connection closed")
                self.ws_connected = False
                
            def on_error(ws, error):
                logger.error(f"WebSocket error: {error}")
                self.ws_connected = False
            
            # Create websocket connection
            self.ws = websocket.WebSocketApp(
                self.mavlink_host,
                on_open=on_open,
                on_close=on_close,
                on_error=on_error
            )
            
            # Start connection in a separate thread
            websocket_thread = threading.Thread(target=self.ws.run_forever)
            websocket_thread.daemon = True
            websocket_thread.start()
            
            # Wait for connection to establish
            timeout = 5.0
            start_time = time.time()
            while not self.ws_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
                
            if not self.ws_connected:
                logger.error("Failed to establish WebSocket connection within timeout")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Error creating WebSocket connection: {e}")
            return False
    
    def send_mavlink_message_ws(self, message_type: str, message_data: dict) -> bool:
        """
        Send a MAVLink message via websocket
        """
        if not self.ws_connected or not self.ws:
            logger.warning("WebSocket not connected, attempting to reconnect...")
            if not self.create_websocket_connection():
                return False
        
        try:
            # Create the message payload
            message = {
                "header": {
                    "system_id": self.vehicle_id,
                    "component_id": self.component_id,
                    "sequence": 0
                },
                "message": {
                    "type": message_type,
                    **message_data
                }
            }
            
            # Send via websocket
            self.ws.send(json.dumps(message))
            logger.debug(f"Successfully sent {message_type} message via WebSocket")

            return True
            
        except Exception as e:
            logger.error(f"Error sending {message_type} via WebSocket: {e}")
            self.ws_connected = False
            return False
    
    def send_vision_position_estimate(self, translation, quaternion, timestamp_us):
        """
        Send VISION_POSITION_ESTIMATE message with pose data
        """
        message_data = {
            "usec": timestamp_us,
            "x": translation.x,
            "y": translation.y, 
            "z": translation.z,
            "roll": 0.0,  # We'll send quaternion separately
            "pitch": 0.0,
            "yaw": 0.0,
            "covariance": [0.0] * 21,  # Unknown covariance
            "reset_counter": 0
        }
        
        return self.send_mavlink_message_ws("VISION_POSITION_ESTIMATE", message_data)
    
    def send_att_pos_mocap(self, translation, quaternion, timestamp_us):
        """
        Send ATT_POS_MOCAP message with both position and orientation data
        """
        message_data = {
            "time_usec": timestamp_us,
            "q": [quaternion.qw, quaternion.qx, quaternion.qy, quaternion.qz],  # [w, x, y, z]
            "x": translation.x,
            "y": translation.y,
            "z": translation.z,
            "covariance": [0.0] * 21  # Unknown covariance
        }
        
        return self.send_mavlink_message_ws("ATT_POS_MOCAP", message_data)
    
    def send_vision_speed_estimate(self, velocity, timestamp_us):
        """
        Send VISION_SPEED_ESTIMATE message with velocity data in world frame
        """
        message_data = {
            "usec": timestamp_us,
            "x": velocity[0],  # X velocity in world frame (m/s)
            "y": velocity[1],  # Y velocity in world frame (m/s)
            "z": velocity[2],  # Z velocity in world frame (m/s)
            "covariance": [0.0] * 9,  # 3x3 velocity covariance matrix (flattened)
            "reset_counter": 0
        }
        
        return self.send_mavlink_message_ws("VISION_SPEED_ESTIMATE", message_data)

    def send_vision_position_delta(self, position_delta, time_delta_us, timestamp_us):
        """
        Send VISION_POSITION_DELTA message with position delta data in body frame
        """
        message_data = {
            "time_usec": timestamp_us,
            "time_delta_usec": time_delta_us,
            "angle_delta": [0.0, 0.0, 0.0],  # [roll, pitch, yaw] in radians - not calculated
            "position_delta": [position_delta[0], position_delta[1], position_delta[2]],  # [x, y, z] in body frame (m)
            "confidence": 100.0  # Confidence percentage (0-100%)
        }
        
        return self.send_mavlink_message_ws("VISION_POSITION_DELTA", message_data)
    
    def send_named_value_float(self, name, value, timestamp_us):
        """
        Send NAMED_VALUE_FLOAT message with quality metrics
        """
        message_data = {
            "time_boot_ms": timestamp_us // 1000,  # Convert microseconds to milliseconds
            "name": name,
            "value": value
        }
        
        return self.send_mavlink_message_ws("NAMED_VALUE_FLOAT", message_data)
    
    def send_quality_metrics(self, quality_data, timestamp_us):
        """
        Send VIO quality metrics as NAMED_VALUE_FLOAT messages
        """
        current_time = time.time()
        
        # Rate limit quality messages
        if current_time - self.last_quality_time < self.quality_interval:
            return True
        
        self.last_quality_time = current_time
        
        # Send tracking quality (0.0 to 1.0)
        self.send_named_value_float("VIO_TRACKING_QUALITY", quality_data.avgTrackingQuality, timestamp_us)
        
        # Send number of tracked features (normalized to 0-1 range, assuming max 1000 features)
        total_features = sum(quality_data.numTrackedFeatures) if quality_data.numTrackedFeatures else 0
        normalized_features = min(total_features / 1000.0, 1.0)  # Normalize to 0-1
        self.send_named_value_float("VIO_TRACKED_FEATURES", normalized_features, timestamp_us)
        
        # Send number of landmarks (normalized to 0-1 range, assuming max 10000 landmarks)
        normalized_landmarks = min(quality_data.numLandmarks / 10000.0, 1.0)
        self.send_named_value_float("VIO_LANDMARKS", normalized_landmarks, timestamp_us)
        
        # Send processing time in seconds
        processing_time_sec = quality_data.processingTimeMs / 1000.0
        self.send_named_value_float("VIO_PROCESSING_TIME", processing_time_sec, timestamp_us)
        
        # Send tracking status (1.0 if tracking, 0.0 if lost)
        tracking_status = 1.0 if quality_data.isTracking else 0.0
        self.send_named_value_float("VIO_TRACKING_STATUS", tracking_status, timestamp_us)
        
        logger.debug(f"Sent quality metrics: tracking_quality={quality_data.avgTrackingQuality:.3f}, "
                    f"features={normalized_features:.3f}, landmarks={normalized_landmarks:.3f}, "
                    f"processing_time={processing_time_sec:.3f}s, tracking_status={tracking_status}")
        
        return True
    
    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to 3x3 rotation matrix
        q = [qw, qx, qy, qz] (w, x, y, z)
        """
        qw, qx, qy, qz = q.qw, q.qx, q.qy, q.qz
        
        # Rotation matrix from quaternion
        R = [
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ]
        return R
    
    def world_to_body_frame(self, world_vector, quaternion):
        """
        Transform a vector from world frame to body frame using quaternion
        """
        # Get rotation matrix (world to body is transpose of body to world)
        R = self.quaternion_to_rotation_matrix(quaternion)
        
        # Transpose to get world-to-body transformation
        R_T = [[R[j][i] for j in range(3)] for i in range(3)]
        
        # Apply transformation: v_body = R_T * v_world
        body_vector = [
            R_T[0][0] * world_vector[0] + R_T[0][1] * world_vector[1] + R_T[0][2] * world_vector[2],
            R_T[1][0] * world_vector[0] + R_T[1][1] * world_vector[1] + R_T[1][2] * world_vector[2],
            R_T[2][0] * world_vector[0] + R_T[2][1] * world_vector[1] + R_T[2][2] * world_vector[2]
        ]
        
        return body_vector
    
    def flu_to_frd_frame(self, flu_vector):
        """
        Transform a vector from Front-Left-Up (FLU) to Front-Right-Down (FRD) frame
        FLU: X=Forward, Y=Left, Z=Up
        FRD: X=Forward, Y=Right, Z=Down
        
        Transformation:
        - X (Forward) stays the same
        - Y (Left) becomes -Y (Right) 
        - Z (Up) becomes -Z (Down)
        """
        return [flu_vector[0], -flu_vector[1], -flu_vector[2]]
    
    def calculate_velocity_and_delta(self, translation, quaternion, timestamp_us):
        """
        Calculate velocity and position delta from position differences over time
        Returns tuple: (world_velocity, body_position_delta, time_delta_us) 
        - world_velocity: velocity in world frame (m/s) for VISION_SPEED_ESTIMATE
        - body_position_delta: position delta in body frame (m) for VISION_POSITION_DELTA
        - time_delta_us: time difference in microseconds
        """
        if self.prev_translation is None or self.prev_timestamp is None:
            # First measurement, can't calculate velocity or delta yet
            self.prev_translation = translation
            self.prev_timestamp = timestamp_us
            return None, None, None
        
        # Calculate time difference in seconds and microseconds
        dt_us = timestamp_us - self.prev_timestamp
        dt = dt_us / 1e6
        
        # Avoid division by very small time differences
        if dt < 0.001:  # Less than 1ms
            return self.smoothed_world_velocity, None, None
        
        # Calculate position delta in world frame
        world_position_delta = [
            translation.x - self.prev_translation.x,
            translation.y - self.prev_translation.y,
            translation.z - self.prev_translation.z
        ]
        
        # Calculate raw velocity in world frame (for VISION_SPEED_ESTIMATE)
        world_velocity = [
            world_position_delta[0] / dt,
            world_position_delta[1] / dt,
            world_position_delta[2] / dt
        ]
        
        # Transform position delta from world frame to body frame (for VISION_POSITION_DELTA)
        body_position_delta = self.world_to_body_frame(world_position_delta, quaternion)
        
        # Convert from FLU (Front-Left-Up) to FRD (Front-Right-Down) frame for ArduPilot
        body_position_delta_frd = self.flu_to_frd_frame(body_position_delta)
        
        # Apply exponential smoothing to world velocity to reduce noise
        if self.smoothed_world_velocity is None:
            self.smoothed_world_velocity = world_velocity
        else:
            alpha = self.velocity_smoothing_alpha
            self.smoothed_world_velocity = [
                alpha * world_velocity[0] + (1 - alpha) * self.smoothed_world_velocity[0],
                alpha * world_velocity[1] + (1 - alpha) * self.smoothed_world_velocity[1],
                alpha * world_velocity[2] + (1 - alpha) * self.smoothed_world_velocity[2]
            ]
        
        # Update previous values for next calculation
        self.prev_translation = translation
        self.prev_timestamp = timestamp_us
        
        return self.smoothed_world_velocity, body_position_delta_frd, dt_us

    def create_camera_rotation_matrix(self):
        """
        Create rotation matrix for camera pointing down at specified angle
        Rotation is around the Y-axis (pitch) to point camera downward
        """
        angle_rad = math.radians(self.camera_angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # Rotation matrix for pitch (rotation around Y-axis)
        # This rotates the camera frame to point downward
        R = [
            [cos_a, 0, sin_a],    # X-axis (forward)
            [0, 1, 0],           # Y-axis (right)
            [-sin_a, 0, cos_a]   # Z-axis (down)
        ]
        return R
    
    def apply_camera_rotation(self, translation, quaternion):
        """
        Apply camera rotation transformation to pose data
        Returns transformed translation and quaternion
        """
        if self.camera_rotation_matrix is None:
            return translation, quaternion
        
        # Apply rotation to translation vector
        R = self.camera_rotation_matrix
        transformed_translation = type(translation)(
            R[0][0] * translation.x + R[0][1] * translation.y + R[0][2] * translation.z,
            R[1][0] * translation.x + R[1][1] * translation.y + R[1][2] * translation.z,
            R[2][0] * translation.x + R[2][1] * translation.y + R[2][2] * translation.z
        )
        
        # For quaternion, we need to compose the rotations
        # Convert camera rotation matrix to quaternion
        camera_qw = math.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
        camera_qx = (R[2][1] - R[1][2]) / (4 * camera_qw)
        camera_qy = (R[0][2] - R[2][0]) / (4 * camera_qw)
        camera_qz = (R[1][0] - R[0][1]) / (4 * camera_qw)
        
        # Quaternion multiplication: result = camera_quat * original_quat
        # This applies camera rotation first, then original rotation
        qw = camera_qw * quaternion.qw - camera_qx * quaternion.qx - camera_qy * quaternion.qy - camera_qz * quaternion.qz
        qx = camera_qw * quaternion.qx + camera_qx * quaternion.qw + camera_qy * quaternion.qz - camera_qz * quaternion.qy
        qy = camera_qw * quaternion.qy - camera_qx * quaternion.qz + camera_qy * quaternion.qw + camera_qz * quaternion.qx
        qz = camera_qw * quaternion.qz + camera_qx * quaternion.qy - camera_qy * quaternion.qx + camera_qz * quaternion.qw
        
        # Normalize quaternion
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        transformed_quaternion = type(quaternion)(qw/norm, qx/norm, qy/norm, qz/norm)
        
        return transformed_translation, transformed_quaternion

    def run(self):
        """
        Main processing loop - receive pose data and send via MAVLink
        """
        logger.info("Starting MAVLink pose publisher with WebSocket connection")
        self._running = True
        
        # Establish WebSocket connection
        if not self.create_websocket_connection():
            logger.error("Failed to establish WebSocket connection, exiting")
            return
        logger.info("WebSocket connection established")
        while self.isRunning() and self._running:
            try:
                # Get transform data
                trans_data = self.inputTrans.get()
                # img_frame = self.inputImg.tryGet()  # Optional image data
                
                if trans_data is not None:
                    translation = trans_data.getTranslation()
                    quaternion = trans_data.getQuaternion()
                    timestamp_us = int(time.time() * 1e6)  # Current time in microseconds
                    
                    # Apply camera rotation transformation if needed
                    translation, quaternion = self.apply_camera_rotation(translation, quaternion)
                    
                    # Calculate velocity (world frame) and position delta (body frame) from position changes
                    world_velocity, body_position_delta, time_delta_us = self.calculate_velocity_and_delta(translation, quaternion, timestamp_us)
                    
                    # Send pose data via MAVLink
                    #self.send_vision_position_estimate(translation, quaternion, timestamp_us)
                    # self.send_att_pos_mocap(translation, quaternion, timestamp_us)
                    
                    # Send velocity and position delta data if available
                    # if world_velocity is not None:
                    #     self.send_vision_speed_estimate(world_velocity, timestamp_us)
                    
                    if body_position_delta is not None and time_delta_us is not None:
                        self.send_vision_position_delta(body_position_delta, time_delta_us, timestamp_us)
                    #     logger.debug(f"Sent pose: pos=({translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}), "
                    #                f"vel=({velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}), "
                    #                f"quat=({quaternion.qw:.3f}, {quaternion.qx:.3f}, {quaternion.qy:.3f}, {quaternion.qz:.3f})")
                    # else:
                    #     logger.debug(f"Sent pose: pos=({translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}), "
                    #                f"quat=({quaternion.qw:.3f}, {quaternion.qx:.3f}, {quaternion.qy:.3f}, {quaternion.qz:.3f}) [no velocity yet]")
                
                # Get quality data
                quality_data = self.inputQuality.tryGet()
                if quality_data is not None:
                    self.send_quality_metrics(quality_data, timestamp_us)
                
                # Small delay to avoid overwhelming the system
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in MAVLink processing loop: {e}")
                time.sleep(0.1)
        
        logger.info("MAVLink pose publisher stopped")
    
    def stop(self):
        """
        Stop the node
        """
        self._running = False
        if self.ws:
            try:
                self.ws.close()
                logger.info("WebSocket connection closed")
            except Exception as e:
                logger.error(f"Error closing WebSocket: {e}")
        self.ws_connected = False

def main():
    """
    Main function with command line argument parsing
    """
    parser = argparse.ArgumentParser(description='MAVLink pose publisher with camera angle support')
    parser.add_argument('--mavlink-host', 
                       default="ws://127.0.0.1:6040/mavlink2rest/ws/mavlink?filter=SEND_ONLY",
                       help='MAVLink WebSocket host URL')
    parser.add_argument('--vehicle-id', 
                       type=int, default=1,
                       help='Vehicle system ID')
    parser.add_argument('--component-id', 
                       type=int, default=195,
                       help='Component ID (default: 195 for MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)')
    parser.add_argument('--camera-angle', 
                       type=float, default=0.0,
                       help='Camera angle in degrees (0 = forward, 45 = 45° down, etc.)')
    parser.add_argument('--quality-rate', 
                       type=float, default=1.0,
                       help='Rate limit for quality messages in Hz (default: 1.0)')
    
    args = parser.parse_args()
    
    # Create and run the MAVLink node
    node = MavlinkNode(
        mavlink_host=args.mavlink_host,
        vehicle_id=args.vehicle_id,
        component_id=args.component_id,
        camera_angle=args.camera_angle,
        quality_rate=args.quality_rate
    )
    
    try:
        node.run()
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, shutting down...")
    finally:
        node.stop()

if __name__ == "__main__":
    main() 