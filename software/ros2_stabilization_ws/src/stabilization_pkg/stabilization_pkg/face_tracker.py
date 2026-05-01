#!/usr/bin/env python3
"""
Simple Face Tracking with Proportional Control

Subscribes to:
  /face/bbox              — face bounding box from DNN detector
  /image_with_faces/compressed — video frames

Publishes:
  /face_tracked/compressed        — output frames (square crop with yaw centering)
  /face_tracker/pitch_cmd         — pitch command for gimbal servo (Float32, degrees)

Uses simple proportional (P) control for smooth tracking without Kalman complexity.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from std_msgs.msg import Float32


def _decode_compressed(msg: CompressedImage):
    """Decode CompressedImage to BGR ndarray, or return None on error."""
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


def _encode_compressed(frame: np.ndarray) -> CompressedImage:
    """Encode BGR ndarray to CompressedImage (JPEG)."""
    _, buffer = cv2.imencode('.jpg', frame)
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = np.array(buffer).tobytes()
    return msg


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')

        # Parameters
        self.declare_parameter('fov_horizontal_deg', 100.0)
        self.declare_parameter('fov_vertical_deg', 75.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('out_w', 720)
        self.declare_parameter('max_margin_px', 120)
        self.declare_parameter('p_gain_yaw', 1.5)
        self.declare_parameter('p_gain_pitch', 1.5)
        self.declare_parameter('show_annotations', True)

        self._fov_h = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._fov_v = self.get_parameter('fov_vertical_deg').get_parameter_value().double_value
        self._img_w = self.get_parameter('image_width').get_parameter_value().integer_value
        self._img_h = self.get_parameter('image_height').get_parameter_value().integer_value
        self._out_w = self.get_parameter('out_w').get_parameter_value().integer_value
        self._max_margin_px = self.get_parameter('max_margin_px').get_parameter_value().integer_value
        self._p_gain_yaw = self.get_parameter('p_gain_yaw').get_parameter_value().double_value
        self._p_gain_pitch = self.get_parameter('p_gain_pitch').get_parameter_value().double_value
        self._show_annotations = self.get_parameter('show_annotations').get_parameter_value().bool_value

        # Fisheye focal lengths
        fov_rad_h = math.radians(self._fov_h)
        fov_rad_v = math.radians(self._fov_v)
        self._f_eq_h = (self._img_w / 2.0) / (fov_rad_h / 2.0)
        self._f_eq_v = (self._img_h / 2.0) / (fov_rad_v / 2.0)

        # State
        self._tracking = False
        self._latest_bbox = None
        self._prev_pitch_cmd = 0.0
        self._pitch_smooth_alpha = 0.3  # Smoothing (0-1, lower = smoother)

        # Subscriptions
        self.create_subscription(RegionOfInterest, '/face/bbox', self._bbox_cb, 10)
        self.create_subscription(CompressedImage, '/image_with_faces/compressed', self._frame_cb, 10)

        # Publishers
        self._frame_pub = self.create_publisher(CompressedImage, '/face_tracked/compressed', 10)
        self._pitch_pub = self.create_publisher(Float32, '/face_tracker/pitch_cmd', 10)

        self.get_logger().info(f"Face Tracker (Proportional) started")

    def _bbox_cb(self, msg: RegionOfInterest):
        """Store latest bbox."""
        if msg.width > 0 and msg.height > 0:
            self._latest_bbox = msg
        else:
            self._latest_bbox = None

    def _frame_cb(self, msg: CompressedImage):
        """Process frame and output tracked result."""
        frame = _decode_compressed(msg)
        if frame is None:
            return

        h, w = frame.shape[:2]

        if self._latest_bbox is None:
            # No face detected
            self._frame_pub.publish(_encode_compressed(frame))
            self._pitch_pub.publish(Float32(data=0.0))
            return

        # Get face center from bbox
        cx_px = self._latest_bbox.x_offset + self._latest_bbox.width // 2
        cy_px = self._latest_bbox.y_offset + self._latest_bbox.height // 2

        # Convert pixels to angles
        theta_x = self._pixel_to_angle_h(cx_px)
        theta_y = self._pixel_to_angle_v(cy_px)

        # Proportional control for yaw (horizontal centering)
        dx_raw = self._f_eq_h * math.tan(math.radians(theta_x))
        dx_final = self._p_gain_yaw * dx_raw
        dx_final = max(-float(self._max_margin_px), min(float(self._max_margin_px), dx_final))

        # Apply yaw crop (1:1 square)
        out_w = min(self._out_w, w, h)
        out_h = out_w
        cx = w // 2 + int(dx_final)
        cy = h // 2
        x0 = max(0, min(cx - out_w // 2, w - out_w))
        y0 = max(0, min(cy - out_h // 2, h - out_h))
        frame_out = frame[y0:y0 + out_h, x0:x0 + out_w].copy()

        # Annotations
        if self._show_annotations:
            cv2.putText(frame_out, f"Yaw: {theta_x:.1f}°", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_out, f"Pitch: {theta_y:.1f}°", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_out, f"dx: {dx_final:.1f}px", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)

        # Publish cropped frame
        self._frame_pub.publish(_encode_compressed(frame_out))

        # Pitch command with EMA smoothing
        pitch_raw = max(-40.0, min(40.0, theta_y * self._p_gain_pitch))
        pitch_cmd = (1.0 - self._pitch_smooth_alpha) * self._prev_pitch_cmd + self._pitch_smooth_alpha * pitch_raw
        self._prev_pitch_cmd = pitch_cmd
        self._pitch_pub.publish(Float32(data=pitch_cmd))

    def _pixel_to_angle_h(self, px):
        """Convert pixel x to angle (degrees)."""
        dx_px = px - self._img_w / 2.0
        return math.degrees(math.atan(dx_px / self._f_eq_h))

    def _pixel_to_angle_v(self, py):
        """Convert pixel y to angle (degrees)."""
        dy_px = py - self._img_h / 2.0
        return math.degrees(math.atan(dy_px / self._f_eq_v))


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ============================================================================
# KALMAN FILTER VERSION (COMMENTED OUT FOR REFERENCE)
# ============================================================================
# To re-enable Kalman filtering, uncomment the FaceKalman import and the
# self._kf initialization in __init__, then replace the simple proportional
# logic in _frame_cb with:
#
#   from stabilization_pkg.kalman_face import FaceKalman
#
#   In __init__:
#     self._kf = FaceKalman(dt=0.033, std_acc=30.0, std_meas=5.0)
#
#   In _frame_cb (replace the P control section):
#     # Kalman predict
#     theta_x_est, theta_y_est = self._kf.predict(u=None)
#
#     # Kalman update with measurement
#     self._kf.update([theta_x, theta_y])
#
#     # Use estimated values
#     dx_raw = self._f_eq_h * math.tan(math.radians(theta_x_est))
#     pitch_cmd = max(-40.0, min(40.0, theta_y_est))
#
# Note: Kalman version requires numpy and has shown ROS2 type issues.
# Proportional control is simpler and more stable for current use.
