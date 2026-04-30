#!/usr/bin/env python3
"""
Face Tracking with Gimbal Control

Subscribes to:
  /face/bbox              — face bounding box from detector
  /gimbal/angles          — gimbal angles (for control input prediction)
  /image_with_faces/compressed — video frames

Publishes:
  /face_tracked/compressed        — output frames (square crop with dynamic yaw centering)
  /face_tracker/pitch_cmd         — pitch command for gimbal servo (Float32, degrees)

The Kalman filter tracks face position in angle space, using gimbal motion as
a control input to smoothly predict face motion even when detector drops frames.

Dynamic Yaw Centering:
  Crop window shifts horizontally to keep detected face centered, with PD control
  and 3-frame weighted averaging for smooth motion.

Future Improvements:
  - 9:16 vertical aspect ratio output for more left/right flexibility
  - Crop window biased toward upper frame for full-body tracking
  - Gimbal yaw feedback integration for stabilization (not just centering)
"""

import math
from collections import deque
from threading import Lock

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

from stabilization_pkg.kalman_face import FaceKalman


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
    msg.header.stamp = None  # will be set by caller
    msg.format = 'jpeg'
    msg.data = np.array(buffer).tobytes()
    return msg


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')

        # ------ Parameters ------
        self.declare_parameter('fov_horizontal_deg', 100.0)
        self.declare_parameter('fov_vertical_deg', 75.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('out_w', 720)  # Square output for more yaw flexibility
        self.declare_parameter('max_margin_px', 120)  # Increased for square format
        self.declare_parameter('no_face_hold_sec', -1.0)  # -1 = forever
        self.declare_parameter('p_gain', 2.0)  # Increased to make yaw shift visible
        self.declare_parameter('d_gain', 0.3)
        self.declare_parameter('dt', 0.033)
        self.declare_parameter('show_annotations', True)
        self.declare_parameter('kalman_std_acc', 30.0)
        self.declare_parameter('kalman_std_meas', 5.0)

        self._fov_h = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._fov_v = self.get_parameter('fov_vertical_deg').get_parameter_value().double_value
        self._img_w = self.get_parameter('image_width').get_parameter_value().integer_value
        self._img_h = self.get_parameter('image_height').get_parameter_value().integer_value
        self._out_w = self.get_parameter('out_w').get_parameter_value().integer_value
        self._max_margin_px = self.get_parameter('max_margin_px').get_parameter_value().integer_value
        self._no_face_hold_sec = self.get_parameter('no_face_hold_sec').get_parameter_value().double_value
        self._p_gain = self.get_parameter('p_gain').get_parameter_value().double_value
        self._d_gain = self.get_parameter('d_gain').get_parameter_value().double_value
        self._dt = self.get_parameter('dt').get_parameter_value().double_value
        self._show_annotations = self.get_parameter('show_annotations').get_parameter_value().bool_value
        self._kalman_std_acc = self.get_parameter('kalman_std_acc').get_parameter_value().double_value
        self._kalman_std_meas = self.get_parameter('kalman_std_meas').get_parameter_value().double_value

        # Compute fisheye focal length (equidistant projection)
        fov_rad_h = math.radians(self._fov_h)
        fov_rad_v = math.radians(self._fov_v)
        self._f_eq_h = (self._img_w / 2.0) / (fov_rad_h / 2.0)
        self._f_eq_v = (self._img_h / 2.0) / (fov_rad_v / 2.0)

        # ------ State ------
        self._lock = Lock()
        self._kf = FaceKalman(dt=self._dt, std_acc=self._kalman_std_acc, std_meas=self._kalman_std_meas)
        self._tracking = False
        self._last_face_time = None
        self._last_gimbal_yaw = 0.0
        self._last_gimbal_pitch = 0.0
        self._prev_dx_raw = 0.0  # for D gain on yaw crop
        self._yaw_crop_history = deque(maxlen=3)  # 3-frame weighted average for smoothness

        # ------ Subscriptions ------
        self.create_subscription(RegionOfInterest, '/face/bbox', self._face_bbox_cb, 10)
        self.create_subscription(Vector3Stamped, '/gimbal/angles', self._gimbal_cb, 10)
        self.create_subscription(CompressedImage, '/image_with_faces/compressed', self._frame_cb, 10)

        # ------ Publishers ------
        self._frame_pub = self.create_publisher(CompressedImage, '/face_tracked/compressed', 10)
        self._pitch_pub = self.create_publisher(Float32, '/face_tracker/pitch_cmd', 10)

        self.get_logger().info(
            f"Face Tracker started — FOV {self._fov_h:.0f}°×{self._fov_v:.0f}°, "
            f"out_w={self._out_w}, p_gain={self._p_gain}, d_gain={self._d_gain}"
        )

    # ------ Callbacks ------

    def _face_bbox_cb(self, msg: RegionOfInterest):
        """Receive face bounding box detection."""
        with self._lock:
            # Extract center in pixels
            cx_px = msg.x_offset + msg.width // 2
            cy_px = msg.y_offset + msg.height // 2

            # Convert pixels to angles
            theta_x = self._pixel_to_angle_h(cx_px)
            theta_y = self._pixel_to_angle_v(cy_px)

            # Initialize filter on first detection
            if not self._tracking:
                self._kf.reset(theta_x, theta_y)
                self._tracking = True
                self.get_logger().info(f"Face acquired at ({theta_x:.1f}°, {theta_y:.1f}°)")

            # Compute gimbal delta since last callback
            delta_yaw = -self._last_gimbal_yaw  # reset; will be updated if new gimbal msg arrives
            delta_pitch = -self._last_gimbal_pitch
            self._last_gimbal_yaw = 0.0
            self._last_gimbal_pitch = 0.0

            # Predict with gimbal control input
            if delta_yaw != 0.0 or delta_pitch != 0.0:
                u = [delta_yaw / self._dt, delta_pitch / self._dt]
                self._kf.predict(u=u)
            else:
                self._kf.predict(u=None)

            # Update with measurement
            self._kf.update([theta_x, theta_y])

            self._last_face_time = self.get_clock().now()

    def _gimbal_cb(self, msg: Vector3Stamped):
        """Receive gimbal angles for control input."""
        with self._lock:
            # Store deltas for next predict step
            self._last_gimbal_yaw = msg.vector.z
            self._last_gimbal_pitch = msg.vector.y

    def _frame_cb(self, msg: CompressedImage):
        """Process incoming frame."""
        frame = _decode_compressed(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode frame', throttle_duration_sec=2.0)
            return

        h, w = frame.shape[:2]
        self.get_logger().debug(f'Received frame {w}×{h}, tracking={self._tracking}')

        with self._lock:
            if not self._tracking:
                # No face detected; publish frame as-is (or hold last crop)
                self._frame_pub.publish(_encode_compressed(frame))
                self._pitch_pub.publish(Float32(data=0.0))
                return

            # Predict using gimbal data (in case no new bbox this frame)
            theta_x_est, theta_y_est = self._kf.predict(u=None)

            # Compute yaw crop offset (software yaw centering)
            dx_raw = self._f_eq_h * math.tan(math.radians(theta_x_est))

            # PD control
            dx_p = self._p_gain * dx_raw
            dx_delta = dx_raw - self._prev_dx_raw
            dx_controlled = dx_p - self._d_gain * dx_delta
            self._prev_dx_raw = dx_raw

            # Hard clamp
            m = float(self._max_margin_px)
            dx_controlled = max(-m, min(m, dx_controlled))

            # 3-frame weighted average for smoothness
            self._yaw_crop_history.append(dx_controlled)
            if len(self._yaw_crop_history) == 3:
                dx_final = 0.80 * self._yaw_crop_history[2] + 0.15 * self._yaw_crop_history[1] + 0.05 * self._yaw_crop_history[0]
            elif len(self._yaw_crop_history) == 2:
                dx_final = 0.60 * self._yaw_crop_history[1] + 0.40 * self._yaw_crop_history[0]
            else:
                dx_final = self._yaw_crop_history[0]

            # Apply sliding crop window for yaw
            h, w = frame.shape[:2]
            out_w = min(self._out_w, w)
            cx = w // 2 + int(dx_final)
            x0 = max(0, min(cx - out_w // 2, w - out_w))
            frame_out = frame[:, x0:x0 + out_w].copy()

            # Annotations
            if self._show_annotations:
                cv2.putText(frame_out, f"Yaw: {theta_x_est:.1f}°",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame_out, f"Pitch: {theta_y_est:.1f}°",
                           (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame_out, f"Shift: {dx_final:.1f}px",
                           (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)

            # Publish cropped frame
            out_msg = _encode_compressed(frame_out)
            out_msg.header.stamp = self.get_clock().now().to_msg()
            self._frame_pub.publish(out_msg)
            self.get_logger().debug(f'Published tracked frame {self._out_w}×{h}')

            # Publish pitch command (clamped to ±45°)
            pitch_cmd = max(-45.0, min(45.0, theta_y_est))
            self._pitch_pub.publish(Float32(data=pitch_cmd))

    # ------ Coordinate conversion ------

    def _pixel_to_angle_h(self, px):
        """Convert pixel x-coordinate to horizontal angle (degrees from center)."""
        dx_px = px - self._img_w / 2.0
        return math.degrees(math.atan(dx_px / self._f_eq_h))

    def _pixel_to_angle_v(self, py):
        """Convert pixel y-coordinate to vertical angle (degrees from center)."""
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
