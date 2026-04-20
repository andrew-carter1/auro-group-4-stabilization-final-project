"""
Yaw Stabilizer — Physics-Based Horizontal Translation with Sliding Crop

Subscribes to rolling shutter corrected frames and gimbal yaw, applies yaw stabilization
via horizontal pixel translation derived from the camera's fisheye equidistant projection.

Input:  /rs_corrected_frame/compressed (rolling shutter corrected frames — clean, not side-by-side)
        /gimbal/angles (gimbal yaw in degrees, continuous)
Output: /yaw_stabilized/compressed (cropped to 4:3, yaw-stabilized)

Correction math:
  - Fisheye equidistant: f_eq = (width/2) / (fov_rad/2)
  - Yaw to pixel shift: dx = f_eq * tan(d_yaw_rad)
  - Sliding crop: move crop window left/right to compensate; output is always 4:3 (960×720)
  - No border artifacts — crop is always within the captured frame

Lag compensation:
  - yaw_lag_frames: buffer N incoming frames before processing, so the gimbal history
    that arrives during that window aligns with the frame being stabilized.
    Analogous to the compass_lag_frames in rolling_shutter_node.
    Default 0 (no buffer). Set to match compass_lag_frames if using both nodes together.

Parameters:
  fov_horizontal_deg (float): camera horizontal FOV, default 150.0 (Mobius)
  max_margin_px (int):        max pixels to shift crop center, default 80
  yaw_lag_frames (int):       frame buffer depth for gimbal lag compensation, default 0
  show_annotations (bool):    show circle + text overlays on output, default True
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
import math
from collections import deque


class YawStabilizer(Node):
    def __init__(self):
        super().__init__('yaw_stabilizer')

        # Parameters
        self.declare_parameter('fov_horizontal_deg', 150.0)
        self.declare_parameter('max_margin_px',      80)
        self.declare_parameter('yaw_lag_frames',     0)
        self.declare_parameter('show_annotations',   True)

        self._fov_deg         = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._max_margin_px   = self.get_parameter('max_margin_px').get_parameter_value().integer_value
        self._lag_frames      = self.get_parameter('yaw_lag_frames').get_parameter_value().integer_value
        self._show_annotations = self.get_parameter('show_annotations').get_parameter_value().bool_value

        # Subscriptions
        self.sub_frame = self.create_subscription(
            CompressedImage,
            '/rs_corrected_frame/compressed',
            self.frame_callback,
            10
        )
        self.sub_gimbal = self.create_subscription(
            Vector3Stamped,
            '/gimbal/angles',
            self.gimbal_callback,
            10
        )

        # Publication
        self.pub = self.create_publisher(CompressedImage, '/yaw_stabilized/compressed', 10)

        # State
        self.current_yaw = 0.0
        self.yaw_history = deque(maxlen=30)   # 30-frame moving average for smoothing
        self._frame_buffer: deque = deque()   # for lag compensation

    def gimbal_callback(self, msg):
        """Update current yaw from gimbal magnetometer (continuous, not wrapped)."""
        self.current_yaw = msg.vector.z

    def frame_callback(self, msg):
        """Buffer incoming frames and process with lag compensation."""
        if self._lag_frames > 0:
            self._frame_buffer.append(msg)
            if len(self._frame_buffer) <= self._lag_frames:
                return  # still filling buffer
            msg = self._frame_buffer.popleft()

        self._process(msg)

    def _process(self, msg):
        """Apply yaw stabilization via sliding crop."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        out_w = 960   # output width — 4:3 at 720 height
        out_h = h     # preserve input height (720 if camera is at 1280×720)

        # Smooth yaw over recent frames
        self.yaw_history.append(self.current_yaw)
        smoothed_yaw = float(np.mean(self.yaw_history))

        # Yaw correction: deviation from smoothed baseline → pixel shift
        correction_deg = smoothed_yaw - self.current_yaw
        fov_rad = math.radians(self._fov_deg)
        f_eq = (w / 2.0) / (fov_rad / 2.0)  # equidistant focal length (~489 px at 1280, 150°)
        dx = f_eq * math.tan(math.radians(correction_deg))
        dx = max(min(dx, self._max_margin_px), -self._max_margin_px)

        # Sliding crop: shift window center by -dx to compensate for yaw motion
        cx = w // 2 - int(dx)
        x0 = cx - out_w // 2
        x0 = max(0, min(x0, w - out_w))
        frame_out = frame[:, x0:x0 + out_w].copy()

        if self._show_annotations:
            # Yellow circle at the world-anchor point — stays near center if stabilization works
            anchor_x = w // 2 - x0   # where original frame center maps to in output
            anchor_y = out_h // 2
            cv2.circle(frame_out, (int(anchor_x), anchor_y), 8, (0, 255, 255), 2)
            cv2.line(frame_out, (int(anchor_x) - 12, anchor_y),
                     (int(anchor_x) + 12, anchor_y), (0, 255, 255), 1)
            cv2.line(frame_out, (int(anchor_x), anchor_y - 12),
                     (int(anchor_x), anchor_y + 12), (0, 255, 255), 1)
            cv2.putText(frame_out, f"Yaw: {self.current_yaw:.1f} deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_out, f"dx: {dx:.1f}px",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        _, buffer = cv2.imencode('.jpg', frame_out)
        out_msg = CompressedImage()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.format = 'jpeg'
        out_msg.data = np.array(buffer).tobytes()
        self.pub.publish(out_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YawStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
