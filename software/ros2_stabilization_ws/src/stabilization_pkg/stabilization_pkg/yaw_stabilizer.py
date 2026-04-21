"""
Yaw Stabilizer — EMA Reference + PD Control

Subscribes to rolling shutter corrected frames and gimbal yaw. Applies yaw stabilization
via horizontal pixel translation using a fisheye equidistant projection model.

Pipeline position:
  /rs_corrected_frame/compressed  →  [yaw_stabilizer]  →  /yaw_stabilized/compressed

Stabilization logic:
  1. A slow EMA tracks the "intended" camera heading (reference_alpha controls drift rate).
     Higher alpha = faster return to center (more aggressive jitter rejection).
  2. Correction = (reference_yaw - current_yaw) → converted to pixels via fisheye model.
  3. P gain scales the correction; D gain damps rapid dx changes (prevents oscillation).
  4. dx is averaged over the last 3 frames (50% recent, 30% middle, 20% oldest) for smooth motion.
  5. Output is a 4:3 center-cropped sliding window (out_w × h, default 960 × input_h).

Parameters:
  fov_horizontal_deg (float): camera horizontal FOV. Default 100.0 (Mobius).
  max_margin_px (int):        hard clamp headroom in pixels. Default 80.
  out_w (int):                explicit output crop width. Default 960 (4:3 at 720h).
  yaw_lag_frames (int):       frame buffer for gimbal lag compensation. Default 0.
  reference_alpha (float):    EMA update rate for baseline yaw. Range 0.0–1.0.
                              Higher = faster return to center. Default 0.08.
  p_gain (float):             proportional gain on dx correction. 1.0 = full fisheye correction.
                              Default 1.0.
  d_gain (float):             derivative gain on dx rate-of-change (damping). Default 0.2.
  show_annotations (bool):    show anchor crosshair + yaw/dx text. Default True.
"""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class YawStabilizer(Node):
    def __init__(self):
        super().__init__('yaw_stabilizer')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('fov_horizontal_deg', 100.0)
        self.declare_parameter('max_margin_px',      80)
        self.declare_parameter('out_w',              960)
        self.declare_parameter('yaw_lag_frames',     0)
        self.declare_parameter('reference_alpha',    0.08)
        self.declare_parameter('p_gain',             1.0)
        self.declare_parameter('d_gain',             0.2)
        self.declare_parameter('show_annotations',   True)

        self._fov_deg          = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._max_margin_px    = self.get_parameter('max_margin_px').get_parameter_value().integer_value
        self._out_w            = self.get_parameter('out_w').get_parameter_value().integer_value
        self._lag_frames       = self.get_parameter('yaw_lag_frames').get_parameter_value().integer_value
        self._ref_alpha        = self.get_parameter('reference_alpha').get_parameter_value().double_value
        self._p_gain           = self.get_parameter('p_gain').get_parameter_value().double_value
        self._d_gain           = self.get_parameter('d_gain').get_parameter_value().double_value
        self._show_annotations = self.get_parameter('show_annotations').get_parameter_value().bool_value

        # ------------------------------------------------------------------
        # Subscriptions and publisher
        # ------------------------------------------------------------------
        self.create_subscription(
            CompressedImage, '/rs_corrected_frame/compressed', self.frame_callback, 10)
        self.create_subscription(
            Vector3Stamped, '/gimbal/angles', self.gimbal_callback, 10)

        self.pub = self.create_publisher(CompressedImage, '/yaw_stabilized/compressed', 10)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._current_yaw  = 0.0
        self._reference_yaw = None   # initialised on first gimbal reading
        self._prev_dx_raw  = 0.0    # for D term
        self._dx_history: deque = deque(maxlen=3)  # for 3-frame weighted averaging
        self._frame_buffer: deque = deque()

        self.get_logger().info(
            f"Yaw stabilizer started — EMA + PD with 3-frame weighted averaging, "
            f"ref_alpha={self._ref_alpha}, p={self._p_gain}, d={self._d_gain}, "
            f"margin={self._max_margin_px}px, out_w={self._out_w}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def gimbal_callback(self, msg: Vector3Stamped):
        """Update current yaw and advance the slow EMA reference baseline."""
        self._current_yaw = msg.vector.z
        if self._reference_yaw is None:
            self._reference_yaw = self._current_yaw  # cold-start: align reference to first reading
        else:
            # EMA: reference drifts toward current_yaw at rate ref_alpha
            # Higher alpha = faster return to center
            self._reference_yaw = (
                (1.0 - self._ref_alpha) * self._reference_yaw
                + self._ref_alpha * self._current_yaw
            )

    def frame_callback(self, msg: CompressedImage):
        """Buffer frames for lag compensation, then process."""
        if self._lag_frames > 0:
            self._frame_buffer.append(msg)
            if len(self._frame_buffer) <= self._lag_frames:
                return
            msg = self._frame_buffer.popleft()
        self._process(msg)

    # ------------------------------------------------------------------
    # Processing
    # ------------------------------------------------------------------

    def _process(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        if self._reference_yaw is None:
            return  # no gimbal data yet

        h, w = frame.shape[:2]
        out_w = min(self._out_w, w)  # never wider than input

        # ------------------------------------------------------------------
        # PD control on pixel offset
        # ------------------------------------------------------------------
        correction_deg = self._reference_yaw - self._current_yaw

        fov_rad = math.radians(self._fov_deg)
        f_eq = (w / 2.0) / (fov_rad / 2.0)           # equidistant focal length
        dx_raw = f_eq * math.tan(math.radians(correction_deg))

        # P term
        dx_p = self._p_gain * dx_raw

        # D term — damps rapid changes in the correction
        dx_delta = dx_raw - self._prev_dx_raw
        dx_controlled = dx_p - self._d_gain * dx_delta
        self._prev_dx_raw = dx_raw

        # Hard clamp to ±max_margin_px
        m = float(self._max_margin_px)
        dx_controlled = max(-m, min(m, dx_controlled))

        # Average dx over 3 frames: 50% recent, 30% middle, 20% oldest (weighted for responsiveness)
        self._dx_history.append(dx_controlled)
        if len(self._dx_history) == 3:
            dx_final = 0.80 * self._dx_history[2] + 0.15 * self._dx_history[1] + 0.05 * self._dx_history[0]
        elif len(self._dx_history) == 2:
            dx_final = 0.60 * self._dx_history[1] + 0.40 * self._dx_history[0]
        elif self._dx_history:
            dx_final = self._dx_history[0]
        else:
            dx_final = 0.0

        # ------------------------------------------------------------------
        # Sliding crop
        # ------------------------------------------------------------------
        cx = w // 2 - int(dx_final)
        x0 = max(0, min(cx - out_w // 2, w - out_w))
        frame_out = frame[:, x0:x0 + out_w].copy()

        # ------------------------------------------------------------------
        # Annotations: three crosshairs at fixed INPUT positions (0.25w, 0.5w, 0.75w)
        # Mapped to output coords — dots move as crop window slides
        # ------------------------------------------------------------------
        if self._show_annotations:
            anchor_y = h // 2

            # Crosshairs at fixed input frame positions, mapped to output coordinates
            for frac, color in [(0.25, (100, 100, 255)), (0.50, (0, 255, 255)), (0.75, (255, 100, 100))]:
                input_x = int(frac * w)     # Fixed position in input frame
                output_x = input_x - x0     # Map to output crop coordinates

                # Only draw if visible in output window
                if 0 <= output_x < out_w:
                    cv2.circle(frame_out, (output_x, anchor_y), 6, color, 2)
                    cv2.line(frame_out, (output_x - 10, anchor_y), (output_x + 10, anchor_y), color, 1)
                    cv2.line(frame_out, (output_x, anchor_y - 10), (output_x, anchor_y + 10), color, 1)

            cv2.putText(frame_out, f"Yaw: {self._current_yaw:.1f} deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_out, f"dx: {dx_final:.1f}px  (ref: {self._reference_yaw:.1f})",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            # Debug: show dx history (most recent, 2nd, 3rd)
            dx_str = "dx_hist: ["
            if len(self._dx_history) >= 1:
                dx_str += f"{self._dx_history[-1]:.1f}"
            if len(self._dx_history) >= 2:
                dx_str += f", {self._dx_history[-2]:.1f}"
            if len(self._dx_history) >= 3:
                dx_str += f", {self._dx_history[-3]:.1f}"
            dx_str += "]"
            cv2.putText(frame_out, dx_str, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        # ------------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------------
        _, buffer = cv2.imencode('.jpg', frame_out)
        out_msg = CompressedImage()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.format = 'jpeg'
        out_msg.data = np.array(buffer).tobytes()
        self.pub.publish(out_msg)


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