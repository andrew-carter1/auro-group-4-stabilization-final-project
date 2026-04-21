"""
Yaw Stabilizer — EMA Reference + PD Control + Soft Clamping

Subscribes to rolling shutter corrected frames and gimbal yaw. Applies yaw stabilization
via horizontal pixel translation using a fisheye equidistant projection model.

Pipeline position:
  /rs_corrected_frame/compressed  →  [yaw_stabilizer]  →  /yaw_stabilized/compressed

Stabilization logic:
  1. A slow EMA tracks the "intended" camera heading (reference_alpha controls drift rate).
     Low alpha = slow reference, aggressive correction of jitter.
     High alpha = fast reference, passes through more intentional motion.
  2. Correction = (reference_yaw - current_yaw) → converted to pixels via fisheye model.
  3. P gain scales the correction; D gain damps rapid dx changes (prevents oscillation).
  4. Tanh soft clamping: correction smoothly saturates at ±max_margin_px instead of
     hard-clipping. Naturally pins at the frame edge without a visible snap.
  5. Output is a 4:3 center-cropped sliding window (out_w × h, default 960 × input_h).

Parameters:
  fov_horizontal_deg (float): camera horizontal FOV. Default 150.0 (Mobius).
  max_margin_px (int):        half-width of crop headroom in pixels. Default 80.
                              At 1280 wide: output is 1280 - 2*80 = 1120 px (or set out_w directly).
  out_w (int):                explicit output crop width. Default 960 (4:3 at 720h).
  yaw_lag_frames (int):       frame buffer for gimbal lag compensation. Default 0.
  reference_alpha (float):    EMA update rate for baseline yaw. Range 0.0–1.0.
                              Lower = slower reference, more jitter correction. Default 0.02.
  p_gain (float):             proportional gain on dx correction. 1.0 = full fisheye correction.
                              Lower values reduce overcorrection. Default 1.0.
  d_gain (float):             derivative gain on dx rate-of-change (damping). Default 0.0 (off).
                              Increase if you see oscillation or overshoot during pans.
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
        self.declare_parameter('fov_horizontal_deg', 150.0)
        self.declare_parameter('max_margin_px',      80)
        self.declare_parameter('out_w',              960)
        self.declare_parameter('yaw_lag_frames',     0)
        self.declare_parameter('reference_alpha',    0.02)
        self.declare_parameter('p_gain',             1.0)
        self.declare_parameter('d_gain',             0.0)
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
        self._frame_buffer: deque = deque()

        self.get_logger().info(
            f"Yaw stabilizer started — ref_alpha={self._ref_alpha}, "
            f"p={self._p_gain}, d={self._d_gain}, "
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
            # Slow EMA: reference drifts toward current_yaw at rate ref_alpha
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
        f_eq = (w / 2.0) / (fov_rad / 2.0)           # ~489 px at 1280w, 150° FOV
        dx_raw = f_eq * math.tan(math.radians(correction_deg))

        # P term
        dx_p = self._p_gain * dx_raw

        # D term — damps rapid changes in the correction
        dx_delta = dx_raw - self._prev_dx_raw
        dx_controlled = dx_p - self._d_gain * dx_delta
        self._prev_dx_raw = dx_raw

        # Soft clamping via tanh — smoothly saturates at ±max_margin_px
        m = self._max_margin_px
        if m > 0:
            dx_final = m * math.tanh(dx_controlled / m)
        else:
            dx_final = dx_controlled

        # ------------------------------------------------------------------
        # Sliding crop
        # ------------------------------------------------------------------
        cx = w // 2 - int(dx_final)
        x0 = max(0, min(cx - out_w // 2, w - out_w))
        frame_out = frame[:, x0:x0 + out_w].copy()

        # ------------------------------------------------------------------
        # Annotations
        # ------------------------------------------------------------------
        if self._show_annotations:
            anchor_x = int(w // 2 - x0)   # original center mapped into output
            anchor_y = h // 2
            # Crosshair at world anchor point
            cv2.circle(frame_out, (anchor_x, anchor_y), 8, (0, 255, 255), 2)
            cv2.line(frame_out, (anchor_x - 14, anchor_y), (anchor_x + 14, anchor_y),
                     (0, 255, 255), 1)
            cv2.line(frame_out, (anchor_x, anchor_y - 14), (anchor_x, anchor_y + 14),
                     (0, 255, 255), 1)
            cv2.putText(frame_out, f"Yaw: {self._current_yaw:.1f} deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_out, f"dx: {dx_final:.1f}px  (ref: {self._reference_yaw:.1f})",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

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
