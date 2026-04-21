"""
Demo Comparison Node

Subscribes to the three pipeline stages and publishes side-by-side comparison frames
for display and recording. All panels are center-cropped to 4:3 (out_w × h) so the
side-by-side is a fair comparison with no letterboxing or distortion.

Subscriptions:
  /camera/raw/compressed          — original camera input (from rolling_shutter_node)
  /rs_corrected_frame/compressed  — rolling shutter corrected (from rolling_shutter_node)
  /yaw_stabilized/compressed      — final output (from yaw_stabilizer, already 4:3)

Publications:
  /demo_comparison/compressed      — 2-panel: raw | yaw_stabilized (2×out_w wide)
  /demo_full_pipeline/compressed   — 3-panel: raw | rs_corrected | yaw_stabilized (3×out_w wide)

All panels are resized to the same width (out_w) and height before compositing.
Output frames are published at the rate incoming yaw frames arrive (~30 fps).

Parameters:
  out_w (int):             width of each panel in pixels. Default 960 (4:3 at 720h).
  show_annotations (bool): draw panel labels on comparison frames. Default True.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def _center_crop_4x3(frame: np.ndarray, out_w: int) -> np.ndarray:
    """Center-crop a frame to out_w wide, preserving full height."""
    h, w = frame.shape[:2]
    if w == out_w:
        return frame
    x0 = max(0, (w - out_w) // 2)
    x1 = x0 + out_w
    return frame[:, x0:min(x1, w)].copy()


def _decode(msg: CompressedImage):
    """Decode a CompressedImage message to a BGR ndarray, or return None."""
    np_arr = np.frombuffer(msg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


class DemoComparisonNode(Node):
    def __init__(self):
        super().__init__('demo_comparison_node')

        self.declare_parameter('out_w',            960)
        self.declare_parameter('show_annotations', True)

        self._out_w            = self.get_parameter('out_w').get_parameter_value().integer_value
        self._show_annotations = self.get_parameter('show_annotations').get_parameter_value().bool_value

        # Latest frames from each stage (None until first message arrives)
        self._raw: np.ndarray | None = None
        self._rs:  np.ndarray | None = None
        self._yaw: np.ndarray | None = None

        self.create_subscription(
            CompressedImage, '/camera/raw/compressed',
            self._raw_cb, 10)
        self.create_subscription(
            CompressedImage, '/rs_corrected_frame/compressed',
            self._rs_cb, 10)
        self.create_subscription(
            CompressedImage, '/yaw_stabilized/compressed',
            self._yaw_cb, 10)

        self._pub2 = self.create_publisher(
            CompressedImage, '/demo_comparison/compressed', 10)
        self._pub3 = self.create_publisher(
            CompressedImage, '/demo_full_pipeline/compressed', 10)

    # ------------------------------------------------------------------
    # Subscription callbacks — just store latest frame
    # ------------------------------------------------------------------

    def _raw_cb(self, msg: CompressedImage):
        frame = _decode(msg)
        if frame is not None:
            self._raw = _center_crop_4x3(frame, self._out_w)

    def _rs_cb(self, msg: CompressedImage):
        frame = _decode(msg)
        if frame is not None:
            self._rs = _center_crop_4x3(frame, self._out_w)

    def _yaw_cb(self, msg: CompressedImage):
        frame = _decode(msg)
        if frame is not None:
            self._yaw = _center_crop_4x3(frame, self._out_w)
        # Composite and publish whenever a new yaw frame arrives
        self._publish_comparisons()

    # ------------------------------------------------------------------
    # Compositing
    # ------------------------------------------------------------------

    def _label(self, frame: np.ndarray, text: str) -> np.ndarray:
        """Draw a panel label if annotations are enabled. Returns the frame (modified in place)."""
        if self._show_annotations:
            cv2.putText(frame, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return frame

    def _resize_to(self, frame: np.ndarray, w: int, h: int) -> np.ndarray:
        """Resize frame to exact (w, h) if needed."""
        if frame.shape[1] == w and frame.shape[0] == h:
            return frame
        return cv2.resize(frame, (w, h))

    def _publish_comparisons(self):
        if self._yaw is None:
            return

        h, w = self._yaw.shape[:2]   # reference size — all panels match this

        # ---- 2-panel: raw | yaw ----------------------------------------
        if self._raw is not None:
            raw_p  = self._label(self._resize_to(self._raw.copy(), w, h), 'Original')
            yaw_p  = self._label(self._yaw.copy(), 'Yaw Stabilized')
            comp2  = cv2.hconcat([raw_p, yaw_p])
            self._pub2.publish(self._encode(comp2))

        # ---- 3-panel: raw | rs | yaw -----------------------------------
        if self._raw is not None and self._rs is not None:
            raw_p  = self._label(self._resize_to(self._raw.copy(), w, h), 'Original')
            rs_p   = self._label(self._resize_to(self._rs.copy(),  w, h), 'RS Corrected')
            yaw_p  = self._label(self._yaw.copy(), 'Yaw Stabilized')
            # Scale down to 1/2 height so the 3-panel fits on a normal monitor
            panels = [cv2.resize(p, (w // 2, h // 2)) for p in [raw_p, rs_p, yaw_p]]
            comp3  = cv2.hconcat(panels)
            self._pub3.publish(self._encode(comp3))

    def _encode(self, frame: np.ndarray) -> CompressedImage:
        _, buf = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = np.array(buf).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DemoComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
