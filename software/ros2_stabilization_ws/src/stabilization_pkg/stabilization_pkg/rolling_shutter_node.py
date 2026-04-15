#!/usr/bin/env python3
"""
Rolling Shutter Correction Node

Corrects rolling shutter distortion caused by the camera's row-by-row readout.
During yaw rotation, each row was captured at a slightly different angle, causing
the image to appear skewed/leaned. Yaw is the dominant axis here since roll/pitch
are corrected mechanically by the gimbal.

Applies a per-row horizontal warp using cv2.remap().

THREE INPUT MODES (set via 'input_mode' parameter):
  'capture'    -- opens the camera directly with cv2.VideoCapture (V4L2 + MJPG),
                  no usb_cam node required. Publishes corrected frames as
                  CompressedImage on /rs_corrected_frame/compressed.
  'compressed' -- subscribes to a CompressedImage topic (e.g. from the EMA
                  stabilization node). Publishes CompressedImage output.
  'raw'        -- subscribes to a sensor_msgs/Image topic. Uses cv_bridge.
                  Publishes Image output.

TWO CORRECTION MODES (set via 'mode' parameter):
  'optical_flow' -- software-only. Divides the frame into horizontal bands,
                    estimates optical flow in each band independently, fits a
                    linear model across band dx values to extract the rolling
                    shutter slope. No gimbal required.
  'compass'      -- uses live gimbal yaw rate (/gimbal/angles) plus the fisheye
                    projection model to compute the per-row shift directly.
                    More accurate when gimbal data is available.

LENS MODEL:
  This camera uses a ~170° FOV fisheye lens. The equidistant projection model
  is assumed: r = f_eq * theta, where f_eq = (width/2) / (fov_rad/2).
  A pinhole lens would use tan(theta) instead.

  TODO: perform a proper OpenCV fisheye calibration (cv2.fisheye.calibrate)
  with a checkerboard to get exact k1/k2/k3/k4 distortion coefficients.
  This would improve accuracy near the frame edges where the equidistant
  approximation is weakest.

READOUT TIME CALIBRATION:
  Calibrated at: 1920x1080 16:9, 30 fps, 200 Hz LED
  (Operating resolution for this project: 1280x960 4:3 — different from calibration)

  Measurement: 172 pixels per bright+dark pair = one full LED cycle (5 ms)
  Calculated:  readout_time_1080p = 1080 / (200 * 172) = 31.4 ms
  Per-row:     PER_ROW_READOUT_MS = 31.4 / 1080 ≈ 0.02907 ms/row

  Readout time for any resolution scales automatically from frame height:
    readout_time_ms(H) = PER_ROW_READOUT_MS * H

  Resolutions and their computed readout times:
    1920x1080 (16:9) → 31.4 ms  ← calibration reference
    1280x960  (4:3)  → 27.9 ms  ← primary operating resolution
    1280x720  (16:9) → 20.9 ms
     848x480  (16:9) → 13.9 ms
     640x480  (4:3)  → 14.0 ms

  Note: if the camera bins rows differently between 4:3 and 16:9 modes, the
  per-row constant may differ and should be re-verified with a separate LED test.

Topics published:
  /rs_corrected_frame/compressed  (CompressedImage) — capture and compressed modes
  /rs_corrected_frame/image_raw   (Image)           — raw mode

Parameters:
  input_mode          (str)   -- 'capture', 'compressed', or 'raw'. Default: 'capture'
  mode                (str)   -- 'optical_flow' or 'compass'. Default: 'optical_flow'
  fov_horizontal_deg  (float) -- camera horizontal FOV in degrees. Default: 170.0
  n_bands             (int)   -- horizontal bands for optical_flow mode. Default: 4
  min_features        (int)   -- min tracked points per band. Default: 8
  video_device        (str)   -- V4L2 device path for 'capture' mode. Default: '/dev/video18'
  image_width         (int)   -- capture resolution width.  Default: 1280
  image_height        (int)   -- capture resolution height. Default: 960
  capture_fps         (float) -- capture frame rate. Default: 30.0
"""

import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3Stamped

# ---------------------------------------------------------------------------
# Calibration constants — LED test at 1920x1080 16:9, 200 Hz
# ---------------------------------------------------------------------------
_CALIB_HEIGHT_PX = 1080
_CALIB_LED_HZ    = 200
_CALIB_BAND_PAIR = 172   # pixels per full bright+dark cycle

# Hardware constant: time per sensor row (ms). Scales to any resolution.
# Derived: (1080 / (200 * 172) * 1000) / 1080 ≈ 0.02907 ms/row
PER_ROW_READOUT_MS = (
    _CALIB_HEIGHT_PX / (_CALIB_LED_HZ * _CALIB_BAND_PAIR) * 1000.0
) / _CALIB_HEIGHT_PX


class RollingShutterNode(Node):

    def __init__(self):
        super().__init__('rolling_shutter_node')

        self.declare_parameter('input_mode',          'capture')
        self.declare_parameter('mode',                'optical_flow')
        self.declare_parameter('fov_horizontal_deg',  170.0)
        self.declare_parameter('n_bands',             4)
        self.declare_parameter('min_features',        8)
        self.declare_parameter('video_device',        '/dev/video18')
        self.declare_parameter('image_width',         1280)
        self.declare_parameter('image_height',        960)
        self.declare_parameter('capture_fps',         30.0)

        self._input_mode = self.get_parameter('input_mode').get_parameter_value().string_value
        self._mode       = self.get_parameter('mode').get_parameter_value().string_value
        self._fov_deg    = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._n_bands    = self.get_parameter('n_bands').get_parameter_value().integer_value
        self._min_feat   = self.get_parameter('min_features').get_parameter_value().integer_value

        if self._input_mode not in ('capture', 'compressed', 'raw'):
            self.get_logger().error(f"Unknown input_mode '{self._input_mode}'.")
            raise ValueError(self._input_mode)
        if self._mode not in ('optical_flow', 'compass'):
            self.get_logger().error(f"Unknown mode '{self._mode}'.")
            raise ValueError(self._mode)

        # Previous grayscale frame for optical flow
        self._prev_gray = None

        # Gimbal yaw history: [(timestamp_sec, yaw_deg), ...]
        # Yaw is continuous — NOT wrapped at ±360°
        self._yaw_history = []

        if self._mode == 'compass':
            self.create_subscription(
                Vector3Stamped, '/gimbal/angles', self._gimbal_cb, 10
            )

        # ------------------------------------------------------------------
        # Input wiring
        # ------------------------------------------------------------------
        if self._input_mode == 'capture':
            self._setup_capture()
        elif self._input_mode == 'compressed':
            self.create_subscription(
                CompressedImage,
                '/stabilized_frame/compressed',
                self._compressed_frame_cb,
                10
            )
        else:  # raw
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
            self.create_subscription(
                Image, '/usb_cam/image_raw', self._raw_frame_cb, 10
            )

        # ------------------------------------------------------------------
        # Output publisher
        # ------------------------------------------------------------------
        if self._input_mode == 'raw':
            self._pub = self.create_publisher(Image, '/rs_corrected_frame/image_raw', 10)
        else:
            self._pub = self.create_publisher(
                CompressedImage, '/rs_corrected_frame/compressed', 10
            )

        self.get_logger().info(
            f"Rolling shutter node started — input_mode='{self._input_mode}', "
            f"mode='{self._mode}', FOV={self._fov_deg}°, bands={self._n_bands}"
        )

    # ------------------------------------------------------------------
    # Capture mode setup (mirrors realtime_stabilization.py approach)
    # ------------------------------------------------------------------

    def _setup_capture(self):
        device = self.get_parameter('video_device').get_parameter_value().string_value
        width  = self.get_parameter('image_width').get_parameter_value().integer_value
        height = self.get_parameter('image_height').get_parameter_value().integer_value
        fps    = self.get_parameter('capture_fps').get_parameter_value().double_value

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera opened: {device} at {actual_w}x{actual_h} @ {actual_fps} fps"
        )

        # Seed prev_gray with the first frame so optical flow is ready from frame 2
        success, first_frame = self._cap.read()
        if not success:
            self.get_logger().error("Failed to read first frame from camera!")
            return
        self._prev_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

        self.create_timer(1.0 / fps, self._capture_cb)

    def _capture_cb(self):
        success, frame = self._cap.read()
        if not success:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=1.0)
            return

        corrected = self._process_frame(frame)
        if corrected is None:
            corrected = frame

        _, buffer = cv2.imencode('.jpg', corrected)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data   = np.array(buffer).tobytes()
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _compressed_frame_cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed frame', throttle_duration_sec=2.0)
            return

        corrected = self._process_frame(frame)
        if corrected is None:
            self._pub.publish(msg)
            return

        _, buffer = cv2.imencode('.jpg', corrected)
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data   = np.array(buffer).tobytes()
        self._pub.publish(out)

    def _raw_frame_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}', throttle_duration_sec=2.0)
            return

        corrected = self._process_frame(frame)
        if corrected is None:
            self._pub.publish(msg)
            return

        out = self._bridge.cv2_to_imgmsg(corrected, encoding='bgr8')
        out.header = msg.header
        self._pub.publish(out)

    # ------------------------------------------------------------------
    # Shared processing — returns corrected frame, or None if no change
    # ------------------------------------------------------------------

    def _process_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._mode == 'compass':
            slope = self._slope_from_compass(h, w)
        else:
            slope = self._slope_from_optical_flow(curr_gray, h, w)

        if abs(slope) < 1e-4:
            return None

        return self._apply_remap(frame, slope, h, w)

    # ------------------------------------------------------------------
    # Gimbal callback
    # ------------------------------------------------------------------

    def _gimbal_cb(self, msg: Vector3Stamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # vector.z = yaw degrees, continuous, NOT wrapped at ±360°
        self._yaw_history.append((t, msg.vector.z))
        if len(self._yaw_history) > 2:
            self._yaw_history.pop(0)

    # ------------------------------------------------------------------
    # Fisheye equidistant focal length
    # ------------------------------------------------------------------

    def _f_eq(self, width: int) -> float:
        return (width / 2.0) / (math.radians(self._fov_deg) / 2.0)

    # ------------------------------------------------------------------
    # Readout time
    # ------------------------------------------------------------------

    @staticmethod
    def _readout_time_sec(height: int) -> float:
        return PER_ROW_READOUT_MS * height / 1000.0

    # ------------------------------------------------------------------
    # Slope estimation — compass mode
    # ------------------------------------------------------------------

    def _slope_from_compass(self, h: int, w: int) -> float:
        if len(self._yaw_history) < 2:
            return 0.0
        t1, yaw1 = self._yaw_history[0]
        t2, yaw2 = self._yaw_history[1]
        dt = t2 - t1
        if dt <= 0.0:
            return 0.0
        yaw_rate_px_s = self._f_eq(w) * math.radians((yaw2 - yaw1) / dt)
        return yaw_rate_px_s * self._readout_time_sec(h) / h

    # ------------------------------------------------------------------
    # Slope estimation — optical flow mode
    # ------------------------------------------------------------------

    def _slope_from_optical_flow(self, curr_gray: np.ndarray, h: int, w: int) -> float:
        """
        Divides the frame into n_bands horizontal strips and runs Lucas-Kanade
        optical flow independently in each strip. Fits a linear model to the
        per-band median dx values:

          dx(row) = intercept + slope * row
            intercept = global frame translation
            slope     = rolling shutter distortion in pixels/row

        Interpolation note:
          Linear interpolation between band centers is sufficient for small
          distortions. For large yaw rates or long readout times, a cubic
          spline across band centers would reduce edge artifacts.
        """
        if self._prev_gray is None:
            self._prev_gray = curr_gray
            return 0.0

        band_h = h // self._n_bands
        row_centers = []
        dx_medians  = []

        for i in range(self._n_bands):
            y0 = i * band_h
            y1 = min(y0 + band_h, h)

            pts = cv2.goodFeaturesToTrack(
                self._prev_gray[y0:y1, :],
                maxCorners=60, qualityLevel=0.01, minDistance=10, blockSize=3
            )
            if pts is None or len(pts) < self._min_feat:
                continue

            # Lift band-relative y coords to full-frame coords before tracking
            pts_full = pts.copy()
            pts_full[:, 0, 1] += y0

            curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, curr_gray, pts_full, None
            )
            good = np.where(status.flatten() == 1)[0]
            if len(good) < self._min_feat:
                continue

            dx_vals = (curr_pts[good] - pts_full[good])[:, 0, 0]
            row_centers.append((y0 + y1) / 2.0)
            dx_medians.append(float(np.median(dx_vals)))

        self._prev_gray = curr_gray

        if len(row_centers) < 2:
            return 0.0

        x = np.array(row_centers)
        y = np.array(dx_medians)
        A = np.vstack([np.ones_like(x), x]).T
        coeffs, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        return float(coeffs[1])  # slope = pixels/row

    # ------------------------------------------------------------------
    # Per-row remap
    # ------------------------------------------------------------------

    @staticmethod
    def _apply_remap(frame: np.ndarray, slope: float, h: int, w: int) -> np.ndarray:
        """
        Inverse remap: map_x[row, col] = col - slope*row
        Shift grows linearly from 0 at the top row to slope*(h-1) at the bottom.
        """
        rows  = np.arange(h, dtype=np.float32).reshape(h, 1)
        cols  = np.arange(w, dtype=np.float32).reshape(1, w)
        map_x = (cols - slope * rows).astype(np.float32)
        map_y = np.broadcast_to(rows, (h, w)).copy().astype(np.float32)
        return cv2.remap(frame, map_x, map_y,
                         interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RollingShutterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
