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
  'optical_flow' -- software-only. Tracks features across the full frame using
                    Lucas-Kanade + estimateAffinePartial2D to get a robust global
                    horizontal shift (dx) per frame. Converts dx to a pixel/second
                    velocity, then uses the calibrated readout time to compute the
                    per-row rolling shutter slope. No gimbal required.
  'compass'      -- uses live gimbal yaw rate (/gimbal/angles) plus the fisheye
                    projection model to compute the per-row shift directly.
                    More accurate when gimbal data is available.

UNIFIED SLOPE FORMULA (same for both modes):
  Global pixel velocity (px/s):
    optical_flow:  px_per_sec = dx_per_frame * fps
    compass:       px_per_sec = f_eq * yaw_rate_rad_s

  slope_px_per_row = px_per_sec * readout_time_sec / frame_height

  Both modes run through the same clamp and remap.

LENS MODEL:
  This camera uses a ~150° FOV fisheye lens. The equidistant projection model
  is assumed: r = f_eq * theta, where f_eq = (width/2) / (fov_rad/2).
  A pinhole lens would use tan(theta) instead.

  TODO: perform a proper OpenCV fisheye calibration (cv2.fisheye.calibrate)
  with a checkerboard to get exact k1/k2/k3/k4 distortion coefficients.
  This would improve accuracy near the frame edges where the equidistant
  approximation is weakest. Replace _f_eq() with the actual dr/dtheta
  evaluated at each row's angle from the optical axis.

READOUT TIME CALIBRATION:
  LED measured at 200 Hz. Each test: count vertical pixels spanning 4 bright
  bands + 4 dark bands = 4 full cycles = 20 ms of sensor readout.

  Measured pixels (4 cycles) per resolution:
    640×480:  242 px    720×480:  279 px    848×480:  320 px
    1024×576: 389 px    1024×768: 390 px    1280×960: 486 px
    1920×1080: 743 px
  (1080×720 excluded — K value was a clear outlier, likely a bad measurement)

  Per-row readout time is proportional to 1/width (wider rows take longer
  to clock out). This gives a single constant K_MS_PX:

    per_row_ms           = K_MS_PX / width
    readout_time_ms(H,W) = K_MS_PX * H / W

  K derived per resolution: 52.89, 51.61, 53.00, 52.58, 52.67, 51.69
  Mean K_MS_PX ≈ 52.4 ms

  Primary operating resolution: 1280×720 (16:9, 30 fps)
    readout_time = 52.4 × 720 / 1280 ≈ 29.5 ms  (fits within 33.3 ms frame period)

Topics published:
  /rs_corrected_frame/compressed  (CompressedImage) — capture and compressed modes
  /rs_corrected_frame/image_raw   (Image)           — raw mode
  /rs_diagnostics                 (String)          — diagnostics mode only; comma-separated values
                                                      (yaw, yaw_rate, compass_shift, flow_shift, applied)

Parameters:
  input_mode          (str)   -- 'capture', 'compressed', or 'raw'. Default: 'capture'
  mode                (str)   -- 'optical_flow' or 'compass'. Default: 'optical_flow'
  fov_horizontal_deg  (float) -- camera horizontal FOV in degrees. Default: 150.0
  min_features        (int)   -- min tracked points for optical_flow mode. Default: 12
  video_device        (str)   -- V4L2 device path for 'capture' mode. Default: '/dev/video4'
  image_width         (int)   -- capture resolution width.  Default: 1280
  image_height        (int)   -- capture resolution height. Default: 720
  capture_fps         (float) -- capture frame rate (also used as assumed fps for other
                                 input modes). Default: 30.0
  show_comparison     (bool)  -- side-by-side original vs corrected output (capture mode).
                                 Default: False
  max_shift_pct       (float) -- maximum total pixel shift (top-to-bottom) as a fraction
                                 of frame width. Prevents runaway corrections.
                                 Default: 0.10 (10% of width = 128 px at 1280 wide)
  slope_ema_alpha     (float) -- EMA smoothing on slope. 0.0 = off (raw per-frame slope).
                                 Higher = more smoothing, more lag. Default: 0.0
  compass_delay_sec   (float) -- (compass mode only) seconds to look back in gimbal
                                 history when syncing to a video frame. Default: 0.0
  compass_lag_frames  (int)   -- (compass mode only) number of frames to buffer before
                                 processing. Compensates for gimbal magnetometer filter lag.
                                 Set to ~4 if compass peaks ~133ms behind optical flow.
                                 Default: 0 (no buffer)
  diagnostics         (bool)  -- publish /rs_diagnostics (std_msgs/String) at frame rate
                                 with yaw, yaw_rate, compass_shift, flow_shift, and applied
                                 values. Useful for timing analysis. Default: False
"""

import math
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped

# ---------------------------------------------------------------------------
# Calibration constant — LED measurements at 200 Hz across 7 resolutions
# ---------------------------------------------------------------------------
# The full sensor is always read at all output resolutions; lower resolutions
# are the same sensor data scaled down. Because wider rows have more sensor
# pixels to clock out, per-video-row readout time is proportional to 1/width:
#
#   per_row_ms           = K_MS_PX / width
#   readout_time_ms(H,W) = K_MS_PX * H / W
#
# K derived from 7 resolutions (1080×720 excluded — outlier measurement):
#   640×480: 52.89   720×480: 51.61   848×480: 53.00
#   1024×576/768: 52.58 avg   1280×960: 52.67   1920×1080: 51.69
#   Mean ≈ 52.4 ms
K_MS_PX = 52.4


class RollingShutterNode(Node):

    def __init__(self):
        super().__init__('rolling_shutter_node')

        self.declare_parameter('input_mode',          'capture')
        self.declare_parameter('mode',                'optical_flow')
        self.declare_parameter('fov_horizontal_deg',  150.0)
        self.declare_parameter('min_features',        12)
        self.declare_parameter('video_device',        '/dev/video4')
        self.declare_parameter('image_width',         1280)
        self.declare_parameter('image_height',        720)
        self.declare_parameter('capture_fps',         30.0)
        self.declare_parameter('show_comparison',     False)
        self.declare_parameter('max_shift_pct',       0.10)
        self.declare_parameter('slope_ema_alpha',     0.0)
        self.declare_parameter('compass_delay_sec',   0.0)
        self.declare_parameter('compass_lag_frames',  0)
        self.declare_parameter('diagnostics',         False)
        self.declare_parameter('show_annotations',    True)

        self._input_mode      = self.get_parameter('input_mode').get_parameter_value().string_value
        self._mode            = self.get_parameter('mode').get_parameter_value().string_value
        self._fov_deg         = self.get_parameter('fov_horizontal_deg').get_parameter_value().double_value
        self._min_feat        = self.get_parameter('min_features').get_parameter_value().integer_value
        self._show_comparison = self.get_parameter('show_comparison').get_parameter_value().bool_value
        self._max_shift_pct   = self.get_parameter('max_shift_pct').get_parameter_value().double_value
        self._ema_alpha       = self.get_parameter('slope_ema_alpha').get_parameter_value().double_value
        self._compass_delay   = self.get_parameter('compass_delay_sec').get_parameter_value().double_value
        self._lag_frames      = self.get_parameter('compass_lag_frames').get_parameter_value().integer_value
        self._diagnostics       = self.get_parameter('diagnostics').get_parameter_value().bool_value
        self._show_annotations  = self.get_parameter('show_annotations').get_parameter_value().bool_value
        self._fps               = self.get_parameter('capture_fps').get_parameter_value().double_value

        if self._input_mode not in ('capture', 'compressed', 'raw'):
            self.get_logger().error(f"Unknown input_mode '{self._input_mode}'.")
            raise ValueError(self._input_mode)
        if self._mode not in ('optical_flow', 'compass'):
            self.get_logger().error(f"Unknown mode '{self._mode}'.")
            raise ValueError(self._mode)

        # Previous grayscale frame for optical flow
        self._prev_gray = None

        # EMA state for slope smoothing (used when slope_ema_alpha > 0)
        self._smoothed_slope = 0.0

        # Diagnostics publisher (only active when diagnostics=True)
        if self._diagnostics:
            self._diag_pub = self.create_publisher(String, '/rs_diagnostics', 10)

        # Frame buffer for compass lag compensation (stores (frame, frame_time) tuples)
        self._frame_buffer: deque = deque()

        # Gimbal yaw history: deque of (timestamp_sec, yaw_deg), newest at right.
        # Yaw is continuous — NOT wrapped at ±360°
        # Sized to hold ~10 s of 60 Hz data so the delay buffer can look back far.
        self._yaw_history: deque = deque(maxlen=600)

        # Store applied slope for comparison frame overlay
        self._last_slope = 0.0  # px/row
        self._slope_history: deque = deque(maxlen=3)  # 3-frame weighted averaging for overlay display

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
        # Output publishers
        # ------------------------------------------------------------------
        if self._input_mode == 'raw':
            self._pub = self.create_publisher(Image, '/rs_corrected_frame/image_raw', 10)
        else:
            self._pub = self.create_publisher(
                CompressedImage, '/rs_corrected_frame/compressed', 10
            )
            # Separate comparison topic — side-by-side only when show_comparison=True
            # The primary topic always carries the clean corrected frame for downstream nodes
            self._comparison_pub = self.create_publisher(
                CompressedImage, '/rs_comparison/compressed', 10
            ) if self._show_comparison else None

        # Raw camera frame publisher — the frame as captured, before any RS processing.
        # Published after the lag buffer so it is temporally aligned with rs_corrected_frame.
        # Used by demo_comparison_node for end-to-end before/after display.
        self._raw_pub = self.create_publisher(
            CompressedImage, '/camera/raw/compressed', 10
        ) if self._input_mode == 'capture' else None

        self.get_logger().info(
            f"Rolling shutter node started — input_mode='{self._input_mode}', "
            f"mode='{self._mode}', FOV={self._fov_deg}°, "
            f"max_shift={self._max_shift_pct*100:.0f}%, ema_alpha={self._ema_alpha}"
        )

    # ------------------------------------------------------------------
    # Capture mode setup (mirrors realtime_stabilization.py approach)
    # ------------------------------------------------------------------

    def _setup_capture(self):
        device = self.get_parameter('video_device').get_parameter_value().string_value
        width  = self.get_parameter('image_width').get_parameter_value().integer_value
        height = self.get_parameter('image_height').get_parameter_value().integer_value

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        actual_w   = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
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

        self.create_timer(1.0 / self._fps, self._capture_cb)

    def _capture_cb(self):
        success, frame = self._cap.read()
        if not success:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=1.0)
            return

        frame_time = self.get_clock().now().nanoseconds * 1e-9

        # Buffer frames to compensate for gimbal magnetometer filter lag
        if self._lag_frames > 0:
            self._frame_buffer.append((frame, frame_time))
            if len(self._frame_buffer) <= self._lag_frames:
                return  # still filling buffer
            frame, frame_time = self._frame_buffer.popleft()

        # Publish raw frame (temporally aligned with processed output after lag buffer)
        if self._raw_pub is not None:
            _, raw_buf = cv2.imencode('.jpg', frame)
            raw_msg = CompressedImage()
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.format = 'jpeg'
            raw_msg.data = np.array(raw_buf).tobytes()
            self._raw_pub.publish(raw_msg)

        corrected = self._process_frame(frame, frame_time)
        if corrected is None:
            corrected = frame

        # Always publish clean corrected frame for downstream nodes (e.g. yaw_stabilizer)
        _, buffer = cv2.imencode('.jpg', corrected)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data   = np.array(buffer).tobytes()
        self._pub.publish(msg)

        # Comparison frame published separately so downstream nodes always get clean input
        if self._show_comparison and self._comparison_pub is not None:
            h, w = frame.shape[:2]
            half_w, half_h = w // 2, h // 2
            orig_small = cv2.resize(frame,     (half_w, half_h))
            corr_small = cv2.resize(corrected, (half_w, half_h))
            cv2.putText(orig_small, 'Original',     (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(corr_small, 'RS Corrected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if self._show_annotations:
                # Compute weighted average of slope: 50% recent, 30% middle, 20% oldest
                if len(self._slope_history) == 3:
                    avg_slope = 0.50 * self._slope_history[2] + 0.30 * self._slope_history[1] + 0.20 * self._slope_history[0]
                elif len(self._slope_history) == 2:
                    avg_slope = 0.60 * self._slope_history[1] + 0.40 * self._slope_history[0]
                elif self._slope_history:
                    avg_slope = self._slope_history[0]
                else:
                    avg_slope = 0.0

                # Green vertical reference lines — both panels (at quarter points)
                for panel in (orig_small, corr_small):
                    cv2.line(panel, (half_w // 4, 0),     (half_w // 4, half_h - 1),     (0, 220, 0), 1)
                    cv2.line(panel, (3 * half_w // 4, 0), (3 * half_w // 4, half_h - 1), (0, 220, 0), 1)

                # Yellow slope indicator on orig_small — pivots from center
                mid_x = half_w // 2
                mid_y = half_h // 2
                total_shift = int(avg_slope * (half_h - 1))
                half_shift = total_shift / 2.0
                cv2.line(orig_small,
                         (int(mid_x - half_shift), 5),
                         (int(mid_x + half_shift), half_h - 5),
                         (0, 255, 255), 2)

                # Yellow vertical line on corr_small (perfectly corrected)
                cv2.line(corr_small, (mid_x, 5), (mid_x, half_h - 5), (0, 255, 255), 2)
                cv2.putText(corr_small,
                            f"Shift: {total_shift:+d}px",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            comparison_frame = cv2.hconcat([orig_small, corr_small])
            _, cmp_buf = cv2.imencode('.jpg', comparison_frame)
            cmp_msg = CompressedImage()
            cmp_msg.header.stamp = self.get_clock().now().to_msg()
            cmp_msg.format = 'jpeg'
            cmp_msg.data = np.array(cmp_buf).tobytes()
            self._comparison_pub.publish(cmp_msg)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _compressed_frame_cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed frame', throttle_duration_sec=2.0)
            return

        frame_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        corrected = self._process_frame(frame, frame_time)
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

        frame_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        corrected = self._process_frame(frame, frame_time)
        if corrected is None:
            self._pub.publish(msg)
            return

        out = self._bridge.cv2_to_imgmsg(corrected, encoding='bgr8')
        out.header = msg.header
        self._pub.publish(out)

    # ------------------------------------------------------------------
    # Shared processing — returns corrected frame, or None if no change
    # ------------------------------------------------------------------

    def _process_frame(self, frame: np.ndarray, frame_time: float = 0.0):
        h, w = frame.shape[:2]
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._mode == 'compass':
            slope = self._slope_from_compass(h, w, frame_time)
        else:
            slope = self._slope_from_global_flow(curr_gray, h, w)

        # In diagnostic mode, also compute the other slope for comparison
        diag_flow_slope = 0.0
        if self._diagnostics and self._mode == 'compass':
            diag_flow_slope = self._slope_from_global_flow(curr_gray, h, w)

        raw_slope = slope
        slope = self._clamp_slope(slope, h, w)

        # Optional EMA smoothing across frames
        if self._ema_alpha > 0.0:
            self._smoothed_slope = (self._ema_alpha * slope
                                    + (1.0 - self._ema_alpha) * self._smoothed_slope)
            slope = self._smoothed_slope

        # Publish diagnostics
        if self._diagnostics:
            self._publish_diagnostics(raw_slope, slope, diag_flow_slope, h, w)

        # Store slope for comparison frame overlay (even if negligible)
        self._last_slope = slope
        self._slope_history.append(slope)

        if abs(slope) < 1e-6:
            return None

        return self._apply_remap(frame, slope, h, w)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _publish_diagnostics(self, raw_slope: float, applied_slope: float,
                             flow_slope: float, h: int, w: int):
        """Publish diagnostic values to /rs_diagnostics for tuning."""
        # Get latest yaw info from history
        yaw_deg = 0.0
        yaw_rate_deg_s = 0.0
        if len(self._yaw_history) >= 2:
            t1, y1 = self._yaw_history[-2]
            t2, y2 = self._yaw_history[-1]
            yaw_deg = y2
            dt = t2 - t1
            if dt > 0:
                yaw_rate_deg_s = (y2 - y1) / dt

        # Convert slopes to total pixel shift for readability
        compass_shift_px = raw_slope * (h - 1)
        flow_shift_px = flow_slope * (h - 1)
        applied_shift_px = applied_slope * (h - 1)

        msg = String()
        msg.data = (
            f"yaw={yaw_deg:.1f}deg | "
            f"yaw_rate={yaw_rate_deg_s:.1f}deg/s | "
            f"compass_shift={compass_shift_px:.1f}px | "
            f"flow_shift={flow_shift_px:.1f}px | "
            f"applied={applied_shift_px:.1f}px | "
            f"buffer={len(self._frame_buffer)}/{self._lag_frames}"
        )
        self._diag_pub.publish(msg)

    # ------------------------------------------------------------------
    # Gimbal callback
    # ------------------------------------------------------------------

    def _gimbal_cb(self, msg: Vector3Stamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # vector.z = yaw degrees, continuous, NOT wrapped at ±360°
        # deque(maxlen=300) handles eviction automatically
        self._yaw_history.append((t, msg.vector.z))

    # ------------------------------------------------------------------
    # Fisheye equidistant focal length
    # ------------------------------------------------------------------

    def _f_eq(self, width: int) -> float:
        # TODO: once lens calibration (cv2.fisheye.calibrate) is complete, replace
        # this approximation with the actual dr/dtheta from k1-k4 coefficients,
        # evaluated at each row's angle from the optical axis.
        return (width / 2.0) / (math.radians(self._fov_deg) / 2.0)

    # ------------------------------------------------------------------
    # Readout time
    # ------------------------------------------------------------------

    @staticmethod
    def _readout_time_sec(height: int, width: int) -> float:
        return K_MS_PX * height / (width * 1000.0)

    # ------------------------------------------------------------------
    # Slope clamp
    # ------------------------------------------------------------------

    def _clamp_slope(self, slope: float, h: int, w: int) -> float:
        """Limit total top-to-bottom shift to max_shift_pct * frame_width."""
        limit = self._max_shift_pct * w / h
        return max(-limit, min(limit, slope))

    # ------------------------------------------------------------------
    # Slope estimation — optical flow mode (global shift)
    # ------------------------------------------------------------------

    def _slope_from_global_flow(self, curr_gray: np.ndarray, h: int, w: int) -> float:
        """
        Estimate rolling shutter slope from full-frame global optical flow.

        Uses goodFeaturesToTrack + calcOpticalFlowPyrLK + estimateAffinePartial2D
        across the entire frame (same pattern as realtime_stabilization.py) to get
        a robust global horizontal shift dx per frame.

        Converts to slope via the unified formula:
          px_per_sec = dx * fps
          slope      = px_per_sec * readout_time_sec / height

        The rolling shutter effect adds readout_time/frame_period of the global
        motion as a differential shift between the top and bottom of the frame.

        TODO: once a pixel-to-degree mapping is calibrated for this lens,
        convert dx to an estimated yaw_rate_rad_s and use the compass formula
        for unit-consistent slope (currently uses dx*fps directly).

        Sign note: if panning right produces a positive dx but the remap applies
        correction in the wrong direction, negate dx here (THIS WAS DONE).
        """
        if self._prev_gray is None:
            self._prev_gray = curr_gray
            return 0.0

        pts = cv2.goodFeaturesToTrack(
            self._prev_gray,
            maxCorners=200, qualityLevel=0.01, minDistance=30, blockSize=3
        )
        if pts is None:
            self._prev_gray = curr_gray
            return 0.0

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, curr_gray, pts, None
        )
        good = status.flatten() == 1
        if good.sum() < self._min_feat:
            self._prev_gray = curr_gray
            return 0.0

        matrix, _ = cv2.estimateAffinePartial2D(pts[good], curr_pts[good])
        self._prev_gray = curr_gray

        if matrix is None:
            return 0.0

        dx = matrix[0, 2]           # global horizontal translation (px/frame)
        # ADDED: negate DX
        dx = -dx

        # da = math.atan2(matrix[1, 0], matrix[0, 0])  # in-plane rotation — unused
        px_per_sec = dx * self._fps
        return px_per_sec * self._readout_time_sec(h, w) / h

    # ------------------------------------------------------------------
    # Slope estimation — compass mode
    # ------------------------------------------------------------------

    def _slope_from_compass(self, h: int, w: int, frame_time: float) -> float:
        """
        Compute rolling shutter slope from gimbal yaw rate.

        When compass_lag_frames > 0, the frame has been buffered so the latest
        gimbal entries now correspond to this frame's physical motion — just use
        the two most recent yaw_history entries.

        When compass_lag_frames == 0, falls back to the timestamp-based lookup
        using compass_delay_sec.

        px_per_sec = f_eq * yaw_rate_rad_s
        slope      = px_per_sec * readout_time_sec / height
        """
        if len(self._yaw_history) < 2:
            return 0.0

        if self._lag_frames > 0:
            # Frame is buffered — latest gimbal data corresponds to this frame's motion
            t1, yaw1 = self._yaw_history[-2]
            t2, yaw2 = self._yaw_history[-1]
        else:
            # Timestamp-based lookup (for when no frame buffer is used)
            target_t = frame_time - self._compass_delay
            history = self._yaw_history
            hi = len(history) - 1
            while hi > 0 and history[hi][0] > target_t:
                hi -= 1

            lo = max(hi - 1, 0)
            hi = max(hi, 1)
            t1, yaw1 = history[lo]
            t2, yaw2 = history[hi]

        dt = t2 - t1
        if dt <= 0.0:
            return 0.0

        yaw_rate_rad_s = math.radians((yaw2 - yaw1) / dt)
        px_per_sec = self._f_eq(w) * yaw_rate_rad_s
        return px_per_sec * self._readout_time_sec(h, w) / h

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
