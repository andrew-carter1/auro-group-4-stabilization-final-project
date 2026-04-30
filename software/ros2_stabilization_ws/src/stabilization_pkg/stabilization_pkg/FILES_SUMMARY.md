# Stabilization Package Files Summary

This document provides a high-level overview of all modules in the `stabilization_pkg/stabilization_pkg/` directory.

---

## `__init__.py`
**Purpose:** Package initialization file (empty/minimal).

---

## `stabilization_node.py`
**Class:** `MyFaceDetNode`  
**Type:** Legacy face detection node  
**Primary Role:** Subscribe to raw camera images and detect faces

### Key Features
- Subscribes to `/image_raw` (raw `sensor_msgs/Image` topic)
- Loads a Haar Cascade classifier from `models/haarcascade.xml`
- Detects faces using `cv2.detectMultiScale()` with parameters:
  - `scaleFactor=1.1`
  - `minNeighbors=5`
  - `minSize=(30, 30)`
- Draws green bounding boxes around detected faces
- Publishes annotated image to `output/image` (uncompressed `sensor_msgs/Image`)
- Uses a mutex lock for thread-safe frame processing
- Handles frame drops gracefully with logging

### Status
**Deprecated.** This is an older implementation. The main stabilization pipeline uses `realtime_stabilization.py` and `face_detection_node.py` instead.

---

## `face_detection_node.py`
**Class:** `ImageProcessor`  
**Type:** Modern face detection node  
**Primary Role:** Subscribe to stabilized (compressed) frames and detect faces

### Key Features
- Subscribes to `/stabilized_frame/compressed` (already-stabilized compressed images)
- Loads Haar Cascade from `~/haarcascade_frontalface_default.xml`
- Detects faces with `cv2.detectMultiScale(gray, 1.1, 5)`
- Draws green bounding boxes (2px thick) around detected faces
- Publishes annotated frame as CompressedImage to `/image_with_faces/compressed`
- Preserves the input message's header (timestamp)
- Uses cv_bridge for encoding/decoding

### Performance
- Operates on pre-stabilized frames
- Compresses output as JPEG for network efficiency

---

## `realtime_stabilization.py`
**Class:** `StabilizationNode`  
**Type:** Core real-time stabilization engine  
**Primary Role:** Capture live video and apply EMA-based optical flow stabilization

### Stabilization Algorithm
1. **Feature Detection:** `cv2.goodFeaturesToTrack()` (200 corners, minDistance=30)
2. **Optical Flow Tracking:** Lucas-Kanade (`cv2.calcOpticalFlowPyrLK()`)
3. **Motion Estimation:** `cv2.estimateAffinePartial2D()` → partial affine (prevents shear)
4. **EMA Smoothing:** On cumulative trajectory (not per-frame deltas)
   - X/Y smoothing: `ALPHA_XY = 0.05`
   - Yaw smoothing: `ALPHA_YAW = 0.02`
5. **Warp & Stabilize:** Apply inverse transformation via `cv2.warpAffine()`
6. **Edge Hiding:** `fix_border()` applies 1.1× zoom crop to hide black edges
7. **Sharpening:** Unsharp mask (Gaussian blur at σ=3 with gain 2.0)

### Webcam Setup
- V4L2 device 0 (default USB webcam)
- Resolution: 320×240
- Format: MJPG
- FPS: 15
- Buffer size: 1 (always capture latest frame)

### Publishing
- `/stabilized_frame/compressed` — final stabilized+sharpened output (CompressedImage)
- `/raw_frame/compressed` — raw uncorrected input (CompressedImage)
- `/gimbal/angles` — yaw angle extracted from optical flow (Vector3Stamped, z only)

### Display
- 3-panel side-by-side: Original | Stabilized | Sharpened
- Shake history graph (100-frame deque) with X/Y/Yaw traces
- Displays estimated yaw angle

### Key Parameters
- `MIN_FEATURES = 20` — minimum tracked points required for valid motion estimate
- Frame timing: 30ms timer (≈33 fps intended, actual ~15 fps from camera)

---

## `demo_comparison_node.py`
**Class:** `DemoComparisonNode`  
**Type:** Visualization/demo helper  
**Primary Role:** Composite frames from multiple pipeline stages for side-by-side comparison

### Subscriptions
- `/camera/raw/compressed` — original camera input (from rolling_shutter_node)
- `/rs_corrected_frame/compressed` — rolling shutter corrected output
- `/yaw_stabilized/compressed` — final yaw-stabilized output (4:3 aspect ratio)

### Publishing
- `/demo_comparison/compressed` — 2-panel: raw | yaw_stabilized (2×out_w wide)
- `/demo_full_pipeline/compressed` — 3-panel: raw | rs_corrected | yaw_stabilized (3×out_w wide, scaled to 1/2 height)

### Key Features
- All panels center-cropped to 4:3 aspect ratio for fair comparison
- Resized to matching width/height before compositing
- Optional panel labels ("Original", "RS Corrected", "Yaw Stabilized")
- Publishes whenever a new yaw frame arrives (~30 fps)

### Parameters
- `out_w (int)` — width of each panel. Default: 960 (4:3 at 720h)
- `show_annotations (bool)` — draw panel labels. Default: True

---

## `gimbal_node.py`
**Class:** `GimbalNode`  
**Type:** Hardware interface to Storm BCG gimbal  
**Primary Role:** Read roll/pitch/yaw and gyro rates from gimbal via serial

### Serial Communication
- Protocol: SimpleBGC Serial API v2 (Storm BCG gimbal)
- Command: `CMD_REALTIME_DATA` (0x44) → returns sensor angles + gyro
- Port: `/dev/ttyACM0` (default, configurable)
- Baudrate: 115200 (default)
- Timeout: 15ms (fast enough for 60 Hz, ~5ms for ~68-byte response at 115200 baud)
- Update Rate: 60 Hz via ROS2 timer

### Calibration Constants
- Roll/Pitch: 0.18°/unit (rounded to 1 decimal place)
- Yaw: 0.1°/unit (continuous, NOT wrapped at ±360°)

### Publishing
- `/gimbal/angles` — Vector3Stamped (x=roll°, y=pitch°, z=yaw° continuous)
- `/gimbal/gyro` — Vector3Stamped (x=roll gyro raw, y=pitch gyro raw, z=0)

### Key Features
- Robust packet parsing with CRC validation
- Automatic reconnection on serial failure (throttled warn logs)
- Cold-start delay: 2 seconds after opening port (gimbal initialization)
- Yaw accumulation: Rotating continuously increments past ±360° (must handle in consumers)

### Parameters
- `port (string)` — serial port. Default: '/dev/ttyACM0'
- `baudrate (int)` — baud rate. Default: 115200

---

## `rolling_shutter_node.py`
**Class:** `RollingShutterNode`  
**Type:** Rolling shutter distortion correction  
**Primary Role:** Correct skew caused by camera's row-by-row readout during camera motion

### Problem
During yaw rotation, each row is captured at a slightly different angle, causing the image to appear horizontally skewed/leaned. Roll/Pitch are corrected mechanically by the gimbal, so yaw is the dominant correctable axis.

### Three Input Modes
1. **'capture'** — Opens camera directly (V4L2+MJPG), no usb_cam node required
2. **'compressed'** — Subscribes to CompressedImage topic (e.g., from EMA stabilization)
3. **'raw'** — Subscribes to sensor_msgs/Image, uses cv_bridge

### Two Correction Modes
1. **'optical_flow'** — Software-only global horizontal shift estimation
   - Uses Lucas-Kanade + `estimateAffinePartial2D` (same as realtime_stabilization.py)
   - Converts per-frame pixel shift `dx` to yaw-equivalent slope
   - Works without gimbal hardware
   
2. **'compass'** — Gimbal yaw rate + fisheye projection model
   - Subscribes to `/gimbal/angles` for live yaw rate
   - More accurate when gimbal available
   - Supports lag compensation (`compass_lag_frames`)

### Rolling Shutter Slope Formula (Both Modes)
```
px_per_sec = optical_flow: dx * fps
           | compass:      f_eq * yaw_rate_rad_s

slope_px_per_row = px_per_sec * readout_time_sec / frame_height
```

### Readout Time Calibration
- LED-measured at 200 Hz across 7 resolutions
- Per-row readout time proportional to 1/width
- `readout_time_ms(H,W) = K_MS_PX * H / W`
- `K_MS_PX ≈ 52.4 ms` (mean from measurements)
- At 1280×720 @ 30 fps: readout time ≈ 29.5 ms

### Fisheye Lens Model
- Assumes equidistant projection: `r = f_eq * theta`
- `f_eq = (width/2) / (fov_rad/2)`
- TODO: Replace with proper calibration (cv2.fisheye.calibrate) using checkerboard

### Publishing
- `/rs_corrected_frame/compressed` or `/rs_corrected_frame/image_raw` (depending on input mode)
- `/camera/raw/compressed` — raw input (capture mode only, temporally aligned after lag buffer)
- `/rs_comparison/compressed` — side-by-side before/after (if `show_comparison=True`)
- `/rs_diagnostics` — debugging values (if `diagnostics=True`): yaw, yaw_rate, compass_shift, flow_shift, applied

### Key Parameters
- `input_mode` — 'capture', 'compressed', or 'raw'. Default: 'capture'
- `mode` — 'optical_flow' or 'compass'. Default: 'optical_flow'
- `fov_horizontal_deg` — camera FOV. Default: 150.0
- `image_width`, `image_height` — capture resolution. Default: 1280×720
- `capture_fps` — frame rate. Default: 30.0
- `max_shift_pct` — max total pixel shift as % of width (prevents runaway). Default: 0.10 (10%)
- `slope_ema_alpha` — EMA smoothing on slope. Default: 0.0 (off)
- `compass_delay_sec` — timestamp lookback for gimbal sync. Default: 0.0
- `compass_lag_frames` — frame buffer for gimbal lag (e.g., ~4 if compass peaks ~133ms behind flow). Default: 0
- `diagnostics` — publish `/rs_diagnostics`. Default: False

---

## `yaw_stabilizer.py`
**Class:** `YawStabilizer`  
**Type:** Yaw stabilization via horizontal crop  
**Primary Role:** Apply active yaw stabilization using gimbal feedback + EMA + PD control

### Pipeline Position
```
/rs_corrected_frame/compressed  →  [yaw_stabilizer]  →  /yaw_stabilized/compressed
```

### Stabilization Logic
1. **EMA Reference Baseline** — Tracks "intended" camera heading
   - Higher `reference_alpha` = faster return to center (more aggressive jitter rejection)
   - Cold-start: reference initialized to first gimbal yaw reading
   
2. **PD Control** — Corrects heading deviation
   - `correction_deg = reference_yaw - current_yaw`
   - Convert to pixels via fisheye equidistant model: `dx_raw = f_eq * tan(correction_deg)`
   - P term: `dx_p = p_gain * dx_raw` (proportional correction)
   - D term: `dx_delta = dx_raw - prev_dx_raw` (damps rapid changes)
   - Controlled output: `dx_controlled = dx_p - d_gain * dx_delta`
   - Hard clamp: `±max_margin_px`
   
3. **Weighted Frame Averaging** — Smooth output motion
   - 3-frame weighted average: 80% recent, 15% middle, 5% oldest
   - Fallback weights for fewer frames (60/40 for 2 frames, 100/0 for 1)
   
4. **Sliding Crop Window** — Output is center-cropped 4:3 aspect
   - Output width: `out_w` (default 960)
   - Crop center `cx = w/2 - int(dx_final)` (moves left/right as stabilization adjusts)
   - Extracts `frame[:, x0:x0+out_w]` (always fits within frame bounds)

### Annotations
- Three fixed-position crosshairs (25%, 50%, 75% of input width)
  - Colors: Red (25%), Yellow (50%), Cyan (75%)
  - Mapped to output crop coords — dots move as window slides
- Text overlays:
  - Current yaw angle
  - Final dx pixel offset (and reference yaw)
  - dx history (recent 3 frames)

### Publishing
- `/yaw_stabilized/compressed` — CompressedImage, 4:3 center-cropped, JPEG encoded

### Key Parameters
- `fov_horizontal_deg` — camera horizontal FOV. Default: 100.0 (Mobius camera)
- `max_margin_px` — hard clamp headroom. Default: 80
- `out_w` — explicit output crop width. Default: 960 (4:3 at 720h)
- `yaw_lag_frames` — frame buffer for gimbal lag compensation. Default: 0
- `reference_alpha` — EMA update rate (0.0–1.0, higher = faster return to center). Default: 0.08
- `p_gain` — proportional gain (1.0 = full fisheye correction). Default: 1.0
- `d_gain` — derivative gain (damping). Default: 0.2
- `show_annotations` — display crosshairs + yaw/dx text. Default: True

### Subscriptions
- `/rs_corrected_frame/compressed` — rolling-shutter-corrected frames
- `/gimbal/angles` — gimbal yaw (Vector3Stamped, z component)

---

## Summary of Data Flow

```
[USB Webcam V4L2]
        ↓
[realtime_stabilization.py]  ← primary optical flow stabilization
    ├→ /stabilized_frame/compressed
    ├→ /raw_frame/compressed
    └→ /gimbal/angles (yaw from optical flow)
        ↓
[rolling_shutter_node.py]  ← rolling shutter correction (fisheye-aware)
    ├→ /rs_corrected_frame/compressed
    ├→ /camera/raw/compressed
    └→ /rs_comparison/compressed (optional)
        ↓
[yaw_stabilizer.py]  ← yaw stabilization with gimbal feedback + EMA + PD
    └→ /yaw_stabilized/compressed (final output, 4:3 crop)
        ↓
[demo_comparison_node.py]  ← visualization
    ├→ /demo_comparison/compressed (2-panel: raw | yaw_stabilized)
    └→ /demo_full_pipeline/compressed (3-panel: raw | rs_corrected | yaw_stabilized)

[/dev/ttyACM0 (Serial Gimbal)]
        ↓
[gimbal_node.py]  ← reads roll/pitch/yaw + gyro
    ├→ /gimbal/angles
    └→ /gimbal/gyro
```

---

## Key Algorithmic Choices

### Stabilization (realtime_stabilization.py)
- **EMA on cumulative trajectory** (not per-frame delta) prevents lag accumulation
- **Partial affine** (no shear) prevents unrealistic distortion from outliers
- **1.1× zoom crop** hides warp artifacts at frame edges

### Rolling Shutter Correction (rolling_shutter_node.py)
- **Fisheye equidistant model** approximates wide-angle (150° FOV) lens
- **Per-row linear warp** via cv2.remap() is efficient
- **Readout time calibration** empirically measured at 200 Hz

### Yaw Stabilization (yaw_stabilizer.py)
- **EMA reference baseline** drifts toward gimbal reading at tunable rate (resists false gimbal drift)
- **PD control** (proportional + derivative) allows smooth, damped correction
- **3-frame weighted average** smooths output without excessive lag
- **Sliding 4:3 crop** preserves cinematic aspect while stabilizing

### Face Detection
- **Haar Cascade classifier** — fast, suitable for real-time on Jetson
- Operates on pre-stabilized frames (simpler than raw video)

