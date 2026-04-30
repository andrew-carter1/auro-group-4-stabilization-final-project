# Plan: Face-Tracking Gimbal Control

## Context
We want to detect faces in the camera stream and:
1. **Yaw (software):** shift the crop window to keep the face horizontally centered.
2. **Pitch (hardware):** send PWM commands to a Storm BCG gimbal via an ESP32 over UART to keep the face vertically centered.

A Kalman filter (constant-velocity, based on HW5/Part_BC's KF_2D) smooths noisy face-detector output and predicts face position across frames. Sudden gimbal yaw movements are fed as control inputs so the filter can anticipate how the face will move in the image before the next detection arrives.

Roll control is out of scope for now (leave TODOs).

---

## Architecture

```
[rolling_shutter_node]
    /rs_corrected_frame/compressed
        ↓
[face_detection_node]  (MODIFIED — also publishes bbox)
    /face/bbox           (RegionOfInterest: x, y, w, h in pixels)
    /image_with_faces/compressed
        ↓
[face_tracker.py]  (NEW) ← /gimbal/angles (yaw/pitch for control input)
    /face_tracked/compressed     (software-yaw-cropped output)
    /face_tracker/pitch_cmd      (std_msgs/Float32, degrees -45..+45)
        ↓
[uart_gimbal_servo.py]  (NEW)
    → UART → ESP32 → PWM signal → gimbal pitch servo
```

---

## Files to Create / Modify

### 1. Modify `face_detection_node.py`
**File:** `stabilization_pkg/face_detection_node.py`

- Import `sensor_msgs.msg.RegionOfInterest`
- Add a second publisher: `self.bbox_pub = create_publisher(RegionOfInterest, '/face/bbox', 10)`
- In `listener_callback`, after detecting faces:
  - If at least one face detected: pick largest bbox (`max(faces, key=lambda f: f[2]*f[3])`)
  - Publish `RegionOfInterest(x_offset=x, y_offset=y, width=w, height=h, do_rectify=False)`
  - Note: `RegionOfInterest` has no header (no timestamp); downstream must tolerate this
- When no face detected: do NOT publish on `/face/bbox` — absence of message signals no detection
- Keep existing `/image_with_faces/compressed` publication unchanged

---

### 2. New `kalman_face.py`
**File:** `stabilization_pkg/kalman_face.py`
**Based on:** `HW5/Part_BC/KF_2D.py`

**State:** `[theta_x, theta_y, vx, vy]` — angles (degrees) from frame center + angular velocity (deg/s)

**State transition (A):**
```
A = [[1, 0, dt, 0 ],
     [0, 1,  0, dt],
     [0, 0,  1,  0],
     [0, 0,  0,  1]]
```

**Control input (B × u):** gimbal angle delta since last frame
```
B = [[dt, 0 ],
     [0,  dt],
     [1,  0 ],
     [0,  1 ]]
u = [-delta_yaw_deg/dt, -delta_pitch_deg/dt]   # apparent face velocity from camera movement
```
Why: if the camera pans right by Δyaw, the face appears to shift left in the frame by Δyaw deg.
This is equivalent to the face having velocity `-delta_yaw / dt`. Feeding this as control input lets
the filter predict the new face position even if detector misses a frame.

**Measurement matrix (H):** measures only theta_x, theta_y (not velocity):
```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

**Noise tuning (starting values — will need empirical tuning):**
- `Q`: derived from continuous white-noise acceleration model (same as KF_2D)
  - `std_acc = 30 deg/s²` — faces can change direction reasonably quickly
- `R`: `x_std_meas = y_std_meas = 5.0 deg` — Haar cascade is somewhat noisy
- `P`: initial = identity * 500 (large initial uncertainty)

**Interface:**
```python
class FaceKalman:
    def __init__(self, dt, std_acc=30.0, std_meas=5.0)
    def predict(self, u=None) -> (theta_x, theta_y)   # u = [delta_yaw, delta_pitch]
    def update(self, z) -> (theta_x, theta_y)         # z = [theta_x_meas, theta_y_meas]
    def reset(self, theta_x, theta_y)                 # cold-start from first detection
```

---

### 3. New `face_tracker.py`
**File:** `stabilization_pkg/face_tracker.py`

**Node:** `FaceTrackerNode`

**Subscriptions:**
- `/face/bbox` (sensor_msgs/RegionOfInterest) — raw detections
- `/gimbal/angles` (geometry_msgs/Vector3Stamped) — for control input

**Publications:**
- `/face_tracked/compressed` (CompressedImage) — software-yaw-cropped output
- `/face_tracker/pitch_cmd` (std_msgs/Float32, degrees in [-45, +45])

**Parameters:**
- `fov_horizontal_deg` (float): camera horizontal FOV. Default: 100.0 (Mobius)
- `fov_vertical_deg` (float): camera vertical FOV. Default: 75.0 (estimate)
- `image_width` (int): input frame width. Default: 1280
- `image_height` (int): input frame height. Default: 720
- `out_w` (int): output crop width. Default: 960 (4:3)
- `max_margin_px` (int): hard crop clamp. Default: 80
- `no_face_hold_sec` (float): hold pitch command this long before freezing. Default: -1 (forever)
- `p_gain` (float): proportional gain on yaw crop. Default: 1.0
- `d_gain` (float): derivative gain on yaw crop (damp oscillation). Default: 0.2
- `dt` (float): assumed filter time step. Default: 0.033 (30 fps)

**Core Logic:**

```
Pixel offset to angle:
  fov_rad = radians(fov_horizontal_deg)
  f_eq    = (image_width / 2.0) / (fov_rad / 2.0)
  theta_x = degrees(atan((cx_px - image_width/2) / f_eq))   # positive = right of center
  theta_y = degrees(atan((cy_px - image_height/2) / f_eq))  # positive = below center

On each /face/bbox message:
  1. Compute theta_x, theta_y from bbox center pixel
  2. If first detection: reset Kalman with this measurement
  3. Compute delta_yaw, delta_pitch from latest gimbal reading (diff since last callback)
  4. kf.predict(u=[-delta_yaw, -delta_pitch])
  5. kf.update([theta_x, theta_y])
  6. _last_face_time = now; _tracking = True

On each /rs_corrected_frame/compressed (also subscribed):
  If _tracking:
    1. Predict (time-since-last-detection based predict if no new bbox)
    2. Get est_theta_x, est_theta_y from filter
    3. Software yaw: dx_px = f_eq * tan(radians(est_theta_x))
                   Apply PD + clamp, produce crop x offset
    4. Pitch cmd: publish est_theta_y clamped to [-45, 45] on /face_tracker/pitch_cmd
    5. Apply crop, encode, publish on /face_tracked/compressed
  Else:
    Publish frame uncropped (or hold last crop position)
```

**Hold behavior:** When no face detected, `_tracking` remains True and filter keeps predicting using gimbal control inputs. Pitch command holds at last estimated value. Software yaw holds last crop position. Never returns to neutral automatically (per user spec).

**TODO (roll):** Left as comment stub — would require facial landmark detection to estimate head tilt, then map to a separate gimbal roll command axis.

---

### 4. New `uart_gimbal_servo.py`
**File:** `stabilization_pkg/uart_gimbal_servo.py`

**Node:** `UartGimbalServoNode`

**Subscription:** `/face_tracker/pitch_cmd` (std_msgs/Float32)

**Parameters:**
- `port` (string): UART device for ESP32. Default: '/dev/ttyUSB0' (configure at runtime)
- `baudrate` (int): Default 115200
- `min_deg` (float): physical lower limit. Default: -45.0
- `max_deg` (float): physical upper limit. Default: +45.0

**Protocol (ESP32 side):** ASCII: `"P:{duty:.1f}\n"` where duty is 0.0–100.0
```
duty = (clamp(deg, -45, 45) + 45.0) / 90.0 * 100.0
```

**Features:**
- Same robust reconnect pattern as `gimbal_node._connect()` (retry on serial error)
- Throttled warn log when ESP32 not connected
- Safe shutdown: on node destroy, send `"P:50.0\n"` (neutral) before closing port

---

### 5. New `KALMAN_APPROACH.md`
**File:** `software/ros2_stabilization_ws/src/stabilization_pkg/KALMAN_APPROACH.md`

Copy relevant KF_2D equations from HW5/Part_BC and explain how we adapt them for angular face tracking (angle state, fisheye projection, gimbal control input). Include parameter tuning guidance and extension notes for IMM/adaptive variants.

---

### 6. Update `setup.py`
**File:** `software/ros2_stabilization_ws/src/stabilization_pkg/setup.py`

Add entry points:
```python
'face_tracker = stabilization_pkg.face_tracker:main',
'uart_gimbal_servo = stabilization_pkg.uart_gimbal_servo:main',
```

---

## Coordinate Conventions

| Axis | Positive Direction | Physical Gimbal |
|---|---|---|
| theta_x (yaw) | Face is right of center | → software crop shifts right |
| theta_y (pitch) | Face is below center | → pitch gimbal down (ESP32) |

Pitch duty cycle:
- -45° (camera all the way up) → 0% duty
- 0° (level) → 50% duty
- +45° (camera all the way down) → 100% duty

---

## Gimbal Delta as Kalman Control Input

This is the key insight: when `/gimbal/angles` reports that yaw changed by `Δyaw` degrees since the last frame, any stationary object appears to shift by `-Δyaw` in angular frame-space. Feeding this as `u = [-Δyaw, -Δpitch]` into the Kalman predict step lets the filter compensate for camera motion even during frames where the detector produces no output. This makes the crop window much smoother during fast camera pans.

---

## Open Questions / TODOs Left for Later

- **Roll control:** Would require facial landmark detector (e.g., MediaPipe Face Mesh or dlib) to extract head-tilt angle. Send roll PWM separately (±90° range per original gimbal spec).
- **Gimbal fusion:** Fuse actual gimbal angles into Kalman state (currently vision-only).
- **IMM/adaptive filter:** If face tracking oscillates during sudden stops, add maneuver detection (Singer model or IMM) on top of CV Kalman.
- **ESP32 protocol:** Finalize UART message format with hardware team; adjust `uart_gimbal_servo.py` if needed.
- **Calibrate FOV:** Measure actual camera FOV rather than assuming 100°; also calibrate fisheye k1/k2 per rolling_shutter_node's TODO.
- **Face detector upgrade:** Haar cascade may be too slow/noisy for fast tracking. Consider YOLO-face or MediaPipe for higher frame rate.

---

## Testing Plan

1. **Unit test the Kalman filter** (`kalman_face.py`) standalone:
   - Feed synthetic straight-line face motion, verify filter tracks it smoothly
   - Inject "missed frames" with only control input, verify prediction quality

2. **Face bbox topic check:**
   - `ros2 topic echo /face/bbox` with a face in frame — verify ROI updates
   - Verify `/face/bbox` is silent when no face present

3. **Yaw crop test (no gimbal):**
   - Run `face_tracker` without ESP32 connected (pitch_cmd topic should still publish)
   - Move face left/right — verify `/face_tracked/compressed` crop window follows

4. **Pitch command range:**
   - Face at top of frame → cmd should approach -45
   - Face at bottom → cmd should approach +45
   - Face at center → cmd near 0

5. **ESP32 integration:**
   - Connect to ESP32, verify `uart_gimbal_servo` sends PWM via scope/serial monitor
   - Physically verify gimbal moves in correct direction

6. **End-to-end:**
   - Full launch: rolling_shutter → face_detection → face_tracker → uart_servo
   - Person walks in front of camera: gimbal pitches to keep face vertical center; yaw crop follows horizontal center
