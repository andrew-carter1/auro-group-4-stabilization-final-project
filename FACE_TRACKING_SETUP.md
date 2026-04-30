# Face Tracking Setup & Testing Guide

## Status Summary

All core face tracking components have been implemented and are ready for testing:

✅ **face_tracker.py** — Kalman filter + gimbal control input + yaw crop + PD control  
✅ **kalman_face.py** — Constant-velocity Kalman filter (angle + angular velocity)  
✅ **uart_gimbal_servo.py** — UART → PWM servo driver (duty cycle mapping)  
✅ **face_detection_node.py** — Modified to publish `/face/bbox` (RegionOfInterest)  
✅ **setup.py** — Entry points added for face_tracker and uart_gimbal_servo  
✅ **face_tracking_test_launch.py** — Complete launch script for testing  
✅ **ESP32_UART_PWM_GUIDE.md** — Hardware integration guide for teammate  

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│ USB Camera (e.g., Mobius @ 1280×720 30fps MJPEG)            │
└─────────────────┬────────────────────────────────────────────┘
                  │
        ┌─────────▼──────────────────────────────┐
        │ rolling_shutter_node                   │
        │ (optical_flow mode: no gimbal required)│
        │ Publishes stabilized frames @ 1280×720 │
        └─────────┬──────────────────────────────┘
                  │
        /rs_corrected_frame/compressed
                  │
        ┌─────────▼────────────┐
        │ face_detection_node  │
        │ Haar Cascade         │
        └─────────┬────────────┘
                  │
        /face/bbox (RegionOfInterest)
        /image_with_faces/compressed
                  │
        ┌─────────▼──────────────────────┐
        │   face_tracker                 │
        │   Kalman CV model (pitch only) │
        └─────────┬──────────────────────┘
                  │
        /face_tracked/compressed (yaw crop)
        /face_tracker/pitch_cmd (Float32, ±45°)
                  │
        ┌─────────▼────────────┐
        │  uart_gimbal_servo   │
        │  (UART → PWM)        │
        └─────────┬────────────┘
                  │
        /dev/ttyUSB0 (ESP32)
                  │
        ┌─────────▼─────────┐
        │ ESP32 Firmware    │
        │ • UART parser     │
        │ • PWM generator   │
        │ • Servo driver    │
        └─────────┬─────────┘
                  │
        Gimbal Pitch Servo
```

---

## Launch Script: `face_tracking_test_launch.py`

Located in: `software/ros2_stabilization_ws/src/stabilization_pkg/launch/face_tracking_test_launch.py`

### Quick Start

```bash
cd software/ros2_stabilization_ws
source install/setup.bash

# Build (if not already done)
colcon build

# Run with defaults (camera /dev/video4, gimbal /dev/ttyUSB0)
ros2 launch stabilization_pkg face_tracking_test_launch.py

# Or specify custom gimbal port
ros2 launch stabilization_pkg face_tracking_test_launch.py gimbal_port:=/dev/ttyACM0

# Tune Kalman filter noise
ros2 launch stabilization_pkg face_tracking_test_launch.py kalman_std_acc:=40.0 kalman_std_meas:=3.0

# Tune yaw crop PD control
ros2 launch stabilization_pkg face_tracking_test_launch.py face_track_p_gain:=1.5 face_track_d_gain:=0.3
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `video_device` | auto (Mobius) or `/dev/video4` | Camera device path |
| `servo_port` | `/dev/ttyUSB0` | Serial port for ESP32 gimbal servo |
| `fov_horizontal_deg` | `100.0` | Camera horizontal FOV (degrees) |
| `max_shift_pct` | `0.10` | Rolling shutter correction strength (0–1) |
| `face_track_p_gain` | `1.0` | Yaw crop proportional gain (↑ = more aggressive) |
| `face_track_d_gain` | `0.2` | Yaw crop derivative damping (↑ = smoother) |
| `kalman_std_acc` | `30.0` | Kalman accel noise (deg/s²); ↑ = smoother |
| `kalman_std_meas` | `5.0` | Kalman measurement noise (deg); ↑ = trust detector less |
| `servo_min_deg` | `-45.0` | Servo lower limit (degrees) |
| `servo_max_deg` | `45.0` | Servo upper limit (degrees) |
| `show_annotations` | `true` | Show yaw/pitch text on output frames |

---

## What Each Node Does

### 1. **rolling_shutter_node** (optical_flow mode)
- Captures video from USB camera directly via V4L2 (1280×720 @ 30 fps)
- Runs Lucas-Kanade optical flow across frame pairs
- Estimates and corrects rolling shutter skew via optical flow (no gimbal dependency)
- Publishes `/rs_corrected_frame/compressed` — stabilized frames

### 2. **face_detection_node**
- Runs Haar Cascade on rolling-shutter-corrected frames
- **Publishes `/face/bbox`** — RegionOfInterest (x, y, w, h) of largest detected face
- Publishes `/image_with_faces/compressed` — visualization with boxes drawn

### 3. **face_tracker**
- Subscribes to:
  - `/face/bbox` — face detection output
  - `/rs_corrected_frame/compressed` — video frames from rolling_shutter_node
- **Kalman filter:** tracks face angle in image space with constant-velocity model
- **Yaw crop:** applies PD control to shift crop window horizontally (centers face)
- **Pitch command:** outputs `/face_tracker/pitch_cmd` clamped to ±45°
- **Publications:**
  - `/face_tracked/compressed` — yaw-cropped frame with annotations
  - `/face_tracker/pitch_cmd` (std_msgs/Float32) — gimbal pitch command in degrees

### 4. **uart_gimbal_servo**
- Subscribes to `/face_tracker/pitch_cmd`
- Converts angle → duty cycle (0–100%)
  - -45° → 0% (servo all the way up)
  - 0°   → 50% (neutral)
  - +45° → 100% (servo all the way down)
- Sends UART commands to ESP32: `"P:{duty:.1f}\n"`
- **Safe shutdown:** sends `"P:50.0\n"` (neutral) before closing serial port

---

## Testing Checklist

### Phase 1: Pre-Flight Checks (before running)

- [ ] Camera detected: `v4l2-ctl --list-devices` (should show Mobius or video device)
- [ ] ESP32 connected to computer: `ls -l /dev/ttyUSB0` or `/dev/ttyACM0`
- [ ] ESP32 programmed with UART receiver code (see **ESP32_UART_PWM_GUIDE.md**)
- [ ] Gimbal servo powered and mechanically free (test moving it by hand)

### Phase 2: Topic Check (nodes running)

```bash
# Terminal 1: Launch
ros2 launch stabilization_pkg face_tracking_test_launch.py

# Terminal 2: Monitor topics
ros2 topic list  # Verify all topics appear
ros2 topic echo /gimbal/angles
ros2 topic echo /face/bbox  # Should publish when face in frame
ros2 topic echo /face_tracker/pitch_cmd
```

### Phase 3: Visual Verification

- **rqt_image_view window** should show `/face_tracked/compressed`
  - Look for yaw crop (frame narrows as face moves side-to-side)
  - Annotations show "Yaw: XX.X°" and "Pitch: YY.Y°"

### Phase 4: Gimbal Servo Test

```bash
# In separate terminal, manually publish pitch commands
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: -20.0'  # pitch up
sleep 2
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: 0.0'    # neutral
sleep 2
ros2 topic pub /face_tracker/pitch_cmd std_msgs/Float32 'data: 20.0'   # pitch down
```
- Watch gimbal servo move smoothly through range
- **If servo doesn't move:** check ESP32 firmware, UART wiring, power supply

### Phase 5: End-to-End Test

1. Move face left/right in front of camera
   - **Expected:** `/face_tracked/compressed` crop window follows face horizontally
   - **Note:** software yaw crop only; gimbal yaw not involved

2. Move face up/down in front of camera
   - **Expected:** `/face_tracker/pitch_cmd` changes from -45 to +45
   - **Expected:** Gimbal servo pitches to keep face centered vertically
   
3. Pan gimbal by hand (rotate camera left/right)
   - **Expected:** Kalman filter predicts face motion even if detector skips a frame
   - **Expected:** Crop window stays smoother when using gimbal delta as control input

---

## Tuning Guide

### Kalman Filter (if tracking is jittery)

**Increase smoothing (face position less noisy):**
```bash
ros2 launch stabilization_pkg face_tracking_test_launch.py \
  kalman_std_acc:=50.0 kalman_std_meas:=8.0
```
- Higher `kalman_std_acc` = assume face can move faster (less filtering)
- Higher `kalman_std_meas` = detector is noisier (trust it less)
- ↓ **Decrease std values** to smooth more; increase to respond faster

### Yaw Crop (if crop window oscillates)

**Reduce oscillation with more damping:**
```bash
ros2 launch stabilization_pkg face_tracking_test_launch.py \
  face_track_p_gain:=0.8 face_track_d_gain:=0.4
```
- `p_gain` controls how fast crop moves to center face
- `d_gain` damping reduces overshoot
- ↓ **Lower p_gain** for slower, smoother tracking; higher for faster response
- ↓ **Raise d_gain** to dampen oscillations

---

## Common Issues & Fixes

| Issue | Likely Cause | Fix |
|-------|--------------|-----|
| `/face/bbox` never publishes | Face not detected or wrong input topic | Check face_detection_node input topic (should be `/rs_corrected_frame/compressed`) |
| Gimbal servo doesn't move | UART not working, ESP32 offline | Verify ESP32 program, check `/dev/ttyUSB0`, run UART test in guide |
| Crop window jitters | Kalman filter oscillating | Increase `kalman_std_meas` or `kalman_std_acc` |
| Gimbal overshoots | PD gains too aggressive | Lower `face_track_p_gain`, raise `face_track_d_gain` |
| Face detection is slow | Haar Cascade on big frames | Reduce camera resolution or switch to YOLO-face (future work) |

---

## File Changes Summary

### Modified

**`face_detection_node.py`**
- Changed input topic from `/stabilized_frame/compressed` → `/rs_corrected_frame/compressed`
- Now part of rolling_shutter + face tracking pipeline

### Created

**Launch**
- `launch/face_tracking_test_launch.py` — complete face tracking pipeline

**Documentation**
- `ESP32_UART_PWM_GUIDE.md` — detailed ESP32 firmware guide for teammate
- `FACE_TRACKING_SETUP.md` — this file

---

## Next Steps

1. **Build the project:**
   ```bash
   cd software/ros2_stabilization_ws
   colcon build
   ```

2. **Prepare ESP32:**
   - Follow **ESP32_UART_PWM_GUIDE.md** (share with hardware teammate)
   - Upload firmware that receives `P:{duty}\n` and outputs PWM

3. **Test gimbal servo UART:**
   ```bash
   python3 << 'EOF'
   import serial
   ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
   ser.write(b'P:50.0\n')  # Neutral
   ser.close()
   EOF
   ```

4. **Run face tracking launch:**
   ```bash
   ros2 launch stabilization_pkg face_tracking_test_launch.py
   ```

5. **Monitor and tune:**
   - Watch rqt_image_view for crop behavior
   - Adjust Kalman and PD gains as needed
   - Verify gimbal servo follows pitch commands

---

## Architecture Notes

### Kalman Filter for Smooth Tracking

The constant-velocity Kalman filter tracks face angle position and angular velocity:
- **State:** [theta_x, theta_y, vx, vy] — angles and angular velocities
- **Measurement:** face detection bbox center (converted to degrees)
- **Prediction:** fills gaps when detector misses frames

Even without gimbal control input, the velocity terms let the filter extrapolate face motion smoothly across frames.

### Yaw Crop Strategy

- **Soft correction** via Kalman tracking (face position estimate)
- **Hard clamp** to ±max_margin_px (prevent excessive crop drift)
- **PD control** (proportional gain for speed, derivative for damping)

Result: smooth, predictable tracking without oscillation.

### Pitch Command Range

Pitch command is published as Float32 in degrees [-45, 45], directly mapping to:
- -45° → servo fully up (0% duty)
- 0°   → servo neutral (50% duty)  
- +45° → servo fully down (100% duty)

---

## References

- **FACE_TRACKING_PLAN.md** — original architecture & algorithm specification
- **kalman_face.py** — Kalman filter implementation (constant-velocity model)
- **face_tracker.py** — ROS2 node with gimbal control input integration
- **uart_gimbal_servo.py** — UART ↔ PWM driver
- **ESP32_UART_PWM_GUIDE.md** — hardware firmware guide (for teammate)

---

Good luck with testing! 🚀
