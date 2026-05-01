# Project Pivot Plan: Hardware Gimbal Failure

**Date**: April 30, 2026  
**Status**: CRITICAL — Gimbal fried, project due soon  
**Team**: Group 4 (Stabilization)

---

## Executive Summary

Storm BCG gimbal failure requires immediate pivot. Two viable paths forward:
1. **Stepper Motor Gimbal** — hardware-based pitch/roll control with custom servo
2. **Software-Only Face Following** — gimbal-less, using advanced CV techniques (Kalman, motion detection, optical flow)

This document captures all work done, debugging findings, and clear next steps.

---

## Part I: What We Built (Pre-Failure)

### Working Systems ✅

1. **DNN Face Detection** (`face_detection_dnn_node.py`)
   - SSD-based face detector running at ~30 fps
   - Publishes `/face/bbox` (bounding box) and `/image_with_faces/compressed`
   - Confidence threshold: 0.3 (tunable)
   - **Status**: Stable, production-ready

2. **Proportional Control Face Tracker** (`face_tracker.py`)
   - Takes DNN bounding box → computes pitch/yaw commands
   - Simple proportional control (no Kalman, no complex state)
   - EMA smoothing for pitch (alpha=0.3) to dampen oscillation
   - Publishes `/face_tracker/pitch_cmd` (Float32, degrees -40 to +40)
   - Publishes `/face_tracked/compressed` (1:1 square crop with yaw centering)
   - **Status**: Stable, working well

3. **Gimbal Servo UART Interface** (`uart_gimbal_servo.py`)
   - Subscribes to `/face_tracker/pitch_cmd`
   - Sends "P:XX.X" format commands at 10 Hz (fixed timer)
   - ESP32 parses and drives PWM to servo
   - Gracefully handles missing ESP32 (doesn't crash)
   - Publishes `/uart_gimbal/esp32_echo` for debugging
   - **Status**: Working, but gimbal hardware failed

4. **Rolling Shutter Correction** (`rolling_shutter_node.py`)
   - Per-row horizontal warp compensation
   - Compass-based (gimbal magnetometer) + optical flow hybrid
   - Publishes `/rs_corrected_frame/compressed`
   - **Status**: Working but secondary to gimbal failure

5. **Yaw Stabilization** (`yaw_stabilizer.py`)
   - Follow-at-margin algorithm (gimbal-based)
   - Software yaw correction via horizontal pixel translation
   - EMA reference + PD control
   - **Status**: Working but now irrelevant (no gimbal)

### Key Debugging Findings 🔍

#### Face Tracking Oscillation Issue
**Problem**: Pitch command cycled 50%↔100% continuously (0°↔40°), causing gimbal to hunt.  
**Root Cause**: Frame-to-frame jitter in face vertical position (theta_y) caused by:
- Face detection box jumping ±few pixels vertically
- DNN detector inconsistency at low confidence
- No temporal smoothing

**Solution Applied**: EMA smoothing on pitch_cmd (alpha=0.3)
```python
pitch_cmd = (1.0 - alpha) * prev_pitch + alpha * pitch_raw
```
Result: Smooth, stable convergence to centered position.

#### Gimbal Direction Issue
**Problem**: Face following pushed subject out of frame (wrong direction).  
**Root Cause**: Sign flip in yaw crop window calculation.  
**Solution Applied**: Changed line 213 in face_tracker from `cx = w // 2 - int(dx_final)` to `cx = w // 2 + int(dx_final)`.

#### ROS2 Timestamp Serialization Crash
**Problem**: face_tracker crashed with `builtin_interfaces__msg__time__convert_from_py` assertion error.  
**Root Cause**: ROS2 Python/C extension type mismatch when setting `msg.header.stamp`.  
**Solution Applied**: Removed timestamp setting entirely (message still publishes with default timestamp).

#### ESP32 PWM Range Issue
**Problem**: PWM command "P:99.9" resulted in ~10% duty instead of 99%.  
**Root Cause**: Servo library (`writeMicroseconds()`) had incorrect min/max calibration for this servo.  
**Solution Applied**: Switched to raw PWM using `analogWrite()` or ESP32 `ledcWrite()` with direct 0-255 mapping.

#### Confidence Threshold Tuning
**Finding**: Default 0.5 was too strict (missed many faces).  
**Fix Applied**: Lowered to 0.3 → face detection much more responsive.

---

## Part II: Gimbal Failure Analysis

**Hardware**: Storm BCG gimbal (2-axis, locked firmware)  
**Failure Mode**: Electrical failure (likely power surge or servo burnout)  
**Consequence**: Cannot command pitch/roll anymore; gimbal control path is dead

**What We Lost**:
- Physical pitch/roll stabilization
- Gimbal angle feedback (magnetometer, gyro)
- Servo command output

**What Still Works**:
- Camera + DNN detector
- Software stabilization (rolling shutter, yaw via optical flow)
- Face detection + tracking logic
- ROS2 infrastructure

---

## Part III: Pivot Options

### Option A: Stepper Motor Gimbal (Hardware-Based) ⚙️

**Approach**: Build/use a stepper-motor-driven 2-axis gimbal.

**Pros**:
- Full position control (no servo speed limits)
- Precise feedback via stepper encoder/microsteps
- Can hold position indefinitely
- Familiar to team (custom servo work done previously)

**Cons**:
- Requires mechanical integration (mount camera)
- Stepper driver board + power supply
- Requires torque calculation (camera weight + lens)
- May be slower than servo (50 Hz max vs. servo 100+ Hz)
- Integration time: 2–3 days

**Technical Path**:
1. Design 2-axis gimbal (pan-tilt) with stepper motors
2. Interface stepper drivers to Jetson (GPIO or dedicated driver board)
3. Reuse face_tracker.py pitch/yaw commands → map to stepper angles
4. New node: `stepper_gimbal_node.py` (subscribe to `/face_tracker/pitch_cmd`, drive steppers)
5. Add feedback (optical encoders or just step count) for diagnostics

**Deliverable**:
- Working gimbal that follows detected face
- Before/after video showing smooth tracking
- Performance metrics (pan speed, steady-state error)

**Estimated Effort**: 2–3 days (if mechanical parts available)

---

### Option B: Software-Only Face Following (Gimbal-Less) 💻

**Approach**: No gimbal. Use advanced CV + motion estimation to "smart crop" the video in real-time so face stays centered.

**Techniques**:
1. **Kalman Filtering** (on face position)
   - Smooth jitter, predict motion
   - Already have KalmanFace implementation (commented in code)
   - Would reduce oscillation without gimbal feedback

2. **Optical Flow Motion Detection**
   - Detect dominant motion in frame (camera shake, subject motion)
   - Separate "intentional" motion (pan following subject) from "unintentional" (shake)
   - Use to compute smart crop offset

3. **Multi-Frame Tracking** (temporal consistency)
   - If face missing one frame, interpolate position
   - Prevents jittery crop from single-frame noise

4. **Region-of-Interest (ROI) Prediction**
   - Predict where subject will be next frame based on velocity
   - Crop window leads the subject (feedforward)
   - Smoother than reactive cropping

**Pros**:
- No hardware needed
- Uses existing DNN detector
- Reuses optical flow infrastructure (already running)
- Can be implemented entirely in software

**Cons**:
- Can only stabilize in 2D (crop), not 3D (gimbal)
- Limited by image boundaries (can't pan past edges)
- Doesn't look as "dynamic" as gimbal (feels zoomed-in)
- Kalman has had ROS2 type issues (reason we switched to proportional)

**Technical Path**:
1. **Quick Win (Day 1)**: Re-enable Kalman filtering
   - Uncomment kalman_face.py and face_tracker Kalman code
   - Fix ROS2 timestamp issue (remove header.stamp)
   - Test: smoother face tracking without gimbal

2. **Smart Crop (Day 2)**: Implement predicted ROI
   - Use face velocity (from Kalman) to predict position
   - Compute "leading" crop window (ahead of face motion)
   - Blend with detected position (hybrid predictive + reactive)

3. **Optical Flow Integration (Day 3)**: Detect and separate motions
   - Analyze optical flow in non-face regions
   - If dominant flow exists outside face box → camera/gimbal motion (not subject)
   - Adjust crop velocity to follow the dominant motion intelligently

**Deliverable**:
- Smooth, stable face-centered video (software crop only)
- Kalman-filtered bounding box visualization
- Before/after comparison: jerky detection vs. smooth tracking
- Quantitative metric: face position variance frame-to-frame

**Estimated Effort**: 1–2 days (Kalman alone is 1 day; motion detection +1)

---

### Option C: Hybrid (Best of Both) 🎯

**Approach**: Stepper gimbal + software smart-crop for final polish.

**Steps**:
1. Build stepper gimbal (Option A) — rough tracking
2. Add Kalman + smart crop (Option B) — smooth jitter, predict motion
3. Gimbal does coarse tracking, software does fine stabilization

**Result**: Best video quality, demonstrates both hardware and software mastery.

**Pros**:
- Gimbal handles large motions (100+ pixels/frame)
- Software smooths residual jitter
- Covers curriculum: mechanics + controls + CV

**Cons**:
- Highest complexity
- Most integration risk
- Needs both hardware AND software stable

**Estimated Effort**: 3–4 days (serial tasks, high risk)

---

## Part IV: Recommended Path Given Time Crunch

### Priority 1: Option B (Software-Only, Kalman) — START IMMEDIATELY ⏱️

**Why**:
- Lowest hardware risk (can't break anything)
- Reuses 90% of existing code
- Kalman implementation already in codebase (just commented out)
- Can have working demo in 1 day
- Good story: "Smart software crop replaces gimbal"

**Day 1 Deliverable**:
- Uncomment Kalman code
- Fix timestamp issue
- Test face tracking with Kalman smoothing
- Record before/after (proportional vs. Kalman)

**Day 2 Stretch**:
- Implement optical flow motion detection
- Publish diagnostics showing "detected motion vector"

**Day 3 Polish**:
- Optimize Kalman parameters (tuning) for stable crop
- Add metrics overlay (face position variance, tracking confidence)

---

### Priority 2: Option A (Stepper Gimbal) — IF TIME PERMITS

If Kalman demo is solid by EOD Day 1 and you have mechanical parts:
- Start gimbal design Day 2
- Stepper driver code is simple (reuse uart_gimbal_servo pattern)
- Integration by EOD Day 3

If gimbal isn't ready by Day 2 evening, **stick with software-only** (Option B). A polished software solution beats a half-finished gimbal.

---

## Part V: Implementation Roadmap

### Immediate (Next 4 hours)

- [ ] Decide: Option A, B, or C
- [ ] If Option B: Uncomment Kalman code in face_tracker.py
- [ ] Fix timestamp issue (remove header.stamp from _encode_compressed)
- [ ] Rebuild and test
- [ ] Record baseline video (proportional control issues visible)

### Day 1 (Next 8 hours)

**If Option B**:
- [ ] Tune Kalman parameters (std_acc, std_meas)
- [ ] Test face tracking smoothness
- [ ] Record comparison video (before/after)
- [ ] Add metrics: face position variance, tracking latency
- [ ] Document what Kalman achieved

**If Option A**:
- [ ] CAD/sketch gimbal design (2-axis stepper mount)
- [ ] Order/collect parts if not in lab
- [ ] Start mechanical assembly

### Day 2

**If Option B**:
- [ ] Integrate optical flow motion detection
- [ ] Publish `/motion_vector` topic showing detected motion
- [ ] Test in real scenario (moving camera, moving subject)
- [ ] Begin polish (parameter tuning, visualization)

**If Option A**:
- [ ] Complete mechanical assembly
- [ ] Stepper driver board + wiring
- [ ] Write stepper_gimbal_node.py
- [ ] Integrate with face_tracker commands
- [ ] Test simple tracking (no Kalman yet)

### Day 3

**Polish & Testing**:
- [ ] Optimize whichever path you chose
- [ ] Record final demo video
- [ ] Write up technical summary
- [ ] Prepare presentation slides

---

## Part VI: Code Changes Needed

### For Option B (Kalman)

**In face_tracker.py**:
1. Uncomment import at top:
   ```python
   from stabilization_pkg.kalman_face import FaceKalman
   ```

2. In `__init__`, uncomment:
   ```python
   self._kf = FaceKalman(dt=0.033, std_acc=30.0, std_meas=5.0)
   self._tracking = True
   ```

3. In `_frame_cb`, replace proportional logic with:
   ```python
   # Kalman predict
   theta_x_est, theta_y_est = self._kf.predict(u=None)
   
   # Kalman update
   self._kf.update([theta_x, theta_y])
   
   # Use estimates
   dx_raw = self._f_eq_h * math.tan(math.radians(theta_x_est))
   pitch_cmd = max(-40.0, min(40.0, theta_y_est * 1.0))  # No p_gain needed
   ```

4. Remove timestamp line if present:
   ```python
   # DELETE: out_msg.header.stamp = self.get_clock().now().to_msg()
   ```

5. Add to _encode_compressed:
   ```python
   msg.header.stamp.sec = 0  # Bypass type issue
   ```

**Parameters to tune** (in launch file):
```python
'kalman_std_acc': 20.0,      # Lower = smoother but slower response
'kalman_std_meas': 3.0,      # Lower = trusts measurements more
```

---

### For Option A (Stepper Gimbal)

**New file: stepper_gimbal_node.py**
```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO  # or Jetson.GPIO for Jetson
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class StepperGimbalNode(Node):
    def __init__(self):
        super().__init__('stepper_gimbal')
        
        # Define GPIO pins for stepper drivers
        self.pitch_pin_step = 17
        self.pitch_pin_dir = 27
        self.yaw_pin_step = 22
        self.yaw_pin_dir = 23
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in [self.pitch_pin_step, self.pitch_pin_dir, self.yaw_pin_step, self.yaw_pin_dir]:
            GPIO.setup(pin, GPIO.OUT)
        
        # Current position (in steps)
        self.pitch_steps = 0
        self.yaw_steps = 0
        
        # Subscribe to gimbal commands
        self.create_subscription(Float32, '/face_tracker/pitch_cmd', self._pitch_cb, 10)
        
        self.get_logger().info("Stepper Gimbal Node started")
    
    def _pitch_cb(self, msg: Float32):
        """Convert pitch angle to stepper steps and drive motor."""
        target_steps = int(msg.data * 10)  # e.g., 10 steps per degree
        delta = target_steps - self.pitch_steps
        
        if delta > 0:
            GPIO.output(self.pitch_pin_dir, GPIO.HIGH)
            for _ in range(delta):
                GPIO.output(self.pitch_pin_step, GPIO.HIGH)
                GPIO.output(self.pitch_pin_step, GPIO.LOW)
                # Add delay here (e.g., 10ms for 100 Hz)
        elif delta < 0:
            GPIO.output(self.pitch_pin_dir, GPIO.LOW)
            for _ in range(-delta):
                GPIO.output(self.pitch_pin_step, GPIO.HIGH)
                GPIO.output(self.pitch_pin_step, GPIO.LOW)
        
        self.pitch_steps = target_steps
```

---

## Part VII: Risk Assessment & Fallback

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Kalman still crashes | Medium | High | Have proportional-only backup; fall back to smart crop without Kalman |
| Stepper gimbal mechanical issues | High | High | Start early; have fallback to software-only |
| Time runs out | High | High | Prioritize Kalman (Day 1), gimbal is optional |
| Face detection fails (confidence too low) | Low | Medium | Pre-recorded test video as backup demo |

**Fallback Plan**: If everything fails, submit software-stabilization (rolling shutter) + DNN detection as "robust video analysis pipeline." Demonstrates CV skills even without gimbal/tracking.

---

## Part VIII: Success Metrics

### Option B (Software Kalman)
- ✅ Smooth face-centered crop video (no jitter visible)
- ✅ Face stays in frame even with hand-shaking camera
- ✅ Quantitative: face position variance < 20 pixels (vs. ~50 pixels with proportional)
- ✅ Before/after comparison showing smoothness improvement

### Option A (Stepper Gimbal)
- ✅ Gimbal responds to face motion (pitch/yaw follow)
- ✅ Face stays roughly centered (within ±50 pixels)
- ✅ Smooth motion, no jitter or hunting
- ✅ Live demo + recorded video

### Option C (Hybrid)
- ✅ All of the above
- ✅ Gimbal + software working together (gimbal for coarse, software for fine)

---

## Summary & Next Steps

**Right now**: Pick Option A or B. **Recommend B** (software-only Kalman) for speed and safety.

**Next 30 minutes**:
1. Uncomment Kalman code
2. Fix timestamp issue
3. Rebuild and verify it doesn't crash

**If it works**: You have 2 full days to polish and add optical flow.  
**If it crashes**: Fall back to proportional control (we know it works) and focus on smart crop without Kalman.

**Document what you learn**. This pivot—recovering from hardware failure using software—is actually a strong story for a capstone project.

Good luck! 🚀
