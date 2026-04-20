# Gimbal Integration with Video Stabilization

## Overview

The Storm BCG gimbal provides real-time measurements of camera orientation via its onboard magnetometer and gyro, published on `/gimbal/angles` at 30 Hz. The gimbal **mechanically stabilizes roll and pitch** but has no hardware yaw control. Yaw drift is measured but must be corrected via software.

The primary camera is a **Mobius action camera** (150° horizontal FOV, ~1280×720 typical) feeding into the rolling shutter correction and realtime stabilization pipelines.

---

## Current Implementation: yaw_stabilizer.py

A separate ROS2 node (`yaw_stabilizer.py`) performs gimbal-based yaw correction:

**Architecture:**
1. Subscribes to `/raw_frame/compressed` (unprocessed video from realtime_stabilization.py)
2. Subscribes to `/gimbal/angles` (yaw from gimbal magnetometer)
3. Applies a **rotation correction** based on smoothed gimbal yaw
4. Publishes corrected frames to `/yaw_stabilized/compressed`

**Current approach:**
```python
# Smooth the yaw reading over 30 frames
smoothed_yaw = np.mean(yaw_history)
correction = (smoothed_yaw - current_yaw) * 5  # gain factor

# Apply rotation correction
M = cv2.getRotationMatrix2D((w//2, h//2), correction, 1.0)
frame = cv2.warpAffine(frame, M, (w, h))
```

**Status:** This implementation works but is being **refined** to:
- Use horizontal **translation** (`dx`) instead of in-plane rotation (the original GIMBAL_INTEGRATION plan)
- Derive the correction gain from the camera's fisheye model rather than a hardcoded multiplier

---

## Original Plan: Yaw-to-Pixel Conversion

Converting yaw delta to a pixel shift requires the horizontal field of view and the camera's projection model.

**Fisheye equidistant model** (150° FOV, Mobius):
```
f_eq = (image_width / 2) / (fov_horizontal_radians / 2)
     = 640 / (2.618 rad / 2)       [at 1280×720]
     ≈ 488 pixels (equidistant focal length)

dx_pixels = f_eq * tan(d_yaw_radians)
```

At 150° FOV and 1280 wide: 1° of yaw ≈ **9 pixels** horizontally. This is a significant
and measurable correction — without it, yaw drift will walk the frame off-center over time.

**Alternative: pinhole lens approximation** (less accurate near edges, simpler):
```
f_px = (image_width / 2) / tan(fov_horizontal / 2)
dx_pixels = f_px * tan(d_yaw_radians)
```

The equidistant model is preferred for wide-FOV (>120°) fisheye lenses because it matches
the actual sensor behavior more closely.

---

## Rate Mismatch & Synchronization

- **realtime_stabilization.py** runs at ~30 fps (timer 0.033 s)
- **gimbal_node** publishes at 30 Hz
- **yaw_stabilizer** subscribes to frames (async, processes on arrival)

Both are running at roughly the same rate on a single system. The gimbal publishes its latest
yaw reading for every frame the camera captures, so synchronization is implicit. In the callback,
yaw_stabilizer always reads `self.current_yaw` (the most recent gimbal message), which is
appropriate for per-frame correction.

---

## Sign Convention

Verify the direction of correction before trusting it:

- **Gimbal yaw**: typically increases when drone nose swings **right** (clockwise from above)
  - `msg.vector.z` increases → yaw increases
- **In the video**: drone nose swinging right → scene moves **left** → dx should be negative
- **Correction**: if scene moves left, we want to pan right (positive correction)

Testing:
1. Mount gimbal with camera pointing forward
2. Slowly rotate gimbal clockwise (yaw+)
3. Watch the video output — does it visually correct by panning right?
4. If correction goes wrong direction, negate the yaw or dx

Current yaw_stabilizer uses `(smoothed_yaw - current_yaw) * 5`, which applies a positive
gain to the yaw error. Verify this sign is correct in practice.

---

## Future Work: Aligning yaw_stabilizer to the Plan

The current yaw_stabilizer.py applies **rotation correction**. To match the original
GIMBAL_INTEGRATION plan more closely (and likely improve quality):

1. **Compute dx from yaw rate**:
   ```python
   d_yaw = yaw2 - yaw1
   dx_pixels = f_eq * math.tan(math.radians(d_yaw))
   ```

2. **Apply dx as horizontal translation** (same as rolling shutter correction):
   ```python
   map_x = col - dx_pixels / height_samples * row
   # (maps horizontal motion to per-row shift, matching rolling shutter approach)
   ```

3. **Use gimbal data to constrain, not replace, optical flow**:
   - Gimbal yaw rate gives the true camera rotation (high confidence)
   - Optical flow from realtime_stabilization gives translation (can drift)
   - Blend: `final_dx = YAW_WEIGHT * dx_yaw + (1 - YAW_WEIGHT) * dx_optical_flow`

This would integrate yaw more deeply into the stabilization pipeline and avoid fighting
the existing EMA smoothing logic.

---

## Quick Reference: Gimbal Output Format

From gimbal_node.py:

| Topic | Type | Fields | Rate |
|---|---|---|---|
| `/gimbal/angles` | geometry_msgs/Vector3Stamped | x=roll°, y=pitch°, z=yaw° (continuous) | 30 Hz |
| `/gimbal/gyro` | geometry_msgs/Vector3Stamped | x=roll_rate, y=pitch_rate, z=0 (yaw not in packet) | 30 Hz |

- **Yaw is continuous** — does not wrap at ±360°. Rotating clockwise accumulates > +360°
- **Roll/Pitch** precision: 0.18°/unit (≈ 0.1° after rounding)
- **Yaw** precision: 0.1°/unit (full resolution)
