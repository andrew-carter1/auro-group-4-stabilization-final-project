# Gimbal Integration with Video Stabilization

## Overview

The Storm BCG gimbal provides real-time measurements of camera orientation via its onboard magnetometer and gyro, published on `/gimbal/angles` at 30 Hz. The gimbal **mechanically stabilizes roll and pitch** but has no hardware yaw control. Yaw drift is measured but must be corrected via software.

The primary camera is a **Mobius action camera** (150° horizontal FOV, ~1280×720 typical) feeding into the rolling shutter correction and realtime stabilization pipelines.

---

## Current Implementation: yaw_stabilizer.py (Follow-at-Margin)

A separate ROS2 node (`yaw_stabilizer.py`) performs gimbal-based yaw correction via horizontal pixel translation:

**Architecture:**
1. Subscribes to `/rs_corrected_frame/compressed` (rolling shutter corrected video)
2. Subscribes to `/gimbal/angles` (yaw from gimbal magnetometer)
3. Computes required yaw correction using fisheye equidistant model
4. Applies **follow-at-margin control**: freezes reference when within crop window, slides at margins
5. Publishes corrected frames to `/yaw_stabilized/compressed` (960×720, 4:3 cropped)

**Control algorithm:**
```python
correction_deg = reference_yaw - current_yaw
f_eq = (width / 2.0) / (fov_rad / 2.0)  # ~489 px at 1280w, 130° FOV
dx_raw = f_eq * math.tan(math.radians(correction_deg))

if abs(dx_raw) > max_margin_px:
    # Exceeds crop headroom — slide reference to hold at ±margin
    target_dx = copysign(max_margin_px, dx_raw)
    reference_yaw = current_yaw + degrees(atan(target_dx / f_eq))
    dx_final = target_dx
else:
    # Within margin — hold reference frozen, apply full correction
    dx_final = dx_raw

# Sliding crop (no warping, no border artifacts)
cx = width // 2 - int(dx_final)
x0 = max(0, min(cx - out_w // 2, width - out_w))
frame_out = frame[:, x0:x0 + out_w]
```

**Key properties:**
- **Jitter resistance**: Small wobbles within `max_margin_px` are fully corrected without drifting
- **Pan passthrough**: Large intentional motions are not fought — crop follows at the margin
- **Edge hold**: After a pan stops, the crop stays at the edge (no decay drift)
- **No oscillation**: Natural clamping via reference sliding; no damping or soft clamp needed

**Status:** ✅ **Implemented** — horizontal translation with follow-at-margin control (replaces old rotation + EMA approach)

---

## Yaw-to-Pixel Conversion (Fisheye Equidistant Model)

Converting yaw delta to a pixel shift requires the horizontal field of view and the camera's projection model.

**Fisheye equidistant model** (130° FOV, Mobius):
```
f_eq = (image_width / 2) / (fov_horizontal_radians / 2)
     = 640 / (2.268 rad / 2)       [at 1280 wide, 130° FOV]
     ≈ 565 pixels (equidistant focal length)

dx_pixels = f_eq * tan(d_yaw_radians)
```

At 130° FOV and 1280 wide: 1° of yaw ≈ **10 pixels** horizontally.

**Usage in yaw_stabilizer:**
```python
correction_deg = reference_yaw - current_yaw
dx_raw = f_eq * math.tan(math.radians(correction_deg))
```

The equidistant model is preferred for wide-FOV (>120°) fisheye lenses because it matches
the actual sensor behavior more closely than pinhole approximation.

**Status**: ✅ Implemented in yaw_stabilizer.py

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

The yaw_stabilizer uses fisheye equidistant projection to convert yaw error to pixel shift:

- **Gimbal yaw**: increases when camera nose swings **right** (clockwise from above)
  - `msg.vector.z` increases → yaw increases
- **Correction error**: `correction_deg = reference_yaw - current_yaw`
  - If gimbal yawed right (current > reference), error is negative
  - Negative error → negative dx via `atan()` → crop shifts left
  - Left crop shift makes scene appear to pan right, compensating for the rightward yaw ✓
- **Follow-at-margin**: Once correction reaches ±max_margin, reference slides to follow
  - Maintains sign and magnitude, just clamps to margin

**Verification**: 
1. Pan gimbal slowly right (yaw increases)
2. Observe yaw_stabilizer output: should pan left (compensating)
3. Crop should slide to the right edge and hold there
4. Pan gimbal back left: crop slides back toward center

The sign is baked into the fisheye model and should be correct as-is.

---

## Rolling Shutter + Yaw Integration

The rolling shutter node (`rolling_shutter_node.py`) operates in **compass mode**, using gimbal yaw rate to compute per-row horizontal warp. This corrects the inherent camera skew during yaw motion.

The yaw_stabilizer then operates on the rolling shutter corrected output, providing a second stage of stabilization via the follow-at-margin control. Together:

1. **Rolling shutter (first stage)**: Corrects row-by-row skew based on gimbal yaw rate
2. **Yaw stabilizer (second stage)**: Resists residual jitter and holds camera heading

**Recent improvements (April 2026):**
- **2-frame slope averaging** in rolling shutter node to smooth overlay noise
- **Green horizontal reference lines** in RS comparison panel for visual clarity
- **Follow-at-margin control** in yaw stabilizer (replaces EMA drift)

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

---

## Latest Improvements (April 2026)

**Rolling Shutter Enhancements:**
- 2-frame slope averaging for smoother overlay display (reduces frame-to-frame noise)
- Green horizontal reference lines on both comparison panels to visually highlight rolling shutter distortion
- Slope indicator now pivots from center (symmetric top/bottom) rather than top-only

**Yaw Stabilizer Redesign:**
- Replaced EMA-based reference drift with follow-at-margin control
- Reference yaw is frozen when correction is within crop margin (resists jitter)
- Reference slides only when correction exceeds margin (lets pans through naturally)
- Removed PID tuning parameters (`reference_alpha`, `p_gain`, `d_gain`) — control law is now deterministic
- Removed tanh soft clamping — follow-at-margin design naturally handles edge clamping

**Result:** Smoother jitter rejection, no "rubber-band" effect after pans, cleaner tuning (only `max_margin_px` matters)
