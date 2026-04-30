# Changes: RS Overlay Improvements + Yaw Stabilizer Follow-at-Margin Control

## Summary

Two major improvements to the stabilization pipeline:

1. **Rolling shutter overlay**: 2-frame slope averaging + green horizontal reference lines for visual clarity
2. **Yaw stabilizer**: Switched from EMA-based reference drift to follow-at-margin control (freeze reference when within crop window, only slide when hitting margins)

---

## 1. Rolling Shutter Node (`rolling_shutter_node.py`)

### Changes

**1a. Slope history tracking**
- Added `self._slope_history: deque = deque(maxlen=2)` in `__init__` to store the last 2 slopes
- In `_process_frame`, appended slope to history: `self._slope_history.append(slope)`

**1b. Overlay improvements**
- Compute averaged slope for display: `avg_slope = sum(self._slope_history) / len(self._slope_history) if self._slope_history else 0.0`
- Added **two thin green horizontal reference lines** on both sides of the comparison panels at `y=5` and `y=half_h-5`:
  ```python
  for panel in (orig_small, corr_small):
      cv2.line(panel, (0, 5),          (half_w - 1, 5),          (0, 220, 0), 1)
      cv2.line(panel, (0, half_h - 5), (half_w - 1, half_h - 5), (0, 220, 0), 1)
  ```
- Yellow slope line now uses `avg_slope` instead of `self._last_slope` for display (smooths out frame-to-frame noise)
- The yellow line in the original panel tilts against the green references to visually communicate rolling shutter distortion
- The yellow line in the corrected panel stays vertical, proving correction worked

### Visual Benefit

The green horizontal lines provide visual anchors. The tilted yellow line against horizontal green lines clearly shows the RS slope at a glance, without needing to mentally decipher pixel values.

---

## 2. Yaw Stabilizer (`yaw_stabilizer.py`)

### Algorithm Change: EMA → Follow-at-Margin

**Old behavior (EMA)**:
- `reference_yaw` constantly drifted toward `current_yaw` via exponential moving average
- After any correction, the correction slowly decayed to zero, causing a "rubber-band" effect where the crop drifted back to center
- Problem: hard to dial in tuning; correction always faded

**New behavior (Follow-at-Margin)**:
- `reference_yaw` is **frozen** when the required correction is within the crop window (`±max_margin_px`)
- `reference_yaw` only slides when the correction **exceeds** the crop window
- Once a pan reverses, the crop re-freezes at the edge until motion comes back

**Key properties**:
- **Jitter resistance**: Gimbal wobble within the margin is fully corrected without drifting the crop
- **Pan passthrough**: Large pans are not fought — the crop slides to follow the motion at the margin
- **No oscillation**: No tanh damping or derivative term needed; the follow-at-margin naturally clamps

### Code Changes

**gimbal_callback** — simplified to just track current yaw:
```python
def gimbal_callback(self, msg: Vector3Stamped):
    self._current_yaw = msg.vector.z
    if self._reference_yaw is None:
        self._reference_yaw = self._current_yaw
```

**_process** — replaced PD control with follow-at-margin:
```python
correction_deg = self._reference_yaw - self._current_yaw
fov_rad = math.radians(self._fov_deg)
f_eq = (w / 2.0) / (fov_rad / 2.0)
dx_raw = f_eq * math.tan(math.radians(correction_deg))

m = float(self._max_margin_px)
if m > 0 and abs(dx_raw) > m:
    # Exceeds margin — slide reference to hold at ±margin
    target_dx = math.copysign(m, dx_raw)
    target_correction_deg = math.degrees(math.atan(target_dx / f_eq))
    self._reference_yaw = self._current_yaw + target_correction_deg
    dx_final = target_dx
else:
    # Within margin — hold reference frozen, apply full correction
    dx_final = dx_raw
```

**Parameter cleanup**:
- Removed `reference_alpha`, `p_gain`, `d_gain` — no longer used
- Kept: `fov_horizontal_deg`, `max_margin_px`, `out_w`, `yaw_lag_frames`, `show_annotations`
- Removed `self._prev_dx_raw` state variable

**Annotation update**:
- Changed text to show correction error in degrees: `f"dx: {dx_final:.1f}px  err: {correction_deg:.2f}deg"`
- More useful for understanding what's being corrected

### Why This Works

The follow-at-margin design is physically intuitive:
- Think of the crop window as having "sticky" edges with a margin inside (max_margin_px)
- Small wobbles (jitter) stay within the margin → crop holds steady, full correction applied
- Large, intentional motion (pan) exceeds the margin → crop slides to follow at the edge
- When motion stops, the crop stays at the edge (no decaying drift)

This is much more predictable and easier to tune than EMA drift.

---

## 3. Launch File Updates

Both `demo_rs_yaw_launch.py` and `demo_rs_yaw_clean_launch.py`:

**Removed launch arguments**:
```python
DeclareLaunchArgument('reference_alpha', ...)
DeclareLaunchArgument('p_gain', ...)
DeclareLaunchArgument('d_gain', ...)
```

**Updated yaw_stabilizer node parameters** — now only includes:
```python
'fov_horizontal_deg':  130.0,
'max_margin_px':       LaunchConfiguration('max_margin_px'),
'out_w':               960,
'yaw_lag_frames':      LaunchConfiguration('yaw_lag_frames'),
'show_annotations':    LaunchConfiguration('yaw_annotations'),
```

**Comment in annotated launch** updated to reflect follow-at-margin behavior.

---

## Verification

All changes compile successfully:
```
Summary: 2 packages finished [2.11s]
```

### Expected Behavior on Next Run

1. **Rolling shutter comparison panel**:
   - Green horizontal lines visible on both sides
   - Yellow line tilts during yaw rotation
   - Tilt much less noisy than before (2-frame average smoothing)
   - On corrected side, yellow line is perfectly vertical

2. **Yaw stabilizer**:
   - Pan the camera slowly → crop slides at the margin edge, image holds steady at edge
   - Pan back → crop re-freezes, doesn't drift back to center
   - Small wobbles → crop barely moves, disturbance is corrected
   - No rubber-band effect after pans stop

3. **Tuning** (via launch args):
   - `max_margin_px:=50` — tighter crop, more aggressive hold
   - `max_margin_px:=120` — looser crop, easier pans
   - `compass_lag_frames:=1` or `compass_lag_frames:=2` — adjust RS timing as needed

---

## Files Modified

- `stabilization_pkg/stabilization_pkg/rolling_shutter_node.py` — slope history + green lines
- `stabilization_pkg/stabilization_pkg/yaw_stabilizer.py` — complete rewrite (follow-at-margin)
- `stabilization_pkg/launch/demo_rs_yaw_launch.py` — removed PID args, updated comments
- `stabilization_pkg/launch/demo_rs_yaw_clean_launch.py` — removed PID args
