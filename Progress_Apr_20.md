# Progress Report — April 20, 2026

## Summary

Since April 8, the team has completed a **rolling shutter correction subsystem** (new), improved gimbal integration, refined video stabilization, and begun integrating gimbal yaw feedback for 3-DOF virtual stabilization.

The project now has two distinct video processing pipelines:
1. **EMA-based real-time stabilization** (`realtime_stabilization.py`) for general use
2. **Rolling shutter correction** (`rolling_shutter_node.py`) for high-motion scenarios

---

## Completed Work

### Rolling Shutter Correction Subsystem (New)

**Core Implementation**
- `rolling_shutter_node.py`: Complete rewrite from band-based optical flow to unified global-shift model
  - **Optical flow mode**: Lucas-Kanade tracking + `estimateAffinePartial2D` on full frame → global dx → slope
  - **Compass mode**: Gimbal yaw rate → fisheye projection → per-row shift directly
  - Unified formula: `slope_px_per_row = px_per_sec * readout_time_sec / frame_height`
  - Parameters: `max_shift_pct` (clamp), `slope_ema_alpha` (smoothing), `compass_lag_frames` (buffer)

**LED Sensor Calibration**
- Measured readout time at 200 Hz LED frequency across 7 resolutions
- Method: count vertical pixels in 4 bright + 4 dark bands (20 ms of sensor activity)
- Result: K_MS_PX = 52.4 ms (derived from measurements; fits as constant across all resolutions)
- Documentation: `ROLLING_SHUTTER_CALIBRATION.md` (comprehensive with setup, observations, future improvements)

**Launch Files** (all using Mobius camera, 1280×720)
- `rolling_shutter_raw_test_launch.py`: Optical flow mode, direct capture, side-by-side output
- `rolling_shutter_compass_launch.py`: Compass mode, gimbal-driven, no EMA stabilization
- `rolling_shutter_compass_diagnostics_launch.py`: Compass mode + `/rs_diagnostics` topic for timing analysis

**Diagnostics & Tuning**
- `/rs_diagnostics` topic (std_msgs/String): yaw, yaw_rate, compass_shift, flow_shift, applied, buffer_depth
- Frame buffer (`compass_lag_frames`): compensates for gimbal magnetometer internal filter (~133ms lag)
  - User sets frame count; at runtime, buffers N frames before processing
  - Allows gimbal data to "catch up" to video frame's physical motion
- Max shift clamp (`max_shift_pct`): prevents bottom-of-frame skipping and runaway corrections

---

### Gimbal Node Improvements

**Graceful Reconnection**
- `gimbal_node.py` now survives missing gimbal at startup
- `_connect()` method wraps serial open in try/except, retries every 5s
- Node stays alive and logs throttled warning instead of crashing
- Enables testing without gimbal connected; auto-recovers when plugged in

---

### Real-Time Stabilization Enhancements

**Sharpening & Deblurring**
- Added unsharp mask (Gaussian blur) after EMA stabilization in `realtime_stabilization.py`
- Function: `sharpen(frame)` — reduces blur, increases perceived sharpness

**Split EMA Tuning**
- Separated trajectory smoothing: `ALPHA_XY = 0.05` (translation), `ALPHA_YAW = 0.02` (rotation)
- Allows independent tuning of X/Y pan vs. rotation jitter
- Rotation tuned lower (0.02) for responsiveness; translation tuned higher for stability

**Topic Publishing**
- `/raw_frame/compressed`: unprocessed frames (feeds yaw_stabilizer)
- `/gimbal/angles`: yaw angle as optical flow estimates it (for downstream fusion)

---

### Yaw Stabilizer (New, In Progress)

**Current Implementation**
- Separate ROS2 node (`yaw_stabilizer.py`)
- Subscribes to `/raw_frame/compressed` and `/gimbal/angles`
- Applies **rotation correction** based on smoothed gimbal yaw (30-frame moving average)
- Publishes to `/yaw_stabilized/compressed`
- Currently uses hardcoded gain (`* 5`); being refined for physics-based model

**Planned Refinement**
- Replace rotation with **horizontal translation** model (from GIMBAL_INTEGRATION.md)
- Derive gain from camera FOV and fisheye projection, not empirical tuning
- See `GIMBAL_INTEGRATION.md` "Future Work" section for details

---

### Custom Gimbal Hardware (Parallel Project)

**Servo-Based PID Control**
- Location: `hardware/servoNoGUI/`
- Manual PID tuning for servo actuation (separate from Storm BCG)
- Demonstrates custom control authority and direct motor control
- Side project for learning/demonstration; not primary flight hardware

---

## Documentation Updates

- `GIMBAL_INTEGRATION.md`: Rewritten to reflect Mobius camera (1280×720, 150° FOV), yaw_stabilizer.py current state, and original dx-translation plan
- `rolling_shutter_node.py`: Docstring updated with all parameters (including compass_lag_frames, diagnostics)
- `rolling_shutter_compass_launch.py`: Header comments updated with compass_lag_frames tuning example
- `PROJECT_DIRECTION.md`: Updated "What We Have Built" and hardware constraints sections

---

## Key Technical Insights

**Why global flow replaced bands**: Rolling shutter differential (~2-5 px) is swamped by global panning motion (~50+ px) at typical speeds. Band-based optical flow couldn't reliably extract that tiny differential from noisy tracking. Global `estimateAffinePartial2D` measures the large dx easily, then applies physics formula to derive slope.

**Why frame buffer works for compass lag**: Gimbal's internal magnetometer filtering adds ~133ms latency to yaw values (measurements are from 133ms in the past). Buffering video frames delays processing, so by the time we process a frame, the gimbal data reflecting its physical motion has arrived. Uses latest history entries directly (no timestamp lookup needed).

**Why split EMA**: Rotation jitter is visually objectionable and should be smoothed aggressively. Pan translation can be intentional and should preserve user intent. Separate tuning lets rotation respond to shake while pans feel responsive.

---

## Testing & Verification

**Rolling Shutter** (optical_flow mode)
- Tested at 1280×720 with manual panning
- Side-by-side output shows original vs. corrected
- Correction visible but requires careful pan motion for observation

**Rolling Shutter** (compass mode)
- Diagnostics mode confirms gimbal data available and flowing
- Frame buffer (compass_lag_frames=4) delays output by ~133ms (acceptable)
- Peak detection tool shows compass_shift and flow_shift aligning when buffer active

**Gimbal Node**
- Tested with gimbal connected: publishes at 30 Hz, yaw continuous (no wrap)
- Tested with gimbal disconnected: retries gracefully, reconnects on plug-in

---

## Remaining Work / Next Steps

### Near-term (this sprint)
- Refine yaw_stabilizer to use physics-based horizontal translation (not hardcoded rotation gain)
- Integrate yaw_stabilizer with rolling shutter compass mode
- Test full pipeline: realtime_stabilization → yaw_stabilizer → rolling_shutter_node (or custom gimbal output)

### Medium-term (if time permits)
- IMU + optical flow fusion (Kalman filter or complementary filter)
  - IMU for high-frequency rotation, optical flow for low-frequency drift
  - Would give true 3-DOF virtual stabilization from 2-DOF gimbal
- Face detection + gimbal tracking (P-controller to lock faces in frame)
- Adaptive EMA: stronger smoothing during jitter, weaker during intentional pans

### Long-term (future semesters / research)
- Proper OpenCV fisheye calibration (cv2.fisheye.calibrate) with checkerboard
- Gimbal singularity mitigation (command smoothing at >90° pitch)
- Blur reduction via temporal frame alignment
- Multi-resolution pyramid for large motion tracking

---

## Metrics & Status

| Component | Status | Metric |
|---|---|---|
| Real-time stabilization | ✅ Working | ~30 fps, responsive to motion |
| Rolling shutter (optical flow) | ✅ Working | detects global dx, applies slope |
| Rolling shutter (compass) | ✅ Working | diagnoses available, frame buffer functional |
| Gimbal integration | ✅ Mostly done | receives yaw; yaw_stabilizer being refined |
| LED calibration | ✅ Complete | K_MS_PX = 52.4 across 7 resolutions |
| Diagnostics | ✅ Available | /rs_diagnostics publishes at 30 fps |
| Custom gimbal | 🔄 In progress | manual PID tuning underway |

---

## Code Quality Notes

- rolling_shutter_node.py is clean and modular (three input modes, two correction modes)
- gimbal_node.py gracefully handles disconnection (good UX)
- realtime_stabilization.py benefits from split EMA (more tunable)
- yaw_stabilizer.py is functional but needs physics-based model (in progress)
- All launch files are self-documenting and use LaunchConfiguration for runtime tuning

---

## Lessons Learned

1. **Global motion measurement is simpler and more robust than band-based differentials** — especially when global motion dominates the signal.
2. **Gimbal magnetometer filtering adds significant latency** — frame buffering is a practical solution when you can't change the hardware.
3. **Separate EMA smoothing coefficients are worth the extra tuning effort** — different motion types need different filtering.
4. **Modular node design (yaw_stabilizer as separate node) is more flexible** than trying to integrate everything into one pipeline.
