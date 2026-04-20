# Project Direction & Next Steps

## Current Project Status

### What We Have Built
- **Software-based stabilization**: Optical flow + partial affine transform + EMA smoothing (ALPHA=0.05) on cumulative trajectory
- **Gimbal hardware integration**: Storm BCG 8-bit gimbal with PID tuning (poles, motor config)
- **Gimbal communication**: Python script to decode and read raw SimpleBGC protocol packets (roll, pitch, yaw, IMU data)
- **Camera setup**: Wide field-of-view action camera mounted on gimbal
- **Computing target**: Nvidia Jetson (ROS2 ready, PWM-capable)

### Hardware Constraint We Can't Change
The gimbal firmware is **locked** — we cannot access or modify its internal stabilization logic. We can only:
- Send **target pitch angle** (-45° to +40°)
- Send **target roll angle** (±360°)
- Receive **IMU data** (roll/pitch/yaw rates, estimated angles)

This means gimbal improvements must come from **software-level control** (what angles we command), not firmware-level tuning.

### Key Challenge Identified
The gimbal exhibits instability when base pitch exceeds ~90° (where the pitch axis effectively becomes a yaw axis — gimbal singularity). The TA (David) flagged this as a potential research contribution if solvable.

---

## TA Feedback (David)

### Project Classification: "Research-Oriented"
- Move beyond basic stabilization toward "bleeding edge" techniques
- Focus on what distinguishes a robotics capstone from a standard project

### Suggested Advanced Techniques
1. **Blur reduction** — compensate for motion blur via temporal/frame alignment
2. **Rolling shutter reduction** — handle frame readout artifacts from rapid motion
3. **Gimbal singularity mitigation** — improve behavior at >90° pitch (the shake problem)

### Constraint Acknowledgment
David understood that gimbal firmware is locked — any solution must be **software-only control strategy**.

---

## Your Key Insights

### Software Yaw via Wide FOV + IMU Fusion
- Camera has ~150°+ FOV (wide angle) — contains useful information in the periphery
- Gimbal only provides 2 physical axes (pitch/roll)
- **Software-computed yaw** can use:
  - IMU yaw gyro rate from gimbal
  - Optical flow in periphery to detect pan-induced motion
  - Timestamp-synchronized fusion of both
- This effectively gives a **3-DOF virtual stabilization** from 2-DOF hardware

### Face Tracking for Smart Framing
- Detect faces, compute required pan/tilt to center subject
- Command gimbal angles to keep face locked in frame
- Prevents subject drift during camera motion
- Adds **intentional motion** (follows subject) vs. **unwanted motion** (shake)

---

## Realistic 4–5 Day Goals (Group of 4)

### ⭐ Must-Do (High Impact, Lower Risk)

#### 1. **PWM Control on Jetson** (1–2 days, 1 person)
- Implement hardware PWM output on Jetson (GPIO or PWM pin)
- Write ROS2 node: `gimbal_control_node`
  - Subscribe to desired pitch/roll angles
  - Map angles (pitch: -45 to +40, roll: ±360) to PWM duty cycles
  - Send PWM at ~50 Hz to gimbal
- **Deliverable**: Gimbal responds to ROS2 commands from Jetson (not just debug scripts)
- **Why now**: Prerequisite for all downstream work

#### 2. **IMU Data → ROS2 Topic** (1–2 days, 1 person)
- Package existing gimbal reader into proper ROS2 node (`gimbal_imu_node`)
- Publish on `/gimbal/imu` (IMUData or custom msg with: roll_rate, pitch_rate, yaw_rate, angles)
- Add basic frame synchronization with camera timestamps
- **Deliverable**: Clean topic-based interface for IMU data
- **Why now**: Enables fusion work and removes dependency on script-based hacks

---

### 🔥 High-Value Stretch (Pick 1–2 Based on Interest)

#### Option A: **IMU + Optical Flow Fusion for Yaw Stabilization** (2–3 days, 2 people)
**Goal**: Demonstrate 3-DOF virtual stabilization from 2-DOF gimbal

**Steps**:
1. Take optical flow output from existing `realtime_stabilization.py` (x/y motion in pixels)
2. Project pixel motion to world motion using camera intrinsics + known gimbal angles
3. Fuse with IMU yaw rate:
   - If `optical_flow_yaw > IMU_yaw_rate` → likely camera pan (desired) → don't fight it
   - If `optical_flow_yaw ≈ IMU_yaw_rate` → likely gimbal shake (undesired) → apply software crop/pan
4. Output: `yaw_correction_angle` to command gimbal roll to compensate for world yaw drift
5. **Benchmark**: Side-by-side video of original vs. 3-DOF stabilized (pans kept, shakes removed)

**Why impactful**: Directly addresses the "software yaw" idea; shows sensor fusion (class curriculum); bleeding-edge for a 2-axis gimbal.

#### Option B: **Face Detection + Gimbal Tracking** (2–3 days, 2 people)
**Goal**: Lock camera on detected face; command gimbal to keep face centered

**Steps**:
1. Enhance existing `face_detection_node`
2. Compute face center bounding box
3. Map face position (pixels) to gimbal command angle offset:
   - If face center offset left → increase roll to pan left
   - If face center offset up → increase pitch to tilt up
4. Add smooth ramp (P-controller): `angle_cmd = base_angle + K_p * center_error`
5. **Deliverable**: Live demo — face moves, gimbal follows; no shake
6. **Metric**: How many seconds until gimbal locks onto face; tracking smoothness

**Why impactful**: Combines computer vision + control (class topics); practical demo (impressive visually); natural progression from detection to tracking.

#### Option C: **Rolling Shutter Reduction via Frame Alignment** (2–3 days, 2 people)
**Goal**: Detect and compensate for rolling shutter artifacts during fast gimbal motion

**Steps**:
1. During high gimbal angular velocity, detect rolling shutter distortion (warped lines)
2. Use IMU rate as proxy for expected distortion magnitude
3. Apply per-scanline motion compensation:
   - Top scanlines: apply smaller warp correction (less time to move)
   - Bottom scanlines: apply larger warp correction (more time to move)
4. Blend back with optical flow-based compensation
5. **Deliverable**: Slow-motion video of gimbal yaw + rolling shutter before/after
6. **Metric**: Straightness of vertical edges in output frame

**Why impactful**: Addresses TA's "rolling shutter" suggestion; uses IMU data (class curriculum); visible quality improvement.

---

### 🎯 Nice-to-Have (If Time Permits)

- **Gimbal singularity handling**: When pitch > 85°, pre-emptively smooth gimbal command ramps to avoid instability. May require tuning or empirical limits.
- **Blur reduction via multi-frame stacking**: Align + blend frames during high acceleration to reduce motion blur (temporal super-resolution).
- **Adaptive EMA smoothing**: Increase ALPHA during detected motion (pans) to preserve intent; lower during jitter.

---

## Suggested 4–5 Day Sprint Plan

### Day 1
- **Morning**: PWM control on Jetson (person 1) + IMU → ROS2 node (person 2)
- **Afternoon**: Both working in parallel; merge into unified system
- **Goal**: By EOD, gimbal responds to ROS2 commands; IMU data on `/gimbal/imu` topic

### Days 2–4
- **Split into two parallel tracks**:
  - **Track A** (2 people): Pick one of [Option A, B, or C] above; build and benchmark
  - **Track B** (2 people or overlap): Either support Track A OR iterate on complementary features
- **Daily**: 15-min standup on blockers; integrate frequently
- **Goal**: Working demo by Day 4, EOD

### Day 5
- **Morning**: Final testing, edge cases, video recording
- **Afternoon**: Prepare presentation; document code; clean up
- **Goal**: Polished submission with clear metrics and before/after demos

---

## Recommended Priority (If You Can Only Pick One)

**Start with PWM + IMU node** (non-negotiable; enables everything else).

**Then pivot to Option A (IMU + Optical Flow Yaw)** because:
1. Directly uses gimbal hardware already in hand
2. Demonstrates sensor fusion (curriculum alignment)
3. Tangible metric: vertical straightness, less yaw drift
4. Reuses existing optical flow code (lower risk)
5. Addresses gimbal limitation creatively (software solution to hardware constraint)

---

## Success Metrics

### For PWM + IMU (Tier 1)
- ✅ Gimbal responds to angle commands from ROS2
- ✅ IMU data published at >20 Hz with <50ms latency
- ✅ Synchronization between camera frame timestamp and IMU read

### For Selected Stretch Goal (Tier 2)
- ✅ Before/after video comparison (measurable improvement in metric)
- ✅ Quantitative result: e.g., "yaw drift reduced by 65%", "face tracking lock-on time <2s"
- ✅ Code is modular; new node integrates cleanly with existing ROS2 stack

### For Presentation
- ✅ Clear explanation of constraint (locked firmware, 2-DOF hardware)
- ✅ Show how software addresses it (creative, within scope)
- ✅ Live demo OR high-quality recorded video with metrics overlaid

---

## Open Questions / Risk Mitigation

| Risk | Mitigation |
|---|---|
| PWM on Jetson not straightforward | Check Nvidia docs early; may need kernel module or GPIO library (wiringPi alternative) |
| IMU packet format still not 100% reliable | Keep gimbal reader script; periodically validate against inclinometer or known angles |
| Optical flow + IMU fusion is complex | Start with simple sum; validate each component separately before fusing |
| Face detection is slow | Test on Jetson (GPU available?); pre-size frames to 320×240 if needed |
| Gimbal shake at >90° is unfixable in software | Set realistic expectation: goal is to **reduce**, not eliminate; document the physical limit |

---

## Summary

Your project is at an inflection point:
- ✅ You've solved the "make hardware work" problem (gimbal communication, software stabilization)
- ✅ You've identified the constraint (locked firmware → software-only solutions)
- ❌ Now you need to show research-level contributions (not just "stabilize a camera")

**The winning path**: PWM + IMU node (Day 1–2) → IMU + Optical Flow Yaw OR Face Tracking (Days 2–5).

This pivots from "build the system" to "solve a novel problem creatively within constraints" — exactly what the TA is looking for in a capstone project.

Good luck! 🚀
