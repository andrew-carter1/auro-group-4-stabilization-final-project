# Launch Configurations Overview

## Quick Reference

| Launch File | Purpose | Gimbal | Viewers | Key Nodes |
|---|---|---|---|---|
| `stabilization_launch.py` | Legacy: basic stabilization + face detection | ❌ | 1 | stabilization_node, face_detection_node |
| `combined_launch.py` | Full USB camera + face detection pipeline | ❌ | 1 | usb_cam, stabilization_node, face_detection_node |
| `rolling_shutter_compass_launch.py` | RS correction only (gimbal yaw driven) | ✅ | 1 | gimbal_node, rolling_shutter_node |
| `rolling_shutter_compass_diagnostics_launch.py` | RS correction + diagnostics output | ✅ | 1 | gimbal_node, rolling_shutter_node |
| `rolling_shutter_raw_test_launch.py` | RS correction via optical flow (no gimbal) | ❌ | 1 | rolling_shutter_node |
| `demo_rs_yaw_launch.py` | Full RS + yaw stabilization (annotated) | ✅ | 2 | gimbal_node, rolling_shutter_node, yaw_stabilizer, demo_comparison_node |
| `demo_rs_yaw_clean_launch.py` | Full RS + yaw stabilization (clean output) | ✅ | 1 | gimbal_node, rolling_shutter_node, yaw_stabilizer, demo_comparison_node |

---

## Detailed Configs

### `stabilization_launch.py`
**Status**: Legacy  
**Gimbal Required**: No  
**Output Topics**:
- `/output/image` — stabilized frame (from stabilization_node)
- (Face detection output)

**Nodes**: stabilization_node, face_detection_node, rqt_image_view  
**Notes**: Basic optical flow stabilization. No rolling shutter correction, no yaw feedback.

---

### `combined_launch.py`
**Status**: Legacy  
**Gimbal Required**: No  
**Output Topics**:
- `/camera/image_raw` — raw USB camera
- `/output/image` — stabilized frame
- (Face detection output)

**Nodes**: usb_cam, stabilization_node, face_detection_node, rqt_image_view  
**Notes**: Adds USB camera driver to stabilization pipeline.

---

### `rolling_shutter_compass_launch.py`
**Status**: Active  
**Gimbal Required**: ✅ Storm BCG at `/dev/ttyACM0`

**Tuning Arguments**:
```
video_device              /dev/video4 (auto-detected Mobius)
gimbal_port              /dev/ttyACM0
compass_delay_sec        0.0         (time sync offset, increase to reduce vertical edge lean)
compass_lag_frames       2           (gimbal filter latency buffer)
max_shift_pct            0.10        (128 px limit at 1280 wide)
```

**Output Topics**:
- `/camera/raw/compressed` — raw camera (lag-aligned)
- `/rs_corrected_frame/compressed` — rolling shutter corrected

**Viewers**: 1 (rs_corrected_frame side-by-side)

**Rolling Shutter Node Params**:
- FOV: **100.0°** (baseline, compass mode)
- Resolution: 1280×720 @ 30 fps
- Mode: compass (gimbal yaw-driven)

**Notes**: Baseline rolling shutter correction using gimbal yaw. K_MS_PX = 52.4 ms, readout ≈ 29.5 ms.

---

### `rolling_shutter_compass_diagnostics_launch.py`
**Status**: Active (diagnostic variant)  
**Gimbal Required**: ✅ Storm BCG

**Tuning Arguments** (same as rolling_shutter_compass_launch.py):
```
compass_delay_sec        0.0
compass_lag_frames       4           (⚠️ different from compass_launch: 4 vs 2)
max_shift_pct            0.10
```

**Output Topics**:
- `/camera/raw/compressed` — raw camera
- `/rs_corrected_frame/compressed` — corrected frame
- `/rs_diagnostics` — real-time stats (yaw, yaw_rate, compass_shift, flow_shift, applied_shift)

**Viewers**: 1 (rs_corrected_frame side-by-side)

**Rolling Shutter Node Params**:
- FOV: **130.0°** (updated from 150°)
- Resolution: 1280×720 @ 30 fps
- Mode: compass
- diagnostics: True

**Notes**: Same as compass_launch but publishes `/rs_diagnostics` for real-time tuning. View with `ros2 topic echo /rs_diagnostics`.

---

### `rolling_shutter_raw_test_launch.py`
**Status**: Active  
**Gimbal Required**: ❌ (optical flow only)

**Tuning Arguments**:
```
video_device              /dev/video4 (auto-detected Mobius)
max_shift_pct            0.10        (0.10 = 128 px at 1280 wide)
slope_ema_alpha          0.0         (EMA on slope; 0.0 = off)
```

**Output Topics**:
- `/rs_corrected_frame/compressed` — rolling shutter corrected via optical flow

**Viewers**: 1 (rs_corrected_frame side-by-side)

**Rolling Shutter Node Params**:
- FOV: **130.0°** (updated from 150°)
- Resolution: 1280×720 @ 30 fps
- Mode: optical_flow
- min_features: 12

**Notes**: Isolated RS testing without gimbal. Useful for validating optical flow correction independently.

---

### `demo_rs_yaw_launch.py`
**Status**: Active (primary demo)  
**Gimbal Required**: ✅ Storm BCG

**Tuning Arguments**:
```
video_device              /dev/video4 (auto-detected Mobius)
gimbal_port              /dev/ttyACM0
compass_delay_sec        0.0
compass_lag_frames       1           (⚠️ latency: 1 vs 4 in clean variant)
max_shift_pct            0.10
max_margin_px            80          (yaw_stabilizer hardclamp)
yaw_lag_frames           0           (yaw stabilizer lag buffer)
reference_alpha          0.06        (EMA update rate; higher = faster return to center)
p_gain                   1.0         (proportional correction)
d_gain                   0.2         (derivative damping)
rs_annotations           true        (rolling shutter side-by-side labels)
yaw_annotations          true        (yaw stabilizer labels + stats)
cmp_annotations          true        (comparison node labels)
```

**Output Topics**:
- `/camera/raw/compressed` — raw camera
- `/rs_corrected_frame/compressed` — rolling shutter corrected
- `/rs_comparison/compressed` — side-by-side RS with slope lines
- `/yaw_stabilized/compressed` — yaw-stabilized output (960×720 cropped)
- `/demo_comparison/compressed` — raw | yaw comparison (2-panel)
- `/demo_full_pipeline/compressed` — raw | rs | yaw (3-panel, half-size)

**Viewers**: 2
1. `/rs_comparison` — RS correction diagnostic (slope lines + green references)
2. `/demo_full_pipeline` — end-to-end 3-panel view

**Nodes**:
- gimbal_node
- rolling_shutter_node (compass mode, FOV 130°)
- yaw_stabilizer (EMA + PD control, FOV 100°)
- demo_comparison_node
- 2× rqt_image_view

**Notes**: Full annotated pipeline. Primary demo configuration. Shows all intermediate stages.

---

### `demo_rs_yaw_clean_launch.py`
**Status**: Active (clean output variant)  
**Gimbal Required**: ✅ Storm BCG

**Tuning Arguments** (same as demo_rs_yaw_launch.py):
```
compass_lag_frames       4           (⚠️ different from launch: 4 vs 1)
reference_alpha          0.06        (updated from 0.08)
```

**Output Topics** (same as annotated, but without annotations):
- `/camera/raw/compressed`
- `/rs_corrected_frame/compressed`
- `/yaw_stabilized/compressed`
- `/demo_comparison/compressed`

**Viewers**: 1 (clean demo_comparison only, no RS diagnostics)

**Nodes**:
- gimbal_node
- rolling_shutter_node (compass mode, FOV 130°)
- yaw_stabilizer (EMA + PD control, FOV 100°)
- demo_comparison_node
- 1× rqt_image_view

**Key Differences from Annotated**:
- `show_annotations` disabled on all nodes
- Only one viewer (raw | yaw 2-panel, no labels)
- compass_lag_frames: 1 → **4** (latency compensation difference)
- No RS diagnostic side-by-side view

**Notes**: Clean output for recording/presentation. All data processing identical to annotated variant.

---

## Consistency Changes (2026-04-30)

### FOV Standardization
- Rolling shutter correction standardized to **130°** (except compass_launch at 100°)
  - Rationale: Consistent across optical flow and compass-driven RS correction
  - Exception: `rolling_shutter_compass_launch.py` left at 100° (baseline mode)
  
### Reference Alpha (Yaw Stabilizer)
- Updated to **0.06** in both demo launches (was 0.08)
  - Effect: Faster return to center, more aggressive jitter rejection
  - See yaw_stabilizer.py line 12: `Higher alpha = faster return to center`

### Compass Lag Frames (Latency Tuning)
- Left as-is per launch; observed latency differences justified per-config tuning:
  - demo_rs_yaw_clean_launch.py: **4 frames** (≈133 ms gimbal filter lag)
  - demo_rs_yaw_launch.py: **1 frame**
  - rolling_shutter_compass_launch.py: **2 frames**
  - rolling_shutter_compass_diagnostics_launch.py: **4 frames**

---

## Yaw Stabilizer Reference Crosshairs

**File**: `stabilization_pkg/yaw_stabilizer.py`  
**Status**: Fixed (2026-04-30)

Crosshairs now anchored to **input image** positions (0.25w, 0.5w, 0.75w) instead of output crop. They remain stable on the fisheye reference grid and only draw when visible in the active crop window.

---

## Running the Demos

```bash
cd software/ros2_stabilization_ws/
source install/setup.bash

# Full annotated pipeline (primary demo)
ros2 launch stabilization_pkg demo_rs_yaw_launch.py

# Clean output for recording
ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py

# RS correction only (compass mode, gimbal required)
ros2 launch stabilization_pkg rolling_shutter_compass_launch.py

# RS correction with diagnostics
ros2 launch stabilization_pkg rolling_shutter_compass_diagnostics_launch.py

# RS correction via optical flow (no gimbal)
ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py

# Tune compass sync delay
ros2 launch stabilization_pkg rolling_shutter_compass_launch.py compass_delay_sec:=0.03

# Tune yaw stabilizer reference alpha (faster return)
ros2 launch stabilization_pkg demo_rs_yaw_launch.py reference_alpha:=0.08
```
