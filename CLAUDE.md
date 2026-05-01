# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2-based autonomous robotics project with three loosely coupled systems:
1. **Video stabilization** — software-based gimbal stabilization using optical flow
2. **Drone control** — PX4 offboard control via MAVROS
3. **Gimbal communication** — serial interface to a Storm BCG gimbal

---

## Build & Run Commands

### Software Stack (Rolling Shutter + Yaw Stabilization + Face Detection DNN)

```bash
cd software/ros2_stabilization_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# Run with rolling shutter + yaw stabilization + DNN face detection
ros2 launch stabilization_pkg demo_rs_yaw_launch.py

# Or clean version without annotations
ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py
```

### Hardware Stack (Drone Control)

```bash
cd hardware/px4_ws/
colcon build
source install/setup.bash

# Launch MAVROS (see hardware/README.txt for full SITL demo walkthrough)
ros2 run drone_control offboard_node
```

### Tests & Linting

```bash
# From a workspace root (software or hardware)
colcon test
colcon test-result --verbose

# Individual package lint check
ament_flake8 src/<package_name>
ament_pep257 src/<package_name>
```

### Gimbal Reader (Standalone Script)

```bash
cd examples/gimbal_communication/
pip install pyserial
python3 gimbal_raw_reader.py  # reads roll/pitch/yaw from /dev/ttyACM0
```

---

## Architecture

### Software Workspace (`software/ros2_stabilization_ws/src/`)

**stabilization_pkg** — complete video stabilization and detection pipeline:
- `rolling_shutter_node.py` — corrects rolling shutter distortion using compass gimbal data (or optical flow), applies per-row horizontal warp via cv2.remap().
- `yaw_stabilizer.py` — applies yaw stabilization via EMA reference tracking + PD control, sliding crop window at 960×720 resolution.
- `gimbal_node.py` — reads gimbal angles from Storm BCG board at 60 Hz via SimpleBGC Serial API, publishes to `/gimbal/angles`.
- `face_detection_dnn_node.py` — DNN-based face detection (YOLO or similar), publishes detected faces.
- `demo_comparison_node.py` — creates side-by-side and multi-panel comparison views for visualization.
- Launch files: `demo_rs_yaw_launch.py` (annotated) and `demo_rs_yaw_clean_launch.py` (clean output).

### Hardware Workspace (`hardware/px4_ws/src/drone_control/`)

**offboard_node.py** — maintains a 2m hover:
- Publishes `PoseStamped` to `/mavros/setpoint_position/local` at 20 Hz
- Subscribes to `/mavros/state`
- Calls `/mavros/cmd/arming` (CommandBool) and `/mavros/set_mode` (SetMode) services
- See `hardware/README.txt` for full 4-terminal SITL demo sequence

### Gimbal Communication (`examples/gimbal_communication/`)

Serial interface to Storm BCG gimbal at `/dev/ttyACM0`, 115200 baud, SimpleBGC Serial API v2:
- Command `CMD_REALTIME_DATA` (0x44) returns roll/pitch/yaw angles and gyro data at ~10 Hz
- Packet format: `[0x3E][CMD][SIZE][CRC][DATA...][CRC]`
- Roll/Pitch resolution: 0.18°/unit; Yaw: 0.1°/unit

### Key Topics & Services

| Name | Type | Direction |
|---|---|---|
| `/gimbal/angles` | Vector3Stamped | pub by gimbal_node |
| `/rs_corrected_frame/compressed` | CompressedImage | pub by rolling_shutter_node |
| `/yaw_stabilized/compressed` | CompressedImage | pub by yaw_stabilizer |
| `/demo_full_pipeline/compressed` | CompressedImage | pub by demo_comparison_node |
| `/mavros/state` | mavros_msgs/State | sub by offboard_node |
| `/mavros/setpoint_position/local` | PoseStamped | pub by offboard_node |
| `/mavros/cmd/arming` | CommandBool (srv) | called by offboard_node |
| `/mavros/set_mode` | SetMode (srv) | called by offboard_node |

---

## Stabilization Algorithm Details

The core algorithm in `realtime_stabilization.py`:
1. `cv2.goodFeaturesToTrack` — 200 corners, minDistance=30
2. `cv2.calcOpticalFlowPyrLK` — Lucas-Kanade tracking
3. `cv2.estimateAffinePartial2D` — partial affine (prevents unrealistic shear)
4. EMA on **cumulative trajectory** (not per-frame delta): `smooth = ALPHA * smooth + (1-ALPHA) * raw`
5. 1.1× zoom crop to hide black border edges from warping

See `Progress_Apr_8.md` for design rationale and planned improvements (IMU+vision fusion, face-aware motion masking, adaptive smoothing).

---

## Important Notes

- Gimbal node runs at 60 Hz (configurable); yaw data fed to both rolling_shutter_node and yaw_stabilizer.
- Rolling shutter correction: compass mode (gimbal-based) is preferred over optical_flow mode.
- Yaw stabilization uses EMA reference tracking + PD control with weighted 3-frame dx averaging.
- Launch files default `yaw_lag_frames=0`; can be increased for gimbal lag compensation.
- PX4-Autopilot must be cloned separately into `hardware/` (not committed to this repo).
- `examples/real_time_stabilization/` contains older algorithm prototypes for reference.
