# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2-based autonomous robotics project with three loosely coupled systems:
1. **Video stabilization** — software-based gimbal stabilization using optical flow
2. **Drone control** — PX4 offboard control via MAVROS
3. **Gimbal communication** — serial interface to a Storm BCG gimbal

---

## Build & Run Commands

### Software Stack (Stabilization + Face Detection)

```bash
cd software/ros2_stabilization_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# Run nodes
ros2 run stabilization_pkg stabilization_node
ros2 run stabilization_pkg realtime_stabilization
ros2 run face_detection face_detection_node

# Or use the launch file (starts stabilization + face detection + rqt_image_view)
ros2 launch stabilization_pkg stabilization_launch.py
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

**stabilization_pkg** — main stabilization node:
- `realtime_stabilization.py` — captures from USB webcam (V4L2 device 0, 320×240 @ 15fps MJPG), runs Lucas-Kanade optical flow + partial affine estimation, applies EMA smoothing (ALPHA=0.05) on cumulative trajectory, then publishes to `/stabilized_frame/compressed`. Also displays a side-by-side view with shake graph.
- `stabilization_node.py` — legacy node that subscribes to `/image_raw` and publishes to `output/image`.
- `launch/stabilization_launch.py` — starts stabilization_node, face_detection_node, and rqt_image_view.

**face_detection** — subscribes to stabilized frames and runs Haar Cascade detection, publishes to `/image_with_faces/compressed`.

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
| `/stabilized_frame/compressed` | CompressedImage | pub by stabilization_pkg |
| `/image_with_faces/compressed` | CompressedImage | pub by face_detection |
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

- The `stabilization_pkg/setup.py` has no `entry_points` yet — nodes are run directly with `ros2 run` using the installed scripts, or invoked manually.
- PX4-Autopilot must be cloned separately into `hardware/` (not committed to this repo).
- `software/hw.py` is a standalone (non-ROS2) face detection script for quick testing.
- `examples/real_time_stabilization/` contains older algorithm prototypes for reference.
