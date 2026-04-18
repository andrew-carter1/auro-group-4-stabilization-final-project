# Gimbal Integration with Stabilization Node

## Overview

The current `realtime_stabilization.py` is **vision-only**: it estimates camera motion from optical flow alone and corrects it with an EMA filter. The gimbal node (`gimbal_node.py`) publishes physical angle measurements from the Storm BCG hardware on `/gimbal/angles`.

The gimbal controls roll and pitch mechanically, but **has no hardware yaw control**. Yaw (compass direction, from the magnetometer) drifts freely and is the primary target for software correction. The gimbal reads yaw to **0.1° resolution**, making it precise enough to use as a direct correction signal.

The stabilization warp matrix is built from `[dx, dy, da]`:
- `dx` — horizontal translation (pixels)
- `dy` — vertical translation (pixels)
- `da` — in-plane rotation (radians)

Yaw drift shows up in the image primarily as **horizontal drift (`dx`)**. When the drone yaws right, the scene appears to pan left in the frame.

---

## Yaw-to-Pixel Conversion

Converting a yaw delta to a pixel shift requires knowing the horizontal FOV. The current camera is ~160° horizontal FOV at 320px wide.

Using a simple pinhole approximation:

```
f_px = (image_width / 2) / tan(FOV_horizontal / 2)
     = 160 / tan(80°)
     ≈ 28.2 pixels (effective focal length)

dx_pixels = f_px * tan(d_yaw_radians)
```

At 160° FOV, 1° of yaw ≈ **0.5 pixels** at 320px width. This is a small but cumulative correction — without it, slow yaw drift will walk the frame off-center over time.

> If the camera FOV is measured more precisely later, update `FOV_HORIZONTAL_DEG` below.

---

## Integration: Yaw Correction in `realtime_stabilization.py`

### Step 1 — Subscribe to gimbal angles

```python
from geometry_msgs.msg import Vector3Stamped
import math

# In __init__:
FOV_HORIZONTAL_DEG = 160.0  # update if measured more precisely
self.f_px = (self.width / 2.0) / math.tan(math.radians(FOV_HORIZONTAL_DEG / 2.0))

self.latest_gimbal = None
self.prev_yaw_rad = None

self.create_subscription(
    Vector3Stamped,
    '/gimbal/angles',
    self._gimbal_cb,
    10
)

def _gimbal_cb(self, msg):
    self.latest_gimbal = msg
```

### Step 2 — Apply yaw correction in `process_frame`

After the optical flow block computes `current_transform`, override `dx` with the yaw-derived correction:

```python
if self.latest_gimbal is not None:
    yaw_rad = math.radians(self.latest_gimbal.vector.z)

    if self.prev_yaw_rad is not None:
        d_yaw = yaw_rad - self.prev_yaw_rad
        dx_yaw = self.f_px * math.tan(d_yaw)

        # Replace optical flow horizontal translation with physical measurement.
        # To blend instead: current_transform[0] = YAW_WEIGHT * dx_yaw + (1 - YAW_WEIGHT) * current_transform[0]
        current_transform[0] = dx_yaw

    self.prev_yaw_rad = yaw_rad
```

### Blending vs. replacing

| Approach | When to use |
|---|---|
| Full replacement (`current_transform[0] = dx_yaw`) | Gimbal yaw is trusted; optical flow prone to drift |
| Blend with weight | Tuning phase — start at weight 0.0 (pure optical flow) and increase |
| Fallback only | Use yaw correction only when optical flow feature count drops below `MIN_FEATURES` |

Start with a blend weight of `0.0` to confirm nothing breaks, then increase.

---

## Roll and Pitch (Secondary)

Since the gimbal *mechanically* corrects roll and pitch, the camera should already have minimal roll/pitch motion by the time optical flow sees the frame. These are lower priority than yaw, but if mechanical stabilization is imperfect:

- **Roll** (`vector.x`) → maps to `da` (in-plane rotation). Delta roll in radians ≈ correction angle directly.
- **Pitch** (`vector.y`) → maps to `dy` (vertical translation), similar conversion to yaw using vertical FOV.

---

## Rate Mismatch

The stabilization node runs at ~15 fps. The gimbal node runs at 30 Hz. `_gimbal_cb` fires roughly 2× per stabilization frame — this is fine because the callback just caches the latest value and `process_frame` always reads the most recent one. No synchronization needed in a single-threaded executor.

---

## Sign Convention

Verify before trusting the correction:

- Gimbal yaw increases clockwise (looking down) — drone nose swings right → `vector.z` increases
- In the image, drone nose swinging right → scene moves left → `dx` should be negative

You may need to negate:
```python
dx_yaw = -self.f_px * math.tan(d_yaw)
```

Test by slowly rotating the rig clockwise and checking whether the correction moves the frame in the right direction.

---

## Quick Start Checklist

1. `colcon build && source install/setup.bash`
2. Terminal 1: `ros2 run stabilization_pkg gimbal_node`
3. Terminal 2: `ros2 topic echo /gimbal/angles` — confirm yaw is changing as expected
4. Apply Step 1 and Step 2 changes to `realtime_stabilization.py` with blend weight `0.0` first
5. Verify no regressions, then increase weight
6. Check sign convention (see above) if correction appears inverted
