# Gimbal Integration with Stabilization Node

## Overview

The current `realtime_stabilization.py` is **vision-only**: it estimates camera motion from optical flow alone and corrects it with an EMA filter. The gimbal node (`gimbal_node.py`) publishes physical angle measurements from the Storm BCG hardware on `/gimbal/angles`. Fusing these two sources should improve stabilization — especially when the scene is low-texture or fast-moving, which both cause optical flow to degrade.

The key insight is that the stabilization warp matrix is built from a 3-vector `[dx, dy, da]`:
- `dx`, `dy` — translation, estimated by optical flow
- `da` — rotation (radians), estimated by optical flow

The gimbal **roll angle** is a direct physical measurement of `da`. Pitch and yaw can inform `dy` and `dx` as secondary signals.

---

## Integration Approaches

### Option A: Gimbal-only rotation (simplest)

Replace the optical-flow rotation estimate with the gimbal roll delta.

**Changes to `realtime_stabilization.py`:**

1. Add a subscription to `/gimbal/angles` and cache the latest reading:

```python
from geometry_msgs.msg import Vector3Stamped
import math

# In __init__:
self.latest_gimbal = None
self.prev_gimbal_roll = None
self.create_subscription(
    Vector3Stamped,
    '/gimbal/angles',
    self._gimbal_cb,
    10
)

def _gimbal_cb(self, msg):
    self.latest_gimbal = msg
```

2. In `process_frame`, after the optical flow block, override `da` with the gimbal roll delta:

```python
# After current_transform is estimated from optical flow...

if self.latest_gimbal is not None:
    roll_deg = self.latest_gimbal.vector.x
    roll_rad = math.radians(roll_deg)

    if self.prev_gimbal_roll is not None:
        da_gimbal = roll_rad - self.prev_gimbal_roll
        # Replace optical flow rotation with physical measurement
        current_transform[2] = da_gimbal

    self.prev_gimbal_roll = roll_rad
```

**Trade-offs:**
- Simple, one change
- Fully trusts gimbal hardware — if the gimbal drifts or glitches, the warp will too
- Loses the EMA smoothing benefit on rotation (since gimbal is already stabilized hardware)

---

### Option B: Blended rotation (recommended starting point)

Weight optical flow and gimbal roll together. This lets you tune confidence in each source.

```python
GIMBAL_WEIGHT = 0.7  # 0.0 = pure optical flow, 1.0 = pure gimbal

if self.latest_gimbal is not None and self.prev_gimbal_roll is not None:
    roll_rad = math.radians(self.latest_gimbal.vector.x)
    da_gimbal = roll_rad - self.prev_gimbal_roll
    current_transform[2] = (
        GIMBAL_WEIGHT * da_gimbal +
        (1.0 - GIMBAL_WEIGHT) * current_transform[2]
    )
    self.prev_gimbal_roll = roll_rad
```

This is safer during testing — a `GIMBAL_WEIGHT` of 0.0 falls back to the existing behavior exactly.

---

### Option C: Gimbal as fallback when optical flow fails

Use optical flow rotation when tracking is healthy, and fall back to gimbal when feature count drops below threshold. This is the most robust for production use.

```python
if len(idx) >= self.MIN_FEATURES and self.latest_gimbal is not None:
    # Optical flow is healthy — blend as in Option B
    ...
elif self.latest_gimbal is not None and self.prev_gimbal_roll is not None:
    # Too few features — use gimbal alone
    roll_rad = math.radians(self.latest_gimbal.vector.x)
    current_transform[2] = roll_rad - self.prev_gimbal_roll
    self.prev_gimbal_roll = roll_rad
```

---

## Rate Mismatch

The stabilization node runs at ~15 fps (timer at 33 ms). The gimbal node runs at 30 Hz. This means `_gimbal_cb` fires roughly 2x per stabilization frame.

- The subscription callback just caches the latest value (`self.latest_gimbal`), so no data is lost — `process_frame` always reads the most recent gimbal angle when it runs.
- No synchronization primitives are needed since both run in the same single-threaded executor.

If you move to a `MultiThreadedExecutor` later, add a `threading.Lock` around `self.latest_gimbal`.

---

## Coordinate Frame

Verify sign conventions before trusting the blend:

| Gimbal field | `vector.x` = roll | `vector.y` = pitch | `vector.z` = yaw |
|---|---|---|---|
| Positive direction | clockwise (from camera's POV) | nose up | nose right |

The optical flow `da` is in radians, positive = counter-clockwise (OpenCV convention). You may need to negate the gimbal roll:

```python
da_gimbal = -(roll_rad - self.prev_gimbal_roll)
```

Test this by tilting the rig clockwise while watching the correction direction.

---

## Quick Start Checklist

1. `colcon build && source install/setup.bash`
2. In one terminal: `ros2 run stabilization_pkg gimbal_node`
3. In another: `ros2 topic echo /gimbal/angles` — confirm data is flowing
4. Apply Option B changes to `realtime_stabilization.py` with `GIMBAL_WEIGHT = 0.0` first and verify nothing breaks
5. Gradually increase `GIMBAL_WEIGHT` while watching the stabilized output
6. Check sign convention (see above) if the correction looks inverted
