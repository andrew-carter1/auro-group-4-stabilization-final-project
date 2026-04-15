# Rolling Shutter Calibration

## What is Rolling Shutter?

CMOS camera sensors do not capture the entire frame at a single instant.
Instead, they read out one row at a time, top to bottom, over a period called
the **readout time**. On a fast-moving platform (like a drone), this means
the top and bottom of any given frame were captured at slightly different
camera orientations. The result is a skewed or "leaned" image — objects that
should be vertical appear slanted.

For this project, yaw is the dominant axis because roll and pitch are
corrected mechanically by the gimbal. Yaw drift is left to software correction.

---

## Calibration Setup

To measure the sensor readout time, a single LED was blinked at a known
frequency using a microcontroller. The camera was pointed at the LED in a
darkened room while held completely still.

Because the camera reads rows sequentially, rows that were scanned while the
LED was ON appear bright, and rows scanned during the OFF phase appear dark.
This produces visible horizontal bands across the captured frame.

**Setup parameters:**
- LED frequency: **200 Hz** (period = 5 ms, half-period = 2.5 ms per band)
- Camera mode: **1920×1080, 16:9, 30 fps**

---

## Observed Result

The captured frames showed alternating bright and dark horizontal bands of
uniform width, as expected. The brightness of each bright band varied with the
angle of the LED relative to the camera (intensity falloff), but the
**combined width of one bright + one dark band was consistent** at
approximately **172 pixels** regardless of LED angle.

![Alternating bright and dark bands from 200 Hz LED calibration](rolling_shutter.png)

Each bright+dark pair represents one full LED cycle (5 ms), spread across
172 rows of the sensor.

---

## Readout Time Calculation

```
readout_time = frame_height / (LED_frequency × pixels_per_cycle)

readout_time = 1080 / (200 Hz × 172 px/cycle)
             = 1080 / 34400
             = 0.03140 seconds
             ≈ 31.4 ms
```

Per-row readout time (hardware constant):

```
per_row_ms = 31.4 ms / 1080 rows ≈ 0.02907 ms/row
```

---

## Scaling to Other Resolutions

Because the per-row readout time is a fixed hardware property of the sensor,
the total readout time for any resolution scales linearly with frame height:

```
readout_time_ms(H) = 0.02907 ms/row × H rows
```

| Resolution  | Aspect | Height | Readout Time |
|-------------|--------|--------|--------------|
| 1920 × 1080 | 16:9   | 1080   | 31.4 ms      |
| 1280 × 720  | 16:9   |  720   | 20.9 ms      |
|  854 × 480  | 16:9   |  480   | 14.0 ms      |
| 1280 × 960  | 4:3    |  960   | 27.9 ms      |
|  960 × 720  | 4:3    |  720   | 20.9 ms      |
|  640 × 480  | 4:3    |  480   | 14.0 ms      |

> **Note:** The 4:3 mode uses more of the sensor's physical height than the
> 16:9 crop mode. If the camera reads out a different number of raw sensor
> rows internally when switching aspect ratios, the per-row constant may
> differ slightly between modes and should be verified with a separate LED
> test in 4:3 mode.

---

## How This Is Used in the Code

The constant `PER_ROW_READOUT_MS` in [rolling_shutter_node.py](stabilization_pkg/rolling_shutter_node.py)
is derived directly from this calibration. The node auto-detects the
incoming frame height and computes the appropriate readout time at runtime —
no manual parameter needed for resolution changes.

In `optical_flow` mode, the readout time sets the scale for interpreting
the rolling shutter slope estimated from scanline band optical flow.

In `compass` mode, it determines how much total pixel shift accumulates
between the first and last row of the frame at the measured yaw rate.

---

## Future Work

The calibration above assumes the sensor reads rows at a perfectly uniform
rate. In practice, some sensors have a brief blanking interval at the end of
each frame before the next readout begins. If the band widths are not
perfectly uniform across the frame, a row-by-row timing model (rather than
a single linear constant) may be needed.

Additionally, the 170° fisheye lens introduces significant radial distortion.
A full lens calibration using `cv2.fisheye.calibrate` with a checkerboard
pattern would provide per-pixel distortion coefficients, improving the
accuracy of the yaw-to-pixel conversion — especially near the frame edges
where the equidistant approximation currently used becomes less accurate.
