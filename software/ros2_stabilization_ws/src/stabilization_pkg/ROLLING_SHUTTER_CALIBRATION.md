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
- LED frequency: **200 Hz** (period = 5 ms, half-period = 2.5 ms per band) -
  ACTUALLY 5.04,  BUT 5.00 estimate is close enough
- Measurement: count the vertical pixels spanning **4 bright + 4 dark bands**
  = 4 full LED cycles = **20 ms** of sensor readout time

The test was repeated at every resolution the camera supports.

---

## Observed Result

Alternating bright and dark horizontal bands appear across the frame, as
expected. Each group of 4 bright bands + 4 dark bands spans 20 ms of readout.

![Alternating bright and dark bands from 200 Hz LED calibration](rolling_shutter.png)

---

## Measurements

Vertical pixels spanning 4 bright + 4 dark bands at each resolution:

| Resolution  | Aspect | Measured (px) | Readout Time        |
|-------------|--------|---------------|---------------------|
| 1920 × 1080 | 16:9   | 743           | (1080/743) × 20 = 29.1 ms |
| 1280 × 960  | 4:3    | 486           | (960/486)  × 20 = 39.5 ms |
| 1080 × 720  | 3:2    | 491           | (720/491)  × 20 = 29.3 ms |
| 1024 × 768  | 4:3    | 390           | (768/390)  × 20 = 39.4 ms |
| 1024 × 576  | 16:9   | 389           | (576/389)  × 20 = 29.6 ms |
|  848 × 480  | 16:9   | 320           | (480/320)  × 20 = 30.0 ms |
|  720 × 480  | 3:2    | 279           | (480/279)  × 20 = 34.4 ms |
|  640 × 480  | 4:3    | 242           | (480/242)  × 20 = 39.7 ms |

---

## Key Insight: Readout Time Depends on Width

The full sensor is always active at every output resolution — lower resolutions
are the same sensor data scaled down. Because wider output rows have more sensor
pixels to clock out per row, the per-output-row readout time is proportional to
**1 / width**. This means the total frame readout time scales with the **aspect
ratio** (height / width) rather than height alone.

A single constant **K_MS_PX** captures this:

```
per_row_ms           = K_MS_PX / width
readout_time_ms(H,W) = K_MS_PX × H / W
```

K values derived per resolution:

| Resolution  | K = readout_ms × W / H |
|-------------|-------------------------|
| 1920 × 1080 | 51.69                   |
| 1280 × 960  | 52.67                   |
| 1024 × 768  | 52.51                   |
| 1024 × 576  | 52.66                   |
|  848 × 480  | 53.00                   |
|  720 × 480  | 51.61                   |
|  640 × 480  | 52.89                   |

> **1080 × 720** was excluded — its K value (44.0) was a clear outlier,
> likely due to a bad measurement.

**Mean K_MS_PX ≈ 52.4 ms**

---

## Readout Times Using K = 52.4

```
readout_time_ms = 52.4 × height / width
```

| Resolution  | Aspect | Readout Time |
|-------------|--------|--------------|
| 1920 × 1080 | 16:9   | 29.5 ms      |
| 1280 × 960  | 4:3    | 39.3 ms      |
| 1024 × 768  | 4:3    | 39.3 ms      |
| 1024 × 576  | 16:9   | 29.5 ms      |
|  848 × 480  | 16:9   | 29.7 ms      |
|  720 × 480  | 3:2    | 34.9 ms      |
|  640 × 480  | 4:3    | 39.3 ms      |

> **Note on 4:3 modes:** The ~39 ms readout time exceeds one 30 fps frame
> period (33.3 ms). This likely means the camera runs at ~25 fps in 4:3
> modes. The readout time measurement is still valid — it reflects the
> physical sensor behavior at that mode.

---

## How This Is Used in the Code

The constant `K_MS_PX` in [rolling_shutter_node.py](stabilization_pkg/rolling_shutter_node.py)
is derived directly from this calibration. The node auto-detects the
incoming frame dimensions and computes the appropriate readout time at
runtime — no manual parameter needed for resolution changes.

In `optical_flow` mode, the readout time sets the scale for interpreting
the rolling shutter slope estimated from scanline band optical flow.

In `compass` mode, it determines how much total pixel shift accumulates
between the first and last row of the frame at the measured yaw rate.

---

## Known Issues and Observations

### Correction is under/over-applied (more pronounced shutter than expected)

The optical flow estimator fits a single global slope `dx(row) = intercept + slope × row`
across all bands. This linear model assumes the distortion is uniform and continuous,
but real rolling shutter distortion during fast or unpredictable motion can be
non-linear. Possible contributors:

- **LK tracking fails at large displacements.** Lucas-Kanade optical flow assumes
  small inter-frame motion. A sudden yaw snap moves features far enough that the
  tracker loses them or returns wrong matches, producing a bad slope estimate
  that over- or under-corrects.
- **Fisheye non-linearity.** The equidistant model (`r = f × θ`) is an approximation.
  Near the frame edges, the true mapping diverges, so the same angular yaw rate
  produces a different pixel shift than the linear model predicts. This means the
  correction applied at the edges is wrong even if the slope is correct at the center.
- **Vibration aliases as rolling shutter.** High-frequency vibration that the gimbal
  does not damp (especially propeller-frequency oscillations) appears as a rolling
  shutter-like skew to the optical flow estimator, so it tries to "correct" something
  that is actually random noise.

### Bottom ~1/4 of the frame "skips" during fast movement

The accumulated remap shift grows linearly with row number. At the bottom of the
frame, `shift = slope × (height − 1)` is at its maximum. Several things can go wrong:

- **Border replication.** When the warp maps bottom rows to pixel coordinates
  outside the frame, `cv2.remap` with `BORDER_REPLICATE` fills those pixels by
  repeating the edge column, which looks like a frozen or "skipped" strip.
- **Slope is overestimated during sudden motion.** A single bad band (where LK
  tracking fails) can pull the linear regression to an extreme slope, causing the
  bottom rows to be warped far off-screen.
- **Too few bands at the bottom.** With 6 bands and a 960-pixel-tall frame, each
  band covers 160 rows. If the bottom band has few trackable features (e.g., sky
  or a featureless region), that data point is dropped, leaving the regression
  extrapolating unconstrained into the bottom of the frame.

---

## Future Improvements

### Use gimbal magnetometer (yaw) for correction — highest priority

The gimbal already measures absolute yaw via its onboard magnetometer and publishes
it to `/gimbal/angles`. Using this directly would eliminate the optical flow
estimation step entirely:

```
yaw_rate = (yaw_now − yaw_prev) / dt          # from /gimbal/angles
shift_per_row = f_eq × yaw_rate_rad × readout_time / height
```

This is the `compass` mode already implemented in `rolling_shutter_node.py`.
Advantages over optical flow:

- **Not affected by fast or unpredictable motion** — the gimbal measures the
  physical yaw regardless of scene content.
- **No feature tracking failures** — works in low-texture scenes.
- **No slope overestimation** — the slope comes from a physical sensor, not a
  regression over noisy optical flow values.
- **Matches the correction to the actual cause** — rolling shutter is caused by
  camera rotation, not by image content changes.

The main requirement is that the gimbal yaw data arrives at sufficient rate (≥30 Hz)
and is timestamped accurately relative to the image frames.

### Clamp the maximum remap shift

Add a safety clamp on the computed slope so that the total shift at the bottom
row never exceeds some fraction of the frame width (e.g., 10%). This prevents
a single bad optical flow estimate from causing the bottom-of-frame skipping
artifact, at the cost of under-correcting in extreme cases.

### Increase band count or use feature density weighting

More bands (e.g., 10–12 instead of 6) give the linear regression more data
points and reduce the impact of any one bad band. Alternatively, weight each
band's contribution to the regression by the number of successfully tracked
features in that band, so bands with more reliable flow have more influence.

### Proper fisheye lens calibration

A full OpenCV fisheye calibration (`cv2.fisheye.calibrate`) with a checkerboard
pattern would replace the equidistant approximation with per-pixel distortion
coefficients. This would improve correction accuracy near the edges of the 150°
FOV where the approximation is weakest.
