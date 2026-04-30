# Kalman Filter Approach for Face Tracking

## Overview

This document explains the constant-velocity (CV) Kalman filter used in the gimbal face-tracking system. The filter is based on the linear Kalman filter implementations from `HW5/Part_BC/KF_2D.py` and `KF_4D.py`, adapted for angular face position tracking in the image.

---

## State Model

The filter tracks face position as **angles from the image center**, plus angular velocities:

$$x = \begin{bmatrix} \theta_x \\ \theta_y \\ v_x \\ v_y \end{bmatrix}$$

Where:
- $\theta_x$ = horizontal angle from image center (degrees; positive = right)
- $\theta_y$ = vertical angle from image center (degrees; positive = down)
- $v_x$ = horizontal angular velocity (degrees/second)
- $v_y$ = vertical angular velocity (degrees/second)

### State Transition (Predict)

Constant-velocity model with discrete time step $dt$:

$$x_{k+1} = A \cdot x_k + B \cdot u_k + w_k$$

Where:

$$A = \begin{bmatrix} 1 & 0 & dt & 0 \\ 0 & 1 & 0 & dt \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$$B = \begin{bmatrix} dt & 0 \\ 0 & dt \\ 1 & 0 \\ 0 & 1 \end{bmatrix}$$

Control input:

$$u_k = \begin{bmatrix} -\Delta \text{yaw}_k / dt \\ -\Delta \text{pitch}_k / dt \end{bmatrix}$$

(Gimbal angle deltas from the gimbal sensor; negative because camera motion causes apparent opposite face motion in the frame.)

Process noise $w_k \sim \mathcal{N}(0, Q)$ models face acceleration uncertainty.

### Measurement Model

Only position is directly measured (Haar cascade detects bounding box center in pixels, then converted to angles):

$$z = H \cdot x$$

Where:

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

$$z = \begin{bmatrix} \theta_{x,\text{meas}} \\ \theta_{y,\text{meas}} \end{bmatrix}$$

Measurement noise $v_k \sim \mathcal{N}(0, R)$ models detector uncertainty (Haar cascade jitter).

---

## Kalman Filter Equations

### Predict Step

```
x_pred = A * x + B * u
P_pred = A * P * A^T + Q
```

Returns predicted position: $[\theta_{x,\text{pred}}, \theta_{y,\text{pred}}]$ (first two states).

### Update Step

```
y = z - H * x_pred           # innovation / residual
S = H * P_pred * H^T + R     # innovation covariance
K = P_pred * H^T * S^(-1)    # Kalman gain
x = x_pred + K * y           # state update
P = (I - K * H) * P_pred     # covariance update
```

Returns updated position: $[\theta_{x,\text{est}}, \theta_{y,\text{est}}]$.

---

## Noise Covariances

### Process Noise (Q)

Derived from continuous white-noise acceleration model. For each dimension (decoupled):

$$Q = \begin{bmatrix} dt^4/4 & dt^3/2 \\ dt^3/2 & dt^2 \end{bmatrix} \cdot \sigma_{\text{acc}}^2$$

Where $\sigma_{\text{acc}}$ is the standard deviation of face acceleration (degrees/sec²).

**Default:** $\sigma_{\text{acc}} = 30 \text{ deg/s}^2$
- Assumes face can change direction reasonably quickly (realistic for head motion)
- Tune lower if face moves smoothly; higher if jerky/sudden

For the full 4×4 matrix, apply this block to both [position, velocity] pairs:

$$Q = \begin{bmatrix} Q_{xx} & 0 & 0 & 0 \\ 0 & Q_{yy} & 0 & 0 \\ 0 & 0 & Q_{xx} & 0 \\ 0 & 0 & 0 & Q_{yy} \end{bmatrix}$$

Where $Q_{xx}$ and $Q_{yy}$ are the 2×2 blocks for each axis.

### Measurement Noise (R)

Haar cascade detector jitter. Diagonal, per-axis:

$$R = \begin{bmatrix} \sigma_x^2 & 0 \\ 0 & \sigma_y^2 \end{bmatrix}$$

**Defaults:** $\sigma_x = \sigma_y = 5.0 \text{ degrees}$
- Haar cascade detection has ~±5° noise at 1280×720 resolution
- Tune down if camera/image quality is very high; up if noisy/low-res

### Initial Error Covariance (P)

Start with large uncertainty, allowing quick convergence:

$$P_0 = I \cdot 500$$

The filter is "born confused" but quickly learns from measurements. After ~5–10 updates, $P$ converges to steady-state.

---

## Coordinate Transformation: Pixels ↔ Angles

### Pixel to Angle

Convert face bounding box center (pixels) to angle:

1. Compute equidistant fisheye focal length:
$$f_{\text{eq}} = \frac{\text{width}/2}{\text{fov}_{\text{rad}}/2} = \frac{w/2}{\arctan(\text{fov}_{\text{deg}}/2) \cdot 180/\pi}$$

2. Center-relative pixel offsets:
$$\Delta x_{\text{px}} = x_{\text{center}} - \frac{\text{width}}{2}$$
$$\Delta y_{\text{px}} = y_{\text{center}} - \frac{\text{height}}{2}$$

3. Convert to angles:
$$\theta_x = \arctan(\Delta x_{\text{px}} / f_{\text{eq}}) \quad \text{(degrees)}$$
$$\theta_y = \arctan(\Delta y_{\text{px}} / f_{\text{eq}}) \quad \text{(degrees)}$$

### Angle to Pixel (Inverse)

For output (crop window offset or gimbal command):

$$\Delta x_{\text{px}} = f_{\text{eq}} \cdot \tan(\theta_x \text{ radians})$$
$$\Delta y_{\text{px}} = f_{\text{eq}} \cdot \tan(\theta_y \text{ radians})$$

**FOV Parameters (from existing code):**
- Mobius camera (face_tracker.py default): 100° horizontal, ~75° vertical
- Rolling shutter node: 150° horizontal (fisheye)
- Yaw stabilizer: varies (100–130°)

Measure actual FOV with a calibration target (checkerboard), or use the values provided in launch configs.

---

## Control Input: Gimbal Motion Prediction

The key insight is to use gimbal angle deltas as a **predictive control input** to the Kalman filter.

### Motivation

When the gimbal (or camera platform) rotates by $\Delta \text{yaw}$ degrees between frames:
1. Any stationary object in the world appears to shift in the image by $-\Delta \text{yaw}$ degrees
2. This is equivalent to the object having an apparent velocity of $-\Delta \text{yaw} / dt$ in the frame
3. Feed this as a control input so the filter predicts the face's new position even if the detector misses a frame

### Implementation

```python
# Latest gimbal angles
yaw_k, pitch_k = gimbal_angles

# Gimbal delta since last predict
delta_yaw   = yaw_k - yaw_prev
delta_pitch = pitch_k - pitch_prev

# Control input (apparent face velocity from camera motion)
u = np.array([
    -delta_yaw / dt,      # apparent horizontal angular velocity
    -delta_pitch / dt     # apparent vertical angular velocity
])

# Predict with control input
x_pred = A @ x + B @ u
P_pred = A @ P @ A.T + Q
```

**Effect:** Between detector frames, the filter smoothly interpolates using gimbal data alone. When the detector provides a new measurement, the filter updates. This produces smooth tracking even with slow or noisy face detection (e.g., Haar cascade at 10–15 Hz).

---

## Tuning Guidelines

### When to Adjust Q (Process Noise)

| Symptom | Adjustment |
|---|---|
| Filter **lags** behind fast face movement | Decrease $\sigma_{\text{acc}}$ (increase trust in const-velocity model) |
| Filter **jitters** (chases noisy measurements) | Increase $\sigma_{\text{acc}}$ (allow more acceleration) |
| Gimbal oscillates during smooth pans | Increase $\sigma_{\text{acc}}$ to permit higher steady-state velocity |

### When to Adjust R (Measurement Noise)

| Symptom | Adjustment |
|---|---|
| Filter **lags** behind measurements | Decrease $\sigma_x, \sigma_y$ (trust measurements more) |
| Filter **chases jitter** in detector | Increase $\sigma_x, \sigma_y$ (smooth out detector noise) |
| Gimbal command is **jerky** on each new detection | Increase $R$ to ignore noisy outlier detections |

### When to Adjust P₀ (Initial Covariance)

| Symptom | Adjustment |
|---|---|
| Filter is **slow to converge** at startup | Decrease $P_0$ (start with more confidence) |
| Filter **overshoots** on first few frames | Increase $P_0$ (be more conservative initially) |

**Rule of thumb:** Start with the defaults, then do a few passes of tuning on real video. Record face detection + filter output, then tweak one parameter at a time.

---

## Extension: Adaptive / Interacting Multiple Model (IMM)

The current constant-velocity model works well for smooth face motion. If you observe oscillation or sluggishness during sudden stops or direction changes, consider adding:

### Interacting Multiple Model (IMM)

Run two Kalman filters in parallel:
1. **CV (Constant Velocity):** assumes face velocity is constant
2. **Singer (or other constant-acceleration model):** assumes face can accelerate

Use a Markov chain to switch between models based on measurement likelihood. This allows the filter to "choose" between smooth tracking (CV) and responsive acceleration (CA) automatically.

**Reference:** Bar-Shalom, Li, & Kirubarajan, *Estimation with Applications to Tracking and Navigation*.

### Maneuver Detection

Alternatively, detect when the measurement innovation is "large" (outlier) and temporarily boost $Q$ or $P$ to allow rapid adaptation.

---

## Summary: The Complete Filter Cycle

```
PREDICT STEP:
  Input:  x_prev, P_prev, u (gimbal control)
  Output: x_pred, P_pred
  
UPDATE STEP:
  Input:  x_pred, P_pred, z (face detection measurement)
  Output: x_est, P_est

CYCLE:
  1. Gimbal publishes /gimbal/angles
  2. Face detection publishes /face/bbox (pixel coords → convert to angle z)
  3. Face tracker:
     a. If new gimbal message: call predict(u = [Δyaw/dt, Δpitch/dt])
     b. If new face detection: call update(z)
     c. Extract θ_x, θ_y from filter state
     d. Output gimbal pitch command, software yaw crop offset
```

---

## References

- `HW5/Part_BC/KF_2D.py` — base constant-velocity 2D Kalman filter
- `HW5/Part_BC/KF_4D.py` — constant-velocity 4D Kalman filter (position + size)
- Bar-Shalom, Li, & Kirubarajan (2001). *Estimation with Applications to Tracking and Navigation*. Wiley.
- OpenCV fisheye calibration docs: https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html
