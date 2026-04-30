"""
Kalman Filter for Face Tracking — Constant Velocity Model

State: [theta_x, theta_y, vx, vy]
  - theta_x, theta_y: angles from image center (degrees)
  - vx, vy: angular velocity (degrees/second)

Measurement: [theta_x, theta_y] from face detection

Control Input: gimbal angle deltas — when camera pans, face appears to shift
  u = [-delta_yaw / dt, -delta_pitch / dt]
"""

import numpy as np


class FaceKalman:
    def __init__(self, dt=0.033, std_acc=30.0, std_meas=5.0):
        """
        Initialize constant-velocity Kalman filter for face tracking.

        Args:
            dt (float): time step in seconds (default 0.033 ≈ 30 Hz)
            std_acc (float): standard deviation of acceleration (deg/s²). Default 30.
            std_meas (float): standard deviation of measurement noise (degrees). Default 5.
        """
        self.dt = dt
        self.std_acc = std_acc
        self.std_meas = std_meas

        # State: [theta_x, theta_y, vx, vy]
        self.x = np.zeros((4, 1))

        # State transition matrix (constant velocity)
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Control input matrix
        self.B = np.array([
            [dt, 0],
            [0, dt],
            [1, 0],
            [0, 1]
        ], dtype=np.float32)

        # Measurement matrix (measure position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        # Process noise Q — derived from continuous white-noise acceleration model
        # Q_block = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * std_acc^2
        q_factor = std_acc * std_acc
        q_block = np.array([
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2]
        ]) * q_factor

        # Full Q (diagonal blocks for x and y)
        self.Q = np.zeros((4, 4), dtype=np.float32)
        self.Q[0:2, 0:2] = q_block
        self.Q[2:4, 2:4] = q_block

        # Measurement noise R
        self.R = np.array([
            [std_meas**2, 0],
            [0, std_meas**2]
        ], dtype=np.float32)

        # Error covariance — start with large uncertainty
        self.P = np.eye(4, dtype=np.float32) * 500.0

    def predict(self, u=None):
        """
        Predict next state using constant-velocity model + gimbal control input.

        Args:
            u (2-tuple or None): control input [delta_yaw/dt, delta_pitch/dt].
                If None, assume zero control (no gimbal motion).

        Returns:
            (theta_x, theta_y): predicted angles in degrees.
        """
        if u is not None:
            u = np.array(u).reshape((2, 1))
            self.x = self.A @ self.x + self.B @ u
        else:
            self.x = self.A @ self.x

        self.P = self.A @ self.P @ self.A.T + self.Q

        # Return position (first two states)
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, z):
        """
        Update state with face detection measurement.

        Args:
            z (2-tuple): measured angles [theta_x, theta_y] in degrees.

        Returns:
            (theta_x, theta_y): updated angles in degrees.
        """
        z = np.array(z).reshape((2, 1))

        # Innovation
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        I = np.eye(4, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P

        # Return updated position
        return float(self.x[0, 0]), float(self.x[1, 0])

    def reset(self, theta_x, theta_y):
        """
        Reset filter state to initial measurement (cold start).

        Args:
            theta_x (float): initial angle x (degrees).
            theta_y (float): initial angle y (degrees).
        """
        self.x = np.array([[theta_x], [theta_y], [0], [0]], dtype=np.float32)
        self.P = np.eye(4, dtype=np.float32) * 500.0

    def get_velocity(self):
        """Return current estimated angular velocity (vx, vy) in deg/s."""
        return float(self.x[2, 0]), float(self.x[3, 0])

    def get_state(self):
        """Return full state vector as (theta_x, theta_y, vx, vy)."""
        return tuple(float(self.x[i, 0]) for i in range(4))
