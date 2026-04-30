#!/usr/bin/env python3
"""
UART Gimbal Servo Control via ESP32

Subscribes to:
  /face_tracker/pitch_cmd  — pitch command in degrees [-45, +45]

Sends UART commands to ESP32 that drives the gimbal pitch servo:
  "P:{duty:.1f}\n"  where duty is 0.0–100.0 (percentage)

Mapping:
  -45° → 0% duty (camera all the way up)
  0°   → 50% duty (neutral)
  +45° → 100% duty (camera all the way down)
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class UartGimbalServoNode(Node):
    def __init__(self):
        super().__init__('uart_gimbal_servo')

        # ------ Parameters ------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('min_deg', -45.0)
        self.declare_parameter('max_deg', 45.0)

        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._min_deg = self.get_parameter('min_deg').get_parameter_value().double_value
        self._max_deg = self.get_parameter('max_deg').get_parameter_value().double_value

        # ------ Serial connection ------
        self.ser = None
        self._connect()

        # ------ Subscriptions ------
        self.create_subscription(Float32, '/face_tracker/pitch_cmd', self._pitch_cmd_cb, 10)

        self.get_logger().info(
            f"UART Gimbal Servo started on {self._port} @ {self._baudrate} baud, "
            f"range [{self._min_deg:.0f}, {self._max_deg:.0f}]°"
        )

    def _connect(self) -> bool:
        """Open serial connection to ESP32. Returns True on success."""
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=1.0
            )
            self.get_logger().info(f"Connected to ESP32 on {self._port}")
            return True
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(
                f"ESP32 not available on {self._port}: {e}. Will retry.",
                throttle_duration_sec=5.0
            )
            return False

    def _deg_to_duty(self, deg: float) -> float:
        """Convert gimbal angle (degrees) to PWM duty cycle (0–100%)."""
        # Clamp to valid range
        deg_clamped = max(self._min_deg, min(self._max_deg, deg))

        # Linear mapping: min_deg → 0%, max_deg → 100%
        duty = ((deg_clamped - self._min_deg) / (self._max_deg - self._min_deg)) * 100.0
        return duty

    def _pitch_cmd_cb(self, msg: Float32):
        """Receive pitch command and send to ESP32."""
        if not self.ser or not self.ser.is_open:
            if not self._connect():
                return

        pitch_deg = msg.data
        duty = self._deg_to_duty(pitch_deg)

        # Send command
        try:
            cmd = f"P:{duty:.1f}\n"
            self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(
                f"Serial write error: {e}. Will attempt reconnect.",
                throttle_duration_sec=2.0
            )
            self.ser.close()
            self.ser = None

    def destroy_node(self):
        """Close serial connection on shutdown."""
        if self.ser and self.ser.is_open:
            try:
                # Send neutral command before closing
                self.ser.write(b"P:50.0\n")
            except Exception:
                pass
            self.ser.close()
            self.get_logger().info("Serial connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UartGimbalServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
