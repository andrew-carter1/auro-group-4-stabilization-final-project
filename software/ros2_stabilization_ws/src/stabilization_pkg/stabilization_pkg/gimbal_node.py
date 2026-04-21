#!/usr/bin/env python3
"""
Storm BCG Gimbal ROS2 Node
Reads roll/pitch/yaw and gyro rates from the gimbal over serial.

Topics published:
  /gimbal/angles  (geometry_msgs/Vector3Stamped)
    x = roll  (degrees, 1 decimal place)
    y = pitch (degrees, 1 decimal place)
    z = yaw   (degrees, continuous — see note below)

  /gimbal/gyro  (geometry_msgs/Vector3Stamped)
    x = roll  angular velocity (raw units — unit type not yet determined)
    y = pitch angular velocity (raw units — unit type not yet determined)
    z = 0     (gimbal does not output yaw gyro in CMD_REALTIME_DATA)

NOTE — Yaw accumulation:
  Yaw is reported as a continuous value and does NOT wrap at ±360°.
  Rotating clockwise continuously will increment past +360°, +720°, etc.
  Counter-clockwise goes past -360°, -720°, etc.
  Consumers must handle this if a bounded [-180°, +180°) or [0°, 360°)
  representation is needed (use modulo arithmetic).

Parameters:
  port     (string) -- serial port, default '/dev/ttyACM0'
  baudrate (int)    -- baud rate,   default 115200
"""

import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import serial


class GimbalNode(Node):
    HEADER = 0x3E
    CMD_REALTIME_DATA = 0x44

    # Timeout short enough to keep up with 60 Hz (16.7 ms per cycle).
    # At 115200 baud a ~68-byte response takes ~5 ms, so 15 ms leaves margin.
    SERIAL_TIMEOUT = 0.015

    def __init__(self):
        super().__init__('gimbal_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.pub_angles = self.create_publisher(Vector3Stamped, '/gimbal/angles', 10)
        self.pub_gyro   = self.create_publisher(Vector3Stamped, '/gimbal/gyro',   10)

        self._port     = port
        self._baudrate = baudrate
        self.ser       = None
        self._connect()

        # Running at 60 Hz
        self.create_timer(1.0 / 60.0, self.timer_cb)

    def _connect(self) -> bool:
        """
        Attempt to open the serial port. Returns True on success.
        Safe to call repeatedly — closes any existing connection first.
        """
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self.SERIAL_TIMEOUT,
                dsrdtr=True
            )
            import time; time.sleep(2)  # let the gimbal finish initializing
            self.get_logger().info(f'Connected to gimbal on {self._port}')
            return True
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(
                f'Gimbal not available on {self._port}: {e}. Will retry.',
                throttle_duration_sec=5.0
            )
            return False

    def _build_packet(self, cmd: int) -> bytes:
        size = 0
        header_crc = (cmd + size) & 0xFF
        return bytes([self.HEADER, cmd, size, header_crc, 0])

    def _read_gimbal(self) -> dict:
        # Discard any queued bytes from a previous cycle so we always parse
        # the freshest response, not a stale one that arrived late.
        self.ser.reset_input_buffer()
        self.ser.write(self._build_packet(self.CMD_REALTIME_DATA))

        # Read the 4-byte header first to discover payload size.
        header = self.ser.read(4)
        if len(header) < 4 or header[0] != self.HEADER:
            return None

        size = header[2]

        # Read payload + trailing data CRC byte.
        rest = self.ser.read(size + 1)
        if len(rest) < size:
            return None

        payload = rest[:size]

        if len(payload) < 44:
            return None

        # Byte offsets validated experimentally (see examples/gimbal_communication/SETUP.md)
        roll_raw       = struct.unpack('<h', payload[0:2])[0]
        roll_gyro_raw  = struct.unpack('<h', payload[2:4])[0]   # raw — units TBD
        pitch_raw      = struct.unpack('<h', payload[6:8])[0]
        pitch_gyro_raw = struct.unpack('<h', payload[8:10])[0]  # raw — units TBD
        yaw_raw        = struct.unpack('<h', payload[42:44])[0]

        return {
            # Roll/pitch rounded to 1 decimal — sub-0.1° is noise at 0.18°/unit resolution
            'roll_deg':       round(roll_raw  * 0.18, 1),
            'pitch_deg':      round(pitch_raw * 0.18, 1),
            # Yaw: full resolution (0.1°/unit), continuous, NOT wrapped — see docstring
            'yaw_deg':        yaw_raw * 0.1,
            'roll_gyro_raw':  roll_gyro_raw,
            'pitch_gyro_raw': pitch_gyro_raw,
        }

    def timer_cb(self):
        if not self.ser or not self.ser.is_open:
            self._connect()
            return

        data = self._read_gimbal()
        if data is None:
            self.get_logger().warn('No data from gimbal', throttle_duration_sec=1.0)
            return

        now = self.get_clock().now().to_msg()

        angles_msg = Vector3Stamped()
        angles_msg.header.stamp    = now
        angles_msg.header.frame_id = 'gimbal'
        angles_msg.vector.x = data['roll_deg']
        angles_msg.vector.y = data['pitch_deg']
        angles_msg.vector.z = data['yaw_deg']
        self.pub_angles.publish(angles_msg)

        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp    = now
        gyro_msg.header.frame_id = 'gimbal'
        gyro_msg.vector.x = float(data['roll_gyro_raw'])
        gyro_msg.vector.y = float(data['pitch_gyro_raw'])
        gyro_msg.vector.z = 0.0  # yaw gyro not available in this packet
        self.pub_gyro.publish(gyro_msg)

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
