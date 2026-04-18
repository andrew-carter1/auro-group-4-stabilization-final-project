#!/usr/bin/env python3
"""
Storm BCG Gimbal ROS2 Node
Reads roll/pitch/yaw from the gimbal over serial and publishes to /gimbal/angles.

Topic: /gimbal/angles (geometry_msgs/Vector3Stamped)
  x = roll  (degrees)
  y = pitch (degrees)
  z = yaw   (degrees, continuous)

Parameters:
  port     (string)  -- serial port, default '/dev/ttyACM0'
  baudrate (int)     -- baud rate,   default 115200
"""

import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import serial


class GimbalNode(Node):
    HEADER = 0x3E
    CMD_REALTIME_DATA = 0x44

    def __init__(self):
        super().__init__('gimbal_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.pub = self.create_publisher(Vector3Stamped, '/gimbal/angles', 10)

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            dsrdtr=True
        )
        time.sleep(2)
        self.get_logger().info(f'Connected to gimbal on {port}')

        # Poll at 10 Hz to match gimbal output rate
        self.create_timer(0.1, self.timer_cb)

    def _build_packet(self, cmd: int) -> bytes:
        size = 0
        header_crc = (cmd + size) & 0xFF
        return bytes([self.HEADER, cmd, size, header_crc, 0])

    def _read_gimbal(self) -> dict:
        self.ser.write(self._build_packet(self.CMD_REALTIME_DATA))

        response = b''
        for _ in range(30):
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)

        if len(response) < 48:
            return None

        payload = response[4:]

        roll_raw = struct.unpack('<h', payload[0:2])[0]
        pitch_raw = struct.unpack('<h', payload[6:8])[0]
        yaw_raw = struct.unpack('<h', payload[42:44])[0]

        return {
            'roll_deg':  roll_raw  * 0.18,
            'pitch_deg': pitch_raw * 0.18,
            'yaw_deg':   yaw_raw   * 0.1,
        }

    def timer_cb(self):
        data = self._read_gimbal()
        if data is None:
            self.get_logger().warn('No data from gimbal', throttle_duration_sec=5.0)
            return

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal'
        msg.vector.x = data['roll_deg']
        msg.vector.y = data['pitch_deg']
        msg.vector.z = data['yaw_deg']
        self.pub.publish(msg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
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
