#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        self.current_state = State()

        # Subscriber: Listen to the drone's state (mode, armed, connected)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10)

        # Publisher: Send target position setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        # Service Clients: For mode switching and arming
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to become available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        # Define the target position (X: 0, Y: 0, Z: 2 meters)
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0

        # Create a timer to run the loop at 20Hz (0.05 seconds)
        self.timer = self.create_timer(0.05, self.timer_cb)
        
        # Track time to avoid spamming service requests
        self.last_req = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def timer_cb(self):
        # 1. ALWAYS publish the setpoint (Requirement for OFFBOARD mode)
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.pose)

        # If not connected to the flight controller, do nothing yet
        if not self.current_state.connected:
            return

        now = self.get_clock().now()
        time_diff = (now - self.last_req).nanoseconds / 1e9  # Convert to seconds

        # 2. Try to switch to OFFBOARD mode (every 5 seconds if not in it)
        if self.current_state.mode != "OFFBOARD" and time_diff > 5.0:
            self.get_logger().info("Requesting OFFBOARD mode...")
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
            self.last_req = now

        # 3. If in OFFBOARD mode but not armed, try to arm (every 5 seconds)
        elif self.current_state.mode == "OFFBOARD" and not self.current_state.armed and time_diff > 5.0:
            self.get_logger().info("Requesting Arming...")
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.last_req = now


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
        
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
