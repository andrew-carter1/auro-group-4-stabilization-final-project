"""
Rolling Shutter Correction — Compass (Gimbal Yaw) Launch
Requires the Storm BCG gimbal connected via USB.

Starts:
  1. realtime_stabilization  — captures webcam, publishes /stabilized_frame/compressed
  2. gimbal_node             — reads gimbal serial data, publishes /gimbal/angles
  3. rolling_shutter_node    — compass mode, uses yaw rate for per-row correction
  4. rqt_image_view          — displays the corrected output

Calibration: 1920x1080 16:9, 200 Hz LED  |  Operating resolution: 1280x960 4:3
Gimbal port:    /dev/ttyACM0 (override with gimbal_port arg if different)

To run:
  ros2 launch stabilization_pkg rs_compass_launch.py

  # Override gimbal port:
  ros2 launch stabilization_pkg rs_compass_launch.py gimbal_port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gimbal_port_arg = DeclareLaunchArgument(
        'gimbal_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Storm BCG gimbal'
    )

    return LaunchDescription([
        gimbal_port_arg,

        # 1. EMA stabilization — provides /stabilized_frame/compressed
        Node(
            package='stabilization_pkg',
            executable='realtime_stabilization',
            name='stabilization_node',
            output='screen'
        ),

        # 2. Gimbal serial reader — provides /gimbal/angles (yaw at 30 Hz)
        Node(
            package='stabilization_pkg',
            executable='gimbal_node',
            name='gimbal_node',
            output='screen',
            parameters=[{
                'port':     LaunchConfiguration('gimbal_port'),
                'baudrate': 115200,
            }]
        ),

        # 3. Rolling shutter correction using live yaw rate
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'mode':               'compass',
                'fov_horizontal_deg': 170.0,
                'n_bands':            4,      # unused in compass mode, kept for parity
                'min_features':       8,      # unused in compass mode
            }]
        ),

        # 4. Visualize corrected output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame/compressed']
        ),
    ])
