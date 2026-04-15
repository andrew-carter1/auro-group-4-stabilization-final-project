"""
Rolling Shutter Correction — Software-Only Launch
(no gimbal required)

Starts:
  1. realtime_stabilization  — captures webcam, publishes /stabilized_frame/compressed
  2. rolling_shutter_node    — optical flow mode, publishes /rs_corrected_frame/compressed
  3. rqt_image_view          — displays the corrected output

Calibration: 1920x1080 16:9, 200 Hz LED  |  Operating resolution: 1280x960 4:3
  Per-row constant (0.02907 ms/row) scales automatically → 1280x960 = ~27.9 ms readout

To run:
  ros2 launch stabilization_pkg rs_software_launch.py

Tuning:
  n_bands        -- increase for finer slope resolution (needs more texture)
  min_features   -- raise if noisy slope estimates, lower if correction drops out

To switch to compass mode (when gimbal is available), use rs_compass_launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. EMA stabilization — provides /stabilized_frame/compressed
        Node(
            package='stabilization_pkg',
            executable='realtime_stabilization',
            name='stabilization_node',
            output='screen'
        ),

        # 2. Rolling shutter correction (optical flow, no gimbal)
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'mode':               'optical_flow',
                'fov_horizontal_deg': 170.0,
                'n_bands':            4,
                'min_features':       8,
            }]
        ),

        # 3. Visualize corrected output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame/compressed']
        ),
    ])
