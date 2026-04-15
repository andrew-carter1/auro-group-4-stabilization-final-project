"""
Rolling Shutter Correction — Direct Capture Test Launch

Opens the camera directly via cv2.VideoCapture (V4L2 + MJPG), bypassing both
the usb_cam node and the EMA stabilization node. Useful for isolated testing
of rolling shutter correction only.

Starts:
  1. rolling_shutter_node -- capture mode: opens camera directly, applies
                             optical flow rolling shutter correction, publishes
                             /rs_corrected_frame/compressed
  2. rqt_image_view       -- displays the corrected output

Calibration note:
  Readout time was calibrated at 1920x1080 16:9 (NOT at 1280x960).
  The per-row constant (0.02907 ms/row) scales automatically to the
  operating resolution: 1280x960 → ~27.9 ms total readout.

To run:
  ros2 launch stabilization_pkg rs_raw_test_launch.py

  # Override device path if the camera is on a different node:
  ros2 launch stabilization_pkg rs_raw_test_launch.py video_device:=/dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video18',
        description='V4L2 device path for cv2.VideoCapture (e.g. /dev/video18)'
    )

    return LaunchDescription([
        video_device_arg,

        # Rolling shutter correction with direct camera capture
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'optical_flow',
                'fov_horizontal_deg':  170.0,
                'n_bands':             4,
                'min_features':        8,
                'video_device':        LaunchConfiguration('video_device'),  # string path e.g. '/dev/video18'
                'image_width':         1280,
                'image_height':        960,
                'capture_fps':         30.0,
            }]
        ),

        # View corrected output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame']
        ),
    ])
