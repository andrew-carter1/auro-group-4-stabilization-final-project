"""
Combined Launch — EMA Stabilization + Optical Flow Rolling Shutter Correction
(no gimbal required)

Runs the full software-only pipeline:
  - EMA stabilization (realtime_stabilization) smooths out frame-to-frame
    camera shake and publishes the result as /stabilized_frame/compressed.
  - rolling_shutter_node subscribes to that output and applies optical flow
    rolling shutter correction on top.

The Mobius camera device is detected automatically at launch time by searching
v4l2-ctl --list-devices for a device whose name contains "Mobius".

Starts:
  1. realtime_stabilization  — captures webcam, publishes /stabilized_frame/compressed
  2. rolling_shutter_node    — optical flow mode, publishes /rs_corrected_frame/compressed
  3. rqt_image_view          — displays the corrected output

Calibration: K_MS_PX = 52.4 ms  |  Operating resolution: 1280x960 4:3
  readout_time = K * height / width ≈ 39.3 ms at 1280x960

To run:
  ros2 launch stabilization_pkg combined_launch.py

To use compass mode (gimbal required), use rolling_shutter_compass_launch.py.
To test rolling shutter correction alone (no EMA), use rolling_shutter_raw_test_launch.py.
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_mobius_device(fallback: str = '/dev/video4') -> str:
    """
    Parse v4l2-ctl --list-devices and return the first /dev/videoX listed
    under the 'Mobius' entry. Returns fallback if Mobius is not found.
    """
    try:
        out = subprocess.check_output(
            ['v4l2-ctl', '--list-devices'],
            stderr=subprocess.DEVNULL, text=True
        )
        in_mobius_block = False
        for line in out.splitlines():
            if 'mobius' in line.lower():
                in_mobius_block = True
            elif in_mobius_block:
                stripped = line.strip()
                if stripped.startswith('/dev/video'):
                    return stripped
                elif stripped and not stripped.startswith('/dev/'):
                    break
    except Exception:
        pass
    return fallback


_MOBIUS_DEVICE = _find_mobius_device()


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value=_MOBIUS_DEVICE,
        description='V4L2 device path for the Mobius camera (auto-detected)'
    )

    return LaunchDescription([
        video_device_arg,

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
                'input_mode':          'compressed',
                'mode':                'optical_flow',
                'fov_horizontal_deg':  150.0,
                'n_bands':             6,
                'min_features':        12,
            }]
        ),

        # 3. Visualize corrected output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame']
        ),
    ])
