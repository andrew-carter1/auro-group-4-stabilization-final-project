"""
Rolling Shutter Correction — Direct Capture Test (Optical Flow, No Gimbal)

Opens the Mobius camera directly via cv2.VideoCapture (V4L2 + MJPG), bypassing
both the usb_cam node and the EMA stabilization node. Useful for isolated testing
of rolling shutter correction using optical flow only.

The Mobius camera device is detected automatically at launch time by searching
v4l2-ctl --list-devices for a device whose name contains "Mobius". Falls back
to /dev/video4 if the Mobius is not found.

Starts:
  1. rolling_shutter_node -- capture mode: opens camera directly, applies
                             optical flow rolling shutter correction, publishes
                             side-by-side comparison on /rs_corrected_frame/compressed
  2. rqt_image_view       -- displays the comparison output

Calibration: K_MS_PX = 52.4 ms  |  readout_time = K * height / width
  1280x960 (4:3) → 39.3 ms    1920x1080 (16:9) → 29.5 ms

To run:
  ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py

  # Override device if auto-detection fails:
  ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py video_device:=/dev/video4

To use compass mode (gimbal required), use rolling_shutter_compass_launch.py.
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
                    # Reached the next device's header — Mobius had no video nodes
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

        # Rolling shutter correction with direct camera capture + side-by-side output
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'optical_flow',
                'fov_horizontal_deg':  150.0,
                'n_bands':             6,
                'min_features':        12,
                'video_device':        LaunchConfiguration('video_device'),
                'image_width':         1280,
                'image_height':        960,
                'capture_fps':         30.0,
                'show_comparison':     True,
            }]
        ),

        # View side-by-side comparison (original left, RS corrected right)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame']
        ),
    ])
