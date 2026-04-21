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

Calibration: K_MS_PX = 52.4 ms  |  Operating resolution: 1280×720 (16:9)
  readout_time = 52.4 × 720 / 1280 ≈ 29.5 ms  (fits within 33.3 ms frame period)

Tuning:
  max_shift_pct  -- max pixel shift top-to-bottom as fraction of width (default 0.10 = 128 px)
  slope_ema_alpha -- EMA smoothing on slope; 0.0 = off, higher = smoother but more lag

To run:
  ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py

  # Override device if auto-detection fails:
  ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py video_device:=/dev/video4

  # Tune correction aggressiveness:
  ros2 launch stabilization_pkg rolling_shutter_raw_test_launch.py max_shift_pct:=0.05

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
    max_shift_arg = DeclareLaunchArgument(
        'max_shift_pct',
        default_value='0.10',
        description='Max pixel shift top-to-bottom as fraction of frame width (0.10 = 128 px at 1280 wide)'
    )
    ema_arg = DeclareLaunchArgument(
        'slope_ema_alpha',
        default_value='0.0',
        description='EMA smoothing on slope (0.0 = off, higher = smoother but more lag)'
    )

    return LaunchDescription([
        video_device_arg,
        max_shift_arg,
        ema_arg,

        # Rolling shutter correction with direct camera capture + side-by-side output
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'optical_flow',
                'fov_horizontal_deg':  130.0,
                'min_features':        12,
                'video_device':        LaunchConfiguration('video_device'),
                'image_width':         1280,
                'image_height':        720,
                'capture_fps':         30.0,
                'show_comparison':     True,
                'max_shift_pct':       LaunchConfiguration('max_shift_pct'),
                'slope_ema_alpha':     LaunchConfiguration('slope_ema_alpha'),
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
