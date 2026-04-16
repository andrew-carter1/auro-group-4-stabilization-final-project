"""
Rolling Shutter Correction — Compass Mode (Gimbal Yaw), Direct Capture
Requires the Storm BCG gimbal connected via USB.

Opens the Mobius camera directly via cv2.VideoCapture (V4L2 + MJPG), with no
EMA stabilization. The gimbal yaw rate drives the per-row correction directly.

The Mobius camera device is detected automatically at launch time by searching
v4l2-ctl --list-devices for a device whose name contains "Mobius".

Starts:
  1. gimbal_node          — reads gimbal serial data, publishes /gimbal/angles
  2. rolling_shutter_node — compass mode, direct capture, side-by-side output
  3. rqt_image_view       — displays the corrected output

Calibration: K_MS_PX = 52.4 ms  |  Operating resolution: 1280x960 4:3
  readout_time = K * height / width ≈ 39.3 ms at 1280x960

Tuning the delay:
  compass_delay_sec compensates for the time between when the gimbal measures
  yaw and when the matching video frame arrives. Start at 0.0 and increase in
  small steps (e.g. 0.01) until vertical edges stop leaning.

To run:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py

  # Override gimbal port:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py gimbal_port:=/dev/ttyACM1

  # Tune sync delay:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py compass_delay_sec:=0.03
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
    gimbal_port_arg = DeclareLaunchArgument(
        'gimbal_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Storm BCG gimbal'
    )
    compass_delay_arg = DeclareLaunchArgument(
        'compass_delay_sec',
        default_value='0.0',
        description=(
            'Seconds to look back in gimbal history when syncing to a video frame. '
            'Increase if the correction lags behind visible shutter skew.'
        )
    )

    return LaunchDescription([
        video_device_arg,
        gimbal_port_arg,
        compass_delay_arg,

        # 1. Gimbal serial reader — provides /gimbal/angles (yaw at 30 Hz)
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

        # 2. Rolling shutter correction — direct capture, compass mode
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'compass',
                'fov_horizontal_deg':  150.0,
                'n_bands':             6,       # unused in compass mode, kept for parity
                'min_features':        12,      # unused in compass mode
                'video_device':        LaunchConfiguration('video_device'),
                'image_width':         1280,
                'image_height':        960,
                'capture_fps':         30.0,
                'show_comparison':     True,
                'compass_delay_sec':   LaunchConfiguration('compass_delay_sec'),
            }]
        ),

        # 3. View side-by-side comparison (original left, RS corrected right)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame']
        ),
    ])
