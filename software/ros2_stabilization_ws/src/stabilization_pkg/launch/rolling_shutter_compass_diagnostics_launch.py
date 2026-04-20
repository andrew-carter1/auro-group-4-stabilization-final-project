"""
Rolling Shutter Correction — Compass Mode with Diagnostics

Same as rolling_shutter_compass_launch.py but with diagnostics enabled.
Publishes /rs_diagnostics (std_msgs/String) at frame rate with:
  - yaw: current gimbal yaw reading (degrees, continuous)
  - yaw_rate: computed yaw rate (deg/s)
  - compass_shift: total pixel shift from compass slope (px, top-to-bottom)
  - flow_shift: total pixel shift from optical flow slope (px, for comparison)
  - applied: actual pixel shift applied after clamp + EMA (px)

To view diagnostics in another terminal:
  ros2 topic echo /rs_diagnostics

To run:
  ros2 launch stabilization_pkg rolling_shutter_compass_diagnostics_launch.py

  # Tune delay:
  ros2 launch stabilization_pkg rolling_shutter_compass_diagnostics_launch.py compass_delay_sec:=0.03
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_mobius_device(fallback: str = '/dev/video4') -> str:
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
        description='Seconds to look back in gimbal history when syncing to a video frame.'
    )
    max_shift_arg = DeclareLaunchArgument(
        'max_shift_pct',
        default_value='0.10',
        description='Max pixel shift top-to-bottom as fraction of frame width'
    )
    lag_frames_arg = DeclareLaunchArgument(
        'compass_lag_frames',
        default_value='4',
        description='Frames to buffer for compass-video sync (0=no buffer)'
    )

    return LaunchDescription([
        video_device_arg,
        gimbal_port_arg,
        compass_delay_arg,
        max_shift_arg,
        lag_frames_arg,

        # 1. Gimbal serial reader
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

        # 2. Rolling shutter correction — compass mode + diagnostics
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'compass',
                'fov_horizontal_deg':  150.0,
                'video_device':        LaunchConfiguration('video_device'),
                'image_width':         1280,
                'image_height':        720,
                'capture_fps':         30.0,
                'show_comparison':     True,
                'compass_delay_sec':   LaunchConfiguration('compass_delay_sec'),
                'compass_lag_frames':  LaunchConfiguration('compass_lag_frames'),
                'max_shift_pct':       LaunchConfiguration('max_shift_pct'),
                'diagnostics':         True,
            }]
        ),

        # 3. View side-by-side comparison
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/rs_corrected_frame']
        ),
    ])
