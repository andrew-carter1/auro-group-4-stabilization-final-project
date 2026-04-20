"""
Demo Launch — Rolling Shutter + Yaw Stabilization (Annotated)

Full annotated pipeline for inspection and diagnostics:
  Camera → rolling_shutter_node (compass mode) → yaw_stabilizer → /yaw_stabilized/compressed

Two viewers:
  - /rs_comparison/compressed  — side-by-side RS correction with slope lines
  - /yaw_stabilized/compressed — 4:3 cropped yaw-stabilized output with overlays

To run (annotated, default):
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py

For clean output without any overlays use:
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py

Or disable annotations selectively:
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py rs_annotations:=false
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py yaw_annotations:=false
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py rs_annotations:=false yaw_annotations:=false

Other tuning:
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py gimbal_port:=/dev/ttyACM1
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py compass_lag_frames:=5
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py max_shift_pct:=0.05
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py max_margin_px:=100
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py yaw_lag_frames:=4
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_mobius_device(fallback: str = '/dev/video4') -> str:
    """Return the first /dev/videoX listed under the 'Mobius' entry in v4l2-ctl."""
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
    return LaunchDescription([
        # ----------------------------------------------------------------
        # Launch arguments
        # ----------------------------------------------------------------
        DeclareLaunchArgument('video_device',       default_value=_MOBIUS_DEVICE,
                              description='V4L2 device path for Mobius camera (auto-detected)'),
        DeclareLaunchArgument('gimbal_port',        default_value='/dev/ttyACM0',
                              description='Serial port for the Storm BCG gimbal'),
        DeclareLaunchArgument('compass_delay_sec',  default_value='0.0',
                              description='Seconds to look back in gimbal history for frame sync'),
        DeclareLaunchArgument('max_shift_pct',      default_value='0.10',
                              description='Max RS correction as fraction of frame width'),
        DeclareLaunchArgument('compass_lag_frames', default_value='4',
                              description='Frame buffer for gimbal magnetometer lag (~133ms)'),
        DeclareLaunchArgument('max_margin_px',      default_value='80',
                              description='Max crop-center shift for yaw stabilization (px)'),
        DeclareLaunchArgument('yaw_lag_frames',     default_value='0',
                              description='Frame buffer for yaw stabilizer gimbal lag (frames)'),
        # Annotation controls — per node
        DeclareLaunchArgument('rs_annotations',     default_value='true',
                              description='Show slope lines + shift text in RS comparison'),
        DeclareLaunchArgument('yaw_annotations',    default_value='true',
                              description='Show anchor circle + yaw text on yaw stabilizer output'),

        # ----------------------------------------------------------------
        # 1. Gimbal serial reader
        # ----------------------------------------------------------------
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

        # ----------------------------------------------------------------
        # 2. Rolling shutter — compass mode, direct capture
        #    Publishes:
        #      /rs_corrected_frame/compressed  (clean frame for yaw_stabilizer)
        #      /rs_comparison/compressed       (side-by-side for display)
        # ----------------------------------------------------------------
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
                'diagnostics':         True,
                'compass_delay_sec':   LaunchConfiguration('compass_delay_sec'),
                'compass_lag_frames':  LaunchConfiguration('compass_lag_frames'),
                'max_shift_pct':       LaunchConfiguration('max_shift_pct'),
                'show_annotations':    LaunchConfiguration('rs_annotations'),
            }]
        ),

        # ----------------------------------------------------------------
        # 3. Yaw stabilizer — physics-based sliding crop
        #    Input:  /rs_corrected_frame/compressed
        #    Output: /yaw_stabilized/compressed (960×720, 4:3)
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='yaw_stabilizer',
            name='yaw_stabilizer',
            output='screen',
            parameters=[{
                'fov_horizontal_deg':  150.0,
                'max_margin_px':       LaunchConfiguration('max_margin_px'),
                'yaw_lag_frames':      LaunchConfiguration('yaw_lag_frames'),
                'show_annotations':    LaunchConfiguration('yaw_annotations'),
            }]
        ),

        # ----------------------------------------------------------------
        # 4. Viewers
        # ----------------------------------------------------------------
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rs_viewer',
            arguments=['/rs_comparison/compressed']
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='yaw_viewer',
            arguments=['/yaw_stabilized/compressed']
        ),
    ])
