"""
Demo Launch — Rolling Shutter + Yaw Stabilization (Clean, No Annotations)

Same pipeline as demo_rs_yaw_launch.py but all annotations disabled.
Use for recording clean demo footage or presentation output.

One viewer launched:
  - /demo_comparison/compressed  — clean raw | yaw side-by-side (2-panel, no labels)

To run:
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py

For annotated diagnostics version:
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py

Tuning (same args as annotated launch):
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py compass_lag_frames:=5
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py reference_alpha:=0.12
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py p_gain:=0.8 d_gain:=0.3
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py max_margin_px:=100
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_mobius_device(fallback: str = '/dev/video4') -> str:
    try:
        out = subprocess.check_output(
            ['v4l2-ctl', '--list-devices'], stderr=subprocess.DEVNULL, text=True)
        in_block = False
        for line in out.splitlines():
            if 'mobius' in line.lower():
                in_block = True
            elif in_block:
                s = line.strip()
                if s.startswith('/dev/video'):
                    return s
                elif s and not s.startswith('/dev/'):
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
        DeclareLaunchArgument('video_device',       default_value=_MOBIUS_DEVICE),
        DeclareLaunchArgument('gimbal_port',        default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('compass_delay_sec',  default_value='0.0'),
        DeclareLaunchArgument('compass_lag_frames', default_value='4'),
        DeclareLaunchArgument('max_shift_pct',      default_value='0.10'),
        DeclareLaunchArgument('max_margin_px',      default_value='80'),
        DeclareLaunchArgument('yaw_lag_frames',     default_value='0'),
        DeclareLaunchArgument('reference_alpha',    default_value='0.08'),
        DeclareLaunchArgument('p_gain',             default_value='1.0'),
        DeclareLaunchArgument('d_gain',             default_value='0.2'),

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
        # 2. Rolling shutter — no comparison display, no annotations
        #    Still publishes /camera/raw/compressed and /rs_corrected_frame/compressed
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
                'show_comparison':     False,
                'diagnostics':         False,
                'compass_delay_sec':   LaunchConfiguration('compass_delay_sec'),
                'compass_lag_frames':  LaunchConfiguration('compass_lag_frames'),
                'max_shift_pct':       LaunchConfiguration('max_shift_pct'),
                'show_annotations':    False,
            }]
        ),

        # ----------------------------------------------------------------
        # 3. Yaw stabilizer — no annotations
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='yaw_stabilizer',
            name='yaw_stabilizer',
            output='screen',
            parameters=[{
                'fov_horizontal_deg':  100.0,
                'max_margin_px':       LaunchConfiguration('max_margin_px'),
                'out_w':               960,
                'yaw_lag_frames':      LaunchConfiguration('yaw_lag_frames'),
                'reference_alpha':     LaunchConfiguration('reference_alpha'),
                'p_gain':              LaunchConfiguration('p_gain'),
                'd_gain':              LaunchConfiguration('d_gain'),
                'show_annotations':    False,
            }]
        ),

        # ----------------------------------------------------------------
        # 4. Comparison node — no labels
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='demo_comparison_node',
            name='demo_comparison_node',
            output='screen',
            parameters=[{
                'out_w':            960,
                'show_annotations': False,
            }]
        ),

        # ----------------------------------------------------------------
        # 5. Single viewer — clean raw | yaw comparison
        # ----------------------------------------------------------------
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='viewer',
            arguments=['/demo_comparison/compressed']
        ),
    ])
