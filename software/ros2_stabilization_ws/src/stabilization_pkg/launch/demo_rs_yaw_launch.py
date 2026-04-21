"""
Demo Launch — Rolling Shutter + Yaw Stabilization (Annotated)

Full annotated pipeline with all comparison views:

  Camera → rolling_shutter_node → yaw_stabilizer → yaw_stabilized
         ↓                      ↓                ↓
  /camera/raw   /rs_comparison  /rs_corrected    /yaw_stabilized
                                                       ↓
                                              demo_comparison_node
                                           → /demo_comparison        (raw | yaw, 2-panel)
                                           → /demo_full_pipeline     (raw | rs | yaw, 3-panel)

Viewers launched:
  - /rs_comparison/compressed      side-by-side RS correction with slope lines + green references
  - /demo_full_pipeline/compressed 3-panel end-to-end pipeline view (half-size)

To run (annotated, default):
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py

For clean output without annotations use:
  ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py

Selective annotation control:
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py rs_annotations:=false
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py yaw_annotations:=false

Tuning:
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py compass_lag_frames:=5
  ros2 launch stabilization_pkg demo_rs_yaw_launch.py max_margin_px:=100
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
        DeclareLaunchArgument('compass_lag_frames', default_value='1'),
        DeclareLaunchArgument('max_shift_pct',      default_value='0.10'),
        DeclareLaunchArgument('max_margin_px',      default_value='80'),
        DeclareLaunchArgument('yaw_lag_frames',     default_value='0'),
        # Per-node annotation control
        DeclareLaunchArgument('rs_annotations',     default_value='true'),
        DeclareLaunchArgument('yaw_annotations',    default_value='true'),
        DeclareLaunchArgument('cmp_annotations',    default_value='true'),

        # ----------------------------------------------------------------
        # 1. Gimbal serial reader → /gimbal/angles
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
        # 2. Rolling shutter node
        #    → /camera/raw/compressed        (raw, lag-aligned)
        #    → /rs_corrected_frame/compressed (clean corrected)
        #    → /rs_comparison/compressed     (side-by-side with slope lines)
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode':          'capture',
                'mode':                'compass',
                'fov_horizontal_deg':  130.0,
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
        # 3. Yaw stabilizer (follow-at-margin control)
        #    → /yaw_stabilized/compressed  (960×720, 4:3)
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='yaw_stabilizer',
            name='yaw_stabilizer',
            output='screen',
            parameters=[{
                'fov_horizontal_deg':  130.0,
                'max_margin_px':       LaunchConfiguration('max_margin_px'),
                'out_w':               960,
                'yaw_lag_frames':      LaunchConfiguration('yaw_lag_frames'),
                'show_annotations':    LaunchConfiguration('yaw_annotations'),
            }]
        ),

        # ----------------------------------------------------------------
        # 4. Demo comparison node
        #    → /demo_comparison/compressed      (raw | yaw, 2-panel)
        #    → /demo_full_pipeline/compressed   (raw | rs | yaw, 3-panel half-size)
        # ----------------------------------------------------------------
        Node(
            package='stabilization_pkg',
            executable='demo_comparison_node',
            name='demo_comparison_node',
            output='screen',
            parameters=[{
                'out_w':            960,
                'show_annotations': LaunchConfiguration('cmp_annotations'),
            }]
        ),

        # ----------------------------------------------------------------
        # 5. Viewers
        # ----------------------------------------------------------------
        # RS correction side-by-side (with slope lines)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rs_viewer',
            arguments=['/rs_comparison/compressed']
        ),
        # Full 3-panel pipeline comparison
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='pipeline_viewer',
            arguments=['/demo_full_pipeline/compressed']
        ),
    ])
