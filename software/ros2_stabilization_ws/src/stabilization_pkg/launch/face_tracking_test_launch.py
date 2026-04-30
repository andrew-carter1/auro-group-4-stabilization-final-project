#!/usr/bin/env python3
"""
Face Tracking Test Launch Configuration

Simple end-to-end pipeline for face detection + gimbal servo control:
  1. rolling_shutter_node (optical_flow mode) — direct camera capture @ 1280×720, publishes stabilized frames
  2. face_detection_node — Haar Cascade detection, publishes /face/bbox
  3. face_tracker — Kalman tracking, publishes pitch_cmd + yaw-cropped frames
  4. uart_gimbal_servo — sends PWM commands to ESP32 via UART
  5. rqt_image_view — visualizes /face_tracked/compressed output

Gimbal Required: ❌ for camera; ✅ for pitch servo (ESP32 at /dev/ttyUSB0)

To run:
  ros2 launch stabilization_pkg face_tracking_test_launch.py

Or configure ESP32 port:
  ros2 launch stabilization_pkg face_tracking_test_launch.py servo_port:=/dev/ttyACM0

Tuning examples:
  ros2 launch stabilization_pkg face_tracking_test_launch.py face_track_p_gain:=1.5 face_track_d_gain:=0.3
  ros2 launch stabilization_pkg face_tracking_test_launch.py kalman_std_acc:=40.0
"""

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_mobius_device(fallback: str = '/dev/video4') -> str:
    """Auto-detect Mobius camera device."""
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
        # ================================================================
        # Launch Arguments
        # ================================================================
        DeclareLaunchArgument(
            'video_device',
            default_value=_MOBIUS_DEVICE,
            description='Camera device (auto-detects Mobius, fallback /dev/video4)'),

        DeclareLaunchArgument(
            'servo_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32 gimbal servo'),

        # Rolling shutter parameters (optical flow stabilization)
        DeclareLaunchArgument(
            'max_shift_pct',
            default_value='0.10',
            description='Max rolling shutter correction as fraction of width'),

        # Face tracking parameters
        DeclareLaunchArgument(
            'fov_horizontal_deg',
            default_value='100.0',
            description='Camera horizontal FOV (degrees)'),

        DeclareLaunchArgument(
            'face_track_out_w',
            default_value='960',
            description='Output crop width for yaw centering (pixels)'),

        DeclareLaunchArgument(
            'face_track_p_gain',
            default_value='1.0',
            description='Yaw crop proportional gain (higher = more aggressive)'),

        DeclareLaunchArgument(
            'face_track_d_gain',
            default_value='0.2',
            description='Yaw crop derivative damping (higher = smoother)'),

        DeclareLaunchArgument(
            'kalman_std_acc',
            default_value='30.0',
            description='Kalman filter accel noise (deg/s²), higher = smoother'),

        DeclareLaunchArgument(
            'kalman_std_meas',
            default_value='5.0',
            description='Kalman filter measurement noise (deg), higher = trust detector less'),

        DeclareLaunchArgument(
            'servo_min_deg',
            default_value='-45.0',
            description='Servo physical lower limit (degrees)'),

        DeclareLaunchArgument(
            'servo_max_deg',
            default_value='45.0',
            description='Servo physical upper limit (degrees)'),

        DeclareLaunchArgument(
            'show_annotations',
            default_value='true',
            description='Show yaw/pitch values on output frames'),

        # ================================================================
        # Nodes
        # ================================================================

        # 1. Rolling shutter stabilization (optical flow, no gimbal)
        Node(
            package='stabilization_pkg',
            executable='rolling_shutter_node',
            name='rolling_shutter_node',
            output='screen',
            parameters=[{
                'input_mode': 'capture',
                'mode': 'optical_flow',
                'fov_horizontal_deg': 130.0,
                'min_features': 12,
                'video_device': LaunchConfiguration('video_device'),
                'image_width': 1280,
                'image_height': 720,
                'capture_fps': 30.0,
                'show_comparison': False,
                'max_shift_pct': LaunchConfiguration('max_shift_pct'),
                'slope_ema_alpha': 0.0,
            }],
        ),

        # 2. Face detection (Haar Cascade)
        Node(
            package='stabilization_pkg',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen',
        ),

        # 3. Face tracker (Kalman filter + yaw crop + pitch command)
        Node(
            package='stabilization_pkg',
            executable='face_tracker',
            name='face_tracker',
            output='screen',
            parameters=[{
                'fov_horizontal_deg': LaunchConfiguration('fov_horizontal_deg'),
                'fov_vertical_deg': 75.0,
                'image_width': 1280,
                'image_height': 720,
                'out_w': LaunchConfiguration('face_track_out_w'),
                'max_margin_px': 80,
                'no_face_hold_sec': -1.0,  # hold forever
                'p_gain': LaunchConfiguration('face_track_p_gain'),
                'd_gain': LaunchConfiguration('face_track_d_gain'),
                'dt': 0.033,  # 30 Hz
                'show_annotations': LaunchConfiguration('show_annotations'),
                'kalman_std_acc': LaunchConfiguration('kalman_std_acc'),
                'kalman_std_meas': LaunchConfiguration('kalman_std_meas'),
            }],
        ),

        # 4. UART gimbal servo (reads pitch_cmd, sends PWM via ESP32)
        Node(
            package='stabilization_pkg',
            executable='uart_gimbal_servo',
            name='uart_gimbal_servo',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('servo_port'),
                'baudrate': 115200,
                'min_deg': LaunchConfiguration('servo_min_deg'),
                'max_deg': LaunchConfiguration('servo_max_deg'),
            }],
        ),

        # 5. Image viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_tracker',
            arguments=['/face_tracked/compressed'],
        ),
    ])
