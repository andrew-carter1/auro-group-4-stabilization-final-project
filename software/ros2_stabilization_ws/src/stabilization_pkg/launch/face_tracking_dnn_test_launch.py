#!/usr/bin/env python3
"""
Face Tracking DNN Test Launch (With Servo Control)

Complete pipeline for SSD face detection + gimbal servo control:
  1. face_detection_dnn_node — opens camera, runs SSD detector, publishes frames + /face/bbox
  2. face_tracker — Kalman tracking, publishes pitch_cmd + yaw-cropped frames
  3. uart_gimbal_servo — sends PWM commands to ESP32 via UART
  4. rqt_image_view (×2) — visualizes detection and tracking output

Gimbal Required: ✅ ESP32 at /dev/ttyUSB0 for pitch servo

To run:
  ros2 launch stabilization_pkg face_tracking_dnn_test_launch.py

Or with custom ports:
  ros2 launch stabilization_pkg face_tracking_dnn_test_launch.py \
    video_device:=/dev/video0 servo_port:=/dev/ttyACM0

Tuning examples:
  ros2 launch stabilization_pkg face_tracking_dnn_test_launch.py \
    confidence_threshold:=0.6 face_track_p_gain:=1.5 face_track_d_gain:=0.3 kalman_std_acc:=40.0
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

        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='SSD detector confidence threshold (0.0-1.0)'),

        DeclareLaunchArgument(
            'fov_horizontal_deg',
            default_value='100.0',
            description='Camera horizontal FOV (degrees)'),

        DeclareLaunchArgument(
            'face_track_out_w',
            default_value='720',
            description='Output crop width (square format, 720×720 pixels)'),

        DeclareLaunchArgument(
            'face_track_p_gain',
            default_value='1.0',
            description='Yaw crop proportional gain'),

        DeclareLaunchArgument(
            'face_track_d_gain',
            default_value='0.2',
            description='Yaw crop derivative damping'),

        DeclareLaunchArgument(
            'kalman_std_acc',
            default_value='30.0',
            description='Kalman filter accel noise (deg/s²)'),

        DeclareLaunchArgument(
            'kalman_std_meas',
            default_value='5.0',
            description='Kalman filter measurement noise (deg)'),

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

        # 1. Face detection DNN (opens camera directly, SSD detector)
        Node(
            package='stabilization_pkg',
            executable='face_detection_dnn_node',
            name='face_detection_dnn_node',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': 1280,
                'image_height': 720,
                'capture_fps': 30.0,
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }],
        ),

        # 2. Face tracker (Kalman filtering + yaw crop + pitch command)
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
                'max_margin_px': 120,
                'no_face_hold_sec': -1.0,
                'p_gain': LaunchConfiguration('face_track_p_gain'),
                'd_gain': LaunchConfiguration('face_track_d_gain'),
                'dt': 0.033,
                'show_annotations': LaunchConfiguration('show_annotations'),
                'kalman_std_acc': LaunchConfiguration('kalman_std_acc'),
                'kalman_std_meas': LaunchConfiguration('kalman_std_meas'),
            }],
        ),

        # 3. UART gimbal servo control
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

        # 4. Image viewer for tracking output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_tracked',
            arguments=['/face_tracked/compressed'],
        ),
    ])
