#!/usr/bin/env python3
"""
Face Tracking DNN Demo Launch (No Servo Control)

Direct camera capture pipeline with SSD face detector (from HW1):
  1. face_detection_dnn_node — opens camera, runs SSD detector, publishes frames + /face/bbox
  2. face_tracker — Kalman tracking, publishes pitch_cmd + yaw-cropped frames
  3. rqt_image_view (×2) — visualizes detection and tracking output

No UART, no gimbal servo commands. Good for:
  - Testing SSD face detection quality (more robust than Haar Cascade)
  - Tuning Kalman and PD parameters
  - Debugging yaw crop behavior
  - Monitoring pitch corrections

To run:
  ros2 launch stabilization_pkg face_tracking_dnn_demo_launch.py

Or with custom camera device:
  ros2 launch stabilization_pkg face_tracking_dnn_demo_launch.py video_device:=/dev/video0

Monitor pitch correction:
  python3 software/monitor_pitch_cmd.py

Tune SSD confidence threshold:
  ros2 launch stabilization_pkg face_tracking_dnn_demo_launch.py confidence_threshold:=0.6
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

        # 3. Image viewer for tracking output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_tracked',
            arguments=['/face_tracked/compressed'],
        ),
    ])
