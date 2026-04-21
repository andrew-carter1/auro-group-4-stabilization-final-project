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

Calibration: K_MS_PX = 52.4 ms  |  Operating resolution: 1280×720 (16:9)
  readout_time = 52.4 × 720 / 1280 ≈ 29.5 ms  (fits within 33.3 ms frame period)

Tuning the delay:
  compass_delay_sec compensates for the time between when the gimbal measures
  yaw and when the matching video frame arrives. Start at 0.0 and increase in
  small steps (e.g. 0.01) until vertical edges stop leaning.

  compass_lag_frames compensates for gimbal magnetometer filter latency (~133ms).
  If compass_shift peaks ~4 frames (~133ms) after flow_shift during a fast pan,
  set this to 4. Requires diagnostics mode to observe.

To run:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py

  # Override gimbal port:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py gimbal_port:=/dev/ttyACM1

  # Tune sync delay:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py compass_delay_sec:=0.03

  # Tune frame buffer for magnetometer lag:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py compass_lag_frames:=4

  # Tune max correction:
  ros2 launch stabilization_pkg rolling_shutter_compass_launch.py max_shift_pct:=0.05
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
    max_shift_arg = DeclareLaunchArgument(
        'max_shift_pct',
        default_value='0.10',
        description='Max pixel shift top-to-bottom as fraction of frame width (0.10 = 128 px at 1280 wide)'
    )
    lag_frames_arg = DeclareLaunchArgument(
        'compass_lag_frames',
        default_value='1',
        description='Frames to buffer for compass-video sync (0=no buffer)'
    )

    return LaunchDescription([
        video_device_arg,
        gimbal_port_arg,
        compass_delay_arg,
        max_shift_arg,
        lag_frames_arg,

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
                'fov_horizontal_deg':  130.0,
                'video_device':        LaunchConfiguration('video_device'),
                'image_width':         1280,
                'image_height':        720,
                'capture_fps':         30.0,
                'show_comparison':     True,
                'compass_delay_sec':   LaunchConfiguration('compass_delay_sec'),
                'compass_lag_frames':  LaunchConfiguration('compass_lag_frames'),
                'max_shift_pct':       LaunchConfiguration('max_shift_pct'),
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
