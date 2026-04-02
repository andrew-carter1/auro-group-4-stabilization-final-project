from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Stabilization Node (captures webcam + stabilizes)
        Node(
            package='stabilization_pkg',
            executable='stabilization_node',
            name='stabilization_node'
        ),

        # 2. Face Detection Node
        Node(
            package='stabilization_pkg',
            executable='face_detection_node',
            name='face_detection_node'
        ),

        # 3. RQT to visualize face detection output
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='visualizer'
        )
    ])