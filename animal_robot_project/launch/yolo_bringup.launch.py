from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'animal_robot_project'
    package_dir = get_package_share_directory(package_name)

    return LaunchDescription([
        Node(
            package=package_name,
            executable='yolo_emotion_detection',
            name='yolo_emotion_detection',
            output='screen',
            emulate_tty=True,
        )
    ])
