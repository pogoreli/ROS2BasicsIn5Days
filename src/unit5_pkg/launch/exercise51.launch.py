from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unit5_pkg',
            executable='exercise51',
            output='screen',
            emulate_tty=True),
    ])