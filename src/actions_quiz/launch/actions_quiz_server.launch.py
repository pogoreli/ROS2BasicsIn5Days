from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions_quiz',
            executable='actions_quiz_server_node',
            name='actions_quiz_server_node',
            output='screen'),
    ])