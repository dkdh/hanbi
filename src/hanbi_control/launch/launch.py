from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hanbi_control',
            node_executable='path_pub',
            node_name='path_pub'
        ),
    ])



