from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hanbi_mapping',
            node_executable='client',
            node_name='client'
        ),
        Node(
            package='hanbi_mapping',
            node_executable='run_mapping_custom',
            node_name='run_mapping_custom'
        ),
        Node(
            package='hanbi_mapping',
            node_executable='socket_custom',
            node_name='socket_custom'
        )
    ])



