from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hanbi_control',
            node_executable='path_pub',
            node_name='path_pub'
        ),
        Node(
            package='hanbi_control',
            node_executable='odom',
            node_name='odom'
        ),
        Node(
            package='hanbi_control',
            node_executable='path_tracking_detected',
            node_name='path_tracking_detected'
        ),
        Node(
            package='hanbi_control',
            node_executable='path_tracking_kjh',
            node_name='path_tracking_kjh'
        ),
    ])



