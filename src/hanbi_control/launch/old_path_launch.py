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
            node_executable='path_via',
            node_name='path_via'
        ),
        # Node(
        #     package='hanbi_control',
        #     node_executable='a_star',
        #     node_name='a_star'
        # ),
        # Node(
        #     package='hanbi_ctl_dev',
        #     node_executable='path_tracking_clean',
        #     node_name='path_tracking_clean'
        # ),
    ])



