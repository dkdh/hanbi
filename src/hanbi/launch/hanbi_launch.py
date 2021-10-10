from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hanbi',
            node_executable='a_star_local_path',
            node_name='a_star_local_path'
        ),
        Node(
            package='hanbi',
            node_executable='a_star',
            node_name='a_star'
        ),
        Node(
            package='hanbi',
            node_executable='client',
            node_name='client'
        ),
        Node(
            package='hanbi',
            node_executable='load_map',
            node_name='load_map'
        ),
        # Node(
        #     package='hanbi',
        #     node_executable='make_path',
        #     node_name='make_path'
        # ),
        Node(
            package='hanbi',
            node_executable='odom',
            node_name='odom'
        ),
        Node(
            package='hanbi',
            node_executable='path_pub',
            node_name='path_pub'
        ),
        Node(
            package='hanbi',
            node_executable='path_tracking_patrol',
            node_name='path_tracking_patrol'
        ),
        Node(
            package='hanbi',
            node_executable='perception',
            node_name='perception'
        ),
        Node(
            package='hanbi',
            node_executable='pytorch_detector',
            node_name='pytorch_detector'
        ),
        Node(
            package='hanbi',
            node_executable='socket_custom',
            node_name='socket_custom'
        )
    ])



