from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_odom_map_transform_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'quadrotor/map', 'dragonfly67/odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='init_odom_map_transform_pub2',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'quadrotor/map'],
        ),
    ])
