from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_relays(context, *args, **kwargs):
    original_ns = LaunchConfiguration('original_robot_ns').perform(context)
    target_ns = LaunchConfiguration('target_robot_ns').perform(context)

    pairs = [
        ('odom_relay', 'lidar_odom'),
        ('cloud_relay', 'ground_cloud'),
        ('cloud_relay2', 'tree_cloud'),
        ('cube_relay2', 'car_cuboids_body'),
    ]

    nodes = []
    for name, topic in pairs:
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=name,
            arguments=[f'{original_ns}/{topic}', f'{target_ns}/{topic}'],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('original_robot_ns', default_value='/quadrotor2'),
        DeclareLaunchArgument('target_robot_ns', default_value='/quadrotor4'),
        OpaqueFunction(function=_build_relays),
    ])
