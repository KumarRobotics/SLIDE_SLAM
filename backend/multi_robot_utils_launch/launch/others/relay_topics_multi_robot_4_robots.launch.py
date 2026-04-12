from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_relays(context, *args, **kwargs):
    robot1_ns = LaunchConfiguration('robot1_ns').perform(context)
    robot2_ns = LaunchConfiguration('robot2_ns').perform(context)
    robot3_ns = LaunchConfiguration('robot3_ns').perform(context)
    robot4_ns = LaunchConfiguration('robot4_ns').perform(context)

    first_pairs = [
        ('odom_relay', 'lidar_odom'),
        ('ground_relay', 'ground_cloud'),
        ('tree_relay', 'tree_cloud'),
        ('cube_relay', 'car_cuboids_body'),
    ]
    second_pairs = [
        ('odom_relay1', 'lidar_odom'),
        ('ground_relay1', 'ground_cloud'),
        ('tree_relay1', 'tree_cloud'),
        ('cube_relay1', 'car_cuboids_body'),
    ]

    nodes = []
    for name, topic in first_pairs:
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=name,
            arguments=[f'{robot1_ns}/{topic}', f'{robot3_ns}/{topic}'],
        ))
    for name, topic in second_pairs:
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=name,
            arguments=[f'{robot2_ns}/{topic}', f'{robot4_ns}/{topic}'],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot1_ns', default_value='/quadrotor1'),
        DeclareLaunchArgument('robot2_ns', default_value='/quadrotor2'),
        DeclareLaunchArgument('robot3_ns', default_value='/quadrotor3'),
        DeclareLaunchArgument('robot4_ns', default_value='/quadrotor4'),
        OpaqueFunction(function=_build_relays),
    ])
