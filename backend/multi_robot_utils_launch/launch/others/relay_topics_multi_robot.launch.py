from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_relays(context, *args, **kwargs):
    target_ns = LaunchConfiguration('target_robot_ns').perform(context)

    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            arguments=['/Odometry', f'{target_ns}/lidar_odom'],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cloud_relay',
            arguments=['/ground_cloud', f'{target_ns}/ground_cloud'],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cloud_relay2',
            arguments=['/tree_cloud', f'{target_ns}/tree_cloud'],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cube_relay2',
            arguments=['/car_cuboids_body', f'{target_ns}/car_cuboids_body'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_robot_ns', default_value='/quadrotor1'),
        OpaqueFunction(function=_build_relays),
    ])
