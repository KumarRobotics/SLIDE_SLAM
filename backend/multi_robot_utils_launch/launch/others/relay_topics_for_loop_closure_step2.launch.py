from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_relays(context, *args, **kwargs):
    original_ns = LaunchConfiguration('original_robot_ns').perform(context)
    target_ns = LaunchConfiguration('target_robot_ns').perform(context)

    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            arguments=[f'{original_ns}/sloam/cubes_map', f'{target_ns}/sloam/cubes_map'],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cloud_relay',
            arguments=[
                f'{original_ns}/sloam/submap_cylinder_models',
                f'{target_ns}/sloam/submap_cylinder_models',
            ],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cloud_relay2',
            arguments=['/semantic_meas_sync_odom', f'{target_ns}/semantic_meas_sync_odom'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('original_robot_ns', default_value='/robot_0'),
        DeclareLaunchArgument('target_robot_ns', default_value='/robot_1'),
        OpaqueFunction(function=_build_relays),
    ])
