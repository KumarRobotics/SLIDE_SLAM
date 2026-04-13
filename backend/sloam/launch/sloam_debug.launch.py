"""Ported sloam.launch.debug — wraps sloam_node under gdb."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    rviz_config = PathJoinSubstitution([sloam_share, 'launch', 'rviz', 'decentralized_sloam.rviz'])
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
        Node(
            package='sloam',
            executable='sloam_node',
            name='sloam',
            output='screen',
            prefix=['gdb -ex run --args'],
        ),
    ])
