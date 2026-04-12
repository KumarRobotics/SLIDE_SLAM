"""Ported segmentation.launch.

Note: the original references an executable `sloam_inference_node` that is
not built by this package. This launch file is a thin shim around the
standard `sloam_node` executable preserved for reference.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    sloam_yaml = PathJoinSubstitution([sloam_share, 'params', 'sloam_sim.yaml'])
    sim_yaml = PathJoinSubstitution([sloam_share, 'params', 'sim.yaml'])
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'segmentation.rviz']
    )

    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        Node(
            package='sloam',
            executable='sloam_node',
            name='segmentation',
            output='screen',
            parameters=[sloam_yaml, sim_yaml],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
    ])
