"""Ported run_indoor_large_scale_exploration.launch.

Note: the original references an executable `sloam_active_slam_input_node`
that is not built by the current CMakeLists. This launch file is preserved
for reference but currently launches the standard `sloam_node`.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    # Upstream ROS1 referenced 'params/sloam_active_slam_real_robot.yaml';
    # that file was never shipped in sloam/params/ on master. The only
    # parameter yaml that actually exists is 'params/sloam.yaml', which we
    # use here until someone re-adds a dedicated active-slam config.
    sloam_yaml = PathJoinSubstitution([sloam_share, 'params', 'sloam.yaml'])
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'sloam_active_slam.rviz']
    )
    enable_rviz = LaunchConfiguration('enable_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        Node(
            package='sloam',
            executable='sloam_node',
            name='active_slam_input_node',
            output='screen',
            parameters=[sloam_yaml],
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
