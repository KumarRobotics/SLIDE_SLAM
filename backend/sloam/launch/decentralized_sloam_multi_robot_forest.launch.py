"""Ported decentralized_sloam_multi_robot_forest.launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'decentralized_sloam.rviz']
    )
    decentralized_launch = PathJoinSubstitution(
        [sloam_share, 'launch', 'decentralized_sloam.launch.py']
    )

    enable_rviz = LaunchConfiguration('enable_rviz')
    turn_off_intra = LaunchConfiguration('turn_off_intra_loop_closure')
    turn_off_inter = LaunchConfiguration('turn_off_inter_loop_closure')

    actions = [
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('turn_off_intra_loop_closure', default_value='true'),
        DeclareLaunchArgument('turn_off_inter_loop_closure', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
    ]
    for robot_id in (0, 1, 2):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(decentralized_launch),
                launch_arguments={
                    'turn_off_intra_loop_closure': turn_off_intra,
                    'turn_off_inter_loop_closure': turn_off_inter,
                    'priorTFKnown': 'false',
                    'hostRobot_ID': str(robot_id),
                    'odom_topic': f'/robot{robot_id}/odom',
                }.items(),
            )
        )
    return LaunchDescription(actions)
