"""Ported decentralized_sloam_multi_robot_seven_robots_indoor_outdoor.launch."""

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

    # robots 0..3 use one prior TF, robots 4..7 use another
    robots_a = [
        {'hostRobot_ID': str(i), 'odom_topic': f'/robot{i}/odom',
         'priorTF_x': '-1.0', 'priorTF_y': '0.0', 'priorTF_z': '-0.5'}
        for i in range(4)
    ]
    robots_b = [
        {'hostRobot_ID': str(i), 'odom_topic': f'/robot{i}/odom',
         'priorTF_x': '0.0', 'priorTF_y': '0.0', 'priorTF_z': '0.0'}
        for i in range(4, 8)
    ]

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
    for r in (*robots_a, *robots_b):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(decentralized_launch),
                launch_arguments={
                    'turn_off_intra_loop_closure': turn_off_intra,
                    'turn_off_inter_loop_closure': turn_off_inter,
                    'priorTFKnown': 'true',
                    **r,
                }.items(),
            )
        )
    return LaunchDescription(actions)
