"""Ported decentralized_sloam_indoor_outdoor.launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sloam_share, 'launch', 'single_robot_sloam_test_outdoor.launch.py'])
            ),
            launch_arguments={'turn_off_intra_loop_closure': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sloam_share, 'launch', 'decentralized_sloam_scarab.launch.py'])
            ),
            launch_arguments={
                'hostRobot_ID': '1',
                'odom_topic': '/scarab41/odom_laser',
                'turn_off_intra_loop_closure': 'true',
            }.items(),
        ),
    ])
