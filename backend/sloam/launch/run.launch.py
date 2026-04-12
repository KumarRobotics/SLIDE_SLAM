"""Ported run.launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    return LaunchDescription([
        DeclareLaunchArgument('undistort', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sloam_share, 'launch', 'sloam.launch.py'])
            ),
        ),
    ])
