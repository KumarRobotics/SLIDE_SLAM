"""Ported run_pennovation.launch.

Note: relay_topics_multi_robot.launch does not exist in this package; only
the sloam node is launched here.
"""

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
                PathJoinSubstitution([sloam_share, 'launch', 'sloam.launch.py'])
            ),
        ),
    ])
