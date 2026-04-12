"""Ported launch/unit_tests/place_recognition_test.launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    sloam_yaml = PathJoinSubstitution([sloam_share, 'params', 'sloam.yaml'])

    return LaunchDescription([
        Node(
            package='sloam',
            executable='sloam_place_recognition_test',
            name='place_recognition_test',
            output='screen',
            parameters=[sloam_yaml],
        ),
    ])
