"""ROS2 launch file: ouster lidar driver + decoder + LLOL bring-up."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_decoder'),
                'launch',
                'driver.launch.py',
            ])
        ),
    )

    decoder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_decoder'),
                'launch',
                'decoder.launch.py',
            ])
        ),
        launch_arguments={'replay': 'true'}.items(),
    )

    llol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('llol'),
                'launch',
                'llol.launch.py',
            ])
        ),
    )

    return LaunchDescription([driver, decoder, llol])
