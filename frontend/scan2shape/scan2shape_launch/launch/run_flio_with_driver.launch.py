"""ROS2 launch file: faster_lio + ouster driver bring-up.

NOTE: References the upstream ``faster_lio`` package's ROS2 mapping launch.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drivers_for_faster_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scan2shape_launch'),
                'launch',
                'drivers_for_faster_lio.launch.py',
            ])
        ),
        launch_arguments={
            'robot': 'quadrotor',
            'sensor_hostname': '192.168.100.12',
            'udp_dest': '192.168.100.1',
            'lidar_port': '7502',
            'imu_port': '7503',
            'replay': 'true',
            'lidar_mode': '1024x10',
            'metadata': '',
        }.items(),
    )

    faster_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('faster_lio'),
                'launch',
                'mapping_ouster64.launch.py',
            ])
        ),
    )

    return LaunchDescription([
        drivers_for_faster_lio,
        faster_lio,
    ])
