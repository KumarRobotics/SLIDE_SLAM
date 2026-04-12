"""Ported play_bag_indoor_scarab.launch.

Note: the original ROS1 launch file uses rosbag play on ROS1 .bag files.
ROS2 uses `ros2 bag play` on rosbag2 .db3 files. The bag paths below are
preserved from the original; you will likely need to convert them to ROS2
rosbag2 format (`rosbags-convert ...`) before playback.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'decentralized_sloam.rviz']
    )
    enable_rviz = LaunchConfiguration('enable_rviz')

    scarab40_bag = '/home/jiuzl/bags/abstract_observation_bag/scarab-with-comm/Scarabs-with-comms-VEMS-SLAM-scarab40-EST_2024-01-21-11-19-36'
    scarab41_bag = '/home/jiuzl/bags/abstract_observation_bag/scarab-with-comm/Scarabs-with-comms-VEMS-SLAM-scarab41-EST_2024-01-21-11-19-38'
    scarab45_bag = '/home/jiuzl/bags/abstract_observation_bag/scarab-with-comm/Scarabs-with-comms-VEMS-SLAM-scarab45-EST_2024-01-21-11-19-38'

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', scarab40_bag, '--clock', '--rate', '3',
                 '--remap', '/scarab40/odom_laser:=/robot0/Odometry'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', scarab41_bag, '--rate', '3',
                 '--remap',
                 '/scarab41/odom_laser:=/robot1/Odometry',
                 '/robot0/semantic_meas_sync_odom:=/robot1/semantic_meas_sync_odom'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', scarab45_bag, '--rate', '3',
                 '--remap',
                 '/scarab45/odom_laser:=/robot2/Odometry',
                 '/robot0/semantic_meas_sync_odom:=/robot2/semantic_meas_sync_odom'],
            output='screen',
        ),
    ])
