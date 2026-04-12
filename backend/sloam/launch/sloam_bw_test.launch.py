"""Ported sloam_bw_test.launch.

Note: see play_bag_indoor_scarab.launch.py for the rosbag1 -> rosbag2 caveat.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'decentralized_sloam.rviz']
    )
    enable_rviz = LaunchConfiguration('enable_rviz')

    bag_3rd = '/home/jiuzl/bags/abstract_observation_bag/fined_tuned_bag/robot6-new-from-3rd-parking-lot-around-building-falcon-xmas-slam-pennovation_2023-10-20-13-22-40'
    bag_2nd = '/home/jiuzl/bags/abstract_observation_bag/fined_tuned_bag/robot5-from-2nd-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-00-01'
    bag_1st = '/home/jiuzl/bags/abstract_observation_bag/fined_tuned_bag/robot4-from-1st-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-07-35'

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_3rd, '--start-offset', '80', '--rate', '2',
                 '--remap', '/Odometry:=/robot0/Odometry'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_2nd, '--start-offset', '50', '--rate', '2',
                 '--remap',
                 '/Odometry:=/robot1/Odometry',
                 '/robot0/semantic_meas_sync_odom:=/robot1/semantic_meas_sync_odom'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_1st, '--rate', '2',
                 '--remap',
                 '/Odometry:=/robot2/Odometry',
                 '/robot0/semantic_meas_sync_odom:=/robot2/semantic_meas_sync_odom'],
            output='screen',
        ),
    ])
