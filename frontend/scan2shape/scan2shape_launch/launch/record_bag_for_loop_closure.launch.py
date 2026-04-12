"""ROS2 launch file: record a bag for loop-closure / map merging analysis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


TOPICS = [
    '/robot_0/semantic_meas_sync_odom',
    '/robot_0/vio_odom',
    '/robot_1/semantic_meas_sync_odom',
    '/robot_1/vio_odom',
]


def generate_launch_description():
    dir_arg = DeclareLaunchArgument(
        'dir',
        default_value='/home/sam/bags/xmas-slam-bags/test-map-merging-robot1ANDrobot2')

    out_dir = LaunchConfiguration('dir')

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out_dir] + TOPICS,
        output='screen',
    )

    return LaunchDescription([
        dir_arg,
        record,
    ])
