"""ROS2 launch file: record a bag for confidence statistics analysis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


TOPICS = [
    '/os_node/image',
    '/os_node/camera_info',
    '/os_node/llol_odom/pose',
    '/os_node/llol_odom/sweep',
    '/os_node/llol_odom/path',
    '/tf_static',
    '/tf',
]


def generate_launch_description():
    dir_arg = DeclareLaunchArgument(
        'dir',
        default_value='/home/sam/bags/pennovation-bags/confidence_stats')

    out_dir = LaunchConfiguration('dir')

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out_dir] + TOPICS,
        output='screen',
    )

    return LaunchDescription([
        dir_arg,
        record,
    ])
