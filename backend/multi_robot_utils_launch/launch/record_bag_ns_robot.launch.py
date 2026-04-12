from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dir',
            default_value='/home/sam/bags/xmas-slam-bags/multi-robot',
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', LaunchConfiguration('dir'),
                '/quadrotor1/lidar_odom',
                '/semantic_meas_sync_odom',
            ],
            output='screen',
        ),
    ])
