from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    bag_path = '/home/sam/bags/pennovation-bags/generic_sloam_2_robots_multi_robot_MOST_IMPORTANT_2022-06-30-22-50-33.bag'
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                bag_path,
                '--clock',
                '--topics',
                '/quadrotor1/lidar_odom',
                '/semantic_meas_sync_odom',
                '--remap',
                '/quadrotor1/lidar_odom:=/quadrotor1/lidar_odom',
                '/semantic_meas_sync_odom:=/quadrotor1/semantic_meas_sync_odom',
            ],
            output='screen',
        ),
    ])
