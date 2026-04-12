from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    bag_path = '/home/sam/bags/pennovation-bags/generic_sloam_2_robots_multi_robot_MOST_IMPORTANT_2022-06-30-22-50-33.bag'
    # Topic remappings from ROS1 rosbag play (source:=target) are preserved
    # as CLI remap arguments for ros2 bag play.
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                bag_path,
                '--clock',
                '--topics',
                '/quadrotor2/car_cuboids_body',
                '/quadrotor2/ground_cloud',
                '/quadrotor2/lidar_odom',
                '/quadrotor2/tree_cloud',
                '--remap',
                '/quadrotor2/car_cuboids_body:=/quadrotor1/car_cuboids_body',
                '/quadrotor2/lidar_odom:=/quadrotor1/lidar_odom',
                '/quadrotor2/ground_cloud:=/quadrotor1/ground_cloud',
                '/quadrotor2/tree_cloud:=/quadrotor1/tree_cloud',
            ],
            output='screen',
        ),
    ])
