from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dir_arg = DeclareLaunchArgument(
        'dir',
        default_value='/home/sam/bags/xmas-slam-bags/multi-robot',
    )

    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('dir'),
            '/tf',
            '/tf_static',
            '/cloud_registered',
            '/os_node/segmented_point_cloud_no_destagger',
            '/Odometry',
            '/ground_plane_marker',
            '/cylinder_marker',
            '/car_cuboids',
        ],
        output='screen',
    )

    return LaunchDescription([
        dir_arg,
        recorder,
    ])
