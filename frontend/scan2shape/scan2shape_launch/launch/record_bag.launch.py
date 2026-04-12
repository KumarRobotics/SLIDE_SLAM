"""ROS2 launch file: record a bag of the SlideSLAM topics of interest."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


TOPICS = [
    '/clock',
    '/os_node/llol_odom/pose',
    '/os_node/llol_odom/sweep',
    '/tf',
    '/tf_static',
    '/os_node/segmented_point_cloud_no_destagger',
    '/os_node/segmented_point_cloud_no_destagger/car_prob',
    '/os_node/llol_odom/pose_cov',
    '/os_node/llol_odom/path',
    '/os_node/llol_odom/runtime',
    '/os_node/llol_odom/traj',
    '/os_node/metadata',
    '/os_node/range',
    '/os_node/signal',
    '/quadrotor/odom',
    '/quadrotor/vio/odom',
    '/rosout',
]


def generate_launch_description():
    dir_arg = DeclareLaunchArgument(
        'dir',
        default_value='/home/sam/bags/pennovation-bags/stats_car_confidence_sloam_multi_robot')

    out_dir = LaunchConfiguration('dir')

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out_dir] + TOPICS,
        output='screen',
    )

    return LaunchDescription([
        dir_arg,
        record,
    ])
