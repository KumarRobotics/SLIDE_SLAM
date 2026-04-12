"""ROS2 launch file for the simulated process_cloud_node profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/ddk/ground_truth/odom')

    odom_topic = LaunchConfiguration('odom_topic')

    params_file = PathJoinSubstitution([
        FindPackageShare('scan2shape_launch'),
        'config',
        'process_cloud_node_sim.yaml',
    ])

    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node.py',
        name='process_cloud_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/dragonfly67/quadrotor_ukf/control_odom', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        process_cloud_node,
    ])
