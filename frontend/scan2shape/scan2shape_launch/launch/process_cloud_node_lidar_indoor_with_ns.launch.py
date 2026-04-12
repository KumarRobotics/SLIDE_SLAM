"""ROS2 launch file for the indoor LIDAR process_cloud_node, namespaced per robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/Odometry')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='robot0')

    odom_topic = LaunchConfiguration('odom_topic')
    robot_name = LaunchConfiguration('robot_name')

    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node_lidar_indoor.py',
        name='process_cloud_node',
        output='screen',
        remappings=[
            ('/Odometry', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        robot_name_arg,
        GroupAction([
            PushRosNamespace(robot_name),
            process_cloud_node,
        ]),
    ])
