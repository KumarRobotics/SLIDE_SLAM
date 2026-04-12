"""ROS2 launch file for the outdoor process_cloud_node in KITTI mode, namespaced per robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/Odometry')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='robot0')
    process_cloud_node_name_arg = DeclareLaunchArgument(
        'process_cloud_node_name', default_value='process_cloud_node')

    odom_topic = LaunchConfiguration('odom_topic')
    robot_name = LaunchConfiguration('robot_name')
    process_cloud_node_name = LaunchConfiguration('process_cloud_node_name')

    params_file = PathJoinSubstitution([
        FindPackageShare('scan2shape_launch'),
        'config',
        'process_cloud_node_outdoor_kitti_params.yaml',
    ])

    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node_outdoor.py',
        name=process_cloud_node_name,
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/Odometry', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        robot_name_arg,
        process_cloud_node_name_arg,
        GroupAction([
            PushRosNamespace(robot_name),
            process_cloud_node,
        ]),
    ])
