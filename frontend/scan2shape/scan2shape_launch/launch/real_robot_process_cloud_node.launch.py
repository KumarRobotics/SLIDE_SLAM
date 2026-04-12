"""ROS2 launch file for the real-robot process_cloud_node profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/dragonfly67/quadrotor_ukf/control_odom')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='')

    odom_topic = LaunchConfiguration('odom_topic')
    robot_name = LaunchConfiguration('robot_name')

    # Active profile - the original launch toggled between three yaml files via
    # comments. The full-mission yaml was the active one.
    params_file = PathJoinSubstitution([
        FindPackageShare('scan2shape_launch'),
        'config',
        'process_cloud_node_real_robot_full_mission.yaml',
    ])

    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node.py',
        name='process_cloud_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/odom', odom_topic),
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
