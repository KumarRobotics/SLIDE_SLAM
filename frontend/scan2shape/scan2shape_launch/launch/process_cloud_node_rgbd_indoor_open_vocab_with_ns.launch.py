"""ROS2 launch file for the indoor open-vocab RGB-D process_cloud_node, namespaced per robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/dragonfly67/quadrotor_ukf/control_odom')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='robot0')
    process_cloud_node_name_arg = DeclareLaunchArgument(
        'process_cloud_node_name', default_value='process_cloud_node')
    detect_no_seg_arg = DeclareLaunchArgument(
        'detect_no_seg', default_value='False')

    odom_topic = LaunchConfiguration('odom_topic')
    robot_name = LaunchConfiguration('robot_name')
    process_cloud_node_name = LaunchConfiguration('process_cloud_node_name')
    detect_no_seg = LaunchConfiguration('detect_no_seg')

    params_file = PathJoinSubstitution([
        FindPackageShare('scan2shape_launch'),
        'config',
        'process_cloud_node_indoor_open_vocab_params.yaml',
    ])

    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node.py',
        name=process_cloud_node_name,
        output='screen',
        parameters=[
            params_file,
            {'detect_no_seg': detect_no_seg},
        ],
        remappings=[
            ('/odom', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        robot_name_arg,
        process_cloud_node_name_arg,
        detect_no_seg_arg,
        GroupAction([
            PushRosNamespace(robot_name),
            process_cloud_node,
        ]),
    ])
