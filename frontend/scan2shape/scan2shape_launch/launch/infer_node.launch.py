"""ROS2 launch file for the inference node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    infer_node_name_arg = DeclareLaunchArgument(
        'infer_node_name', default_value='inference_node')

    infer_node_name = LaunchConfiguration('infer_node_name')

    params_file = PathJoinSubstitution([
        FindPackageShare('scan2shape_launch'),
        'config',
        'infer_node_params.yaml',
    ])

    infer_node = Node(
        package='scan2shape_launch',
        executable='infer_node.py',
        name=infer_node_name,
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        infer_node_name_arg,
        infer_node,
    ])
