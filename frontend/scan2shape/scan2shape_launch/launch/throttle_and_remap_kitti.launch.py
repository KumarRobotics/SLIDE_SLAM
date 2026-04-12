"""ROS2 launch file: throttle the KITTI segmented point cloud topic to 2 Hz."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='throttle_semantic_point_cloud',
        output='screen',
        arguments=[
            'messages',
            '/os_node/segmented_point_cloud_no_destagger_high_freq',
            '2.0',
            '/os_node/segmented_point_cloud_no_destagger',
        ],
    )

    return LaunchDescription([
        throttle_node,
    ])
