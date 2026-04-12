"""ROS2 launch file for the SLOAM process_cloud_node profile."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    process_cloud_node = Node(
        package='scan2shape_launch',
        executable='process_cloud_node.py',
        name='process_cloud_node2',
        output='screen',
    )

    # The original launch additionally started a topic_tools throttle node.
    # ROS2 ships topic_tools as the `topic_tools` package and it provides a
    # `throttle` executable.
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='segmentation_throttler',
        arguments=[
            'messages',
            '/os_node/segmented_point_cloud_no_destagger',
            '1.5',
            '/os_node/segmented_point_cloud_no_destagger/throttled',
        ],
    )

    return LaunchDescription([
        process_cloud_node,
        throttle_node,
    ])
