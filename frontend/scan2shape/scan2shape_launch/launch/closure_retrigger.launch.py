"""ROS2 launch file for the loop-closure retrigger node.

(ROS1 legacy note) The original .launch referenced
``pub_loop_closure_retrigger.py`` which does not exist in this package's
script/ directory. This launch file is preserved for parity but will fail
at startup until the script is added back to the package.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    closure_retrigger = Node(
        package='scan2shape_launch',
        executable='pub_loop_closure_retrigger.py',
        name='closure_retrigger',
        output='screen',
    )

    return LaunchDescription([
        closure_retrigger,
    ])
