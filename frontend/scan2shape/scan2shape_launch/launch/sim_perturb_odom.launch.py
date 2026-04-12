"""ROS2 launch file for the simulator odometry perturbation node.

NOTE: The original ROS1 .launch referenced ``add_noise_to_ground_truth_odom.py``
which does not exist in this package's script/ directory. This launch file is
preserved for parity but will fail at startup until the script is added back
to the package.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perturb_node = Node(
        package='scan2shape_launch',
        executable='add_noise_to_ground_truth_odom.py',
        name='add_noise_to_ground_truth_odom',
        output='screen',
        remappings=[
            ('/gt_odom', '/ddk/ground_truth/odom'),
        ],
    )

    return LaunchDescription([
        perturb_node,
    ])
