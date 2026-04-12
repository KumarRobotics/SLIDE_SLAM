from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # step 1 (disabled in original launch file):
    # Node(package='topic_tools', executable='relay', name='odom_relay',
    #      arguments=['/dragonfly67/quadrotor_ukf/control_odom', '/robot_0/vio_odom']),
    # Node(package='topic_tools', executable='relay', name='semantic_meas_sync_odom_relay',
    #      arguments=['/semantic_meas_sync_odom', '/robot_0/semantic_meas_sync_odom']),

    # step 2
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            arguments=['/robot_0/vio_odom', '/robot_1/vio_odom'],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='semantic_meas_sync_odom_relay',
            arguments=['/robot_0/semantic_meas_sync_odom', '/robot_1/semantic_meas_sync_odom'],
        ),
    ])
