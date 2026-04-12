"""ROS2 launch file: LLOL odometry node namespaced under os_node.

(ROS1 legacy note) Ports the original launch one-to-one. The referenced
``llol`` package and its ``sv_node_llol`` executable / config yaml must
exist in the corresponding ament packages for this launch to work.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    tbb_arg = DeclareLaunchArgument('tbb', default_value='0')
    log_arg = DeclareLaunchArgument('log', default_value='0')
    vis_arg = DeclareLaunchArgument('vis', default_value='false')
    rigid_arg = DeclareLaunchArgument('rigid', default_value='true')
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom_llol')

    tbb = LaunchConfiguration('tbb')
    log = LaunchConfiguration('log')
    vis = LaunchConfiguration('vis')
    rigid = LaunchConfiguration('rigid')
    odom_frame = LaunchConfiguration('odom_frame')

    llol_yaml = PathJoinSubstitution([
        FindPackageShare('llol'),
        'config',
        'llol.yaml',
    ])

    llol_node = Node(
        package='llol',
        executable='sv_node_llol',
        name='llol_odom',
        output='screen',
        parameters=[
            llol_yaml,
            {
                'tbb': tbb,
                'log': log,
                'vis': vis,
                'rigid': rigid,
                'odom_frame': odom_frame,
            },
        ],
        remappings=[
            ('~/imu', '/quadrotor/imu'),
            ('~/image', '/quadrotor/image'),
            ('~/camera_info', '/quadrotor/camera_info'),
        ],
    )

    return LaunchDescription([
        debug_arg,
        tbb_arg,
        log_arg,
        vis_arg,
        rigid_arg,
        odom_frame_arg,
        GroupAction([
            PushRosNamespace('os_node'),
            llol_node,
        ]),
    ])
