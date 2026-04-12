"""ROS2 launch file: ouster driver setup intended for faster_lio.

(ROS1 legacy note) This launch ports the original launch one-to-one. The
referenced ouster_ros executables (``os_cloud_node``, ``relay``) and the
decoder include target need to exist in the corresponding ament packages
for this launch to work.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='quadrotor')
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname', default_value='192.168.100.12')
    udp_dest_arg = DeclareLaunchArgument(
        'udp_dest', default_value='192.168.100.1')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='7502')
    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value='7503')
    replay_arg = DeclareLaunchArgument('replay', default_value='true')
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode', default_value='1024x10')
    timestamp_mode_arg = DeclareLaunchArgument(
        'timestamp_mode', default_value='')
    metadata_arg = DeclareLaunchArgument(
        'metadata', default_value='ouster_metadata.json')
    viz_arg = DeclareLaunchArgument('viz', default_value='false')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')

    robot = LaunchConfiguration('robot')
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    udp_dest = LaunchConfiguration('udp_dest')
    lidar_port = LaunchConfiguration('lidar_port')
    imu_port = LaunchConfiguration('imu_port')
    replay = LaunchConfiguration('replay')
    lidar_mode = LaunchConfiguration('lidar_mode')
    metadata = LaunchConfiguration('metadata')
    tf_prefix = LaunchConfiguration('tf_prefix')

    ouster_decoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scan2shape_launch'),
                'launch',
                'ouster_decoder.launch.py',
            ])
        ),
        launch_arguments={
            'robot': robot,
            'sensor_hostname': sensor_hostname,
            'udp_dest': udp_dest,
            'lidar_port': lidar_port,
            'imu_port': imu_port,
            'replay': replay,
            'lidar_mode': lidar_mode,
            'metadata': metadata,
        }.items(),
    )

    os_cloud_node = Node(
        package='ouster_ros',
        executable='os_cloud_node',
        name='os_cloud_node',
        output='screen',
        parameters=[{'tf_prefix': tf_prefix}],
        remappings=[
            ('~/os_config', '/os_node/os_config'),
            ('~/lidar_packets', '/os_node/lidar_packets'),
            ('~/imu_packets', '/os_node/imu_packets'),
        ],
    )

    relay_lidar = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar',
        arguments=['/os1_node/lidar_packets', '/os_node/lidar_packets'],
    )

    relay_imu = Node(
        package='topic_tools',
        executable='relay',
        name='relay_imu',
        arguments=['/os1_node/imu_packets', '/os_node/imu_packets'],
    )

    return LaunchDescription([
        robot_arg,
        sensor_hostname_arg,
        udp_dest_arg,
        lidar_port_arg,
        imu_port_arg,
        replay_arg,
        lidar_mode_arg,
        timestamp_mode_arg,
        metadata_arg,
        viz_arg,
        tf_prefix_arg,
        ouster_decoder_launch,
        GroupAction([
            PushRosNamespace(robot),
            os_cloud_node,
            relay_lidar,
            relay_imu,
        ]),
    ])
