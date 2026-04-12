"""ROS2 launch file: ouster decoder driver.

(ROS1 legacy note) Ports the original launch one-to-one. The referenced
``ouster_decoder`` package and its ``ouster_driver`` executable / decoder
launch must exist in the corresponding ament packages for this launch to
work.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    replay_arg = DeclareLaunchArgument('replay', default_value='false')
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
    timestamp_mode = LaunchConfiguration('timestamp_mode')
    metadata = LaunchConfiguration('metadata')

    os_node = Node(
        package='ouster_decoder',
        executable='ouster_driver',
        name='os_node',
        output='screen',
        parameters=[{
            'lidar_mode': lidar_mode,
            'timestamp_mode': timestamp_mode,
            'replay': replay,
            'sensor_hostname': sensor_hostname,
            'udp_dest': udp_dest,
            'lidar_port': lidar_port,
            'imu_port': imu_port,
            'metadata': metadata,
        }],
    )

    decoder_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_decoder'),
                'launch',
                'decoder.launch.py',
            ])
        ),
        launch_arguments={
            'lidar_ns': robot,
            'replay': 'false',
        }.items(),
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
        os_node,
        decoder_include,
    ])
