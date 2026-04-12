"""Ported single_robot_sloam_test_outdoor.launch."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sloam_share = get_package_share_directory('sloam')
    sloam_yaml = PathJoinSubstitution([sloam_share, 'params', 'sloam.yaml'])
    rviz_config = PathJoinSubstitution(
        [sloam_share, 'launch', 'rviz', 'decentralized_sloam.rviz']
    )
    enable_rviz = LaunchConfiguration('enable_rviz')
    turn_off_intra = LaunchConfiguration('turn_off_intra_loop_closure')
    host_robot_id = '0'

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('hostRobot0_ID', default_value='0'),
        DeclareLaunchArgument('turn_off_intra_loop_closure', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='sloam_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(enable_rviz),
        ),
        Node(
            package='sloam',
            executable='sloam_node',
            name='sloam0',
            output='screen',
            parameters=[
                sloam_yaml,
                {
                    'hostRobotID': int(host_robot_id),
                    'turn_off_intra_loop_closure': turn_off_intra,
                    'priorTFKnown': False,
                    'priorTF_x': 0.0,
                    'priorTF_y': 0.0,
                    'priorTF_z': 0.0,
                },
            ],
            remappings=[
                ('/sloam/cubes_map', f'robot_{host_robot_id}/sloam/cubes_map'),
                ('/sloam/cubes_submap', f'robot_{host_robot_id}/sloam/cubes_submap'),
                ('/sloam/debug/robot0/trajectory', f'robot_{host_robot_id}/sloam/debug/robot0/trajectory'),
                ('/sloam/debug/robot1/trajectory', f'robot_{host_robot_id}/sloam/debug/robot1/trajectory'),
                ('/sloam/cylinders_map', f'robot_{host_robot_id}/sloam/cylinders_map'),
                ('/sloam/submap_cylinder_models', f'robot_{host_robot_id}/sloam/submap_cylinder_models'),
                ('/sloam/map_pose', f'robot_{host_robot_id}/sloam/map_pose'),
                ('/sloam/observation', f'robot_{host_robot_id}/sloam/observation'),
                ('/sloam/optimized_point_landmarks', f'robot_{host_robot_id}/sloam/optimized_point_landmarks'),
                ('/sloam/odom', '/Odometry'),
            ],
        ),
    ])
