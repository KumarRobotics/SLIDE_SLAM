"""Ported single_robot_sloam_test_benchmark.launch."""

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
    host_robot_id = LaunchConfiguration('hostRobot_ID')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('hostRobot_ID', default_value='0'),
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
            name=['sloam', host_robot_id],
            output='screen',
            parameters=[
                sloam_yaml,
                {
                    'hostRobotID': host_robot_id,
                    'priorTFKnown': False,
                    'priorTF_x': 0.0,
                    'priorTF_y': 0.0,
                    'priorTF_z': 0.0,
                    'turn_off_intra_loop_closure': True,
                    'turn_off_inter_loop_closure': True,
                },
            ],
            remappings=[
                ('/sloam/cubes_map', ['robot_', host_robot_id, '/sloam/cubes_map']),
                ('/sloam/cubes_submap', ['robot_', host_robot_id, '/sloam/cubes_submap']),
                ('/sloam/debug/robot0/trajectory', ['robot_', host_robot_id, '/sloam/debug/robot0/trajectory']),
                ('/sloam/debug/robot1/trajectory', ['robot_', host_robot_id, '/sloam/debug/robot1/trajectory']),
                ('/sloam/cylinders_map', ['robot_', host_robot_id, '/sloam/cylinders_map']),
                ('/sloam/submap_cylinder_models', ['robot_', host_robot_id, '/sloam/submap_cylinder_models']),
                ('/sloam/map_pose', ['robot_', host_robot_id, '/sloam/map_pose']),
                ('/sloam/observation', ['robot_', host_robot_id, '/sloam/observation']),
                ('/sloam/optimized_point_landmarks', ['robot_', host_robot_id, '/sloam/optimized_point_landmarks']),
                ('/sloam/odom', ['robot', host_robot_id, '/Odometry']),
            ],
        ),
    ])
