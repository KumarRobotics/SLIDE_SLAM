"""ROS2 launch file for one decentralized sloam robot (ported from decentralized_sloam.launch)."""

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
    odom_topic = LaunchConfiguration('odom_topic')
    turn_off_intra = LaunchConfiguration('turn_off_intra_loop_closure')
    turn_off_inter = LaunchConfiguration('turn_off_inter_loop_closure')
    prior_tf_known = LaunchConfiguration('priorTFKnown')
    prior_tf_x = LaunchConfiguration('priorTF_x')
    prior_tf_y = LaunchConfiguration('priorTF_y')
    prior_tf_z = LaunchConfiguration('priorTF_z')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('hostRobot_ID', default_value='0'),
        DeclareLaunchArgument('odom_topic', default_value='/robot0/odom'),
        DeclareLaunchArgument('turn_off_intra_loop_closure', default_value='false'),
        DeclareLaunchArgument('turn_off_inter_loop_closure', default_value='false'),
        DeclareLaunchArgument('priorTFKnown', default_value='false'),
        DeclareLaunchArgument('priorTF_x', default_value='0.0'),
        DeclareLaunchArgument('priorTF_y', default_value='0.0'),
        DeclareLaunchArgument('priorTF_z', default_value='0.0'),
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
                    'turn_off_intra_loop_closure': turn_off_intra,
                    'turn_off_inter_loop_closure': turn_off_inter,
                    'priorTFKnown': prior_tf_known,
                    'priorTF_x': prior_tf_x,
                    'priorTF_y': prior_tf_y,
                    'priorTF_z': prior_tf_z,
                },
            ],
            remappings=[
                ('/sloam/cubes_map', ['robot_', host_robot_id, '/sloam/cubes_map']),
                ('/sloam/cubes_submap', ['robot_', host_robot_id, '/sloam/cubes_submap']),
                ('/sloam/cylinders_map', ['robot_', host_robot_id, '/sloam/cylinders_map']),
                ('/sloam/submap_cylinder_models', ['robot_', host_robot_id, '/sloam/submap_cylinder_models']),
                ('/sloam/optimized_point_landmarks', ['robot_', host_robot_id, '/sloam/optimized_point_landmarks']),
                ('/sloam/odom', odom_topic),
            ],
        ),
    ])
