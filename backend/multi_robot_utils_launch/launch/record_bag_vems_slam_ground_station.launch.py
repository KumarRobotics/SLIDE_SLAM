from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


TOPICS = [
    '/tf',
    '/tf_static',
    '/robot0/PoseMstPairFromOthers',
    '/robot0/pose_high_freq',
    '/robot0/semantic_meas_sync_odom',
    '/robot0/PoseMstPairFromOthers',
    '/robot1/PoseMstPairFromOthers',
    '/robot2/PoseMstPairFromOthers',
    '/robot3/PoseMstPairFromOthers',
    '/robot4/PoseMstPairFromOthers',
    '/scarab41/odom_laser',
    '/robot1/PoseMstPairFromOthers',
    '/robot1/pose_high_freq',
    '/robot1/semantic_meas_sync_odom',
    '/scarab45/odom_laser',
    '/scarab41/waypoint_generator/waypoints',
    '/scarab45/waypoint_generator/waypoints',
    '/scarab41/exploration_node/planning_vis/frontier',
    '/scarab45/exploration_node/planning_vis/frontier',
    '/scarab41/move_base_simple/goal',
    '/scarab45/move_base_simple/goal',
    '/sloam0/robot0/trajectory',
    '/sloam0/robot1/trajectory',
    '/sloam0/robot2/trajectory',
    '/sloam0/robot3/trajectory',
    '/sloam0/robot4/trajectory',
    '/sloam1/robot0/trajectory',
    '/sloam1/robot1/trajectory',
    '/sloam1/robot2/trajectory',
    '/sloam1/robot3/trajectory',
    '/sloam1/robot4/trajectory',
    '/robot_0/sloam/cubes_map',
    '/robot_0/sloam/cubes_submap',
    '/robot_0/sloam/cylinders_map',
    '/robot_0/sloam/submap_cylinder_models',
    '/robot_0/sloam/optimized_point_landmarks',
    '/robot0/sync_odom_high_freq',
    '/robot0/sloam_to_vio_odom',
    '/robot0/sloam_odom_high_freq',
    '/robot0/pose_high_freq',
    '/robot0/PoseMstPairFromOthers',
    '/robot_1/sloam/cubes_map',
    '/robot_1/sloam/cubes_submap',
    '/robot_1/sloam/cylinders_map',
    '/robot_1/sloam/submap_cylinder_models',
    '/robot_1/sloam/optimized_point_landmarks',
    '/robot1/sync_odom_high_freq',
    '/robot1/sloam_to_vio_odom',
    '/robot1/sloam_odom_high_freq',
    '/robot1/pose_high_freq',
    '/robot11/PoseMstPairFromOthers',
    '/place_recognition_node/matching_results',
]


def _build_recorder(context, *args, **kwargs):
    directory = LaunchConfiguration('dir').perform(context)
    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--compression-mode', 'file',
                '--compression-format', 'zstd',
                '-o', f'{directory}vems-slam',
                *TOPICS,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir', default_value='/home/sam/bags/'),
        OpaqueFunction(function=_build_recorder),
    ])
