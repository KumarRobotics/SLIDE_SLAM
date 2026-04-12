from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_recorder(context, *args, **kwargs):
    directory = LaunchConfiguration('dir').perform(context)
    robot1_ns = LaunchConfiguration('robot1_ns').perform(context)
    robot2_ns = LaunchConfiguration('robot2_ns').perform(context)
    robot3_ns = LaunchConfiguration('robot3_ns').perform(context)
    robot4_ns = LaunchConfiguration('robot4_ns').perform(context)

    topics = []
    for ns in (robot1_ns, robot2_ns, robot3_ns, robot4_ns):
        topics.extend([
            f'{ns}/car_cuboids_body',
            f'{ns}/lidar_odom',
            f'{ns}/ground_cloud',
            f'{ns}/tree_cloud',
        ])

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', directory, *topics],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dir', default_value='/home/sam/bags/xmas-slam-bags'),
        DeclareLaunchArgument('robot1_ns', default_value='/quadrotor1'),
        DeclareLaunchArgument('robot2_ns', default_value='/quadrotor2'),
        DeclareLaunchArgument('robot3_ns', default_value='/quadrotor3'),
        DeclareLaunchArgument('robot4_ns', default_value='/quadrotor4'),
        OpaqueFunction(function=_build_recorder),
    ])
