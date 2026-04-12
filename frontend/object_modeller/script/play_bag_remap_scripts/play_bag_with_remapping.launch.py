from launch import LaunchDescription
from launch.actions import ExecuteProcess


BAG_BASE = "/home/sam/xmas_slam_ws/src/generic-sloam/object_modeller/script/play_bag_remap_scripts"


def _bag_play(bag_path, topic_remaps, clock=False):
    cmd = ["ros2", "bag", "play", bag_path]
    if clock:
        cmd.append("--clock")
    cmd.append("--remap")
    for src, dst in topic_remaps:
        cmd.append(f"{src}:={dst}")
    return ExecuteProcess(cmd=cmd, output="screen")


def generate_launch_description():
    rosbag_play = _bag_play(
        f"{BAG_BASE}/robot1-2023-09-28-10-35-14.bag",
        [
            ("/semantic_meas_sync_odom", "/quadrotor1/semantic_meas_sync_odom"),
            ("/quadrotor1/lidar_odom", "/quadrotor1/lidar_odom"),
        ],
    )
    rosbag_play2 = _bag_play(
        f"{BAG_BASE}/robot2-2023-09-28-10-35-57.bag",
        [
            ("/semantic_meas_sync_odom", "/quadrotor2/semantic_meas_sync_odom"),
            ("/quadrotor1/lidar_odom", "/quadrotor2/lidar_odom"),
        ],
    )
    rosbag_play3 = _bag_play(
        f"{BAG_BASE}/robot3-2023-09-28-10-36-32.bag",
        [
            ("/semantic_meas_sync_odom", "/quadrotor3/semantic_meas_sync_odom"),
            ("/quadrotor1/lidar_odom", "/quadrotor3/lidar_odom"),
        ],
    )
    rosbag_play4 = _bag_play(
        f"{BAG_BASE}/robot4-2023-09-28-10-36-53.bag",
        [
            ("/semantic_meas_sync_odom", "/quadrotor4/semantic_meas_sync_odom"),
            ("/quadrotor1/lidar_odom", "/quadrotor4/lidar_odom"),
        ],
        clock=True,
    )

    return LaunchDescription([
        rosbag_play,
        rosbag_play2,
        rosbag_play3,
        rosbag_play4,
    ])
