"""Play four multi-robot bags in parallel with namespaced topic remaps.

This is a developer bring-up script for replaying a specific 4-robot
recording. The bag directory is parameterised via the SLIDE_SLAM_BAG_BASE
environment variable (no hardcoded /home/<user>/ default — pass an
explicit path so the script works on any machine):

    export SLIDE_SLAM_BAG_BASE=/path/to/bags/dir
    ros2 launch .../play_bag_with_remapping.launch.py

Or via a launch-time arg:

    ros2 launch .../play_bag_with_remapping.launch.py \\
        bag_base:=/path/to/bags/dir

The four bags expected under that directory are:
    robot1-2023-09-28-10-35-14.bag
    robot2-2023-09-28-10-35-57.bag
    robot3-2023-09-28-10-36-32.bag
    robot4-2023-09-28-10-36-53.bag
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _bag_play(bag_path, topic_remaps, clock=False):
    cmd = ["ros2", "bag", "play", bag_path]
    if clock:
        cmd.append("--clock")
    cmd.append("--remap")
    for src, dst in topic_remaps:
        cmd.append(f"{src}:={dst}")
    return ExecuteProcess(cmd=cmd, output="screen")


def _launch_setup(context, *args, **kwargs):
    bag_base = LaunchConfiguration("bag_base").perform(context)
    return [
        _bag_play(
            os.path.join(bag_base, "robot1-2023-09-28-10-35-14.bag"),
            [
                ("/semantic_meas_sync_odom", "/quadrotor1/semantic_meas_sync_odom"),
                ("/quadrotor1/lidar_odom", "/quadrotor1/lidar_odom"),
            ],
        ),
        _bag_play(
            os.path.join(bag_base, "robot2-2023-09-28-10-35-57.bag"),
            [
                ("/semantic_meas_sync_odom", "/quadrotor2/semantic_meas_sync_odom"),
                ("/quadrotor1/lidar_odom", "/quadrotor2/lidar_odom"),
            ],
        ),
        _bag_play(
            os.path.join(bag_base, "robot3-2023-09-28-10-36-32.bag"),
            [
                ("/semantic_meas_sync_odom", "/quadrotor3/semantic_meas_sync_odom"),
                ("/quadrotor1/lidar_odom", "/quadrotor3/lidar_odom"),
            ],
        ),
        _bag_play(
            os.path.join(bag_base, "robot4-2023-09-28-10-36-53.bag"),
            [
                ("/semantic_meas_sync_odom", "/quadrotor4/semantic_meas_sync_odom"),
                ("/quadrotor1/lidar_odom", "/quadrotor4/lidar_odom"),
            ],
            clock=True,
        ),
    ]


def generate_launch_description():
    default_bag_base = os.environ.get("SLIDE_SLAM_BAG_BASE", "")
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_base",
            default_value=default_bag_base,
            description=(
                "Directory containing the four robot*-2023-09-28-*.bag files. "
                "Defaults to $SLIDE_SLAM_BAG_BASE."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
