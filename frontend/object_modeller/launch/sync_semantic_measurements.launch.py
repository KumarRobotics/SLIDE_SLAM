import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="/Odometry")
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot0")

    odom_topic = LaunchConfiguration("odom_topic")
    robot_name = LaunchConfiguration("robot_name")

    pkg_share = get_package_share_directory("object_modeller")
    cylinder_plane_modeller_params = os.path.join(
        pkg_share, "config", "cylinder_plane_modeller_params.yaml")

    point_cloud_ns = PythonExpression(["'/' + '", robot_name, "' + '/'"])

    cylinder_plane_modeller = Node(
        package="object_modeller",
        executable="cylinder_plane_modeller.py",
        name="cylinder_plane_modeller",
        namespace=robot_name,
        output="screen",
        parameters=[cylinder_plane_modeller_params],
        arguments=["--point_cloud_ns", point_cloud_ns],
    )
    sync_cylinder_odom = Node(
        package="object_modeller",
        executable="sync_cylinder_odom.py",
        name="sync_cylinder_odom",
        namespace=robot_name,
        output="screen",
        arguments=["--odom_topic", odom_topic],
    )
    sync_cuboid_odom = Node(
        package="object_modeller",
        executable="sync_cuboid_odom.py",
        name="sync_cuboid_odom",
        namespace=robot_name,
        output="screen",
        arguments=["--odom_topic", odom_topic],
    )
    sync_centroid_odom = Node(
        package="object_modeller",
        executable="sync_centroid_odom.py",
        name="sync_centroid_odom",
        namespace=robot_name,
        output="screen",
        arguments=["--odom_topic", odom_topic],
    )
    sync_all = Node(
        package="object_modeller",
        executable="merge_synced_measurements.py",
        name="sync_all",
        namespace=robot_name,
        output="screen",
    )

    return LaunchDescription([
        odom_topic_arg,
        robot_name_arg,
        cylinder_plane_modeller,
        sync_cylinder_odom,
        sync_cuboid_odom,
        sync_centroid_odom,
        sync_all,
    ])
