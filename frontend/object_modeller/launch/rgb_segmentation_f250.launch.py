from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/dragonfly67/quadrotor_ukf/control_odom")
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot0")
    robot_id_arg = DeclareLaunchArgument(
        "robot_ID", default_value="0")

    odom_topic = LaunchConfiguration("odom_topic")
    robot_name = LaunchConfiguration("robot_name")
    robot_id = LaunchConfiguration("robot_ID")

    rgb_sem_segmentation = Node(
        package="object_modeller",
        executable="detect.py",
        name="rgb_sem_segmentation",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "sim": False,
            "desired_rate": 2.0,
            "confidence_threshold": 0.4,
            "rgb_topic": "camera/color/image_raw/",
            "depth_topic": "camera/depth/image_rect_raw",
            "aligned_depth_topic": "camera/aligned_depth_to_color/image_raw",
            "odom_topic": odom_topic,
            "sync_odom_measurements": True,
            "sync_pc_odom_topic": ["/robot", robot_id, "/sem_detection/sync_pc_odom"],
            "pc_topic": ["/robot", robot_id, "/sem_detection/pointcloud"],
            # For f250 455
            "fx": 390.97088623046875,
            "fy": 390.970886230468,
            "cx": 324.1226806640625,
            "cy": 243.95254516601562,
            "k_depth_scaling_factor": 1000.0,
        }],
    )

    return LaunchDescription([
        odom_topic_arg,
        robot_name_arg,
        robot_id_arg,
        rgb_sem_segmentation,
    ])
