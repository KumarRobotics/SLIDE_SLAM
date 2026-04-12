#!/usr/bin/env python3

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory

from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import StampedRvizMarkerArray, ROSEllipsoid
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber

# This node syncs the measurements from the following topics:
# centroid/ellipsoids (point landmark) measurements
# odometry


class SyncMeasurementsCentroidOdom(Node):
    def __init__(self, args):
        super().__init__("sync_semantic_node_centroid_odom")
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self, Odometry, self.odom_topic)
        self.centroid_sub = Subscriber(
            self, StampedRvizMarkerArray, "chair_cuboids_stamped")
        self.get_logger().info(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))
        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100

        # In ROS2, parameters are local to the node.
        self.declare_parameter("robot_name", "robot0")
        self.declare_parameter("process_cloud_node_name", "process_cloud_node")
        self.declare_parameter("detect_no_seg", False)
        self.robot_name = self.get_parameter("robot_name").value
        self.process_cloud_node_name = self.get_parameter(
            "process_cloud_node_name").value
        self.detect_no_seg = self.get_parameter("detect_no_seg").value

        try:
            scan2shape_share = get_package_share_directory("scan2shape_launch")
        except Exception:
            scan2shape_share = ""

        if self.detect_no_seg:
            self.get_logger().info("Running with open vocabulary object detector")
            cfg = os.path.join(
                scan2shape_share, "config",
                "process_cloud_node_indoor_open_vocab_cls_info.yaml")
        else:
            self.get_logger().info("Running with closed vocabulary object detector")
            cfg = os.path.join(
                scan2shape_share, "config",
                "process_cloud_node_indoor_cls_info.yaml")

        with open(cfg, "r") as file:
            self.cls_full_data = yaml.load(file, Loader=yaml.FullLoader)

        self.cls = {cls_name: self.cls_full_data[cls_name]["id"]
                    for cls_name in self.cls_full_data.keys()}
        # use ApproximateTimeSynchronizer to sync the messages
        self.sync1 = ApproximateTimeSynchronizer(
            [self.centroid_sub, self.odom_sub], queue_size=400, slop=0.01)
        self.sync1.registerCallback(self.sync_callback1)
        # create publishers
        self.sync_meas_pub = self.create_publisher(
            SemanticMeasSyncOdom, "semantic_meas_sync_odom_raw", 10)

    def sync_callback1(self, cuboid_msg, odom_msg):
        self.get_logger().info(
            "Odom and centroid messages received!",
            throttle_duration_sec=7)
        # create a delay for processing
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # take the centroid as the centroid factor
        # centroid_factors is geometry_msgs/Point[]
        ellipsoid_factors = []
        # cuboid factors are MarkerArray
        for cuboid in cuboid_msg.cuboid_rviz_markers.markers:
            ellipsoid_factor = ROSEllipsoid()
            #  the label is in the ns field
            semantic_label_str = cuboid.ns
            ellipsoid_factor.semantic_label = self.cls[semantic_label_str]
            # ellipsoid_factors.scale is float 3
            # ellipsoid_factors.pose is geometry_msgs/Pose
            # ellipsoid_factors.semantic_label is int
            ellipsoid_factor.pose = cuboid.pose
            # set orientation to identity
            ellipsoid_factor.pose.orientation.x = 0.0
            ellipsoid_factor.pose.orientation.y = 0.0
            ellipsoid_factor.pose.orientation.z = 0.0
            ellipsoid_factor.pose.orientation.w = 1.0
            ellipsoid_factor.scale = [
                float(cuboid.scale.x),
                float(cuboid.scale.y),
                float(cuboid.scale.z),
            ]
            ellipsoid_factors.append(ellipsoid_factor)
        sync_msg.ellipsoid_factors = ellipsoid_factors
        # publish the message
        self.sync_meas_pub.publish(sync_msg)
        # put empty cylinder factors
        sync_msg.cylinder_factors = []
        # put empty cuboid factors
        sync_msg.cuboid_factors = []
        # print in green to indicate that the message is published
        self.get_logger().info(
            "\033[92mSynced measurements (centroid only!) published\033[0m",
            throttle_duration_sec=3)


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    # add indoor argument
    ap.add_argument("-o", "--odom_topic", type=str, default="odom",
                    help="odometry topic")
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    parsed, _ = ap.parse_known_args(argv)
    parsed_args = vars(parsed)

    node = SyncMeasurementsCentroidOdom(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print("Node Killed")


if __name__ == "__main__":
    main()
