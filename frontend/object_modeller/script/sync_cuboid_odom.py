#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node

from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import StampedRvizMarkerArray
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber

# helper from merge_synced_measurements.py — avoid importing the module directly so we don't trigger its main
def rviz2cubelist(rviz_cube):
    from sloam_msgs.msg import ROSCube
    from geometry_msgs.msg import Pose
    cubelist = []
    for cube in rviz_cube.markers:
        dims = [float(cube.scale.x), float(cube.scale.y), float(cube.scale.z)]
        semantic_label = -2
        pose = Pose()
        pose.position = cube.pose.position
        pose.orientation = cube.pose.orientation
        ros_cube_msg = ROSCube()
        ros_cube_msg.dim = dims
        ros_cube_msg.semantic_label = semantic_label
        ros_cube_msg.pose = pose
        cubelist.append(ros_cube_msg)
    return cubelist


# This node syncs the measurements from the following topics:
# cuboid measurements
# odometry


class SyncMeasurementsCuboidOdom(Node):
    def __init__(self, args):
        super().__init__("sync_semantic_node_cuboid_odom")
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self, Odometry, self.odom_topic)
        self.get_logger().info(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))
        self.cuboid_sub = Subscriber(
            self, StampedRvizMarkerArray, "cuboid_measurements")
        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100
        # use ApproximateTimeSynchronizer to sync the messages
        self.sync1 = ApproximateTimeSynchronizer(
            [self.cuboid_sub, self.odom_sub], queue_size=200, slop=0.01)
        self.sync1.registerCallback(self.sync_callback1)
        # create publishers
        self.sync_meas_pub = self.create_publisher(
            SemanticMeasSyncOdom, "semantic_meas_sync_odom_raw", 10)

    def sync_callback1(self, cuboid_msg, odom_msg):
        # create a delay for processing
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # fill in the cuboid factors
        sync_msg.cuboid_factors = rviz2cubelist(cuboid_msg.cuboid_rviz_markers)
        # publish the message
        self.sync_meas_pub.publish(sync_msg)
        # put empty cylinder factors
        sync_msg.cylinder_factors = []
        # print in green to indicate that the message is published
        self.get_logger().info(
            "\033[92mSynced measurements (cuboid only!) published\033[0m",
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

    node = SyncMeasurementsCuboidOdom(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print("Node Killed")


if __name__ == "__main__":
    main()
