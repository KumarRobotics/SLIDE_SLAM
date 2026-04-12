#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node

from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import ROSCylinderArray
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber

# This node syncs the measurements from the following topics:
# cylinder measurements
# odometry


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SyncMeasurementsCylinderOdom(Node):
    def __init__(self, args):
        super().__init__("sync_semantic_node_cylinder_odom")
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self, Odometry, self.odom_topic)
        # print in green odom topic
        self.get_logger().debug(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))

        self.cylinder_sub = Subscriber(
            self, ROSCylinderArray, "cylinder_measurements")

        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100

        # use ApproximateTimeSynchronizer to sync the messages
        self.sync2 = ApproximateTimeSynchronizer(
            [self.cylinder_sub, self.odom_sub], queue_size=100, slop=0.01)
        self.sync2.registerCallback(self.sync_callback2)

        # create publishers
        self.sync_meas_pub = self.create_publisher(
            SemanticMeasSyncOdom, "semantic_meas_sync_odom_raw", 10)

    def sync_callback2(self, cylinder_msg, odom_msg):
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # fill in the cylinder factors
        sync_msg.cylinder_factors = cylinder_msg.cylinders
        # put empty cuboid factors (empty MarkerArray)
        sync_msg.cuboid_factors = []
        # put point landmarks as empty
        sync_msg.ellipsoid_factors = []
        # publish the message

        # check if the timestamp is already synced
        odom_time_sec = _stamp_to_sec(odom_msg.header.stamp)
        if odom_time_sec in self.synced_timestamps:
            self.get_logger().warn(
                "\033[93mWarning: timestamp already synced, skipping this to avoid duplicate measurements\033[0m",
                throttle_duration_sec=2)
        else:
            self.sync_meas_pub.publish(sync_msg)
            # print in green to indicate that the message is published
            self.get_logger().info(
                "\033[92mSynced measurements (cylinder only!) published\033[0m",
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

    node = SyncMeasurementsCylinderOdom(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print("Node Killed")


if __name__ == "__main__":
    main()
