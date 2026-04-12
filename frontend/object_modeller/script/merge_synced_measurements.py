#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node

from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import ROSCube
from geometry_msgs.msg import Pose

# This node merges the measurements from the following topics:
# centroid measurements
# cylinder measurements
# cuboid measurements
# odometry
# if a certain type of measurement is not available, it will be empty


def rviz2cubelist(rviz_cube):
    cubelist = []
    # roscube
    # float32[3] dim
    # int8 semantic_label
    # geometry_msgs/Pose pose
    for cube in rviz_cube.markers:
        dims = [float(cube.scale.x), float(cube.scale.y), float(cube.scale.z)]
        # default cuboid label is -2
        semantic_label = -2
        # assemble Pose
        # convert into geometry_msgs/Pose
        pose = Pose()
        pose.position = cube.pose.position
        pose.orientation = cube.pose.orientation
        ros_cube_msg = ROSCube()
        ros_cube_msg.dim = dims
        ros_cube_msg.semantic_label = semantic_label
        ros_cube_msg.pose = pose
        cubelist.append(ros_cube_msg)
    return cubelist


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MergeSyncedMesurements(Node):
    def __init__(self):
        super().__init__("sync_semantic_filter_node")
        self.sync_meas_sub = self.create_subscription(
            SemanticMeasSyncOdom, "semantic_meas_sync_odom_raw",
            self.filter_callback, 10)
        self.sync_meas_pub = self.create_publisher(
            SemanticMeasSyncOdom, "semantic_meas_sync_odom", 10)
        self.max_type_of_measurements = 3  # cylinder, cuboid, centroid
        # seconds, delayed publish, the measurements may come with a delay, we wait for this duration to sync and merge the measurements
        self.time_window_for_filtering = 1.0
        # create a timer to check if we have the oldest message with a time stamp older than current time - time_window_for_filtering
        desired_rate = 100  # Hz
        self.check_timer = self.create_timer(1.0 / desired_rate, self.check_cb)
        self.msg_buffer_limit = 500
        self.past_msgs = []  # deque()
        self.past_msgs_ori = []  # deque()
        self.latest_msg_time = None
        self.last_print_time = self._now_sec()

    def _now_sec(self):
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def filter_callback(self, msg):
        add_msg = True
        msg_time = _stamp_to_sec(msg.header.stamp)
        if self.latest_msg_time is None:
            self.latest_msg_time = msg_time
        else:
            if msg_time < self.latest_msg_time:
                delay = self.latest_msg_time - msg_time
                # print in blue, delay of how many seconds in 2 decimal places
                self.get_logger().info(
                    "\033[94m measurements come with a delay of {:.2f} seconds\033[0m".format(delay))
                # if delay is more than self.time_window_for_filtering, print in red
                if delay > self.time_window_for_filtering:
                    self.get_logger().error(
                        "\033[91mERROR: measurements come with a delay of {:.2f} seconds, which is more than the time window for filtering {:.2f} seconds\033[0m".format(
                            delay, self.time_window_for_filtering))
                    # print in red discard this msg
                    self.get_logger().error(
                        "\033[91mERROR: Discarding this message!\033[0m")
                    add_msg = False
                    # check the type of measurements in the message
                    if msg.cuboid_factors != []:
                        self.get_logger().info(
                            "\033[94mINFO: There are cuboid measurements in the message!\033[0m",
                            throttle_duration_sec=3)
                    if msg.centroid_factors != []:
                        self.get_logger().info(
                            "\033[94mINFO: There are centroid measurements in the message!\033[0m",
                            throttle_duration_sec=3)
                    if msg.cylinder_factors != []:
                        self.get_logger().info(
                            "\033[94mINFO: There are cylinder measurements in the message!\033[0m",
                            throttle_duration_sec=3)
            else:
                self.latest_msg_time = msg_time

        if add_msg:
            # add the message to the end of the list
            self.past_msgs_ori.append(msg)

    def check_cb(self):
        # time the following function
        start_time = self._now_sec()

        # create a copy of the list to avoid modifying the list while iterating through it
        self.past_msgs = copy.deepcopy(self.past_msgs_ori)
        # if length is too long, print in red warning msgs saying this should not happen
        while len(self.past_msgs) > self.msg_buffer_limit:
            self.get_logger().info(
                "\033[91mERROR: More than {} messages in the past_msgs list! Removing the oldest\033[0m".format(
                    self.msg_buffer_limit),
                throttle_duration_sec=2)
            # print this may cause skipping some measurments
            self.get_logger().info(
                "\033[91mERROR: This will cause skipping some measurements!\033[0m",
                throttle_duration_sec=2)
            # remove the oldest message (left)
            self.past_msgs.pop(0)
            self.past_msgs_ori.pop(0)
        if len(self.past_msgs) == 0:
            # if the list is empty, return
            return

        # check if we have the oldest message with a time stamp older than current time - time_window_for_filtering
        oldest_msg = self.past_msgs[0]
        oldest_msg_time = _stamp_to_sec(oldest_msg.header.stamp)
        time_elapsed = self.latest_msg_time - oldest_msg_time
        if time_elapsed > self.time_window_for_filtering:
            if time_elapsed > 2 * self.time_window_for_filtering:
                # print time elapsed too much in red
                self.get_logger().info(
                    "\033[91mERROR: Messages time diff is too much! {:.2f} seconds! Maybe increase the desired_rate! \033[0m".format(
                        time_elapsed),
                    throttle_duration_sec=2)
            # if yes, check_and_publish, and pop the oldest message

            ids_to_remove = self.check_and_publish(oldest_msg)
            for idx in ids_to_remove:
                # sanity check: since we are only appending elements to the end of past_msgs_ori, all elements with idx should be the same in both lists
                if _stamp_to_sec(self.past_msgs[idx].header.stamp) != _stamp_to_sec(self.past_msgs_ori[idx].header.stamp):
                    self.get_logger().error(
                        "\033[91mERROR: The two lists are not the same at the index elements!\033[0m",
                        throttle_duration_sec=2)
                    # print the index
                    # print the two lists one by one
                    print("past_msgs: ")
                    for m in self.past_msgs:
                        print(_stamp_to_sec(m.header.stamp))
                    print("past_msgs_ori: ")
                    for m in self.past_msgs_ori:
                        print(_stamp_to_sec(m.header.stamp))
            # remove elements in ids_to_remove from self.past_msgs_ori
            for idx in sorted(ids_to_remove, reverse=True):
                self.past_msgs_ori.pop(idx)

            # print in blue color remove msg, queue length
            self.get_logger().info(
                "\033[94mRemoving msg, queue length: {}\033[0m".format(
                    len(self.past_msgs_ori)),
                throttle_duration_sec=2)

        else:
            # if no, wait for the next message
            pass

        # print the time taken every 1 seconds
        if self._now_sec() - self.last_print_time > 1.0:
            self.last_print_time = self._now_sec()
        _ = start_time  # keep variable to preserve structure

    def check_and_publish(self, oldest_msg):
        ids_to_remove = [0]  # 0 refers to the oldest message
        oldest_msg_time = _stamp_to_sec(oldest_msg.header.stamp)
        messages_to_merge = [oldest_msg]  # a list of messages to merge
        for msg_idx in range(1, len(self.past_msgs)):
            msg = self.past_msgs[msg_idx]
            # iterate through the past messages, if the time stamp is the same as the oldest message, append to the list so that we will merge them
            if _stamp_to_sec(msg.header.stamp) == oldest_msg_time:
                messages_to_merge.append(msg)
                # add the id to the list of ids to remove
                ids_to_remove.append(msg_idx)
                if len(messages_to_merge) == self.max_type_of_measurements:
                    # all types of measurements are found, break
                    break

        if len(messages_to_merge) > 1:
            merged_msg, merged_msg_types = self.merge_msgs(messages_to_merge)
            # publish the merged message
            self.sync_meas_pub.publish(merged_msg)
            # print in green to indicate that a merged msg with multiple measurements  is published
            # also include types of measurements from merged_msg_types
            self.get_logger().info(
                "\033[92mMeasurements (multiple types: {}) published\033[0m".format(
                    merged_msg_types),
                throttle_duration_sec=2)
            # print in green if all types of measurements are found
            if len(messages_to_merge) == self.max_type_of_measurements:
                self.get_logger().info(
                    "\033[92m+++GOOD: All types of measurements are found!\033[0m",
                    throttle_duration_sec=2)
        else:
            # print in yellow to indicate that a merged msg with single measurement is published
            msg_type = ""
            if messages_to_merge[0].cuboid_factors != []:
                msg_type = "cuboid"
            elif messages_to_merge[0].cylinder_factors != []:
                msg_type = "cylinder"
            elif messages_to_merge[0].ellipsoid_factors != []:
                msg_type = "ellipsoid"
            # print in yellow to indicate that a merged msg with single measurement is published
            self.get_logger().info(
                "\033[93mMeasurements (single type: {}) published\033[0m".format(
                    msg_type),
                throttle_duration_sec=2)
            # publish the single measurement message
            self.sync_meas_pub.publish(messages_to_merge[0])

        # return the ids of the messages to remove
        return ids_to_remove

    def merge_msgs(self, messages_to_merge):
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = messages_to_merge[0].header
        # fill in the odometry
        sync_msg.odometry = messages_to_merge[0].odometry

        # msg types = []
        merged_msg_types = ""
        for msg in messages_to_merge:
            # there should only be one type of measurement in each message
            if msg.cuboid_factors != []:
                # sanity check
                if "cuboid" in merged_msg_types:
                    # print in red to indicate duplicate cuboid measurements corresponding to the same time stamp
                    self.get_logger().error(
                        "\033[91mERROR: Duplicate cuboid measurements corresponding to the same time stamp!\033[0m",
                        throttle_duration_sec=1)
                sync_msg.cuboid_factors = msg.cuboid_factors
                merged_msg_types += "cuboid "
            elif msg.cylinder_factors != []:
                # sanity check
                if "cylinder" in merged_msg_types:
                    # print in red to indicate duplicate cylinder measurements corresponding to the same time stamp
                    self.get_logger().error(
                        "\033[91mERROR: Duplicate cylinder measurements corresponding to the same time stamp!\033[0m",
                        throttle_duration_sec=1)

                sync_msg.cylinder_factors = msg.cylinder_factors
                # iterate through cylinder_factors set default semantic label for cylinder is -1
                for cylinder in sync_msg.cylinder_factors:
                    cylinder.semantic_label = -1

                merged_msg_types += "cylinder "
            elif msg.centroid_factors != []:
                # sanity check
                if "centroid" in merged_msg_types:
                    # print in red to indicate duplicate centroid measurements corresponding to the same time stamp
                    self.get_logger().warn(
                        "\033[91mERROR: Duplicate centroid measurements corresponding to the same time stamp!\033[0m",
                        throttle_duration_sec=2)
                sync_msg.ellipsoid_factors = msg.centroid_factors
                # iterate through centroid_factors set default semantic label for centroid is -1
                for centroid in sync_msg.ellipsoid_factors:
                    if centroid.semantic_label == 0:
                        centroid.semantic_label = 1
                        self.get_logger().warn(
                            "\033[93mWarning: centroid semantic label is 0, meaning it is not set, set to 1\033[0m",
                            throttle_duration_sec=2)
                    else:
                        break
                merged_msg_types += "centroid "
            else:
                self.get_logger().error(
                    "\033[91mError: all factors are empty!\033[0m",
                    throttle_duration_sec=1)

        return sync_msg, merged_msg_types


def main(args=None):
    rclpy.init(args=args)
    node = MergeSyncedMesurements()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print("Node Killed")


if __name__ == "__main__":
    main()
