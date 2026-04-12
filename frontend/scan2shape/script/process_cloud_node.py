#!/usr/bin/env python3

import time
import copy
import os
import sys
import yaml
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_py
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from sloam_msgs.msg import SyncPcOdom

from utils import (
    transform_publish_pc,
    send_tfs,
    make_fields,
    threshold_by_range,
)
from cuboid_utils_indoor import (
    fit_cuboid_indoor,
    cuboid_detection_indoor,
    generate_publish_instance_cloud_indoor,
    cluster_indoor,
    publish_cuboid_and_range_bearing_measurements_final,
)
from object_tracker_utils import track_objects_indoor, publish_markers


def _pointcloud2_to_structured(msg):
    """Read a PointCloud2 message into a structured numpy ndarray.

    Replaces ros_numpy.numpify which is unavailable in ROS2 Jazzy.
    """
    field_names = [f.name for f in msg.fields]
    raw = list(pc2_py.read_points(msg, field_names=field_names, skip_nans=False))
    if len(raw) == 0:
        return np.zeros((0,), dtype=[(n, np.float32) for n in field_names])
    arr = np.array(raw)
    # If the underlying call returned a structured array, return it; otherwise
    # build one ourselves.
    if arr.dtype.names is not None:
        return arr
    dtype = [(n, np.float32) for n in field_names]
    out = np.empty(arr.shape[0], dtype=dtype)
    for i, n in enumerate(field_names):
        out[n] = arr[:, i].astype(np.float32)
    return out


class ProcessCloudNode(Node):
    def __init__(self):
        super().__init__("process_cloud_node")

        # DO NOT CHNAGE THIS TO ANYTHING OTHER THAN /ODOM. This is automatically remapped in the launch file
        self.odom_topic = "/odom"

        # detect_no_seg: if true, meaning only object detection is done, no instance segmentation. This scenario uses YOLO-WORLD
        self.declare_parameter("detect_no_seg", False)
        self.detect_no_seg = self.get_parameter("detect_no_seg").value

        share_dir = get_package_share_directory('scan2shape_launch')
        if self.detect_no_seg:
            self.cls_config_path = os.path.join(
                share_dir, 'config', 'process_cloud_node_indoor_open_vocab_cls_info.yaml')
        else:
            self.cls_config_path = os.path.join(
                share_dir, 'config', 'process_cloud_node_indoor_cls_info.yaml')

        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        self.cls = {cls_name: self.cls_data_all[cls_name]["id"]
                    for cls_name in self.cls_data_all.keys()}

        self.length_cutoffs = {cls_name: tuple(
            self.cls_data_all[cls_name]["length_cutoff"]) for cls_name in self.cls_data_all.keys()}
        self.height_cutoffs = {cls_name: tuple(
            self.cls_data_all[cls_name]["height_cutoff"]) for cls_name in self.cls_data_all.keys()}

        self.class_color = {cls_name: tuple(
            self.cls_data_all[cls_name]["color"]) for cls_name in self.cls_data_all.keys()}

        self.class_model_path = {
            cls_name: self.cls_data_all[cls_name]["mesh_model_path"] for cls_name in self.cls_data_all.keys()}
        self.class_model_scale = {
            cls_name: self.cls_data_all[cls_name]["mesh_model_scale"] for cls_name in self.cls_data_all.keys()}

        self.class_assignment_thresh = {
            cls_name: self.cls_data_all[cls_name]["class_assignment_thresh"] for cls_name in self.cls_data_all.keys()}

        self.color_by_floors = False  # For debugging only, leave it as False
        self.floor_height_thresh = {"floor_1": (
            0.0, 2.5), "floor_2": (3.0, 6.0), "floor_3": (6.0, 15.0)}
        self.floor_color = {"floor_1": (1.0, 0.0, 0.0), "floor_2": (
            0.0, 1.0, 0.0), "floor_3": (0.0, 0.0, 1.0)}
        # this is only for floor-wise object clustering clustering
        self.epsilon_scan = 2.5
        # this is only for floor clustering
        self.min_samples_scan = 1

        ################################## IMPORTANT PARAMS ##################################
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("desired_acc_obj_pub_rate", 1.0)
        self.declare_parameter("expected_segmentation_frequency", 2.0)
        self.declare_parameter("use_sim", False)
        self.declare_parameter("visualize_DBSCAN_results", False)
        self.declare_parameter("valid_range_threshold", 40.0)
        self.declare_parameter("fit_cuboid_dim_thresh", 0.2)
        self.declare_parameter("depth_percentile_lower", 35)
        self.declare_parameter("depth_percentile_upper", 45)
        self.declare_parameter("time_to_initialize_cuboid", 0.75)
        self.declare_parameter("time_to_delete_lost_track_cuboid", 30)
        self.declare_parameter("downsample_res", -1)
        self.declare_parameter("num_instance_point_lim", 10000)
        self.declare_parameter("pc_width", 1024)
        self.declare_parameter("pc_height", 64)
        self.declare_parameter("pc_point_step", 16)

        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.desired_acc_obj_pub_rate = self.get_parameter("desired_acc_obj_pub_rate").value
        self.expected_segmentation_rate = self.get_parameter("expected_segmentation_frequency").value
        self.use_sim = self.get_parameter("use_sim").value
        self.visualize = self.get_parameter("visualize_DBSCAN_results").value
        self.valid_range_threshold = self.get_parameter("valid_range_threshold").value
        self.fit_cuboid_length_thresh = self.get_parameter("fit_cuboid_dim_thresh").value

        depth_percentile_lower = self.get_parameter("depth_percentile_lower").value
        depth_percentile_uppper = self.get_parameter("depth_percentile_upper").value
        self.depth_percentile = (depth_percentile_lower, depth_percentile_uppper)

        time_to_initialize_cuboid = self.get_parameter("time_to_initialize_cuboid").value
        self.tracker_age_thresh_lower = self.expected_segmentation_rate * \
            time_to_initialize_cuboid

        time_to_delete_lost_track_cuboid = self.get_parameter("time_to_delete_lost_track_cuboid").value
        self.num_lost_track_times_thresh = self.expected_segmentation_rate * \
            time_to_delete_lost_track_cuboid

        self.downsample_res = self.get_parameter("downsample_res").value
        self.num_instance_point_lim = self.get_parameter("num_instance_point_lim").value
        self.pc_width = self.get_parameter("pc_width").value
        self.pc_height = self.get_parameter("pc_height").value
        self.pc_point_step = self.get_parameter("pc_point_step").value
        ################################## IMPORTANT PARAMS ENDS ##################################

        # CONTAINERS and VARIABLES
        # IMPORTANT: THESE TWO NEED TO BE UPDATED SIMULTANEOUSLY, THEIR LENGTH SHOULD MATCH!
        # the x, y, length and width of each object
        self.all_objects = []
        # the ObjectTrack instance of each object
        self.all_tracks = []
        self.save_fig_idx = 0
        self.save_fig_counter = 0
        self.processed_scan_idx = -1
        self.prev_acc_obj_pub_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.pc_fields_ = make_fields()

        # PUBLISHERS
        self.segmented_pc_pub = self.create_publisher(
            PointCloud2, "filtered_semantic_segmentation", 1)
        self.cuboid_center_marker_pub = self.create_publisher(
            MarkerArray, "cuboid_centers", 1)
        # TODO(ankit): Check and remove covariance markers if not needed
        self.cuboid_center_cov_pub = self.create_publisher(
            MarkerArray, "cuboid_centers_covariance", 1)
        self.instance_cloud_pub = self.create_publisher(
            PointCloud2, "pc_instance_segmentation_accumulated", 1)
        # TODO(ankit): Current "chair_cuboids" topic is used for publishing all object models. Change this to a more generic name
        self.cuboid_marker_pub = self.create_publisher(
            MarkerArray, "chair_cuboids", 5)
        self.cuboid_marker_body_pub = self.create_publisher(
            MarkerArray, "chair_cuboids_body", 5)
        self.tree_cloud_pub = self.create_publisher(
            PointCloud2, "tree_cloud", 1)
        self.ground_cloud_pub = self.create_publisher(
            PointCloud2, "ground_cloud", 1)
        # TODO(ankit): Check if this is needed
        self.odom_pub = self.create_publisher(
            Odometry, "quadrotor/lidar_odom", 100)

        # frame ids
        if self.use_sim == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "dragonfly67/odom"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "camera"
        else:
            self.range_image_frame = "body"
            self.reference_frame = "world"
            self.undistorted_cloud_frame = "camera"

        # subscriber and publisher
        if self.use_sim == False:
            self.get_logger().info("Running real-world experiments...")
            time.sleep(1)

            self.segmented_pc_sub = self.create_subscription(
                SyncPcOdom, "sem_detection/sync_pc_odom", self.segmented_pc_cb, 1)

            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, 100)

        else:
            self.get_logger().warn(
                "Running simulation experiments. This mode is still under development and is not fully tested. "
                "Switch the self.use_sim flag to False to run real-world experiments which work correctly.")
            time.sleep(10)
            self.segmented_pc_sub = self.create_subscription(
                SyncPcOdom, "sem_detection/sync_pc_odom", self.segmented_pc_cb, 1)
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, 100)

    def sim_segmented_synced_pc_cb(self, chair_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(chair_cloud_msg, None)

    def segmented_pc_cb(self, seg_cloud_msg):
        # Now, this seg_cloud_msg has three parts,
        # Header, cloud, odom (synced with cloud)
        self.odom_from_cloud_msg = seg_cloud_msg.odom
        self.segmented_synced_pc_cb(seg_cloud_msg.cloud, None)

    def segmented_synced_pc_cb(self, segmented_cloud_msg, undistorted_cloud_msg):

        self.processed_scan_idx += 1
        current_raw_timestamp = segmented_cloud_msg.header.stamp
        # create pc from the undistorted_cloud
        segmented_pc = _pointcloud2_to_structured(segmented_cloud_msg)
        # remove nan values
        x_coords = np.nan_to_num(
            segmented_pc['x'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        y_coords = np.nan_to_num(
            segmented_pc['y'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        z_coords = np.nan_to_num(
            segmented_pc['z'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        # fill in the intensity values that represent the class labels
        if self.use_sim:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()
        else:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()

        pc_xyzi_id_conf = np.zeros((x_coords.shape[0], 6))
        pc_xyzi_id_conf[:, 0] = x_coords
        pc_xyzi_id_conf[:, 1] = y_coords
        pc_xyzi_id_conf[:, 2] = z_coords
        pc_xyzi_id_conf[:, 3] = intensities
        pc_xyzi_id_conf[:, 4] = ids
        pc_xyzi_id_conf[:, 5] = confs

        # threshold by range. Remove points that are farther away than self.valid_range_threshold
        valid_indices = threshold_by_range(
            self.valid_range_threshold, pc_xyzi_id_conf)

        if np.sum(valid_indices) == 0:
            self.get_logger().warn(
                "No valid points found after range thresholding. Skipping this scan!!! Make sure the self.valid_range_threshold is set correctly.")
            return

        # apply thresholding
        pc_xyzi_id_conf_thresholded = pc_xyzi_id_conf[valid_indices, :]

        # Use odometry and transform point cloud to world frame and then publish it
        points_world_xyzi_id_conf_depth, points_body_xyzi_id_conf = transform_publish_pc(self,
                                                                                         current_raw_timestamp, pc_xyzi_id_conf_thresholded)

        if points_world_xyzi_id_conf_depth is None or points_body_xyzi_id_conf is None:
            self.get_logger().warn(
                "Failed to transform point cloud to world frame. Skipping this scan!!!")
            self.get_logger().warn(
                "This may be caused due to transform_publish_pc function not performing correctly. Check the above warning messages.")
            self.get_logger().warn(
                "If you are replying bags, try setting use_sim_time to true and add --clock flag to ros2 bag play")
            self.get_logger().warn(
                "It may also be caused by excessive CPU load, play bag with slower rate")

        else:
            for cur_object_class in self.cls.keys():
                # skip background if present
                if cur_object_class == "background":
                    continue
                cur_class_label = self.cls[cur_object_class]
                pc_world_cur_class = points_world_xyzi_id_conf_depth[
                    points_world_xyzi_id_conf_depth[:, 3] == cur_class_label, :]

                # find object instances
                if pc_world_cur_class.shape[0] == 0:
                    continue

                # Fit cuboids to the semantic instances to start the tracking process
                xcs, ycs, lengths, widths, raw_points = fit_cuboid_indoor(
                    self.fit_cuboid_length_thresh, pc_world_cur_class, self.depth_percentile, self.confidence_threshold)

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                if cur_objects.shape[0] != 0:
                    self.all_objects, self.all_tracks = track_objects_indoor(self, cur_class_label, cur_object_class,
                                                                             cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), self.downsample_res, self.num_instance_point_lim)

                    # TODO(ankit): Maybe use age_threshold parameter here
                    publish_markers(self, self.all_tracks, cur_cls_name=cur_object_class,
                                    age_threshold=self.tracker_age_thresh_lower)
                else:
                    self.get_logger().warn(
                        "No valid objects found for object fitting",
                        throttle_duration_sec=7)

                # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
                idx_to_delete = []
                for idx, track in enumerate(self.all_tracks):
                    # if track.age > self.tracker_age_thresh_upper:
                    num_lost_track_times = self.processed_scan_idx - track.last_update_scan_idx
                    if num_lost_track_times > self.num_lost_track_times_thresh:
                        idx_to_delete.append(idx)

                # delete in descending order so that it does not get messed up
                for idx in sorted(idx_to_delete, reverse=True):
                    del self.all_tracks[idx]
                    del self.all_objects[idx]

            # Publishing segmented, tracked, and accumulated instance point cloud
            extracted_instances_xyzl, instance_global_ids = generate_publish_instance_cloud_indoor(self,
                                                                                                   current_raw_timestamp)

            if extracted_instances_xyzl is not None:
                cuboids, cuboid_clus_centroids = cuboid_detection_indoor(self,
                                                                         extracted_instances_xyzl, instance_global_ids)

                if len(cuboids) > 0:

                    now = self.get_clock().now()
                    if self.prev_acc_obj_pub_time is not None and \
                            (now - self.prev_acc_obj_pub_time) < Duration(seconds=1.0 / self.desired_acc_obj_pub_rate):
                        elapsed_sec = (now - self.prev_acc_obj_pub_time).nanoseconds * 1e-9
                        self.get_logger().warn(
                            "Time elapsed since last depth rgb callback is: " + str(elapsed_sec) +
                            " seconds. Skipping current depth image to get desired rate of " +
                            str(self.desired_acc_obj_pub_rate) + " Hz",
                            throttle_duration_sec=5)
                    else:
                        self.prev_acc_obj_pub_time = now

                        if self.color_by_floors == True:
                            cuboid_clus_labels = cluster_indoor(np.array(
                                cuboid_clus_centroids), self.epsilon_scan, self.min_samples_scan, use_2d=False)
                        else:
                            cuboid_clus_labels = None

                        publish_cuboid_and_range_bearing_measurements_final(self, copy.deepcopy(
                            cuboids), cuboid_clus_labels, current_raw_timestamp, self.odom_from_cloud_msg)

    def odom_callback(self, msg):
        send_tfs(self, msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
