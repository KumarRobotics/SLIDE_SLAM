#!/usr/bin/env python3

import os
import sys
import time
import yaml
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_py
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from utils_outdoor import (
    cluster,
    publish_cylinder_cloud,
    publish_ground_cloud,
    transform_publish_pc,
    send_tfs,
    make_fields,
    threshold_by_range,
    calc_dist_to_ground,
)
from cuboid_utils_outdoor import (
    fit_cuboid,
    publish_cuboid_markers,
    cuboid_detection,
    generate_publish_instance_cloud,
)
from object_tracker_utils import publish_markers, track_objects_final


def _pointcloud2_to_structured(msg):
    """Read a PointCloud2 message into a structured numpy ndarray.

    Replaces ros_numpy.numpify which is unavailable in ROS2 Jazzy.
    """
    field_names = [f.name for f in msg.fields]
    raw = list(pc2_py.read_points(msg, field_names=field_names, skip_nans=False))
    if len(raw) == 0:
        return np.zeros((0,), dtype=[(n, np.float32) for n in field_names])
    arr = np.array(raw)
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

        ################################## IMPORTANT PARAMS ##################################
        self.declare_parameter("run_kitti", False)
        self.declare_parameter("ransac_n_points", 50)
        self.declare_parameter("ground_median_increment", 0.3)
        self.declare_parameter("valid_range_threshold", 40.0)
        self.declare_parameter("use_sim", False)
        self.declare_parameter("track_always_visualize", False)
        self.declare_parameter("time_to_initialize_cuboid", 3.0)
        self.declare_parameter("expected_segmentation_frequency", 2.0)
        self.declare_parameter("use_1st_layer_clustering", True)
        self.declare_parameter("use_2nd_layer_clustering", True)
        self.declare_parameter("min_samples_scan_1st_layer", 7)
        self.declare_parameter("epsilon_scan_1st_layer", 0.1)
        self.declare_parameter("epsilon_scan_2nd_layer", 0.5)
        self.declare_parameter("min_samples_scan_2nd_layer", 25)
        self.declare_parameter("fit_cuboid_dim_thresh", 0.5)
        self.declare_parameter("downsample_res", -1)
        self.declare_parameter("num_instance_point_lim", 10000)
        self.declare_parameter("time_to_delete_lost_track_cuboid", 45.0)
        self.declare_parameter("estimate_facing_dir_car", False)
        self.declare_parameter("cluster_and_fix_cuboid_orientation", True)
        self.declare_parameter("visualize_DBSCAN_results", False)
        self.declare_parameter("output_dir_to_save_figs", "./")
        self.declare_parameter("pc_width", 1024)
        self.declare_parameter("pc_height", 64)
        self.declare_parameter("pc_point_step", 16)

        self.run_kitti = self.get_parameter("run_kitti").value

        share_dir = get_package_share_directory('scan2shape_launch')
        if self.run_kitti:
            class_info_yaml_path = os.path.join(
                share_dir, 'config', 'process_cloud_node_outdoor_kitti_class_info.yaml')
        else:
            class_info_yaml_path = os.path.join(
                share_dir, 'config', 'process_cloud_node_outdoor_class_info.yaml')

        with open(class_info_yaml_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        # Creating dictionaries for easy access to class data
        self.cls_label_to_name = {
            self.cls_data_all[cls_name]['id']: cls_name for cls_name in self.cls_data_all.keys()}
        self.cls_name_to_label = {
            cls_name: self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys()}
        self.cls_labels_for_cuboid = [self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys(
        ) if self.cls_data_all[cls_name]['model'] == "cuboid"]
        self.cls_labels_for_cylinder = [self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys(
        ) if self.cls_data_all[cls_name]['model'] == "cylinder"]

        self.cuboid_length_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['length_cutoff']) for cls_label in self.cls_labels_for_cuboid}
        self.cuboid_width_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['width_cutoff']) for cls_label in self.cls_labels_for_cuboid}
        self.cuboid_height_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['height_cutoff']) for cls_label in self.cls_labels_for_cuboid}

        self.clustering_params_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['clustering_params']) for cls_label in self.cls_labels_for_cuboid}

        self.cuboid_assignment_threshold_per_cls_label = {
            cls_label: self.cls_data_all[self.cls_label_to_name[cls_label]]['track_assignment_threshold'] for cls_label in self.cls_labels_for_cuboid}

        # Point cloud fields for creating PointCloud2 messages
        self.pc_fields_ = make_fields()

        # PARAMETERS
        # Consult the params file for more information on each parameter
        self.ransac_n_points = self.get_parameter("ransac_n_points").value
        if self.run_kitti:
            self.get_logger().warn("Running KITTI dataset benchmark code!")
        self.ground_median_increment = self.get_parameter("ground_median_increment").value
        self.valid_range_threshold = self.get_parameter("valid_range_threshold").value
        self.use_sim = self.get_parameter("use_sim").value
        self.track_always_visualize = self.get_parameter("track_always_visualize").value
        time_to_initialize_cuboid = self.get_parameter("time_to_initialize_cuboid").value
        self.expected_segmentation_frequency = self.get_parameter("expected_segmentation_frequency").value
        self.use_2d_1st_layer = self.get_parameter("use_1st_layer_clustering").value
        self.use_2d_2nd_layer = self.get_parameter("use_2nd_layer_clustering").value
        self.min_samples_scan_1st_layer = self.get_parameter("min_samples_scan_1st_layer").value
        self.epsilon_scan_1st_layer = self.get_parameter("epsilon_scan_1st_layer").value
        self.epsilon_scan_2nd_layer = self.get_parameter("epsilon_scan_2nd_layer").value
        self.min_samples_scan_2nd_layer = self.get_parameter("min_samples_scan_2nd_layer").value
        self.fit_cuboid_dim_thresh = self.get_parameter("fit_cuboid_dim_thresh").value
        self.downsample_res = self.get_parameter("downsample_res").value
        self.num_instance_point_lim = self.get_parameter("num_instance_point_lim").value

        self.cuboid_track_age_threshold_per_cls_label = {
            cls_label: self.cls_data_all[self.cls_label_to_name[cls_label]]['track_age_threshold'] * self.expected_segmentation_frequency for cls_label in self.cls_labels_for_cuboid}

        if self.track_always_visualize == True:
            time_to_delete_lost_track_cuboid = np.inf
        else:
            time_to_delete_lost_track_cuboid = self.get_parameter("time_to_delete_lost_track_cuboid").value

        self.num_lost_track_times_thresh = self.expected_segmentation_frequency * \
            time_to_delete_lost_track_cuboid

        self.estimate_facing_dir_car = self.get_parameter("estimate_facing_dir_car").value
        self.cluster_and_fix_cuboid_orientation = self.get_parameter("cluster_and_fix_cuboid_orientation").value

        # save figures of object clustering results from DBSCAN
        self.visualize = self.get_parameter("visualize_DBSCAN_results").value
        self.output_dir = self.get_parameter("output_dir_to_save_figs").value

        self.save_fig_idx = 0
        self.save_fig_counter = 0

        # input point cloud parameters
        self.pc_width = self.get_parameter("pc_width").value
        self.pc_height = self.get_parameter("pc_height").value
        self.pc_point_step = self.get_parameter("pc_point_step").value

        seg_pc_namespace = "/os_node"
        odom_topic = "/Odometry"

        # CONTAINERS
        self.all_objects = []
        self.all_tracks = []
        self.raw_cloud_dict = {}
        self.processed_scan_idx = -1

        # PUBLIHSERS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.segmented_pc_pub = self.create_publisher(
            PointCloud2, "filtered_semantic_segmentation", 1)
        self.cuboid_center_marker_pub = self.create_publisher(
            MarkerArray, "cuboid_centers", 1)
        # TODO(ankit): Check and remove covariance markers if not needed
        self.cuboid_center_cov_pub = self.create_publisher(
            MarkerArray, "cuboid_centers_covariance", 1)
        self.instance_cloud_pub = self.create_publisher(
            PointCloud2, "pc_instance_segmentation_accumulated", 1)
        # TODO(ankit): Currently "car_cuboids" is used for all cuboid objects. Carefully check and change this later
        self.cuboid_marker_pub = self.create_publisher(
            MarkerArray, "car_cuboids", 5)
        self.cuboid_marker_body_pub = self.create_publisher(
            MarkerArray, "car_cuboids_body", 5)
        self.cylinder_cloud_pub = self.create_publisher(
            PointCloud2, "tree_cloud", 1)
        self.ground_cloud_pub = self.create_publisher(
            PointCloud2, "ground_cloud", 1)
        # TODO(ankit): Check if this is needed
        self.odom_pub = self.create_publisher(
            Odometry, "/quadrotor/lidar_odom", 100)

        # defining frame ids
        if self.use_sim == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "odom"
            self.faster_lio_world_frame = "camera_init"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "lidar"
        else:
            self.range_image_frame = "quadrotor"
            self.undistorted_cloud_frame = "quadrotor"
            self.reference_frame = "quadrotor/odom"

        # SUBSCRIBERS
        if self.use_sim == False:
            self.get_logger().info("Running real-world experiments...")
            time.sleep(1)

            self.segmented_pc_sub = self.create_subscription(
                PointCloud2, seg_pc_namespace + "/segmented_point_cloud_no_destagger",
                self.segmented_pc_cb, 10)
            self.odom_sub = self.create_subscription(
                Odometry, odom_topic, self.odom_callback, 100)

        else:
            self.get_logger().warn(
                "Running simulation experiments. This mode is still under development and is not fully tested. "
                "Switch the self.use_sim flag to False to run real-world experiments which work correctly.")
            time.sleep(10)
            # we need to sync point cloud with odometry in order to be able to find the transform precisely
            self.sim_segmentation_sub = message_filters.Subscriber(
                self, PointCloud2, "/quadrotor/fake_lidar/car_cloud")
            self.odom_sub = message_filters.Subscriber(
                self, Odometry, "/quadrotor/odom")
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.sim_segmentation_sub, self.odom_sub], 20, 0.01)
            ts.registerCallback(self.sim_segmented_synced_pc_cb)

    def sim_segmented_synced_pc_cb(self, car_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(car_cloud_msg)

    def segmented_pc_cb(self, seg_cloud_msg):
        self.segmented_synced_pc_cb(seg_cloud_msg)

    def segmented_synced_pc_cb(self, segmented_cloud_msg):
        self.get_logger().info(
            "Segmented point cloud received. Executing callback...",
            throttle_duration_sec=7)

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
            intensities = np.zeros_like(x_coords)
            # NOTE: Hardcoding car label for simulated experiments
            intensities[x_coords != 0] = 5
        else:
            intensities = (segmented_pc['intensity']).flatten()

        pc_xyzi = np.zeros((x_coords.shape[0], 4))
        pc_xyzi[:, 0] = x_coords
        pc_xyzi[:, 1] = y_coords
        pc_xyzi[:, 2] = z_coords
        pc_xyzi[:, 3] = intensities

        # threshold by range. Remove points that are farther away than self.valid_range_threshold
        valid_indices = threshold_by_range(self.valid_range_threshold, pc_xyzi)

        if np.sum(valid_indices) == 0:
            self.get_logger().warn(
                "No valid points found after range thresholding. Skipping this scan!!! Make sure the self.valid_range_threshold is set correctly.")
            return

        # apply range thresholding
        pc_xyzi_thresholded = pc_xyzi[valid_indices, :]

        indices_mask = np.cumsum(valid_indices) - 1

        # Use odometry and transform point cloud to world frame and then publish it
        points_world_xyzi, points_body_xyzi = transform_publish_pc(self,
                                                                   current_raw_timestamp, pc_xyzi_thresholded)

        if points_world_xyzi is None or points_body_xyzi is None:
            self.get_logger().warn(
                "Failed to transform point cloud to world frame. Skipping this scan!!!")
            self.get_logger().warn(
                "This may be caused due to transform_publish_pc function not performing correctly. Check the above warning messages.")
            self.get_logger().warn(
                "If you are replying bags, try setting use_sim_time to true and add --clock flag to ros2 bag play")
            self.get_logger().warn(
                "It may also be caused by excessive CPU load, play bag with slower rate")
            return
        else:
            # Intensity channel is filled with the class label for each segmented point cloud

            # Peforming Ground Plane Fitting (if ground is segmented as a class)
            if "ground" in self.cls_name_to_label.keys():
                points_world_xyzi_ground = points_world_xyzi[points_world_xyzi[:, 3]
                                                             == self.cls_name_to_label["ground"], :]

                if points_world_xyzi_ground.shape[0] > self.ransac_n_points:
                    ground_pcd = o3d.geometry.PointCloud()
                    ground_pcd.points = o3d.utility.Vector3dVector(
                        points_world_xyzi_ground[:, :3])
                    plane_eq, _ = ground_pcd.segment_plane(distance_threshold=0.1,
                                                           ransac_n=self.ransac_n_points, num_iterations=100)
                    [a, b, c, d] = plane_eq
                    self.ground_plane_coeff = np.array([a, b, c, d])
                    self.get_logger().info(
                        f"Ground plane coefficients properly updated: {self.ground_plane_coeff} ...",
                        throttle_duration_sec=7)
                else:
                    self.get_logger().warn(
                        f"Not enough ground points({points_world_xyzi_ground.shape[0]}) found to fit ground plane "
                        f"(atleast {self.ransac_n_points} are needed). Setting ground plane coefficients to simulate flat ground everywhere.",
                        throttle_duration_sec=5)
                    self.get_logger().warn(
                        "If this warning persists, tune the self.ransac_n_points parameter according to the number of ground points.",
                        throttle_duration_sec=5)
                    self.ground_plane_coeff = np.array([0, 0, 1, 0])

                # Preparing points for cylinder fitting
                points_body_xyzi_organized = points_body_xyzi[indices_mask, :]
                points_body_xyzi_organized[valid_indices == 0, :] = np.nan

                points_body_xyzi_cylinder_masked = points_body_xyzi_organized.copy()
                points_body_xyzi_ground_masked = points_body_xyzi_organized.copy()
                non_cylinder_idx = None
                points_body_xyzi_ground_masked[points_body_xyzi_organized[:, 3]
                                               != self.cls_name_to_label["ground"], :] = np.nan

                for cur_class_label in self.cls_labels_for_cylinder:
                    if non_cylinder_idx is None:
                        non_cylinder_idx = points_body_xyzi_organized[:,
                                                                      3] != cur_class_label
                    else:
                        non_cylinder_idx = np.logical_and(
                            non_cylinder_idx, points_body_xyzi_organized[:, 3] != cur_class_label)

                points_body_xyzi_cylinder_masked[non_cylinder_idx, :] = np.nan

                # check if points_body_xyzi_cylinder_masked is a flattened array (num_points x 4) or an organized point cloud (height x width x 4)
                if len(points_body_xyzi_cylinder_masked.shape) == 3:
                    pc_msg_cylinder = publish_cylinder_cloud(self.pc_fields_, points_body_xyzi_cylinder_masked.reshape(
                        (self.pc_height, self.pc_width, 4)), current_raw_timestamp, self.range_image_frame, pc_width=self.pc_width, pc_height=self.pc_height, pc_point_step=self.pc_point_step)
                    self.cylinder_cloud_pub.publish(pc_msg_cylinder)

                    pc_msg_ground = publish_ground_cloud(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
                        (self.pc_height, self.pc_width, 4)), current_raw_timestamp, self.range_image_frame, pc_width=self.pc_width, pc_height=self.pc_height, pc_point_step=self.pc_point_step)
                    self.ground_cloud_pub.publish(pc_msg_ground)
                elif len(points_body_xyzi_cylinder_masked.shape) == 2:
                    pc_msg_cylinder = publish_cylinder_cloud(self.pc_fields_, points_body_xyzi_cylinder_masked, current_raw_timestamp,
                                                             self.range_image_frame, pc_width=points_body_xyzi_cylinder_masked.shape[0], pc_height=1, pc_point_step=self.pc_point_step)
                    self.cylinder_cloud_pub.publish(pc_msg_cylinder)

                    pc_msg_ground = publish_ground_cloud(self.pc_fields_, points_body_xyzi_ground_masked, current_raw_timestamp,
                                                          self.range_image_frame, pc_width=points_body_xyzi_ground_masked.shape[0], pc_height=1, pc_point_step=self.pc_point_step)
                    self.ground_cloud_pub.publish(pc_msg_ground)
                else:
                    raise Exception("points_body_xyzi_cylinder_masked is neither a flattened array nor an organized point cloud. Check the code for errors.")

            else:
                # setting ground plane coeff to simulate flat ground everywhere.
                self.ground_plane_coeff = np.array([0, 0, 1, 0])
                self.get_logger().warn(
                    f"No ground class found in the segmented point cloud. Skipping cylinder model fitting & "
                    f"setting ground plane coefficients to simulate flat ground everywhere ({self.ground_plane_coeff}).",
                    throttle_duration_sec=5)

            # Performing Cuboid Fitting and Tracking
            for cur_class_label in self.cls_labels_for_cuboid:
                points_world_xyzi_cuboid = points_world_xyzi[points_world_xyzi[:, 3]
                                                             == cur_class_label, :]

                if points_world_xyzi_cuboid.shape[0] == 0:
                    self.get_logger().warn(
                        f"No points found to fit cuboid for class {self.cls_label_to_name[cur_class_label]}. Skipping this class for current scan.",
                        throttle_duration_sec=5)
                    continue

                # filter out points that are too close to the ground
                dist_to_ground = calc_dist_to_ground(
                    self, points_world_xyzi_cuboid)
                points_cuboid_valid = points_world_xyzi_cuboid[dist_to_ground >
                                                               self.ground_median_increment, :]

                if points_cuboid_valid.shape[0] == 0:
                    self.get_logger().warn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} points found above ground. Skipping this class for current scan.")
                    self.get_logger().warn(
                        "This is likely due to all points being too close to the ground or an error in the calc_dist_to_ground function. Also try tuning the self.ground_median_increment parameter.")
                    continue

                # Perform two layer clustering to remove noisy points
                labels = cluster(self, points_cuboid_valid, epsilon=self.epsilon_scan_1st_layer,
                                 min_samples=self.min_samples_scan_1st_layer, use_2d=self.use_2d_1st_layer)

                points_cuboid_valid = points_cuboid_valid[labels != -1, :]

                if points_cuboid_valid.shape[0] == 0:
                    self.get_logger().warn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} clusters found after FIRST LAYER of clustering. Skipping this class for current scan. Tuning of parameters may be needed.")
                    continue

                labels = cluster(self, points_cuboid_valid, epsilon=self.clustering_params_per_cls_label[cur_class_label][0],
                                 min_samples=self.clustering_params_per_cls_label[cur_class_label][1], use_2d=self.use_2d_2nd_layer)

                # extract final valid points and their instance labels
                points_cuboid_valid = points_cuboid_valid[labels != -1, :]
                labels = labels[labels != -1]

                if points_cuboid_valid.shape[0] == 0:
                    self.get_logger().warn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} clusters found after SECOND LAYER of clustering. Skipping this class for current scan. Tuning of parameters may be needed.")
                    continue

                # Fit cuboids to the semantic instances to start the tracking process
                raw_points_xyz = points_cuboid_valid[:, :3]
                self.get_logger().info(
                    f"Fitting initial cuboids for {self.cls_label_to_name[cur_class_label]} instances to start tracking...",
                    throttle_duration_sec=7)
                xcs, ycs, lengths, widths, raw_points = fit_cuboid(
                    self.fit_cuboid_dim_thresh, points_cuboid_valid[:, :3], labels)

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                if cur_objects.shape[0] == 0:
                    self.get_logger().warn(
                        f"No valid initial cuboid to start tracking for {self.cls_label_to_name[cur_class_label]}. Skipping this class for current scan.",
                        throttle_duration_sec=5)
                    self.get_logger().warn(
                        "If this warning persists, try tuning the self.fit_cuboid_dim_thresh parameter.",
                        throttle_duration_sec=5)
                    continue

                # Tracking the current class instance across scans
                self.get_logger().info(
                    f"Tracking {self.cls_label_to_name[cur_class_label]} instances across scans...",
                    throttle_duration_sec=7)
                self.all_objects, self.all_tracks = track_objects_final(
                    self, cur_class_label, cur_objects, raw_points, self.all_objects, self.all_tracks, self.processed_scan_idx, self.downsample_res, self.num_instance_point_lim)

                # Publish track centroids as cylinders for visualization
                publish_markers(
                    self, self.all_tracks, cur_cls_name=self.cls_label_to_name[cur_class_label])

            # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
            if self.track_always_visualize == False:
                idx_to_delete = []
                for idx, track in enumerate(self.all_tracks):
                    num_lost_track_times = self.processed_scan_idx - track.last_update_scan_idx
                    if num_lost_track_times > self.num_lost_track_times_thresh:
                        idx_to_delete.append(idx)

                # delete in descending order
                for idx in sorted(idx_to_delete, reverse=True):
                    del self.all_tracks[idx]
                    del self.all_objects[idx]

            extracted_instances_xyzl = generate_publish_instance_cloud(self,
                                                                       current_raw_timestamp)

            if extracted_instances_xyzl is not None:
                cuboids = cuboid_detection(
                    self, extracted_instances_xyzl, current_raw_timestamp)
                if len(cuboids) > 0:
                    publish_cuboid_markers(
                        self, cuboids, current_raw_timestamp)
                else:
                    self.get_logger().warn(
                        "No valid cuboids found from the accumulated instance point cloud. Tune the cuboid dimension thresholds and other parameters in cuboid_detection function if this warning persists.",
                        throttle_duration_sec=5)

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
