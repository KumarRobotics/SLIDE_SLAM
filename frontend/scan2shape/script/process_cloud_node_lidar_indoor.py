#!/usr/bin/env python3
# title         :
# description   :
# author        :Xu Liu and Ankit Prabhu

# type: sensor_msgs/PointCloud2
# topic of interest: /os_node/llol_odom/sweep
# field of interest: header/stamp (secs and nsecs)
# in the segmentation result folder, the pcds are named by point_cloud_secs+nsecs.pcd
# goal: find the point cloud in /os_node/llol_odom/sweep topic that has matched timestamp of the pcds, and save them as world_frame_point_cloud_secs+nsecs.pcd

import time
import argparse
import copy
import glob
import os
import sys
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import label
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener, Buffer, TransformBroadcaster

# (ROS1 legacy note) This module is broken upstream — it imports several
# helpers from utils_outdoor / cuboid_utils_indoor that do not exist there
# even in the original source tree (publish_tree_cloud,
# publish_ground_cloud_and_fake_cubes,
# publish_cuboid_and_range_bearing_measurements{,_with_models,_by_floor*}).
# Those call sites are commented out below or guarded by feature flags so
# the module loads, but the imports themselves are kept disabled with
# try/except to allow this node to start. Fixing the missing helpers is
# outside the scope of the ROS2 port.
from utils_outdoor import (
    show_clusters,
    publish_ground_cloud,
    transform_publish_pc,
    send_tfs,
    publish_accumulated_cloud,
    make_fields,
    threshold_by_range,
    calc_dist_to_ground,
)
try:
    from utils_outdoor import publish_tree_cloud, publish_ground_cloud_and_fake_cubes  # type: ignore
except ImportError:
    publish_tree_cloud = None
    publish_ground_cloud_and_fake_cubes = None

from cuboid_utils_indoor import (
    cluster_indoor,
    cuboid_detection_indoor,
    generate_publish_instance_cloud_indoor,
)
try:
    from cuboid_utils_indoor import (  # type: ignore
        publish_cuboid_and_range_bearing_measurements,
        publish_cuboid_and_range_bearing_measurements_with_models,
        publish_cuboid_and_range_bearing_measurements_by_floor,
        publish_cuboid_and_range_bearing_measurements_by_floor_properly,
    )
except ImportError:
    publish_cuboid_and_range_bearing_measurements = None
    publish_cuboid_and_range_bearing_measurements_with_models = None
    publish_cuboid_and_range_bearing_measurements_by_floor = None
    publish_cuboid_and_range_bearing_measurements_by_floor_properly = None

from cuboid_utils_outdoor import fit_cuboid
from object_tracker_utils import track_objects, track_objects_indoor, publish_markers
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc2_py


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
        # # the label corresponds to ground class
        # self.ground_class_label = 2

        # # the label corresponds to vehicle class
        # self.car_class_label = 3 # 3 is for chairs

        # # the label corresponds to tree class
        # self.tree_trunk_class_label = 4 # tables

        self.cls_raw = {"chair":3, "table":4, "floor":2}
        # required if we want the lidar and camera labels to be the same for map merging
        # if they are already the same, you can set as the same as cls_raw
        self.cls = {"chair":1, "table":2}

        # Chair-> Actual Chairs = 0.5
        # Chair-> Sofas = 1.5
        self.class_assignment_thresh = {"chair": 0.5, "table":1.0}


        # # cutoff for filtering out the FINAL false positive cuboids (lower_limit, upper_limit)

        # Chair-> Actual Chairs Length = (0.25, 2.0)
        # Chair-> Sofas = (0.25, 5.0) 

        self.length_cutoffs = {"chair":(0.25, 5.0), "table":(0.75, 5.0)}
        self.height_cutoffs =  {"chair":(0.1, 4.0), "table":(0.001, 3.0)}       
        self.class_model_path = {"chair": "package://sloam/resource/chair.stl", "table": "package://sloam/resource/table2.stl"}
        self.class_color = {"chair": (1.0 , 0.0, 0.0), "table": (0.0 , 1.0, 0.0)}

        self.floor_rand_color = np.random.rand(20, 3)
        self.floor_color = {"floor_1": (1.0 , 0.0, 0.0), "floor_2": (0.0 , 1.0, 0.0), "floor_3": (0.0 , 0.0, 1.0)}
        # the label corresponds to light pole class
        # self.light_pole_class_label = 9 # we do not need this

        # the expected lidar frequency for calculating when to place or remove a track from object assignment
        self.expected_segmentation_frequency = 3 # in Hz
        
        # use simulator or not
        self.use_sim = False

        # save figures of car clustering results
        self.visualize = False
        
        # TUNABLE PARAMETERS
        # threshold for lidar segmentation range
        self.valid_range_threshold = 200
        # TODO currently we are using pano frame which is not really body centered
        
        # the height of considering a point as a ground point 
        # TODO use actual local ground patches, instead of all ground points in the scan
        self.ground_median_increment = 0.40

        # epsilon_scan: determines the radius of clustering when considering neighboring points as single cluster.
        # Higher value results in more bigger cluster but also nosier clusters. Best value for now is between 0.5-1.0
        # Use 1.0 if troubled with false positive detections in tree canopy

        # min_samples_scan : determines the minimum samples needed inside the epsilon radius of the cluster to be clustered together
        # Higher values result in more sparser clusters with less noise
        # Best value for now is 60

        # epsilon_scan_1st_layer and min_samples_scan_1st_layer are for throwing away noisy points, thus they should be small enough
        # epsilon_scan and min_samples_scan are for 2nd stage clustering which is used for fitting cuboids after noisy point removal

        self.min_samples_scan_1st_layer = 10
        self.epsilon_scan_1st_layer = 0.1
        self.epsilon_scan = 0.5 #0.5
        self.min_samples_scan = 60 #60
                 
        self.publish_with_models = False
        self.publish_with_floors = False
        # remove X and Y and cluster just by Z when it comes to floors
        self.publish_with_floors_proper = True

        # Cuboid length and width threshold used to start initial object tracking process
        self.fit_cuboid_length_thresh = 0.2 # same threshold for chairs and tables 
        self.fit_cuboid_width_thresh = 0.2 # same threshold for chairs and tables

        # visualize all car instances (keep track of all and never remove tracks), should be set as False unless you know what you are doing
        self.car_inst_visualize = False
        

        
        if self.car_inst_visualize:
            # Minimum amount of time required for a track to be published as a cuboid
            # Filters out dynamic objects and small flickering false positives
            # 10 for when you want to visualize all cuboids properly 
            self.tracker_age_thresh_lower = 10
        else:
            # Minimum time required to track a cuboid per lidar scan 
            time_to_initialize_cuboid = 1 #1.5 # in seconds
            self.tracker_age_thresh_lower = self.expected_segmentation_frequency * time_to_initialize_cuboid   

        # Lower will delete tracks faster and will cause new tracks to be initiated at the same location
        time_to_delete_lost_track_cuboid = 5 # 7 in seconds
        self.num_lost_track_times_thresh = self.expected_segmentation_frequency * time_to_delete_lost_track_cuboid # running 10hz:20 #20

        # Hungarian assignment cost to associate 2 cuboid centroids as one
        self.assignment_threshold = 1.0 #1.0
        # Resolution to perform voxel downsampling for instance point cloud accumulation
        self.downsample_res = 0.0 # if this is > 0, we will downsample the instance point cloud
        # only keep the most recent num_points_limit_per_instance for any instance
        self.num_instance_point_lim = 10000
        # run forestry data or not
        self.for_forestry_bags = False
        ################################## IMPORTANT PARAMS ENDS ##################################
        
        
        # Leave this as False, it will be automatically set according to callbacks
        self.use_faster_lio = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.pc_fields_ = make_fields()


        # IMPORTANT: THESE TWO NEED TO BE UPDATED SIMULTANEOUSLY, THEIR LENGTH SHOULD MATCH!
        # the x, y, length and width of each object
        self.all_objects = []
        # the ObjectTrack instance of each object
        self.all_tracks = []
        self.save_fig_idx = 0
        self.save_fig_counter = 0
        self.processed_scan_idx = -1
        self.last_stamp = 0
        self.raw_cloud_dict = {}
        self.thresh_by_range_time = np.array([])
        self.transform_pano_to_world_time = np.array([])
        self.clustering_time = np.array([])
        self.fit_cuboid_time = np.array([])
        self.track_objects_time = np.array([])
        self.deleting_tracks_time = np.array([])
        self.accumulated_cloud_time = np.array([])
        self.final_cuboid_detection_time = np.array([])


        self.segmented_pc_pub = self.create_publisher(
            PointCloud2, "process_cloud_node/filtered_semantic_segmentation", 1)
        self.car_convex_hull_pub = self.create_publisher(
            PointCloud2, "car_convex_hull", 100)

        self.cuboid_center_marker_pub = self.create_publisher(
            MarkerArray, "cuboid_centers", 1)
        self.cuboid_center_cov_pub = self.create_publisher(
            MarkerArray, "cuboid_centers_covariance", 1)

        self.instance_cloud_pub = self.create_publisher(
            PointCloud2, "pc_instance_segmentation_accumulated", 1)

        self.cuboid_marker_pub = self.create_publisher(
            MarkerArray, "chair_cuboids", 5)

        self.cuboid_marker_body_pub = self.create_publisher(
            MarkerArray, "chair_cuboids_body", 5)

        self.odom_received = False

        self.tree_cloud_pub = self.create_publisher(
            PointCloud2, "tree_cloud", 1)

        self.ground_cloud_pub = self.create_publisher(
            PointCloud2, "ground_cloud", 1)

        self.odom_pub = self.create_publisher(
            Odometry, "/quadrotor/lidar_odom", 100)


        # frame ids
        # TODO: need to figure out the correct way to find body odom transform!
        if self.use_sim  == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "odom"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "lidar" # pano for Chao LLOL
        else:
            self.range_image_frame = "quadrotor"
            self.undistorted_cloud_frame = "quadrotor" 
            self.reference_frame = "quadrotor/odom"

        # subscriber and publisher
        if self.use_sim == False:
            print("Running real-world experiments...")
            time.sleep(1)
            #  self.segmented_pc_sub = message_filters.Subscriber(
                #  "/os_node/segmented_point_cloud_no_destagger/throttled", PointCloud2)



            # self.segmented_pc_sub = message_filters.Subscriber(
            #     "/os_node/segmented_point_cloud_no_destagger", PointCloud2)
            # self.undistort_cloud_sub = message_filters.Subscriber(
            #     "/os_node/llol_odom/sweep", PointCloud2)

            # ts = message_filters.ApproximateTimeSynchronizer(
            #     [self.segmented_pc_sub, self.undistort_cloud_sub], 20)
            # ts = message_filters.ApproximateTimeSynchronizer(
            #     [self.segmented_pc_sub, self.undistort_cloud_sub], 100, 0.01)
            # ts.registerCallback(self.segmented_synced_pc_cb)
            
            self.segmented_pc_sub = self.create_subscription(
                PointCloud2, "/os_node/segmented_point_cloud_no_destagger",
                self.segmented_pc_cb, 10)

            self.odom_sub = self.create_subscription(
                Odometry, "/Odometry", self.odom_callback, 100)



            # synced version (commented out)
            # self.segmented_pc_sub = message_filters.Subscriber(self, PointCloud2,
            #     "/os_node/segmented_point_cloud_no_destagger")
            # self.odom_sub = message_filters.Subscriber(self, Odometry, "/Odometry")
            # ts = message_filters.ApproximateTimeSynchronizer(
            #     [self.segmented_pc_sub, self.odom_sub], 100, 0.025)
            # ts.registerCallback(self.segmented_pc_with_odom_cb)

            # self.cloud_frame_id = "quadrotor/base_link"
        else:
            print("Running simulation experiments...")
            time.sleep(1)
            # we need to sync point cloud with odometry in order to be able to find the transform precisely
            self.sim_segmentation_sub = message_filters.Subscriber(
                self, PointCloud2, "/quadrotor/fake_lidar/car_cloud")
            self.odom_sub = message_filters.Subscriber(
                self, Odometry, "/quadrotor/odom")
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.sim_segmentation_sub, self.odom_sub], 20, 0.01)
            ts.registerCallback(self.sim_segmented_synced_pc_cb)
        



        





    
    def cluster(self, xyzi, epsilon=1.25, min_samples=100, use_2d=True):
        # reduce to 2d
        if use_2d:
            cloud = xyzi[:, :2]
        else:
            cloud = xyzi[:, :3]
        # ref[https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html#sklearn.cluster.DBSCAN]
        object_clusters = DBSCAN(eps=epsilon, min_samples=min_samples, metric='euclidean', metric_params=None,
                                 algorithm='auto', leaf_size=30, p=None, n_jobs=1).fit(cloud)
        labels = object_clusters.labels_
        self.save_fig_counter += 1
        if self.visualize & (self.save_fig_counter % 10 == 0):
            self.save_fig_idx += 1
            show_clusters(cloud, labels, object_clusters,
                          self.save_fig_idx)
        return labels

    def sim_segmented_synced_pc_cb(self, car_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(car_cloud_msg, None)

    def segmented_pc_cb(self, seg_cloud_msg):
        # this callback will only be used for faster lio
        self.use_faster_lio = True
        self.segmented_synced_pc_cb(seg_cloud_msg, None)

    # synced version
    def segmented_pc_with_odom_cb(self, seg_cloud_msg, odom_msg):
        # this callback will only be used for faster lio
        self.use_faster_lio = True
        self.odom_callback(odom_msg)
        self.segmented_synced_pc_cb(seg_cloud_msg, None)

    def segmented_synced_pc_cb(self, segmented_cloud_msg, undistorted_cloud_msg):
        self.processed_scan_idx = self.processed_scan_idx + 1
        current_raw_timestamp = segmented_cloud_msg.header.stamp
        # create pc from the undistorted_cloud
        segmented_pc = _pointcloud2_to_structured(segmented_cloud_msg)
        if self.use_sim or self.use_faster_lio:
            undistorted_pc = segmented_pc
        else:
            undistorted_pc = _pointcloud2_to_structured(undistorted_cloud_msg)
        # update the x y z to those in the bag
        x_coords = np.nan_to_num(
            undistorted_pc['x'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        y_coords = np.nan_to_num(
            undistorted_pc['y'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        z_coords = np.nan_to_num(
            undistorted_pc['z'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        
        # # fill in the intensity values that represent the class labels
        # if self.use_sim:
        #     intensities= np.zeros_like(x_coords)
        #     intensities[x_coords!=0] = self.car_class_label # car label
        #     # force valid points to be car label
        # else:
        #     intensities = (segmented_pc['intensity']).flatten()
        intensities = (segmented_pc['intensity']).flatten()

        pc_xyzi = np.zeros((x_coords.shape[0], 4))
        pc_xyzi[:, 0] = x_coords
        pc_xyzi[:, 1] = y_coords
        pc_xyzi[:, 2] = z_coords
        pc_xyzi[:, 3] = intensities

        # threshold the point cloud according to the range -- throw away too far to 0-range points
        # print("\n-----------Benchmarking Starts-----------")
        start_time = time.time()
        valid_indices = threshold_by_range(self.valid_range_threshold, pc_xyzi)
        self.thresh_by_range_time = np.append(self.thresh_by_range_time, [time.time() - start_time])
        # print("\n************")
        # print("The instant threshold by range time is {:.7f}".format(self.thresh_by_range_time[-1]))
        # print("The average threshold by range time is {:.7f}".format(self.thresh_by_range_time.mean()))
        # print("************\n")

        if np.sum(valid_indices) == 0:
            print("no valid points found!!!")
            return

        # apply thresholding
        pc_xyzi_thresholded = pc_xyzi[valid_indices, :]


        indices_mask = np.cumsum(valid_indices) - 1
        # indices_original = np.arange(valid_indices.shape[0])

        # apply transform and publish point cloud in the world frame

        start_time = time.time()
        points_world_xyzi, points_body_xyzi = transform_publish_pc(self,
            current_raw_timestamp, pc_xyzi_thresholded)
        self.transform_pano_to_world_time = np.append(self.transform_pano_to_world_time, [time.time() - start_time])
        # print("\n************")
        # print("The instant transform pano to world time is {:.7f}".format(self.transform_pano_to_world_time[-1]))
        # print("The average transform pano to world time is {:.7f}".format(self.transform_pano_to_world_time.mean()))
        # print("************\n")

        if points_world_xyzi is None or points_body_xyzi is None:
            print("failed to find transform, skipping this cloud (see error msg above for exact failed tf)... ")
            print("if you are replaying bags, try passing -p use_sim_time:=true to this node and add --clock to ros2 bag play")
            print("it may also be caused by that your laptop struggles to run real time, play bag with slower rate")

        else:
            # print("successfully find transforms...")
            # intensity is the class label
            points_world_xyzi_cars = points_world_xyzi[points_world_xyzi[:, 3] == self.cls_raw["chair"], :]
            points_world_xyzi_tables = points_world_xyzi[points_world_xyzi[:, 3] == self.cls_raw["table"], :]       
            points_world_xyzi_ground = points_world_xyzi[points_world_xyzi[:, 3] == self.cls_raw["floor"], :]
            # map the labels of chairs and tables
            points_world_xyzi_cars[:, 3] = self.cls["chair"]
            points_world_xyzi_tables[:, 3] = self.cls["table"]



            ##################################### Ground Plane Fitting ##################################
            ransac_n_points = 5
            print("Number of ground points: ", points_world_xyzi_ground.shape[0])
            if points_world_xyzi_ground.shape[0] > ransac_n_points * 10:
                ground_pcd = o3d.geometry.PointCloud()
                ground_pcd.points = o3d.utility.Vector3dVector(points_world_xyzi_ground[:,:3])
                plane_eq, _ = ground_pcd.segment_plane(distance_threshold = 0.1,
                                                       ransac_n=ransac_n_points, num_iterations= 100)
                [a, b, c, d] = plane_eq
                self.ground_plane_coeff = np.array([a,b,c,d])
            else:
                print("not enough ground points, skipping this cloud...")
                return
            # ############################################################################### prepare point clouds for SLOAM tree stuff ###############################################################
            # points_body_xyzi_organized = points_body_xyzi[indices_mask, :]

            # # points_body_xyzi_organized_reshaped = points_body_xyzi_organized.reshape(((64, 1024, 4)))

            # points_body_xyzi_organized[valid_indices == 0, :] = np.NaN

            # # # print the size of points_body_xyzi_organized
            # # print("points_body_xyzi_organized shape: ", points_body_xyzi_organized.shape)

            # points_body_xyzi_tree_pole_masked = points_body_xyzi_organized.copy()
            # points_body_xyzi_ground_masked = points_body_xyzi_organized.copy()
            # non_tree_idx = points_body_xyzi_organized[:, 3] != self.tree_trunk_class_label
            # non_light_pole_idx = points_body_xyzi_organized[:, 3] != self.light_pole_class_label
            # non_tree_or_light_pole_idx = np.logical_and(
            #     non_tree_idx, non_light_pole_idx)
            # points_body_xyzi_tree_pole_masked[non_tree_or_light_pole_idx, :] = np.NaN
            # points_body_xyzi_ground_masked[points_body_xyzi_organized[:, 3]
            #                                != self.ground_class_label, :] = np.NaN
            
            # # destagger
            # # D = np.array([24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0, 24.0, 8.0])
            # points_body_xyzi_tree_pole_masked_reshaped = points_body_xyzi_tree_pole_masked.reshape(
            #     (64, 1024, 4))
            # # for row, shift in enumerate(D):
            # #     print("row",row)
            # #     points_body_xyzi_tree_pole_masked_reshaped[row, :, :] = np.roll(
            # #         points_body_xyzi_tree_pole_masked_reshaped[row, :, :], int(shift), axis=0)
                
            # # points_xyz = np.nan_to_num(scan_data[:, :, :3], nan=0.0).reshape(-1, 3)
            # # points_intensity = np.frombuffer(scan_data[:, :, 3].tobytes(), dtype=np.uint16).reshape(
            # #     points_xyz.shape[0], -1)[:, 1].astype(np.float32)

            # pc_msg_tree = publish_tree_cloud(self.pc_fields_, points_body_xyzi_tree_pole_masked_reshaped, current_raw_timestamp, self.range_image_frame)
            # self.tree_cloud_pub.publish(pc_msg_tree)

            # print("Running on forestry bags is set as: ", self.for_forestry_bags, "(will publish fake cuboids if true).")
            # if self.for_forestry_bags:
            #     pc_msg_ground, fake_cube_body = publish_ground_cloud_and_fake_cubes(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
            #         (64, 1024, 4)), current_raw_timestamp, self.range_image_frame)
            #     self.cuboid_marker_body_pub.publish(fake_cube_body)
            # else:
            #     pc_msg_ground = publish_ground_cloud(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
            #         (64, 1024, 4)), current_raw_timestamp, self.range_image_frame)
            # self.ground_cloud_pub.publish(pc_msg_ground)
            # #############################################################################################################################################################################################

            if points_world_xyzi_ground.shape[0] > 10:
                # ground_median_z = np.median(points_world_xyzi_ground, axis=0)[
                #     2] + self.ground_median_increment
                pass
            else:
                if not self.use_sim:
                    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                    print("less than 10 ground points found, not thresholding, which may result in inaccuracy!!")
                    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                ground_median_z = -1000
            # filter out points that are too close to the ground
            dist_to_ground = calc_dist_to_ground(self, points_world_xyzi_cars)
            points_cars_valid = points_world_xyzi_cars[dist_to_ground>self.ground_median_increment, :]
            dist_to_ground = calc_dist_to_ground(self, points_world_xyzi_tables)
            points_world_xyzi_tables_valid = points_world_xyzi_tables[dist_to_ground>self.ground_median_increment, :]




            # find object instances
            chair_exists = True
            table_exists = True
            if points_cars_valid.shape[0] <= 0:
                chair_exists = False    
                print("skipping the chair fitting for this scan due to no chair points above ground")
            if points_world_xyzi_tables_valid.shape[0] <= 0:
                table_exists = False    
                print("skipping the table fitting for this scan due to no table points above ground")

            cur_start = time.time()
            ################### USE TWO LAYER CLUSTERING ###################
            start_time = time.time()
            
            if chair_exists:
                labels = self.cluster(
                    points_cars_valid, epsilon=self.epsilon_scan_1st_layer, min_samples=self.min_samples_scan_1st_layer, use_2d=True)
                
                points_cars_valid = points_cars_valid[labels != -1, :]   
            
            if table_exists:
                labels_table = self.cluster(
                    points_world_xyzi_tables_valid, epsilon=self.epsilon_scan_1st_layer, min_samples=self.min_samples_scan_1st_layer, use_2d=True)
                
                points_world_xyzi_tables_valid = points_world_xyzi_tables_valid[labels_table != -1, :]

            
            # check if the chairs and tables belong to the any valid cluster
            if points_cars_valid.shape[0] <= 0:
                    chair_exists = False    
                    print("skipping the chair fitting for this scan due to no chair points belong to any cluster")
            if points_world_xyzi_tables_valid.shape[0] <= 0:
                    table_exists = False    
                    print("skipping the table fitting for this scan due to no table points belong to any cluster")


            

            if chair_exists:
                start_time = time.time()
                labels = self.cluster(
                    points_cars_valid, epsilon=self.epsilon_scan, min_samples=self.min_samples_scan, use_2d=True)
                self.clustering_time = np.append(self.clustering_time, [time.time() - start_time])
                # print("\n************")
                # print("The instant clustering time is {:.7f}".format(self.clustering_time[-1]))
                # print("The average clustering time is {:.7f}".format(self.clustering_time.mean()))
                # print("************\n")

                valid_points = points_cars_valid[labels != -1, :]
                labels = labels[labels != -1]
            
                raw_points_xyz = valid_points[:, :3]
            
                start_time = time.time()
                xcs, ycs, lengths, widths, raw_points = fit_cuboid(self.fit_cuboid_length_thresh, 
                self.fit_cuboid_width_thresh, raw_points_xyz, labels)
                self.fit_cuboid_time = np.append(self.fit_cuboid_time, [time.time() - start_time])
                # print("\n************")
                # print("The instant fit cuboid time is {:.7f}".format(self.fit_cuboid_time[-1]))
                # print("The average fit cuboid time is {:.7f}".format(self.fit_cuboid_time.mean()))
                # print("************\n")

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                
                if cur_objects.shape[0] > 0:
                    cur_class_label = self.cls["chair"]
                    cur_object_class = "chair"
                    self.all_objects, self.all_tracks = track_objects_indoor(self, cur_class_label, cur_object_class, 
                    cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), self.downsample_res, self.num_instance_point_lim)
                    self.track_objects_time = np.append(self.track_objects_time, [time.time() - start_time])
                    print("\n************")
                    print("The instant track objects time is {:.7f}".format(self.track_objects_time[-1]))
                    print("The average track objects time is {:.7f}".format(self.track_objects_time.mean()))
                    print("************\n")
                    publish_markers(self, self.all_tracks)
                

            if table_exists:
                start_time = time.time()
                labels_table = self.cluster(
                    points_world_xyzi_tables_valid, epsilon=self.epsilon_scan, min_samples=self.min_samples_scan, use_2d=True)
                self.clustering_time = np.append(self.clustering_time, [time.time() - start_time])
                # print("\n************")
                # print("The instant clustering time is {:.7f}".format(self.clustering_time[-1]))
                # print("The average clustering time is {:.7f}".format(self.clustering_time.mean()))
                # print("************\n")

                valid_points = points_world_xyzi_tables_valid[labels_table != -1, :]
                labels_table = labels_table[labels_table != -1]

                raw_points_xyz = valid_points[:, :3]
            
                start_time = time.time()
                xcs, ycs, lengths, widths, raw_points = fit_cuboid(self.fit_cuboid_length_thresh, 
                self.fit_cuboid_width_thresh, raw_points_xyz, labels_table)
                self.fit_cuboid_time = np.append(self.fit_cuboid_time, [time.time() - start_time])

                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))
                
                if cur_objects.shape[0] > 0:
                    cur_class_label = self.cls["table"]
                    cur_object_class = "table"
                    self.all_objects, self.all_tracks = track_objects_indoor(self, cur_class_label, cur_object_class, 
                    cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), self.downsample_res, self.num_instance_point_lim)
                    self.track_objects_time = np.append(self.track_objects_time, [time.time() - start_time])
                    print("\n************")
                    print("The instant track objects time is {:.7f}".format(self.track_objects_time[-1]))
                    print("The average track objects time is {:.7f}".format(self.track_objects_time.mean()))
                    print("************\n")
                    publish_markers(self, self.all_tracks)




            # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
            start_time = time.time()
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
            self.deleting_tracks_time = np.append(self.deleting_tracks_time, [time.time() - start_time])
            # print("\n************")
            # print("The instant track deletion time is {:.7f}".format(self.deleting_tracks_time[-1]))
            # print("The average track deletion time is {:.7f}".format(self.deleting_tracks_time.mean()))
            # print("************\n")

            start_time = time.time()
            # if self.valid_scan_counter > self.number_scan_accumulation:
            # publish_accumulated_cloud(self,current_raw_timestamp)
            extracted_instances_xyzl, instance_global_ids = generate_publish_instance_cloud_indoor(self,
                current_raw_timestamp)
            self.accumulated_cloud_time = np.append(self.accumulated_cloud_time, [time.time() - start_time])
            # print("\n************")
            # print("The instant accumulated cloud time is {:.7f}".format(self.accumulated_cloud_time[-1]))
            # print("The average accumulated cloud time is {:.7f}".format(self.accumulated_cloud_time.mean()))
            # print("************\n")

            if extracted_instances_xyzl is not None:
                start_time = time.time()
                cuboids, cuboid_clus_centroids = cuboid_detection_indoor(self,
                    extracted_instances_xyzl, instance_global_ids)
                if len(cuboids) > 0:
                    # publish_cuboid_markers(self,
                    #     copy.deepcopy(cuboids), current_raw_timestamp)
                    self.final_cuboid_detection_time = np.append(self.final_cuboid_detection_time, [time.time() - start_time])
                    # print("\n************")
                    # print("The instant final cuboid formation time is {:.7f}".format(self.final_cuboid_detection_time[-1]))
                    # print("The average final cuboid formation time is {:.7f}".format(self.final_cuboid_detection_time.mean()))
                    # print("************\n")

                    cuboid_clus_labels = cluster_indoor(np.array(cuboid_clus_centroids), self.epsilon_scan, self.min_samples_scan, use_2d=False)
                    # TODO: we need to modify this to publish multi-class centroid measurements
                    if self.publish_with_models:
                        publish_cuboid_and_range_bearing_measurements_with_models(self, 
                        copy.deepcopy(cuboids), current_raw_timestamp)#, self.odom_from_cloud_msg)
                    elif self.publish_with_floors:
                        publish_cuboid_and_range_bearing_measurements_by_floor(self, 
                        copy.deepcopy(cuboids), current_raw_timestamp)#, self.odom_from_cloud_msg)
                    elif self.publish_with_floors_proper:
                        assert len(cuboid_clus_labels) == len(cuboids)
                        publish_cuboid_and_range_bearing_measurements_by_floor_properly(self, 
                        copy.deepcopy(cuboids), cuboid_clus_labels, current_raw_timestamp)#, self.odom_from_cloud_msg)
                    else:
                        publish_cuboid_and_range_bearing_measurements(self, 
                        copy.deepcopy(cuboids), current_raw_timestamp)#, self.odom_from_cloud_msg)

                    # print("\n--------Benchmarking Ends---------\n")


    # TODO add the ground fitting from the stats branch

    def odom_callback(self, msg):
        send_tfs(self, msg)
        if self.odom_received == False:
            # print(f'publishing odom as tf')
            self.odom_received = True


def main(args=None):
    rclpy.init(args=args)
    node = ProcessCloudNode()
    print("node started!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("node killed!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
