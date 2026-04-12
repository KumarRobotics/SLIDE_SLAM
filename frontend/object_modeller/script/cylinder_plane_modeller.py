#!/usr/bin/env python3

# subscribe to both /tree_cloud and /ground_cloud

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
import open3d as o3d
import copy
# import rviz marker and marker array
from visualization_msgs.msg import Marker, MarkerArray
# import R from scipy
from scipy.spatial.transform import Rotation as R
from sloam_msgs.msg import ROSCylinder, ROSCylinderArray, StampedRvizMarkerArray
import message_filters

# This node models the tree as a cylinder and the ground as a plane


class CylinderPlaneModeller(Node):
    def __init__(self, args):
        super().__init__("cylinder_plane_modeller")

        self.synced_tree_and_ground_clouds = None
        # make subcriber with queue size 1
        self.point_cloud_ns = args["point_cloud_ns"]

        # SUBSCRIBERS
        # create a synchronizer that subscribes to both tree_cloud and ground_cloud
        # TODO(ankit): since our process cloud node always publish tree_cloud and ground_cloud at the same time, this implementation is accurate, for future implemnetation, consider syncing them
        self.tree_cloud_sub = message_filters.Subscriber(
            self, PointCloud2, self.point_cloud_ns + "tree_cloud")
        self.ground_cloud_sub = message_filters.Subscriber(
            self, PointCloud2, self.point_cloud_ns + "ground_cloud")
        # sync with maximum time difference of 0.01 seconds
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.tree_cloud_sub, self.ground_cloud_sub], 10, 0.01)
        # register one callback named clouds_callback
        self.sync.registerCallback(self.clouds_callback)

        self.cuboid_sub = self.create_subscription(
            MarkerArray, self.point_cloud_ns + "car_cuboids_body",
            self.cuboid_callback, 10)
        self.cuboid_indoor_sub = self.create_subscription(
            MarkerArray, "chair_cuboids_body", self.cuboid_indoor_callback, 10)

        # PUBLISHERS
        self.tree_instance_pub = self.create_publisher(
            PointCloud2, "tree_instance_cloud", 1)
        self.cylinder_marker_pub = self.create_publisher(
            MarkerArray, "cylinder_marker", 1)
        self.ground_plane_marker_pub = self.create_publisher(
            MarkerArray, "ground_plane_marker", 1)
        self.sync_meas_pub = self.create_publisher(
            ROSCylinderArray, "cylinder_measurements", 10)
        self.cuboid_pub = self.create_publisher(
            StampedRvizMarkerArray, "cuboid_measurements", 10)
        self.cuboid_indoor_pub = self.create_publisher(
            StampedRvizMarkerArray, "chair_cuboids_stamped", 10)

        self.pc_fields_ = self.make_fields()

        # ----------------PARAMETERS-------------------
        # TODO(ankit): Delete unused parameters
        self.declare_parameter("angle_cutoff", 15.0)
        self.declare_parameter("radius_cutoff", [0.05, 0.5])
        self.declare_parameter("min_points_per_tree", 15)
        self.declare_parameter("min_points_for_radius", 5)
        self.declare_parameter("min_points_per_ground_patch", 40)
        self.declare_parameter("ground_plane_patch_size", 10)
        self.declare_parameter("num_ground_plane_model_to_keep", 50)
        self.declare_parameter("clus_eps", 0.5)
        self.declare_parameter("clus_min_samples", 10)
        self.declare_parameter("diameter_measure_height_above_ground", 1.3716)
        self.declare_parameter("default_radius", 0.2)
        self.declare_parameter("run_rate", 20.0)

        self.angle_cutoff = np.deg2rad(
            self.get_parameter("angle_cutoff").value)
        _rc = list(self.get_parameter("radius_cutoff").value)
        self.radius_cutoff = (float(_rc[0]), float(_rc[1]))
        self.min_points_per_tree = self.get_parameter("min_points_per_tree").value
        self.min_points_for_radius = self.get_parameter("min_points_for_radius").value
        self.min_points_per_ground_patch = self.get_parameter(
            "min_points_per_ground_patch").value
        self.ground_plane_patch_size = self.get_parameter(
            "ground_plane_patch_size").value
        self.num_ground_plane_model_to_keep = self.get_parameter(
            "num_ground_plane_model_to_keep").value
        self.clus_eps = self.get_parameter("clus_eps").value
        self.clus_min_samples = self.get_parameter("clus_min_samples").value
        self.diameter_measure_height_above_ground = self.get_parameter(
            "diameter_measure_height_above_ground").value
        self.default_radius = self.get_parameter("default_radius").value
        run_rate = float(self.get_parameter("run_rate").value)
        # ----------------------------------------------

        # CONTAINERS
        self.latest_ground_plane_models = []
        self.latest_ground_plane_models_centroids = []
        # tree cloud counter
        self.tree_cloud_counter = 0
        self.tree_cloud_start_time = self.get_clock().now()
        self.average_tree_cloud_rate = 0

        self.tree_cloud_process_timer = self.create_timer(
            1.0 / run_rate, self.tree_cloud_process)

    # Callback to convert the standard MarkerArray to custom StampedRvizMarkerArray which also contains the msg header
    def cuboid_callback(self, msg):
        # create a new message
        stamped_rviz_marker_array = StampedRvizMarkerArray()
        # fill in the header
        stamped_rviz_marker_array.header = msg.markers[-1].header
        # fill in the marker array
        stamped_rviz_marker_array.cuboid_rviz_markers = msg
        # publish
        self.cuboid_pub.publish(stamped_rviz_marker_array)
        self.get_logger().info(
            "Successfully published stamped outdoor cuboid measurements, next will do msg syncing!",
            throttle_duration_sec=7)

    # Same as cuboid_callback but for indoor cuboids
    def cuboid_indoor_callback(self, msg):
        # read the timestamp of the last cuboid in the message
        # publish cuboid with the stamp
        # create a new message
        stamped_rviz_marker_array = StampedRvizMarkerArray()
        # fill in the header
        stamped_rviz_marker_array.header = msg.markers[-1].header
        # fill in the marker array
        stamped_rviz_marker_array.cuboid_rviz_markers = msg
        # publish
        self.cuboid_indoor_pub.publish(stamped_rviz_marker_array)
        self.get_logger().info(
            "Successfully published stamped indoor cuboid measurements, next will do msg syncing!",
            throttle_duration_sec=7)

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _now_sec(self):
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    # Timesynced callback for both tree and ground cloud
    def clouds_callback(self, tree_cloud, ground_cloud):
        self.get_logger().info(
            "Received synced ground cloud and tree cloud",
            throttle_duration_sec=5)
        if self.tree_cloud_counter == 0:
            self.tree_cloud_start_time = self.get_clock().now()
        else:
            elapsed = (self.get_clock().now() -
                       self.tree_cloud_start_time).nanoseconds * 1e-9
            self.average_tree_cloud_rate = self.tree_cloud_counter / max(
                0.01, elapsed)
        self.tree_cloud_counter += 1

        self.synced_tree_and_ground_clouds = [tree_cloud, ground_cloud]

    # make timer back process function
    def tree_cloud_process(self):
        if self.synced_tree_and_ground_clouds is not None:
            start_time = self._now_sec()
            self.cluster(self.synced_tree_and_ground_clouds)
            time_to_process = self._now_sec() - start_time
            # avoid division by zero
            if time_to_process > 0:
                rate = 1.0 / time_to_process
            else:
                rate = 0.0

            if rate < self.average_tree_cloud_rate:
                self.get_logger().warn(
                    "Time to process tree cloud is longer than the incoming tree point cloud rate. Tree and ground cloud will be reset. Tune the clustering parameters or slow down the incoming point cloud. Processing rate: %.2f, Average incoming rate: %.2f"
                    % (rate, self.average_tree_cloud_rate))
                # reset the latest_tree_cloud
                # reset it to avoid processing the same cloud again
                self.synced_tree_and_ground_clouds = None

    def cluster(self, cur_synced_tree_and_ground_clouds):
        latest_tree_cloud = cur_synced_tree_and_ground_clouds[0]
        cur_ground_points = np.array(list(pc2.read_points(
            cur_synced_tree_and_ground_clouds[1],
            skip_nans=True,
            field_names=("x", "y", "z"))))
        original_cloud = latest_tree_cloud
        # extract xyz from the cloud
        cloud_points = np.array(list(pc2.read_points(
            original_cloud, skip_nans=True,
            field_names=("x", "y", "z"))))
        # check if tree cloud points is more than a given threshold
        # TODO(ankit): remove hardcoded value for minimum tree points in cloud
        if cloud_points.shape[0] > 40:  # 40 is the minimum number of points in the tree cloud
            projected_points = cloud_points
            valid_indices = np.ones(cloud_points.shape[0], dtype=bool)
            tree_cloud_header = original_cloud.header
        else:
            self.get_logger().warn(
                "Not enough points in the tree cloud. Skipping this cloud...",
                throttle_duration_sec=3)
            return

        self.get_logger().info("Clustering tree instance cloud",
                               throttle_duration_sec=5)
        db = DBSCAN(eps=self.clus_eps,
                    min_samples=self.clus_min_samples).fit(projected_points)
        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        # extract the points in each cluster
        tree_instance_cloud = []
        for i in range(n_clusters_):
            # index the points in the original cloud (i.e. cloud_points)
            # that belong to the i-th cluster
            tree_instance_cloud.append(
                cloud_points[valid_indices, :][labels == i])

        # publish tree instance cloud one by one
        # full_data record all instances, each instance has different intensity
        full_data = None
        # init header
        tree_instance_cloud_msg = PointCloud2()
        # same frame_id and same timestamp as the tree cloud
        tree_instance_cloud_msg.header.frame_id = tree_cloud_header.frame_id
        tree_instance_cloud_msg.header.stamp = tree_cloud_header.stamp
        cylinders = []

        for i in range(n_clusters_):
            # check if tree_instance_cloud[i] is None
            if tree_instance_cloud is None:
                self.get_logger().warn(
                    "Tree instance cloud is None, skipping this cloud...",
                    throttle_duration_sec=2)
                continue

            # fit a cylinder to the points
            radius, ray, root = self.fit_cylinder(
                tree_instance_cloud[i], cur_ground_points, tree_cloud_header, i)

            if radius is not None:
                cylinders.append([radius, ray, root])

            if full_data is None:
                full_data = np.hstack(
                    (tree_instance_cloud[i],
                     np.ones((tree_instance_cloud[i].shape[0], 1)) * i)).astype(np.float32)
            else:
                full_data = np.vstack((full_data, np.hstack(
                    (tree_instance_cloud[i],
                     np.ones((tree_instance_cloud[i].shape[0], 1)) * i)).astype(np.float32)))

            self.get_logger().info(
                "Successfully published tree instance cloud",
                throttle_duration_sec=5)

        # visualize the cylinders in rviz
        if len(cylinders) > 0:
            self.visualize_cylinders(cylinders, tree_cloud_header)
            # create ROSCylinderArray message
            ros_cylinder_array = ROSCylinderArray()
            # fill in the ROSCylinder message and publish it
            # fill in the cylinders
            temp_id = 0
            for cylinder in cylinders:
                # cylinder factor should have
                # float32[3] root
                # float32[3] ray
                # float64[] radii
                # float32 radius
                # int64 id
                ros_cylinder = ROSCylinder()
                ros_cylinder.root = [float(v) for v in cylinder[2]]
                ros_cylinder.ray = [float(v) for v in cylinder[1]]
                ros_cylinder.radius = float(cylinder[0])
                ros_cylinder.id = temp_id  # TODO: do the tracking to globally assign ID!!!!!!!!!!!!!
                temp_id += 1
                # append to the ROSCylinder message
                ros_cylinder_array.cylinders.append(ros_cylinder)

            # fill in the header
            ros_cylinder_array.header = tree_cloud_header
            # publish
            self.sync_meas_pub.publish(ros_cylinder_array)

        if full_data is not None:
            # init width and height
            tree_instance_cloud_msg.width = int(full_data.shape[0])
            tree_instance_cloud_msg.height = 1

            # init fields
            tree_instance_cloud_msg.fields = self.pc_fields_

            # point step is 16
            tree_instance_cloud_msg.point_step = 16

            # row step is point step * width
            tree_instance_cloud_msg.row_step = (
                tree_instance_cloud_msg.point_step *
                tree_instance_cloud_msg.width)

            # init data
            tree_instance_cloud_msg.data = full_data.tobytes()

            # publish
            self.tree_instance_pub.publish(tree_instance_cloud_msg)

    def visualize_cylinders(self, cylinders, header):
        # first delete all markers
        marker_array = MarkerArray()
        for i in range(100):
            marker = Marker()
            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = "cylinder"
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
        self.cylinder_marker_pub.publish(marker_array)

        # create marker array
        marker_array = MarkerArray()
        # enumerate all cylinders
        for i, cylinder in enumerate(cylinders):
            axis = cylinder[1]
            # normalize the axis
            axis = axis / np.linalg.norm(axis)
            radius = cylinder[0]
            root = cylinder[2]
            # create a cylinder marker
            marker = Marker()
            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = "cylinder"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            # normalized x,y,z displacement to move from root to center of the cylinder
            z_scale = 10
            half_scale = 0.5 * z_scale
            z_shift = half_scale * axis[2]
            x_shift = half_scale * axis[0]
            y_shift = half_scale * axis[1]
            marker.pose.position.x = float(root[0] + x_shift)
            marker.pose.position.y = float(root[1] + y_shift)
            marker.pose.position.z = float(root[2] + z_shift)

            # get a rotation that rotates the z axis to the axis
            rotation_axis, rotation_angle = self.compute_rotation(
                axis, np.array([0, 0, 1]))
            rotation = R.from_rotvec(rotation_angle * rotation_axis)

            # invert the rotation
            rotation = rotation.inv()
            # convert to quaternion
            quaternion = rotation.as_quat()
            marker.pose.orientation.x = float(quaternion[0])
            marker.pose.orientation.y = float(quaternion[1])
            marker.pose.orientation.z = float(quaternion[2])
            marker.pose.orientation.w = float(quaternion[3])

            marker.scale.x = float(radius * 2)
            marker.scale.y = float(radius * 2)
            marker.scale.z = float(z_scale * 0.65)

            marker.color.a = 0.9
            # colored by i
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            # append to marker array
            marker_array.markers.append(marker)
        # publish
        self.cylinder_marker_pub.publish(marker_array)

    def normalize(self, v):
        """Normalize a vector."""
        return v / np.linalg.norm(v)

    def compute_rotation(self, vec_1, vec_2):
        """Compute rotation axis and angle between two vectors."""
        # Ensure A and B are normalized
        A = self.normalize(vec_1)
        B = self.normalize(vec_2)

        # Compute the rotation axis
        rotation_axis = np.cross(A, B)
        rotation_axis = self.normalize(rotation_axis)

        # Compute the rotation angle
        rotation_angle = np.arccos(np.dot(A, B))
        return rotation_axis, rotation_angle

    def fit_cylinder(self, points, cur_ground_points, tree_cloud_header, cylinder_id):

        if points.shape[0] > self.min_points_per_tree:
            # get the centroid (median) of points
            centroid = np.median(points, axis=0)
            # search the latest point ground cloud, crop a patch of size self.ground_plane_patch_size x self.ground_plane_patch_size around the centroid
            dist_xy = np.linalg.norm(
                cur_ground_points[:, :2] - centroid[:2], axis=1)
            valid_points_idx = dist_xy < self.ground_plane_patch_size / 2
            local_ground_points = cur_ground_points[valid_points_idx]

            # only do ground fitting if there are enough points
            if local_ground_points.shape[0] > self.min_points_per_ground_patch:
                # fit a plane model to the local ground points
                ground_plane_coeff = self.ground_fitting(
                    local_ground_points, ransac_n_points=5)
            else:
                self.get_logger().warn(
                    "Not enough points for ground plane fitting. Setting ground plane to None. Current ground points: %d, Required ground points: %d"
                    % (local_ground_points.shape[0],
                       self.min_points_per_ground_patch),
                    throttle_duration_sec=7)
                ground_plane_coeff = None

            if ground_plane_coeff is None and len(self.latest_ground_plane_models) > 1:
                # search the existing ground plane models, if there is one within self.ground_path_size * 2, take it as the ground plane model
                self.get_logger().info(
                    "No local ground plane model found, searching the closeby ground plane models...",
                    throttle_duration_sec=5)
                # create an array from latest_ground_plane_models
                latest_ground_plane_models_centroids_array = np.array(
                    self.latest_ground_plane_models_centroids)
                # get the distance to the centroid
                distance_to_centroid = np.linalg.norm(
                    latest_ground_plane_models_centroids_array - centroid, axis=1)
                # find the min distance, if it is less than self.ground_plane_patch_size * 2, take it as the ground plane model
                assert len(self.latest_ground_plane_models_centroids) == len(
                    self.latest_ground_plane_models)
                if np.min(distance_to_centroid) < self.ground_plane_patch_size * 2:
                    ground_plane_coeff = self.latest_ground_plane_models[np.argmin(
                        distance_to_centroid)]
                else:
                    self.get_logger().warn(
                        "No closeby ground plane model found, discarding the current axis ray.",
                        throttle_duration_sec=7)
                    return None, None, None

            elif ground_plane_coeff is not None:
                # visualize the ground plane
                ground_centroid = np.median(local_ground_points, axis=0)
                self.visualize_ground_plane(
                    ground_plane_coeff, ground_centroid, tree_cloud_header, cylinder_id)
                # add this to the latest ground plane models
                self.latest_ground_plane_models.append(ground_plane_coeff)
                self.latest_ground_plane_models_centroids.append(
                    ground_centroid)
                if len(self.latest_ground_plane_models) > self.num_ground_plane_model_to_keep:
                    # pop the oldest ground plane model
                    self.latest_ground_plane_models.pop(0)
                    self.latest_ground_plane_models_centroids.pop(0)
            else:
                # print in red color
                self.get_logger().warn(
                    "No ground plane model detected, and no existing ground plane models.",
                    throttle_duration_sec=5)
                return None, None, None

            # fit a line to the points, which is the axis ray
            axis = self.fit_line(points, ground_plane_coeff)
            if axis is None:
                return None, None, None

            # get the radius
            a, b, c, d = ground_plane_coeff
            distance_to_ground = np.abs(np.dot(points, np.array(
                [a, b, c])) + d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)
            # pick the points that have distance to the ground plane close to diameter_measure_height_above_ground
            valid_indices = np.abs(
                distance_to_ground - self.diameter_measure_height_above_ground) < 0.3
            diameter_points = points[valid_indices, :]

            if diameter_points.shape[0] < self.min_points_for_radius:
                self.get_logger().warn(
                    "Not enough points to calculate the radius. Current points: {}, Minimum required points: {}".format(
                        diameter_points.shape[0], self.min_points_for_radius),
                    throttle_duration_sec=5)
                return None, None, None

            else:
                # get the diameter, which is the largest distance between two points in diameter_points
                # get the distance matrix
                distance_matrix = np.linalg.norm(
                    diameter_points[:, None, :] - diameter_points[None, :, :], axis=-1)
                # get the largest distance
                diameter = np.max(distance_matrix)
                # centroid of the diameter points
                representative_point = np.mean(diameter_points, axis=0)

            min_diameter = self.radius_cutoff[0] * 2
            max_diameter = self.radius_cutoff[1] * 2

            if diameter < min_diameter or diameter > max_diameter:
                self.get_logger().warn(
                    "Diameter out of range ({}). Setting as default. Min diameter: {}, Max diameter: {}".format(
                        diameter, min_diameter, max_diameter),
                    throttle_duration_sec=3)
                radius = self.default_radius
            else:
                radius = 0.5 * diameter

            # get the root which is the intersection of the axis ray and the ground plane
            # intropolate the point along axis to the ground plane to get the root
            root_z = (
                d - a * representative_point[0] - b * representative_point[1]) / c
            root = representative_point - axis * root_z
            return radius, axis, root
        else:
            self.get_logger().warn(
                "Not enough points to fit a cylinder. Current points: %d, Minimum required points: %d"
                % (points.shape[0], self.min_points_per_tree),
                throttle_duration_sec=3)
            return None, None, None

    def fit_line(self, points, ground_plane_coeff):
        # fit a line to the points
        # return the axis ray
        # calculate the centroid of the points
        centroid = np.median(points, axis=0)
        # subtract the centroid from the points
        centered_points = points - centroid
        # calculate the covariance matrix
        covariance_matrix = np.dot(centered_points.T, centered_points)
        # calculate the eigenvalues and eigenvectors of the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
        # the eigenvector with the largest eigenvalue is the axis ray
        axis = eigenvectors[:, np.argmax(eigenvalues)]
        a, b, c, d = ground_plane_coeff
        ground_plane_axis = np.array([a, b, c])
        # if the angle between axis ray and ground plane axis is more than discard the axis ray
        angle = np.arccos(np.dot(axis, ground_plane_axis))
        if angle > self.angle_cutoff:
            self.get_logger().warn(
                "Axis ray is more than threshold from ground plane axis, discarding the axis ray",
                throttle_duration_sec=3)
            if angle * 2 < self.angle_cutoff:
                # make axis lie in the middle of axis and ground plane axis
                axis = (axis + ground_plane_axis) / 2
                # normalize the axis
                axis = axis / np.linalg.norm(axis)
            return None
        else:
            return axis

    def make_fields(self):
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields

    def ground_fitting(self, points, ransac_n_points=5):

        ground_pcd = o3d.geometry.PointCloud()
        ground_pcd.points = o3d.utility.Vector3dVector(points)
        plane_eq, _ = ground_pcd.segment_plane(
            distance_threshold=0.1, ransac_n=ransac_n_points, num_iterations=100)
        [a, b, c, d] = plane_eq
        ground_plane_coeff = np.array([a, b, c, d])

        return ground_plane_coeff

    def visualize_ground_plane(self, ground_plane_coeff, ground_centroid, header, id):

        markerarray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = "ground_plane"
        marker.id = int(id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(ground_centroid[0])
        marker.pose.position.y = float(ground_centroid[1])
        marker.pose.position.z = float(ground_centroid[2])
        # get a rotation that rotates the z axis to the ground plane normal
        rotation_axis, rotation_angle = self.compute_rotation(
            ground_plane_coeff[:3], np.array([0, 0, 1]))
        rotation = R.from_rotvec(rotation_angle * rotation_axis)
        # invert the rotation
        rotation = rotation.inv()
        # convert to quaternion
        quaternion = rotation.as_quat()
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        marker.scale.x = float(self.ground_plane_patch_size * 0.5)
        marker.scale.y = float(self.ground_plane_patch_size * 0.5)
        marker.scale.z = 0.01
        marker.color.a = 0.4
        # color the ground plane in ochre
        marker.color.r = 0.8
        marker.color.g = 0.4
        marker.color.b = 0.1
        markerarray.markers.append(marker)
        self.ground_plane_marker_pub.publish(markerarray)


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("--point_cloud_ns", type=str, default="",
                    help="point_cloud_ns namespace")
    # strip ROS args (everything after --ros-args)
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    parsed, _ = ap.parse_known_args(argv)
    parsed_args = vars(parsed)

    node = CylinderPlaneModeller(parsed_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
