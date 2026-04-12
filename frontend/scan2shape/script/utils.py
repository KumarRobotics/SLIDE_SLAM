#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# TODO(ankit): Figure out the purpose of appending parent and grandparent directories to sys.path
# make module semantic_exploration visible by adding it to python path
# import sys
# import os
# dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# sys.path.append(dir)
# dir2 = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(dir2)


def show_clusters(cloud_mat_2d, labels, tree_clustering, save_fig_idx, output_dir):

    lower_lim = np.median(cloud_mat_2d, axis=0) - 30
    upper_lim = np.median(cloud_mat_2d, axis=0) + 30

    # visualize raw data first
    plt.xlim([lower_lim[0], upper_lim[0]])
    plt.ylim([lower_lim[1], upper_lim[1]])
    plt.scatter(cloud_mat_2d[:, 0], cloud_mat_2d[:, 1], s=0.03)
    plt.savefig(output_dir +
                "raw_data_{0:04d}.png".format(save_fig_idx))
    plt.clf()

    # visualize clustering result -- ref [https://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#sphx-glr-auto-examples-cluster-plot-dbscan-py]
    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    # print('Estimated number of clusters: %d' % n_clusters_)
    # print('Estimated number of noise points: %d' % n_noise_)

    # Black color is removed and is used for noise points instead.
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [1, 1, 1, 1]
        class_member_mask = (labels == k)

        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[tree_clustering.core_sample_indices_] = True

        xy = cloud_mat_2d[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor=tuple(col), markersize=1)

        xy = cloud_mat_2d[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor=tuple(col), markersize=0.5)

    plt.xlim([lower_lim[0], upper_lim[0]])
    plt.ylim([lower_lim[1], upper_lim[1]])

    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.savefig(output_dir +
                "clustered_data_{0:04d}.png".format(save_fig_idx))
    plt.clf()

# Currently not used
# ---------------------------------------------------------------------
# def cloud2array(cloud_msg, squeeze=True):
#     dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
#     # parse the cloud into an array
#     cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
#     # remove the dummy fields that were added
#     cloud_arr = cloud_arr[
#         [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
#     if squeeze and cloud_msg.height == 1:
#         return np.reshape(cloud_arr, (cloud_msg.width,))
#     else:
#         return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))
# ---------------------------------------------------------------------


def publish_cylinder_cloud(pc_fields, cylinder_cloud, timestamp, frame_id, pc_width=1024, pc_height=64, pc_point_step=16):
    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp
    header.frame_id = frame_id
    pc_msg.header = header
    pc_msg.width = pc_width
    pc_msg.height = pc_height
    pc_msg.point_step = pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = pc_fields
    full_data = cylinder_cloud.astype(np.float32)
    pc_msg.data = full_data.tobytes()
    return pc_msg


def publish_ground_cloud(pc_fields, ground_cloud, timestamp, frame_id, pc_height=64, pc_width=1024, pc_point_step=16):

    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp
    header.frame_id = frame_id
    pc_msg.header = header
    pc_msg.height = pc_height
    if pc_height != 1:
        pc_msg.width = pc_width
    else:
        pc_msg.width = ground_cloud.shape[0]
    pc_msg.point_step = pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = pc_fields
    full_data = ground_cloud.astype(np.float32)
    pc_msg.data = full_data.tobytes()

    return pc_msg


def calc_dist_to_ground(process_cloud_node_object, points):

    numerator = np.abs(points[:, 0] * process_cloud_node_object.ground_plane_coeff[0]
                       + points[:, 1] *
                       process_cloud_node_object.ground_plane_coeff[1]
                       + points[:, 2] *
                       process_cloud_node_object.ground_plane_coeff[2]
                       + process_cloud_node_object.ground_plane_coeff[3] * np.ones(points.shape[0]))
    denominator = np.linalg.norm(
        process_cloud_node_object.ground_plane_coeff[:3])
    dist_to_groud = numerator / denominator
    return dist_to_groud


def transform_publish_pc(process_cloud_node_object, current_timestamp, pc_xyzi_id_conf_thres_camera):
    # transform semantic cloud, apply filtering, and publish the segmented and filtered point cloud in world frame
    H_world_pano = np.zeros((4, 4))

    try:
        # transform data from the source_frame into the target_frame
        tf_msg = process_cloud_node_object.tf_buffer.lookup_transform(
            process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.reference_frame, current_timestamp)
        t_world_pano = (tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z)
        quat_world_pano = (tf_msg.transform.rotation.x,
                           tf_msg.transform.rotation.y,
                           tf_msg.transform.rotation.z,
                           tf_msg.transform.rotation.w)
        r_world_pano = R.from_quat(quat_world_pano)
        H_world_pano_rot = r_world_pano.as_matrix()
        H_world_pano_trans = np.array(t_world_pano)
        # transformation matrix from world to robot
        H_world_pano[:3, :3] = H_world_pano_rot
        H_world_pano[:3, 3] = H_world_pano_trans
        H_world_pano[3, 3] = 1

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        process_cloud_node_object.get_logger().warn(
            "Cannot find TF from " + process_cloud_node_object.reference_frame +
            " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
        return None, None

    points_pano_xyz = pc_xyzi_id_conf_thres_camera[:, :3]

    # convert to homogeneous representation
    points_pano_xyz_homg = np.ones((points_pano_xyz.shape[0], 4))
    points_pano_xyz_homg[:, :3] = points_pano_xyz
    points_world_xyz_homg = (np.linalg.pinv(
        H_world_pano) @ points_pano_xyz_homg.T).T

    factor_stacked = np.repeat(
        points_world_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
    # normalize
    points_world_xyz = np.divide(
        points_world_xyz_homg[:, :3], factor_stacked)

    points_world_xyzi_id_conf_depth = np.zeros(
        (pc_xyzi_id_conf_thres_camera.shape[0], 7))
    points_world_xyzi_id_conf_depth[:, :3] = points_world_xyz
    points_world_xyzi_id_conf_depth[:,
                                    3:6] = pc_xyzi_id_conf_thres_camera[:, 3:6]
    # keep track of depth
    points_world_xyzi_id_conf_depth[:, 6] = pc_xyzi_id_conf_thres_camera[:, 2]

    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = current_timestamp
    header.frame_id = process_cloud_node_object.reference_frame
    pc_msg.header = header
    pc_msg.width = points_world_xyzi_id_conf_depth.shape[0]
    pc_msg.height = 1
    pc_msg.point_step = process_cloud_node_object.pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = process_cloud_node_object.pc_fields_
    full_data = points_world_xyzi_id_conf_depth[:, :4].astype(np.float32)
    pc_msg.data = full_data.tobytes()

    process_cloud_node_object.segmented_pc_pub.publish(pc_msg)
    process_cloud_node_object.get_logger().info(
        "Filtered segmented point cloud published",
        throttle_duration_sec=5)

    # prepare point clouds for cylinder fitting (currently not used)
    if process_cloud_node_object.use_sim == False:
        H_body_pano = np.zeros((4, 4))
        try:
            # transform data in the source_frame into the target_frame
            tf_msg2 = process_cloud_node_object.tf_buffer.lookup_transform(
                process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.range_image_frame, current_timestamp)
            t_body_pano = (tf_msg2.transform.translation.x,
                           tf_msg2.transform.translation.y,
                           tf_msg2.transform.translation.z)
            quat_body_pano = (tf_msg2.transform.rotation.x,
                              tf_msg2.transform.rotation.y,
                              tf_msg2.transform.rotation.z,
                              tf_msg2.transform.rotation.w)
            r_body_pano = R.from_quat(quat_body_pano)
            H_body_pano_rot = r_body_pano.as_matrix()
            H_body_pano_trans = np.array(t_body_pano)
            H_body_pano[:3, :3] = H_body_pano_rot
            H_body_pano[:3, 3] = H_body_pano_trans
            H_body_pano[3, 3] = 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            process_cloud_node_object.get_logger().warn(
                "Cannot find TF from " + process_cloud_node_object.range_image_frame +
                " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
            return None, None

        points_body_xyz_homg = (np.linalg.pinv(
            H_body_pano) @ points_pano_xyz_homg.T).T

        factor_stacked = np.repeat(
            points_body_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
        # normalize
        points_body_xyz = np.divide(
            points_body_xyz_homg[:, :3], factor_stacked)
    else:
        if process_cloud_node_object.range_image_frame == process_cloud_node_object.undistorted_cloud_frame:
            points_body_xyz = points_pano_xyz_homg[:, :3]
        else:
            raise Exception(
                "range_image_frame is different from undistorted_cloud_frame for running simulation, which is incorrect!!")

    points_body_xyzi = np.zeros((pc_xyzi_id_conf_thres_camera.shape[0], 6))
    points_body_xyzi[:, :3] = points_body_xyz
    points_body_xyzi[:, 3:] = pc_xyzi_id_conf_thres_camera[:, 3:]
    ####################################################################################################################################################################################

    return points_world_xyzi_id_conf_depth, points_body_xyzi


def _make_transform_stamped(stamp, parent_frame, child_frame, translation, rotation):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(translation[0])
    t.transform.translation.y = float(translation[1])
    t.transform.translation.z = float(translation[2])
    t.transform.rotation.x = float(rotation[0])
    t.transform.rotation.y = float(rotation[1])
    t.transform.rotation.z = float(rotation[2])
    t.transform.rotation.w = float(rotation[3])
    return t


def send_tfs(process_cloud_node_object, msg):

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(msg.header.stamp, "quadrotor/map", "quadrotor/odom",
                                (0, 0, 0), (0, 0, 0, 1)))

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(msg.header.stamp, "dragonfly67/odom", "quadrotor/map",
                                (0, 0, 0), (0, 0, 0, 1)))

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(msg.header.stamp, "odom", "dragonfly67/odom",
                                (0, 0, 0), (0, 0, 0, 1)))

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(msg.header.stamp, "world", "odom",
                                (0, 0, 0), (0, 0, 0, 1)))

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(
            msg.header.stamp,
            process_cloud_node_object.reference_frame,
            process_cloud_node_object.range_image_frame,
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)))

    odom_msg = msg
    process_cloud_node_object.odom_pub.publish(odom_msg)

    rot_mat = np.array([[0.0000000, -1.0000000,  0.0000000],
                        [0.0000000,  0.0000000, -1.0000000],
                        [1.0000000,  0.0000000,  0.0000000]])
    rot_cam_body_quat = R.from_matrix(rot_mat.T).as_quat()

    process_cloud_node_object.odom_broadcaster.sendTransform(
        _make_transform_stamped(
            msg.header.stamp,
            process_cloud_node_object.range_image_frame,
            process_cloud_node_object.undistorted_cloud_frame,
            (0, 0, 0),
            (rot_cam_body_quat[0], rot_cam_body_quat[1],
             rot_cam_body_quat[2], rot_cam_body_quat[3])))


def publish_accumulated_cloud(process_cloud_node_object, timestamp):
    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp
    header.frame_id = process_cloud_node_object.reference_frame
    pc_msg.header = header
    pc_msg.width = process_cloud_node_object.accumulated_semantic_cloud.shape[0]
    pc_msg.height = 1
    pc_msg.point_step = process_cloud_node_object.pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = process_cloud_node_object.pc_fields_
    full_data = process_cloud_node_object.accumulated_semantic_cloud.astype(
        np.float32)
    pc_msg.data = full_data.tobytes()
    process_cloud_node_object.accumulated_cloud_pub.publish(pc_msg)
    process_cloud_node_object.get_logger().info(
        "Accumulated semantic point cloud published",
        throttle_duration_sec=5)


def make_fields():
    # create fields for the point cloud publisher
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


# TODO(ankit): Delete after testing if not needed
# def make_fields_instance_seg():
#     # manually add fiels by Ian
#     fields = []
#     field = PointField()
#     field.name = 'x'
#     field.count = 1
#     field.offset = 0
#     field.datatype = PointField.FLOAT32
#     fields.append(field)

#     field = PointField()
#     field.name = 'y'
#     field.count = 1
#     field.offset = 4
#     field.datatype = PointField.FLOAT32
#     fields.append(field)

#     field = PointField()
#     field.name = 'z'
#     field.count = 1
#     field.offset = 8
#     field.datatype = PointField.FLOAT32
#     fields.append(field)

#     field = PointField()
#     field.name = 'intensity'
#     field.count = 1
#     field.offset = 12
#     field.datatype = PointField.FLOAT32
#     fields.append(field)

#     field = PointField()
#     field.name = 'id'
#     field.count = 1
#     field.offset = 16
#     field.datatype = PointField.FLOAT32
#     fields.append(field)


#     field = PointField()
#     field.name = 'confidence'
#     field.count = 1
#     field.offset = 20
#     field.datatype = PointField.FLOAT32
#     fields.append(field)
#     return fields


def threshold_by_range(valid_range_threshold, pc_xyzi, valid_range_threshold_lower=0.1):
    points_pano_xyz = pc_xyzi[:, :3]
    range_values = np.linalg.norm(points_pano_xyz, axis=1)
    # filter out points at 0
    valid_indices = (range_values >= valid_range_threshold_lower)
    # filter out points larger than valid_range_threshold
    valid_indices = np.logical_and(
        (range_values < valid_range_threshold), valid_indices)
    return valid_indices
