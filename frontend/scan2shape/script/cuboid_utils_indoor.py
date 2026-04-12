#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.transform import Rotation as R
import tf2_ros
from sklearn.decomposition import PCA
import open3d as o3d
import copy
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def generate_publish_instance_cloud_indoor(process_cloud_node_object, timestamp):
    instances_xyzl = []
    global_track_ids = []
    current_instance_num = -1
    safe_to_publish = False

    if len(process_cloud_node_object.all_tracks) > 0:
        for track in process_cloud_node_object.all_tracks:
            if track.age > process_cloud_node_object.tracker_age_thresh_lower:
                current_instance_num += 1
                current_xyz = track.all_raw_points
                cur_class_label = track.class_label
                instances_xyzl.append([current_xyz, cur_class_label])
                global_track_ids.append(track.track_idx)
                # just for visualization
                current_i = np.ones(
                    (track.all_raw_points.shape[0], 1)) * current_instance_num
                current_xyzi = np.hstack((current_xyz, current_i))

                if current_instance_num == 0:
                    valid_points_labels = current_xyzi
                    safe_to_publish = True
                else:
                    valid_points_labels = np.vstack(
                        (valid_points_labels, current_xyzi))

    else:
        print(
            "no valid cluster found in current accumulated point cloud!")
        return None, None

    # publish
    if safe_to_publish:
        pc_msg_2 = PointCloud2()
        # define our own header to publish in the world frame
        header = Header()
        header.stamp = timestamp  # rospy.Time.now()
        header.frame_id = process_cloud_node_object.reference_frame
        pc_msg_2.header = header
        pc_msg_2.width = valid_points_labels.shape[0]
        pc_msg_2.height = 1
        pc_msg_2.point_step = 16
        pc_msg_2.row_step = pc_msg_2.width * pc_msg_2.point_step
        pc_msg_2.fields = process_cloud_node_object.pc_fields_
        full_data_2 = valid_points_labels.astype(np.float32)
        pc_msg_2.data = full_data_2.tobytes()
        process_cloud_node_object.instance_cloud_pub.publish(pc_msg_2)

        process_cloud_node_object.get_logger().info(
            "Published segmented and accumulated instance cloud",
            throttle_duration_sec=5)

        instances_xyzl_copied = copy.deepcopy(instances_xyzl)
        global_track_ids_copied = copy.deepcopy(global_track_ids)

        return instances_xyzl_copied, global_track_ids_copied
    else:
        return None, None


def cuboid_detection_indoor(process_cloud_node_object, instances_xyzl, instance_global_ids):

    cuboids = []
    cuboid_clus_centroids = []
    assert len(instances_xyzl) == len(instance_global_ids)
    # TODO(ankit): Currently always doing PCA. Make this universal for this function or properly give an option to disable it
    do_pca = True
    for instance_id, instance_xyzl in enumerate(instances_xyzl):
        instance_xyz = instance_xyzl[0]
        cur_class_label = instance_xyzl[1]
        # find the corresponding class name, which is process_cloud_node_object.object_classes that has the value equal to cur_class_label
        for key, value in process_cloud_node_object.cls.items():
            if value == cur_class_label:
                cur_object_class = key
                break

        if do_pca:
            #################################### PCA ###################################
            pca = PCA(n_components=2)
            # Skipping PCA if less than 50 points for the instance
            if instance_xyz.shape[0] > 50:
                o_pcd = o3d.geometry.PointCloud()
                o_pcd.points = o3d.utility.Vector3dVector(instance_xyz)
                # make this try except block to avoid the error when the convex hull computation fails
                try:
                    hull_points = o_pcd.compute_convex_hull()[1]
                    instance_hull_points = instance_xyz[hull_points, :]
                except:
                    instance_hull_points = instance_xyz
            else:
                instance_hull_points = instance_xyz

            pca.fit(instance_hull_points[:, :2])

            components = pca.components_
            x = np.array([components[0, 0], components[0, 1], 0])
            x = x / np.linalg.norm(x)
            z = np.array([0, 0, 1])
            y = np.cross(z, x)
            y = y / np.linalg.norm(y)

            # get the rotation
            # use arctan2 to avoid angle ambiguity, further, we constrain the yaw to [0,pi) so that the heading of object does not jump
            yaw = np.arctan2(x[1], x[0])
            # record the raw rotation which will be used for centroid projection since it is representing the exact rotation from PCA to world frame
            raw_yaw = yaw

            if yaw < 0:
                yaw += np.pi
            if yaw == np.pi:
                yaw -= np.pi

            # rotation from PCA frame to world frame -- will be used to find centroid as well
            r_pca_world_raw = R.from_rotvec(raw_yaw * z)

            x_projections = instance_xyz @ x
            max_x = np.max(x_projections)
            min_x = np.min(x_projections)
            length = max_x - min_x

            y_projections = instance_xyz @ y
            max_y = np.max(y_projections)
            min_y = np.min(y_projections)
            width = max_y - min_y

            z_projections = instance_xyz @ z
            max_z = np.max(z_projections)
            min_z = np.min(z_projections)
            height = max_z - min_z

            # Calculate the centroid found based on body frame coords
            x_centroid_pca = 0.5*(max_x + min_x)
            y_centroid_pca = 0.5*(max_y + min_y)
            z_centroid_pca = 0.5*(max_z + min_z)
            centroid_pca = np.array(
                [x_centroid_pca, y_centroid_pca, z_centroid_pca])
            centroid_world = r_pca_world_raw .as_matrix() @ centroid_pca

            r_pca_world = R.from_rotvec(yaw * z)
            orient_world = r_pca_world.as_quat()
            #################################### PCA ###################################

        else:
            # Not using PCA
            x = np.array([1, 0, 0])
            z = np.array([0, 0, 1])
            y = np.array([0, 1, 0])

            # use arctan2 to avoid angle ambiguity, further, we constrain the yaw to [0,pi) so that the heading of car does not jump
            yaw = np.arctan2(x[1], x[0])
            # record the raw rotation which will be used for centroid projection since it is representing the exact rotation from PCA to world frame
            raw_yaw = yaw

            if yaw < 0:
                yaw += np.pi
            if yaw == np.pi:
                yaw -= np.pi

            # take the max - min along x axis
            length = np.max(instance_xyz[:, 0]) - np.min(instance_xyz[:, 0])
            # take the max - min along y axis
            width = np.max(instance_xyz[:, 1]) - np.min(instance_xyz[:, 1])
            # take the max - min along z axis
            height = np.max(instance_xyz[:, 2]) - np.min(instance_xyz[:, 2])

            # Calculate the centroid
            x_centroid = np.median(instance_xyz[:, 0])
            y_centroid = np.median(instance_xyz[:, 1])
            z_centroid = np.median(instance_xyz[:, 2])
            centroid_world = np.array([x_centroid, y_centroid, z_centroid])
            orient_world = [0, 0, 0, 1]

        flag = False
        # TODO(ankit): Currently only comparing length and not width. Now that PCA is used, add width comparison as well
        flag = (length > process_cloud_node_object.length_cutoffs[cur_object_class][0] and length < process_cloud_node_object.length_cutoffs[cur_object_class][1]
                and height > process_cloud_node_object.height_cutoffs[cur_object_class][0] and height < process_cloud_node_object.height_cutoffs[cur_object_class][1])
        if flag:
            current_cube = {}
            current_cube['dimensions'] = np.array([length, width, height])
            current_cube['centroid'] = centroid_world
            current_cube['orientation'] = orient_world  # unit quaternion
            current_cube['global_id'] = instance_global_ids[instance_id]
            current_cube['class_label_str'] = cur_object_class
            cuboids.append(current_cube)
            cuboid_clus_centroids.append(
                np.array([0.0, 0.0, centroid_world[2]]))
    return (cuboids, cuboid_clus_centroids)


def fit_cuboid_indoor(fit_cuboid_length_thresh, input_pc, depth_percentile, confidence_threshold):
    # input_pc: x, y, z, intensity, instance_id, confidence, depth
    cloud_mat_3d = input_pc[:, :3]
    instance_ids = input_pc[:, 4]
    confidence_values = input_pc[:, 5]
    depth_values = input_pc[:, 6]
    unique_labels = np.unique(instance_ids)
    xcs = []
    ycs = []
    lengths = []
    widths = []
    raw_points = []
    for k in unique_labels:
        if k == 0:
            continue  # background points or other instances
        class_member_mask = (instance_ids == k)
        if np.sum(class_member_mask) > 0:
            cur_depth_values = depth_values[class_member_mask]
            depth_thres_low = np.percentile(
                cur_depth_values, depth_percentile[0])
            depth_thres_high = np.percentile(
                cur_depth_values, depth_percentile[1])
            confidence = np.median(confidence_values[class_member_mask])
            if confidence > confidence_threshold:
                valid_pts_mask = np.logical_and(
                    (cur_depth_values > depth_thres_low), (cur_depth_values < depth_thres_high))
                if np.sum(valid_pts_mask) > 0:
                    xyzs_instance = cloud_mat_3d[class_member_mask, :]
                    xyzs = xyzs_instance[valid_pts_mask, :]
                    xyzs_valid = xyzs
                    xs = xyzs_valid[:, 0]
                    ys = xyzs_valid[:, 1]
                    x_max = np.max(xs)
                    y_max = np.max(ys)
                    x_min = np.min(xs)
                    y_min = np.min(ys)
                    xc = np.median(xs)
                    yc = np.median(ys)
                    length = x_max - x_min
                    width = y_max - y_min
                    if length > fit_cuboid_length_thresh:
                        xcs.append(xc)
                        ycs.append(yc)
                        lengths.append(length)
                        widths.append(width)
                        raw_points.append(xyzs_valid)

    assert (len(xcs) == len(ycs) == len(lengths)
            == len(widths) == len(raw_points))
    return xcs, ycs, lengths, widths, raw_points


def cluster_indoor(xyzi, epsilon, min_samples, use_2d, cpu_thread_clutering_=2):
    # reduce to 2d
    if use_2d:
        cloud = xyzi[:, :2]
    else:
        cloud = xyzi[:, :3]
    # ref[https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html#sklearn.cluster.DBSCAN]
    object_clusters = DBSCAN(eps=epsilon, min_samples=min_samples, metric='euclidean', metric_params=None,
                             algorithm='auto', leaf_size=30, p=None, n_jobs=cpu_thread_clutering_).fit(cloud)
    labels = object_clusters.labels_
    # components = object_clusters.components_
    # core_sample_indices = object_clusters.core_sample_indices_

    return labels


def publish_cuboid_and_range_bearing_measurements_final(process_cloud_node_object, cuboids, cuboid_clus_labels, current_raw_timestamp, synced_odom=None):
    car_markers = MarkerArray()
    car_markers.markers = []
    car_markers_body = MarkerArray()
    car_markers_body.markers = []
    stamp = current_raw_timestamp

    H_body_world = np.zeros((4, 4), dtype=np.float32)
    try:
        # transform data in the source_frame into the target_frame
        tf_msg = process_cloud_node_object.tf_buffer.lookup_transform(
            process_cloud_node_object.reference_frame, process_cloud_node_object.range_image_frame, current_raw_timestamp)
        t_body_world = (tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z)
        quat_body_world = (tf_msg.transform.rotation.x,
                           tf_msg.transform.rotation.y,
                           tf_msg.transform.rotation.z,
                           tf_msg.transform.rotation.w)
        r_body_world = R.from_quat(quat_body_world)
        H_body_world_rot = r_body_world.as_matrix()
        H_body_world_trans = np.array(t_body_world)
        H_body_world[:3, :3] = H_body_world_rot
        H_body_world[:3, 3] = H_body_world_trans
        # body to world transformation
        H_body_world[3, 3] = 1

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        process_cloud_node_object.get_logger().warn(
            "Cannot find TF from " + process_cloud_node_object.range_image_frame +
            " to " + process_cloud_node_object.reference_frame)
        return

    for idx, cuboid in enumerate(cuboids):
        marker = Marker()
        marker_body = Marker()
        marker.header.frame_id = process_cloud_node_object.reference_frame
        marker.header.stamp = stamp
        if cuboid_clus_labels is not None:
            marker.ns = cuboid['class_label_str'] + \
                "_" + str(cuboid_clus_labels[idx] + 1)
        else:
            marker.ns = cuboid['class_label_str']
        marker.id = idx
        cad_model_exists = False

        if process_cloud_node_object.class_model_path[cuboid['class_label_str']] is not None:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = process_cloud_node_object.class_model_path[
                cuboid['class_label_str']]
            cad_model_exists = True
        else:
            marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker_body = Marker()
        marker_body.header.frame_id = process_cloud_node_object.range_image_frame
        marker_body.header.stamp = stamp
        marker_body.ns = cuboid['class_label_str']
        marker_body.id = idx
        marker_body.type = Marker.SPHERE
        marker_body.action = Marker.ADD

        cuboid_center_homg = np.ones((4,), dtype=float)
        cuboid_center_homg[:3] = cuboid['centroid']
        cuboid_center_body_homg = np.linalg.pinv(
            H_body_world) @ cuboid_center_homg
        cuboid_center_body = cuboid_center_body_homg[:3] / \
            cuboid_center_body_homg[3]

        marker.pose.position.x = cuboid['centroid'][0]
        marker.pose.position.y = cuboid['centroid'][1]
        marker.pose.position.z = cuboid['centroid'][2]

        marker_body.pose.position.x = cuboid_center_body[0]
        marker_body.pose.position.y = cuboid_center_body[1]
        marker_body.pose.position.z = cuboid_center_body[2]

        if process_cloud_node_object.color_by_floors:
            for floor_name, floor_height_thresh in process_cloud_node_object.floor_height_thresh.items():
                if floor_height_thresh[0] <= cuboid['centroid'][2] <= floor_height_thresh[1]:
                    marker.color.a = 0.8  # alpha = 1 means not transparent at all
                    marker.color.r = process_cloud_node_object.floor_color[floor_name][0]
                    marker.color.g = process_cloud_node_object.floor_color[floor_name][1]
                    marker.color.b = process_cloud_node_object.floor_color[floor_name][2]

                    marker_body.color.a = 0.6  # alpha = 1 means not transparent at all
                    marker_body.color.r = process_cloud_node_object.floor_color[floor_name][0]
                    marker_body.color.g = process_cloud_node_object.floor_color[floor_name][1]
                    marker_body.color.b = process_cloud_node_object.floor_color[floor_name][2]

                    marker_colored = True

            if not marker_colored:
                marker.color.a = 0.8
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker_body.color.a = 0.6
                marker_body.color.r = 0.0
                marker_body.color.g = 0.0
                marker_body.color.b = 0.0
        else:
            marker.color.a = 1.0  # alpha = 1 means not transparent at all
            marker.color.r = process_cloud_node_object.class_color[cuboid['class_label_str']][0]
            marker.color.g = process_cloud_node_object.class_color[cuboid['class_label_str']][1]
            marker.color.b = process_cloud_node_object.class_color[cuboid['class_label_str']][2]

            marker_body.color.a = 0.6  # alpha = 1 means not transparent at all
            marker_body.color.r = process_cloud_node_object.class_color[
                cuboid['class_label_str']][0]
            marker_body.color.g = process_cloud_node_object.class_color[
                cuboid['class_label_str']][1]
            marker_body.color.b = process_cloud_node_object.class_color[
                cuboid['class_label_str']][2]

        if cad_model_exists:
            # Hardcoded dimensions for the cad models
            fixed_dim = 0.6
            marker.scale.x = fixed_dim * \
                process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
            marker.scale.y = fixed_dim * \
                process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
            marker.scale.z = fixed_dim * \
                process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
        else:
            marker.scale.x = cuboid['dimensions'][0]
            marker.scale.y = cuboid['dimensions'][1]
            marker.scale.z = cuboid['dimensions'][2]

        marker_body.scale.x = cuboid['dimensions'][0]
        marker_body.scale.y = cuboid['dimensions'][0]
        marker_body.scale.z = cuboid['dimensions'][2]

        marker.pose.orientation.x = cuboid['orientation'][0]
        marker.pose.orientation.y = cuboid['orientation'][1]
        marker.pose.orientation.z = cuboid['orientation'][2]
        marker.pose.orientation.w = cuboid['orientation'][3]

        r_cube_world = R.from_quat(cuboid['orientation'])
        H_cube_world_rot = r_cube_world.as_matrix()
        H_cube_body_rot = H_body_world_rot.T @ H_cube_world_rot
        quat_body_cube = R.from_matrix(H_cube_body_rot).as_quat()
        marker_body.pose.orientation.x = quat_body_cube[0]
        marker_body.pose.orientation.y = quat_body_cube[1]
        marker_body.pose.orientation.z = quat_body_cube[2]
        marker_body.pose.orientation.w = quat_body_cube[3]

        car_markers.markers.append(marker)
        car_markers_body.markers.append(marker_body)

        # Add text to show labels to the marker
        text_marker = Marker()
        text_marker.header.frame_id = process_cloud_node_object.reference_frame
        text_marker.header.stamp = stamp
        text_marker.ns = "text"
        text_marker.id = idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = cuboid['centroid'][0]
        text_marker.pose.position.y = cuboid['centroid'][1]
        text_marker.pose.position.z = cuboid['centroid'][2] + 0.75
        text_marker.pose.orientation.w = 1.0
        text_marker.text = cuboid['class_label_str']
        text_marker.scale.z = 0.5
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        car_markers.markers.append(text_marker)

    process_cloud_node_object.cuboid_marker_pub.publish(car_markers)
    process_cloud_node_object.cuboid_marker_body_pub.publish(car_markers_body)
