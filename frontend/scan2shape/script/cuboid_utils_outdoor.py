#!/usr/bin/env python3
# title         :
# description   :
# author        :Xu Liu  and Ankit Prabhu

import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.transform import Rotation as R
import tf2_ros
from sklearn.decomposition import PCA
import open3d as o3d
import copy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sklearn.cluster import KMeans


def generate_publish_instance_cloud(process_cloud_node_object, timestamp):
    instances_xyzl = []
    current_inst_label = -1
    safe_to_publish = False

    if len(process_cloud_node_object.all_tracks) > 0:
        for track in process_cloud_node_object.all_tracks:
            if track.age > process_cloud_node_object.cuboid_track_age_threshold_per_cls_label[track.class_label]:
                current_inst_label += 1
                current_xyz = track.all_raw_points
                cur_class_label = track.class_label
                current_i = np.ones(
                    (track.all_raw_points.shape[0], 1)) * current_inst_label
                # This point cloud (current_xyzi) is used to visualize the instance cloud
                # The intensity is used to store the label of the instance. This is not the intensity of the point cloud or the class label.
                current_xyzi = np.hstack((current_xyz, current_i))
                instances_xyzl.append([current_xyz, cur_class_label])
                if current_inst_label == 0:
                    valid_points_labels = current_xyzi
                    safe_to_publish = True
                else:
                    valid_points_labels = np.vstack(
                        (valid_points_labels, current_xyzi))

    else:
        process_cloud_node_object.get_logger().info(
            "No valid class object found in segmented accumulated point cloud. If this is unexpected, debug the tracking pipeline.",
            throttle_duration_sec=5)
        return None

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
        pc_msg_2.point_step = process_cloud_node_object.pc_point_step
        pc_msg_2.row_step = pc_msg_2.width * pc_msg_2.point_step
        pc_msg_2.fields = process_cloud_node_object.pc_fields_
        full_data_2 = valid_points_labels.astype(np.float32)
        pc_msg_2.data = full_data_2.tobytes()
        process_cloud_node_object.instance_cloud_pub.publish(pc_msg_2)

        process_cloud_node_object.get_logger().info(
            "Published segmented and accumulated instance cloud",
            throttle_duration_sec=5)

        return copy.deepcopy(instances_xyzl)


def cuboid_detection(process_cloud_node_object, instances_xyzl, current_raw_timestamp, use_convex=True):
    # TODO(ankit): project to ground plane, instead of project to XY plane
    cuboids = []
    for instance_xyzl in instances_xyzl:
        instance_xyz = instance_xyzl[0]
        cur_class_label = instance_xyzl[1]
        pca = PCA(n_components=2)
        # Using convex hull to get the hull points and then fit the PCA
        if use_convex:
            o_pcd = o3d.geometry.PointCloud()
            o_pcd.points = o3d.utility.Vector3dVector(instance_xyz)
            try:
                hull_points = o_pcd.compute_convex_hull()[1]
                instance_hull_points = instance_xyz[hull_points, :]
            except:
                instance_hull_points = instance_xyz
            pca.fit(instance_hull_points[:, :2])
        else:
            pca.fit(instance_xyz[:, :2])

        components = pca.components_
        x = np.array([components[0, 0], components[0, 1], 0])
        x = x / np.linalg.norm(x)
        z = np.array([0, 0, 1])
        y = np.cross(z, x)
        y = y / np.linalg.norm(y)
        # use arctan2 to avoid angle ambiguity, further, we constrain the yaw to [0,pi) so that the heading of cuboid does not jump
        yaw = np.arctan2(x[1], x[0])
        # record the raw rotation which will be used for centroid projection since it is representing the exact rotation from PCA to world frame
        raw_yaw = yaw

        if yaw < 0:
            yaw += np.pi
        if yaw == np.pi:
            yaw -= np.pi

        # rotation from PCA frame to world frame -- will be used to find centroid as well
        r_pca_world_raw = R.from_rotvec(raw_yaw * z)

        # Calculating the length, width, height of the cuboid
        # TODO(ankit): check if changing the percentile values of width also affects the cuboid detection
        x_projections = instance_xyz @ x
        length = np.percentile(x_projections, 99) - \
            np.percentile(x_projections, 1)
        y_projections = instance_xyz @ y
        width = np.percentile(y_projections, 99) - \
            np.percentile(y_projections, 1)
        z_projections = instance_xyz @ z
        height = np.percentile(z_projections, 99) - \
            np.percentile(z_projections, 1)

        # estimate the heading direction for particularly car class
        if process_cloud_node_object.estimate_facing_dir_car:
            process_cloud_node_object.get_logger().warn(
                "Estimating facing direction of the car, disable the flag if there is weird cuboid behavior",
                throttle_duration_sec=10)
            # Estimate the facing direction of the car based on the height of the front and rear part of the car
            # The logic is that the back of the car should have a median height higher than the front of the car
            rear_cut_off = np.percentile(x_projections, 5)  # 5 percentile
            front_cut_off = np.percentile(x_projections, 95)  # 95 percentile
            idx_rear = x_projections <= rear_cut_off
            idx_front = x_projections >= front_cut_off

            front_part_height = z_projections[idx_front]
            rear_part_height = z_projections[idx_rear]
            # highest point, take 75 percentile to be robust to the noise
            front_median_height = np.percentile(front_part_height, 70)
            rear_median_height = np.percentile(rear_part_height, 70)

        # Calculate the centroid found based on body frame coords
        x_centroid_pca = 0.5 * \
            (np.percentile(x_projections, 99) + np.percentile(x_projections, 1))
        y_centroid_pca = 0.5 * \
            (np.percentile(y_projections, 99) + np.percentile(y_projections, 1))
        z_centroid_pca = 0.5 * \
            (np.percentile(z_projections, 99) + np.percentile(z_projections, 1))
        centroid_pca = np.array(
            [x_centroid_pca, y_centroid_pca, z_centroid_pca])

        centroid_world = r_pca_world_raw .as_matrix() @ centroid_pca

        flag = False
        flag = (width > process_cloud_node_object.cuboid_width_cutoff_per_cls_label[cur_class_label][0] and width < process_cloud_node_object.cuboid_width_cutoff_per_cls_label[cur_class_label][1] and length > process_cloud_node_object.cuboid_length_cutoff_per_cls_label[cur_class_label][0]
                and length < process_cloud_node_object.cuboid_length_cutoff_per_cls_label[cur_class_label][1] and height > process_cloud_node_object.cuboid_height_cutoff_per_cls_label[cur_class_label][0] and height < process_cloud_node_object.cuboid_height_cutoff_per_cls_label[cur_class_label][1])
        if flag:
            if process_cloud_node_object.estimate_facing_dir_car:
                if rear_median_height < front_median_height:
                    # flip the orientation since front of the car should have less points than back
                    yaw = yaw+np.pi

            r_pca_world = R.from_rotvec(yaw * z)
            quaternion = r_pca_world.as_quat()

            current_cuboid = {}
            current_cuboid['dimensions'] = np.array([length, width, height])
            current_cuboid['centroid'] = centroid_world
            current_cuboid['orientation'] = quaternion
            current_cuboid['class_name'] = process_cloud_node_object.cls_label_to_name[cur_class_label]
            cuboids.append(current_cuboid)

    return cuboids


def fit_cuboid(fit_cuboid_dim_thresh, cloud_mat_3d, labels):
    unique_labels = np.unique(labels)
    # storing centroid coordinates and dimension of each cuboid
    xcs = []
    ycs = []
    # here we derive length from x values and width from y values
    lengths = []
    widths = []
    raw_points = []
    for k in unique_labels:
        if k == -1:
            continue  # sanity check to filter out noise points
        class_member_mask = (labels == k)
        xyzs = cloud_mat_3d[class_member_mask, :]
        x_max = np.max(xyzs[:, 0])
        y_max = np.max(xyzs[:, 1])
        x_min = np.min(xyzs[:, 0])
        y_min = np.min(xyzs[:, 1])
        xc = 0.5*(x_max + x_min)
        yc = 0.5*(y_max + y_min)
        length = x_max - x_min
        width = y_max - y_min
        min_dim = np.min([length, width])
        if min_dim > fit_cuboid_dim_thresh:
            xcs.append(xc)
            ycs.append(yc)
            lengths.append(length)
            widths.append(width)
            raw_points.append(xyzs)
        else:
            # Free function — no node logger available; using print fallback.
            print(
                f"[fit_cuboid] Discarding current cuboid from tracking process because "
                f"smallest dim ({min_dim}) is less than threshold ({fit_cuboid_dim_thresh})")

    return xcs, ycs, lengths, widths, raw_points


def cluster_cuboid_orientation(cuboids):
    # set cuboid angle to be -45 to 135 degrees (avoid jumping for cuboids that are near 0 or 180 degrees)
    all_yaws = []
    all_rolls = []
    all_pitches = []
    # first convert quaternion to euler angle
    for cuboid in cuboids:
        r = R.from_quat(cuboid['orientation'])
        # get the yaw
        yaw, pitch, roll = r.as_euler('zyx', degrees=False)
        # set yaw to be -45 to 135 degrees
        if yaw < -np.pi/4:
            yaw += np.pi
        if yaw > np.pi*3/4:
            yaw -= np.pi
        all_yaws.append(yaw)
        all_rolls.append(roll)
        all_pitches.append(pitch)

    assert len(all_yaws) == len(cuboids)

    # cluster the yaw using K means
    # Current logic only works for 2 clusters
    # TODO(ankit): Make this more generic
    number_of_yaws_allowed = 2
    if len(all_yaws) > number_of_yaws_allowed:
        # TODO(ankit): change the kernel of Kmeans to be cosine similarity, because, for example, 0 and pi are close
        kmeans = KMeans(n_clusters=number_of_yaws_allowed, random_state=0, n_init=10).fit(
            np.array(all_yaws).reshape(-1, 1))
        # get the cluster centers, set all yaw to be the closest cluster center
        cluster_centers = kmeans.cluster_centers_
        # get the labels
        labels = kmeans.labels_
        # get the size for each cluster
        cluster_sizes = []
        for i in range(number_of_yaws_allowed):
            cluster_sizes.append(np.sum(labels == i))

        # check the distance between each pair of cluster centers, if it is less than 45 degress, merge the two clusters (weighted by the size of the cluster)
        new_cluster_centers = []
        if np.abs(cluster_centers[0] - cluster_centers[1]) < np.pi/4:
            # merge the two clusters
            new_center = (cluster_centers[0]*cluster_sizes[0] + cluster_centers[1]
                          * cluster_sizes[1])/(cluster_sizes[0] + cluster_sizes[1])
            cluster_sizes = [cluster_sizes[0] + cluster_sizes[1]]
            new_cluster_centers.append(new_center)
        else:
            new_cluster_centers = cluster_centers

        # OLD LOGIC
        # check the distance between each pair of cluster centers, if it is less than 45 degress, merge the two clusters (weighted by the size of the cluster)
        # for cur_id, cur_center in enumerate(cluster_centers):
        #     new_center = cur_center
        #     for other_id, other_center in enumerate(cluster_centers):
        #         if np.abs(cur_center - other_center) < np.pi/4:
        #             # merge the two clusters
        #             new_center = (cluster_centers[cur_id]*cluster_sizes[cur_id] + cluster_centers[other_id]*cluster_sizes[other_id])/(cluster_sizes[cur_id] + cluster_sizes[other_id])
        #     new_cluster_centers.append(new_center)

        # get the largest cluster
        largest_cluster = np.argmax(cluster_sizes)
        # get the center of the largest cluster
        largest_cluster_center = new_cluster_centers[largest_cluster]
        if isinstance(largest_cluster_center, np.ndarray):
            largest_cluster_center = largest_cluster_center.item()
        # get the orthogonal cluster center (90 degrees away from the largest cluster center) that lies inside the range of [-45, 135]
        orthogonal_cluster_center = largest_cluster_center + np.pi/2
        if orthogonal_cluster_center > np.pi*3/4:
            orthogonal_cluster_center -= np.pi

        cluster_centers = [largest_cluster_center, orthogonal_cluster_center]

        for id, cuboid in enumerate(cuboids):
            r = R.from_quat(cuboid['orientation'])
            # get the yaw
            yaw = all_yaws[id]
            angle_diff_abs = np.abs(cluster_centers - yaw)
            # cast the larger than 90 degrees to be pi - angle, since for example, 0 degrees is closer to 180 degrees than 90 degrees if we do not specify the heading of the car
            angle_diff_abs[angle_diff_abs > np.pi/2] = np.pi - \
                angle_diff_abs[angle_diff_abs > np.pi/2]
            closest_cluster_center = np.argmin(angle_diff_abs)

            # sanity check: assert the diff is less than 45 degrees
            assert angle_diff_abs[closest_cluster_center] < np.pi/4

            # set the yaw to be the closest cluster center
            cuboid['orientation'] = R.from_euler(
                'zyx', [cluster_centers[closest_cluster_center], all_pitches[id], all_rolls[id]], degrees=False).as_quat()
            # print setting orientation of the cuboid

        return cuboids
    else:
        # Free function — no node logger available; using print fallback.
        print("[cluster_cuboid_orientation] Not enough cuboids to fix cuboid yaw orientation")
        return cuboids


def publish_cuboid_markers(process_cloud_node_object, cuboids, current_raw_timestamp):

    # Cluster and fix cuboid orientation for partially fitted cuboids.
    # For example sometimes the complete cuboid of an object cannot be fit due to missing segmentation points.
    # In such cases, the orientation of the cuboid is not accurate.
    # This function clusters the orientation of the cuboids and sets the orientation of the cuboids to the cluster center.
    # In this way, the orientation of the cuboids is consistent across all cuboids.
    if process_cloud_node_object.cluster_and_fix_cuboid_orientation == True:
        process_cloud_node_object.get_logger().info(
            "Clustering and fixing cuboid orientation. Disable this flag if there is weird cuboid orientation behavior.",
            throttle_duration_sec=5)
        cuboids = cluster_cuboid_orientation(cuboids)

    cuboid_markers = MarkerArray()
    cuboid_markers.markers = []
    cuboid_markers_body = MarkerArray()
    cuboid_markers_body.markers = []

    # Used for deleting markers which are no longer used
    if process_cloud_node_object.track_always_visualize == False:
        marker_del_arr = MarkerArray()
        marker_del = Marker()
        marker_del.header.frame_id = process_cloud_node_object.reference_frame
        marker_del.header.stamp = current_raw_timestamp
        marker_del.action = Marker.DELETEALL
        marker_del_arr.markers.append(marker_del)
        process_cloud_node_object.cuboid_marker_pub.publish(marker_del_arr)

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
            "Cannot find tf from " + process_cloud_node_object.range_image_frame +
            " to " + process_cloud_node_object.reference_frame)
        return

    for idx, cuboid in enumerate(cuboids):
        # Publish cuboid markers both in world and body frames
        marker = Marker()
        marker_body = Marker()
        marker.header.frame_id = process_cloud_node_object.reference_frame
        marker.header.stamp = current_raw_timestamp
        marker.ns = cuboid['class_name']
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker_body = Marker()
        marker_body.header.frame_id = process_cloud_node_object.range_image_frame
        marker_body.header.stamp = current_raw_timestamp
        marker_body.ns = cuboid['class_name'] + "_body"
        marker_body.id = idx
        marker_body.type = Marker.CUBE
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

        marker.scale.x = cuboid['dimensions'][0]
        marker.scale.y = cuboid['dimensions'][1]
        marker.scale.z = cuboid['dimensions'][2]
        marker_body.scale.x = cuboid['dimensions'][0]
        marker_body.scale.y = cuboid['dimensions'][1]
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

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        cuboid_markers.markers.append(marker)

        marker_body.color.a = 0.7
        marker_body.color.r = 0.0
        marker_body.color.g = 0.0
        marker_body.color.b = 1.0
        cuboid_markers_body.markers.append(marker_body)

        # Add text to show class names when visualizing cuboids
        text_marker = Marker()
        text_marker.header.frame_id = process_cloud_node_object.reference_frame
        text_marker.header.stamp = current_raw_timestamp
        text_marker.ns = "text"
        text_marker.id = idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = cuboid['centroid'][0]
        text_marker.pose.position.y = cuboid['centroid'][1]
        text_marker.pose.position.z = cuboid['centroid'][2] + 0.75
        text_marker.pose.orientation.w = 1.0
        text_marker.text = cuboid['class_name']
        text_marker.scale.z = 0.8
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        cuboid_markers.markers.append(text_marker)

    process_cloud_node_object.cuboid_marker_pub.publish(cuboid_markers)
    process_cloud_node_object.cuboid_marker_body_pub.publish(
        cuboid_markers_body)

    process_cloud_node_object.get_logger().info(
        "Published final cuboid markers...",
        throttle_duration_sec=5)
