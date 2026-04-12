#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from assignment import hungarian_assignment
from object_tracker import ObjectTrack


# global variable
track_id = 0


def calculate_cost_matrix(all_objects, cur_objects):
    all_objects_arr = np.asarray(all_objects)
    n1 = all_objects_arr.shape[0]
    n2 = cur_objects.shape[0]
    cost = np.zeros((n1, n2))
    for i2 in np.arange(n2):
        # use xy distance
        distances = np.linalg.norm(
            all_objects_arr[:, :2] - cur_objects[i2, :2], axis=1)
        cost[:, i2] = distances
    return cost


def object_assignment(all_objects, cur_objects, assignment_threshold):
    # cost is just the distance between the centroids of all_objects and cur_objects
    cost = calculate_cost_matrix(all_objects, cur_objects)
    match_inds, lost_inds, new_inds = hungarian_assignment(
        cost, unassigned_cost=assignment_threshold)

    return match_inds, lost_inds, new_inds

# TODO(ankit): Unify all track_objects functions from below
def track_objects_final(process_cloud_node_obj, cur_class_label, cur_objects, cur_objects_raw_points, all_objects, all_tracks, scan_idx, downsample_res=0.3, num_instance_point_lim=50000):

    global track_id
    if scan_idx == 0 or len(all_objects) == 0:
        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age, cur_class_label]))
        return all_objects, all_tracks

    else:
        # length of all_tracks and all_objects should match, keeping all_objects is just for vectorization to speed up
        if len(all_tracks) != len(all_objects):
            raise Exception(
                "Length of all_tracks and all_objects does not match!!")

        # record all objects that have class label as cur_class_label
        # record the index match from all_objects_cur_class to all_objects
        all_objects_cur_class = []
        all_objects_cur_class_to_original_list_match = {}
        for i, object in enumerate(all_objects):
            if object[5] == cur_class_label:
                all_objects_cur_class.append(object)
                index_in_cur_class = len(all_objects_cur_class) - 1
                all_objects_cur_class_to_original_list_match[index_in_cur_class] = i

        if len(all_objects_cur_class) == 0:
            # everything should be new inds and no match inds
            match_ind_sets = []
            lost_inds = []
            new_inds = np.arange(len(cur_objects))
        else:
            match_ind_sets, lost_inds, new_inds = object_assignment(
                all_objects_cur_class, cur_objects, process_cloud_node_obj.cuboid_assignment_threshold_per_cls_label[cur_class_label])

        # matched tracks
        for match in match_ind_sets:
            i_track, i_cur_match = match
            # original index in all_objects
            i_track_original = all_objects_cur_class_to_original_list_match[i_track]
            object_track = all_tracks[i_track_original]  # all_tracks[i_track]
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            object_track.update(x_new, y_new, l_new, w_new,
                                cur_object_raw_points_new, scan_idx)

            all_objects[i_track_original] = np.array(
                [object_track.x, object_track.y, object_track.l, object_track.w, object_track.age, object_track.class_label])

        # new tracks
        for i_cur_new in new_inds:
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [x_new, y_new, l_new, w_new, object_track.age, object_track.class_label]))

        return all_objects, all_tracks


def track_objects_indoor(process_cloud_node_obj, cur_class_label, cur_obj_class, cur_objects, all_objects, all_tracks, scan_idx, cur_objects_raw_points, downsample_res=0.3, num_instance_point_lim=50000):

    global track_id
    if scan_idx == 0 or len(all_objects) == 0:

        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age, cur_class_label]))
        return all_objects, all_tracks

    else:
        # length of all_tracks and all_objects should match, keeping all_objects is just for vectorization to speed up
        if len(all_tracks) != len(all_objects):
            raise Exception(
                "Length of all_tracks and all_objects does not match!!")

        # record all objects that have class label as cur_class_label
        # record the index match from all_objects_cur_class to all_objects
        all_objects_cur_class = []
        all_objects_cur_class_to_original_list_match = {}
        for i, object in enumerate(all_objects):
            if object[5] == cur_class_label:
                all_objects_cur_class.append(object)
                index_in_cur_class = len(all_objects_cur_class) - 1
                all_objects_cur_class_to_original_list_match[index_in_cur_class] = i

        if len(all_objects_cur_class) == 0:
            # everything should be new inds and no match inds
            match_ind_sets = []
            lost_inds = []
            new_inds = np.arange(len(cur_objects))
        else:
            match_ind_sets, lost_inds, new_inds = object_assignment(
                all_objects_cur_class, cur_objects, process_cloud_node_obj.class_assignment_thresh[cur_obj_class])

        # matched tracks
        for match in match_ind_sets:
            i_track, i_cur_match = match
            # original index in all_objects
            i_track_original = all_objects_cur_class_to_original_list_match[i_track]
            object_track = all_tracks[i_track_original]  # all_tracks[i_track]
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            object_track.update(x_new, y_new, l_new, w_new,
                                cur_object_raw_points_new, scan_idx)

            all_objects[i_track_original] = np.array(
                [object_track.x, object_track.y, object_track.l, object_track.w, object_track.age, object_track.class_label])

        # new tracks
        for i_cur_new in new_inds:
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [x_new, y_new, l_new, w_new, object_track.age, object_track.class_label]))

        return all_objects, all_tracks


def track_objects(cur_objects, all_objects, all_tracks, scan_idx, cur_objects_raw_points, assignment_threshold=1.0, downsample_res=0.3, num_instance_point_lim=50000):

    global track_id
    if scan_idx == 0 or len(all_objects) == 0:

        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, num_instance_point_lim)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age]))
        return all_objects, all_tracks

    else:
        # length of all_tracks and all_objects should match, keeping all_objects is just for vectorization to speed up
        if len(all_tracks) != len(all_objects):
            raise Exception(
                "Length of all_tracks and all_objects does not match!!")

        match_ind_sets, lost_inds, new_inds = object_assignment(
            all_objects, cur_objects, assignment_threshold)

        # matched tracks
        for match in match_ind_sets:
            i_track, i_cur_match = match
            object_track = all_tracks[i_track]
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            object_track.update(x_new, y_new, l_new, w_new,
                                cur_object_raw_points_new, scan_idx)
            all_objects[i_track] = np.array(
                [object_track.x, object_track.y, object_track.l, object_track.w, object_track.age])

        # new tracks
        for i_cur_new in new_inds:
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(
                np.array([x_new, y_new, l_new, w_new, object_track.age]))

        return all_objects, all_tracks


def publish_markers(process_cloud_node, all_tracks, cur_cls_name=None, height=1.0, cov_height=0, age_threshold=6):
    # Publish object as a marker in rviz
    object_markers = MarkerArray()
    object_markers.markers = []

    # Publish object location covariance as a marker in rviz
    cov_markers = MarkerArray()
    cov_markers.markers = []

    largest_age = 10
    for i, track in enumerate(all_tracks):
        if track.age < age_threshold:
            continue
        if track.age > largest_age:
            largest_age = track.age
        marker = Marker()
        marker.header.frame_id = "quadrotor/map"
        marker.header.stamp = process_cloud_node.get_clock().now().to_msg()
        if cur_cls_name is None:
            marker.ns = "track_centers"
        else:
            marker.ns = cur_cls_name

        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = track.x
        marker.pose.position.y = track.y
        marker.pose.position.z = height

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2*height

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.5  # alpha = 1 means not transparent at all

        marker.color.r = 0
        # Making older markers more green
        marker.color.g = max(0.3, track.age / largest_age)
        marker.color.b = 0

        object_markers.markers.append(marker)

        # TODO(ankit): Do we really need covariance markers?
        # publish covariance as well
        # (eigValues, eigVectors) = np.linalg.eig(track.xy_cov)
        # # 95% confidence level
        # s = 5.991
        # lengthMajor = 2 * np.sqrt(eigValues[0] * s)
        # lengthMinor = 2 * np.sqrt(eigValues[1] * s)
        # angle = np.arctan2(eigVectors[1, 0], eigVectors[0, 0])

        # cov_marker = Marker()
        # cov_marker.header.frame_id = "quadrotor/map"
        # cov_marker.header.stamp = self.get_clock().now().to_msg()
        # cov_marker.ns = "covariance"
        # cov_marker.id = i
        # cov_marker.type = Marker.SPHERE
        # cov_marker.action = Marker.ADD
        # cov_marker.pose.position.x = track.x
        # cov_marker.pose.position.y = track.y
        # cov_marker.pose.position.z = cov_height

        # cov_marker.pose.orientation.x = 0.0
        # cov_marker.pose.orientation.y = 0.0
        # cov_marker.pose.orientation.z = np.sin(angle*0.5)
        # cov_marker.pose.orientation.w = np.cos(angle*0.5)

        # cov_marker.scale.x = lengthMajor
        # cov_marker.scale.y = lengthMinor
        # cov_marker.scale.z = 0.001

        # cov_marker.color.a = 0.75
        # cov_marker.color.r = 1.0
        # cov_marker.color.g = 0.0
        # cov_marker.color.b = 1.0
        # cov_markers.markers.append(cov_marker)

    process_cloud_node.cuboid_center_marker_pub.publish(object_markers)
    process_cloud_node.cuboid_center_cov_pub.publish(cov_markers)
