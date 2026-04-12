/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu
*
* TODO: License information
*
*/

#include <place_recognition.h>

#include <chrono>

namespace {
template <typename ParamT>
ParamT pr_declare_or_get(rclcpp::Node *node, const std::string &name,
                         const ParamT &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParamT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ParamT>();
}
}  // namespace

// define the constructor of the class GraphMatchNode
PlaceRecognition::PlaceRecognition(rclcpp::Node *node) : node_(node) {
  ns_prefix_ = "place_recognition";
  // create a publisher for visualizing the matching results
  viz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      ns_prefix_ + "/matching_results", 1);
  ParamInit();
}

// define the ParamInit function
void PlaceRecognition::ParamInit() {
  visualize_matching_results = pr_declare_or_get<bool>(
      node_, ns_prefix_ + "/visualize_matching_results", false);
  compute_budget_sec_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/compute_budget_sec", 5.0);
  dilation_factor_ =
      pr_declare_or_get<double>(node_, ns_prefix_ + "/dilation_factor", 1.2);
  match_xy_step_size_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/search_xy_step_size", 0.5);
  double match_yaw_half_range_degrees = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_yaw_half_range", 180.0);
  match_yaw_half_range_ = match_yaw_half_range_degrees * M_PI / 180.;
  disable_yaw_search_ = pr_declare_or_get<bool>(
      node_, ns_prefix_ + "/disable_yaw_search", false);
  double match_yaw_step_size_degrees = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/search_yaw_step_size_degrees", 2.0);
  match_yaw_angle_step_size_ = match_yaw_step_size_degrees * M_PI / 180.;
  match_threshold_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_threshold_position", 0.5);
  match_threshold_dimension_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_threshold_dimension", 1.0);
  ignore_dimension_ = pr_declare_or_get<bool>(
      node_, ns_prefix_ + "/ignore_dimension", false);
  min_loop_closure_overlap_percentage_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/min_loop_closure_overlap_percentage", 0.1);
  // min_num_inliers
  min_num_inliers_ =
      pr_declare_or_get<int>(node_, ns_prefix_ + "/min_num_inliers", 5);
  use_lsq = pr_declare_or_get<bool>(
      node_, ns_prefix_ + "/use_nonlinear_least_squares", true);
  slidematch_min_num_map_objects_to_start_ = pr_declare_or_get<int>(
      node_, ns_prefix_ + "/min_num_map_objects_to_start", 1);
  match_x_half_range_intra_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_x_half_range_intra", 5.0);
  match_y_half_range_intra_ = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_y_half_range_intra", 5.0);
  double match_yaw_half_range_degrees_intra = pr_declare_or_get<double>(
      node_, ns_prefix_ + "/match_yaw_half_range_intra", 10.0);
  match_yaw_half_range_intra_ =
      match_yaw_half_range_degrees_intra * M_PI / 180.;
  match_yaw_half_range_intra_ =
      match_yaw_half_range_degrees_intra * M_PI / 180.;
  // slidegraph namespace is ns_prefix_ + _slidegraph
  std::string slidegraph_ns = ns_prefix_ + "_slidegraph";
  slidegraph_num_inliners_ = pr_declare_or_get<int>(
      node_, slidegraph_ns + "/num_inliners_threshold", 10);
  slidegraph_matching_threshold_ = pr_declare_or_get<double>(
      node_, slidegraph_ns + "/descriptor_matching_threshold", 0.1);
  slidegraph_sigma_ =
      pr_declare_or_get<double>(node_, slidegraph_ns + "/sigma", 0.1);
  slidegraph_epsilon_ =
      pr_declare_or_get<double>(node_, slidegraph_ns + "/epsilon", 0.1);
  slidegraph_min_num_map_objects_to_start_ = pr_declare_or_get<int>(
      node_, slidegraph_ns + "/min_num_map_objects_to_start", 20);

  printParams();
}

void PlaceRecognition::printParams() {
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: visualize_matching_results: " << visualize_matching_results);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: compute_budget_sec_: " << compute_budget_sec_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: dilation_factor_: " << dilation_factor_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: match_xy_step_size_: " << match_xy_step_size_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: match_yaw_half_range_: " << match_yaw_half_range_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: disable_yaw_search_: " << disable_yaw_search_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: match_yaw_angle_step_size_: " << match_yaw_angle_step_size_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: match_threshold_: " << match_threshold_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: match_threshold_dimension_: " << match_threshold_dimension_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: ignore_dimension_: " << ignore_dimension_);
  // RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: min_loop_closure_overlap_percentage_: "
  //                 << min_loop_closure_overlap_percentage_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: min_num_inliers_: " << min_num_inliers_);
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: use_lsq: " << use_lsq);
}

// define the match_maps function
void PlaceRecognition::MatchMaps(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects, Eigen::Matrix3d &R_t_out,
    int &best_num_inliers_out,
    std::vector<Eigen::Vector4d> &map_objects_matched_out,
    std::vector<Eigen::Vector4d> &detection_objects_matched_out) {
  // Apply the sampled transformations on detection_objects
  Eigen::MatrixXd detection_object_xy_homogeneous =
      Eigen::MatrixXd::Ones(query_objects.size(), 3);
  for (int i = 0; i < query_objects.size(); i++) {
    detection_object_xy_homogeneous(i, 0) = query_objects[i][1];
    detection_object_xy_homogeneous(i, 1) = query_objects[i][2];
  }

  Eigen::MatrixXd map_objects_label_xy =
      Eigen::MatrixXd::Zero(reference_objects.size(), 7);
  for (int i = 0; i < reference_objects.size(); i++) {
    map_objects_label_xy(i, 0) = reference_objects[i][0];
    map_objects_label_xy(i, 1) = reference_objects[i][1];
    map_objects_label_xy(i, 2) = reference_objects[i][2];
    map_objects_label_xy(i, 3) = reference_objects[i][3];
    map_objects_label_xy(i, 4) = reference_objects[i][4];
    map_objects_label_xy(i, 5) = reference_objects[i][5];
    map_objects_label_xy(i, 6) = reference_objects[i][6];
  }

  // initialize the best number of inliers
  int best_num_inliers = -10000;
  // initialize the best transformation matrix
  Eigen::Matrix3d best_R_t = Eigen::Matrix3d::Identity();
  // initialize the best matched map objects
  std::vector<Eigen::Vector4d> best_matched_map_objects =
      std::vector<Eigen::Vector4d>();
  // initialize the best matched detection objects
  std::vector<Eigen::Vector4d> best_matched_detection_objects =
      std::vector<Eigen::Vector4d>();

  // TODO: break this down to small functions
  std::vector<double> yaw_candidates;
  if (disable_yaw_search_) {
    yaw_candidates.push_back(0.0);
  } else {
    for (double yaw_raw = -match_yaw_half_range_;
         yaw_raw < match_yaw_half_range_;
         yaw_raw += match_yaw_angle_step_size_) {
      double yaw = yaw_raw;
      yaw_candidates.push_back(yaw);
    }
  }

  // Anytime implementation
  // start with the center (0, 0), gradually expand the search range
  // until reaching either the max time budget or the max search range
  auto start_cpu = std::chrono::high_resolution_clock::now();
  // outer loop step size around 10 x match_xy_step_size_ to make sure we have
  // significant number of samples each time
  double outer_loop_step_size = 10 * match_xy_step_size_;
  double outer_loop_steps_double =
      std::min(match_x_half_range_, match_y_half_range_) / outer_loop_step_size;
  // get the ceiling of outer_loop_steps_double
  int outer_loop_steps = std::ceil(outer_loop_steps_double);
  // this may cause the outer loop step size to be larger than
  // outer_loop_steps_double if match_x_half_range_ != match_y_half_range_ but
  // it should not matter much since this is just for roughly determining the
  // number of steps each time
  double outer_loop_step_size_x =
      match_x_half_range_ / static_cast<double>(outer_loop_steps);
  double outer_loop_step_size_y =
      match_y_half_range_ / static_cast<double>(outer_loop_steps);
  // add sanity check: make sure outer_loop_step_size_x and
  // outer_loop_step_size_y are larger than match_xy_step_size_
  if (outer_loop_step_size_x < match_xy_step_size_ ||
      outer_loop_step_size_y < match_xy_step_size_) {
    RCLCPP_ERROR(node_->get_logger(),
        "Outer loop step size is smaller than match_xy_step_size_, this should "
        "not happen");
    return;
  }

  // Outer loop: gradually expand the search range
  for (int cur_step = 0; cur_step < outer_loop_steps; cur_step++) {
    // Anytime algorithm: check if we have exceeded the time budget
    // get the current time
    auto cur_cpu = std::chrono::high_resolution_clock::now();
    // get the duration in seconds
    double duration =
        std::chrono::duration_cast<std::chrono::seconds>(cur_cpu - start_cpu)
            .count();
    // check if we have exceeded the time budget
    if (duration > compute_budget_sec_) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
          "Exceeded the time budget, break the loop, current duration is: "
          << duration << " seconds, and current step is: " << cur_step);
      break;
    } else {
      RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: Time taken up till now is: "
                      << duration
                      << " seconds, and current step is: " << cur_step);
    }

    std::vector<double> x_candidates;
    std::vector<double> y_candidates;
    double cur_step_double = static_cast<double>(cur_step);
    double x_positive_start = cur_step_double * outer_loop_step_size_x;
    // this is also the end (right boundary) of the previous step in the
    // positive direction
    double x_right_boundary_prev = cur_step_double * outer_loop_step_size_x;
    double x_positive_end = (cur_step_double + 1) * outer_loop_step_size_x;
    double x_negative_start = -(cur_step_double + 1) * outer_loop_step_size_x;
    double x_negative_end = -cur_step_double * outer_loop_step_size_x;
    // this is also the start (left boundary) of the previous step in the
    // negative direction
    double x_left_boundary_prev = -cur_step_double * outer_loop_step_size_x;
    double y_positive_start = cur_step_double * outer_loop_step_size_y;
    // this is also the end  (right boundary) of the previous step in the
    // positive direction
    double y_right_boundary_prev = cur_step_double * outer_loop_step_size_y;
    double y_positive_end = (cur_step_double + 1) * outer_loop_step_size_y;
    double y_negative_start = -(cur_step_double + 1) * outer_loop_step_size_y;
    double y_negative_end = -cur_step_double * outer_loop_step_size_y;
    // this is also the start (left boundary) of the previous step in the
    // negative direction
    double y_left_boundary_prev = -cur_step_double * outer_loop_step_size_y;
    RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: Current region boundaries: x_positive_start: "
                    << x_positive_start
                    << ", x_positive_end: " << x_positive_end
                    << ", x_negative_start: " << x_negative_start
                    << ", x_negative_end: " << x_negative_end);

    // Inner loop: iterate through all x and y within current step
    // Loop at the entire region and remove the center rectangle that has been
    // considered before since we need to get the "ring" region
    for (double x = x_negative_start; x <= x_positive_end;
         x += match_xy_step_size_) {
      for (double y = y_negative_start; y <= y_positive_end;
           y += match_xy_step_size_) {

        // skip the ones that lie in the center rectangle, which we already
        // considered in the previous step note that this need to be conditioned
        // on both X and Y
        if ((x >= x_left_boundary_prev && x <= x_right_boundary_prev) &&
            (y >= y_left_boundary_prev && y <= y_right_boundary_prev)) {
          continue;
        }

        // iterate through all the yaw candidates
        Eigen::Matrix3d cur_R_t = Eigen::Matrix3d::Identity();
        for (double yaw : yaw_candidates) {
          cur_R_t(0, 0) = cos(yaw);
          cur_R_t(0, 1) = -sin(yaw);
          cur_R_t(0, 2) = x;
          cur_R_t(1, 0) = sin(yaw);
          cur_R_t(1, 1) = cos(yaw);
          cur_R_t(1, 2) = y;
          // iterate through the detection objects
          Eigen::MatrixXd cur_detection_objects_transformed =
              Eigen::MatrixXd::Zero(query_objects.size(), 7);
          for (int j = 0; j < query_objects.size(); j++) {
            // apply the transformation on the detection object
            Eigen::Vector3d cur_object_transformed =
                cur_R_t * detection_object_xy_homogeneous.row(j).transpose();
            // normalize the homogeneous coordinates
            cur_object_transformed =
                cur_object_transformed / cur_object_transformed[2];
            // put the label and XY coordinates into the matrix
            cur_detection_objects_transformed(j, 0) = query_objects[j][0];
            cur_detection_objects_transformed(j, 1) = cur_object_transformed[0];
            cur_detection_objects_transformed(j, 2) = cur_object_transformed[1];
            cur_detection_objects_transformed(j, 3) = query_objects[j][3];
            cur_detection_objects_transformed(j, 4) = query_objects[j][4];
            cur_detection_objects_transformed(j, 5) = query_objects[j][5];
            cur_detection_objects_transformed(j, 6) = query_objects[j][6];
          }

          int cur_inliers = 0;
          // record all the matched map objects and detection objects in vector
          std::vector<Eigen::Vector4d> cur_matched_map_objects =
              std::vector<Eigen::Vector4d>();
          std::vector<Eigen::Vector4d> cur_matched_detection_objects =
              std::vector<Eigen::Vector4d>();

          // iterate through all the detection objects, find the best match in
          // the map objects
          for (int cur_idx = 0;
               cur_idx < cur_detection_objects_transformed.rows(); cur_idx++) {
            // take the first two element, which are the XY coordinates
            // take the label from detection_objects_original
            // get all inside map_objects_xyl that (1) has the same label and
            // (2) is within self.match_threshold_ distance to
            // cur_detection_object get the label of cur_detection_object
            double cur_detection_object_label =
                cur_detection_objects_transformed(cur_idx, 0);

            // get the XY coordinates of cur_detection_object
            Eigen::Vector2d cur_detection_object_xy =
                cur_detection_objects_transformed.block<1, 2>(cur_idx, 1);

            // check if there is any object in map_objects_label_xy that has the
            // same label as cur_detection_object, and is within
            // self.match_threshold_ distance to cur_detection_object iterate
            // through all the map objects
            for (int cur_map_idx = 0; cur_map_idx < map_objects_label_xy.rows();
                 cur_map_idx++) {
              // get the current map object label X Y
              Eigen::Vector7d cur_map_object_label_xy =
                  map_objects_label_xy.block<1, 7>(cur_map_idx, 0);
                  
              // first check if the label is the same to be most efficient
              if (cur_map_object_label_xy[0] != cur_detection_object_label) {
                continue;
              } else {
                // check if the distance is within self.match_threshold_
                double x_diff =
                    cur_map_object_label_xy[1] - cur_detection_object_xy[0];
                double y_diff =
                    cur_map_object_label_xy[2] - cur_detection_object_xy[1];
                // avg dim diff
                double avg_dim_diff = 0;
                // for cylinder, we only have one non-zero dimension, so we do
                // not divide by 3 check if both dim 2 and dim 3 are zeros
                if (cur_map_object_label_xy[5] == 0 &&
                    cur_map_object_label_xy[6] == 0) {
                  avg_dim_diff =
                      std::abs(cur_map_object_label_xy[4] -
                               cur_detection_objects_transformed(cur_idx, 4));
                } else {
                  for (int cur_dim_i = 4; cur_dim_i < 7; cur_dim_i++) {
                    avg_dim_diff += std::abs(
                        cur_map_object_label_xy[cur_dim_i] -
                        cur_detection_objects_transformed(cur_idx, cur_dim_i));
                  }
                  avg_dim_diff /= 3;
                }
                // two flags: distance match and dimension match
                bool distance_match =
                    sqrt(x_diff * x_diff + y_diff * y_diff) < match_threshold_;
                bool dimension_match;
                if (ignore_dimension_) {
                  dimension_match = true;
                } else {
                  dimension_match = avg_dim_diff < match_threshold_dimension_;
                }

                if (distance_match && dimension_match) {
                  // if so, add one to the number of inliers
                  cur_inliers++;
                  Eigen::Vector4d cur_map_object(
                      cur_map_object_label_xy[0], cur_map_object_label_xy[1],
                      cur_map_object_label_xy[2], cur_map_object_label_xy[3]);
                  cur_matched_map_objects.push_back(cur_map_object);
                  cur_matched_detection_objects.push_back(Eigen::Vector4d(
                      cur_detection_object_label, query_objects[cur_idx][1],
                      query_objects[cur_idx][2], query_objects[cur_idx][3]));
                  // once inlier is found, break the loop since we only need one
                  // inlier for each detection object
                  break;
                }
              }
            }
          }

          // check if the current number of inliers is larger than the best
          // number of inliers
          if (cur_inliers > best_num_inliers) {
            // update the best number of inliers
            best_num_inliers = cur_inliers;
            // update the best transformation matrix
            best_R_t = cur_R_t;  // sampled_transformations[cur_tf_id];
            // update the best matched map objects
            best_matched_map_objects = cur_matched_map_objects;
            // update the best matched detection objects
            best_matched_detection_objects = cur_matched_detection_objects;
          }
        }
      }
    }
  } 
  // print the best number of inliers ROS INFO
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: best number of inliers: " << best_num_inliers);
  // print the best transformation matrix
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: best transformation matrix: " << best_R_t);
  // assign the best transformation matrix to R_t_out
  R_t_out = best_R_t;
  // assign the best number of inliers to best_num_inliers_out
  best_num_inliers_out = best_num_inliers;
  // assign the best matched map objects to map_objects_matched_out
  map_objects_matched_out = best_matched_map_objects;
  // assign the best matched detection objects to detection_objects_matched_out
  detection_objects_matched_out = best_matched_detection_objects;
}

bool PlaceRecognition::findIntraLoopClosure(
    const std::vector<Eigen::Vector7d> &measurements,
    const std::vector<Eigen::Vector7d> &submap, const SE3 &query_pose,
    const SE3 &candidate_pose, Eigen::Matrix4d &tfFromQuery2Candidate) {
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                       "[findLoopClosure] findLoopClosure Thread Running");
  // end the loop closure if the minimal requirements not met
  if (measurements.size() == 0 || submap.size() == 0) {
    RCLCPP_INFO(node_->get_logger(),"[findLoopClosure] measurements or submap is empty");
    return false;
  }
  int number_of_measurements = measurements.size();
  if (number_of_measurements < 4) {
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: number of detected objects is less than 4, it is: "
                    << number_of_measurements);
    return false;
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: number of detected objects used for loop closure is: "
                    << number_of_measurements);
  }
  // RCLCPP_INFO_STREAM(node_->get_logger(),
  //     "number of object in submap used for loop closure is: " << submap.size());
  // rotate the measurements into map frame as they are in local frame
  // currently
  std::vector<Eigen::Vector7d> measurements_transformed_to_map_frame;

  // query rotation matrix is the
  Eigen::Quaterniond query_orientation = query_pose.unit_quaternion();
  Eigen::Matrix3d query_rotation_matrix = query_orientation.toRotationMatrix();
  // get 4X4 transformation matrix from query_pose
  Eigen::Matrix4d query_pose_matrix = query_pose.matrix().cast<double>();
  // Eigen::Matrix3d query_rotation_matrix = Eigen::Matrix3d::Identity();

  for (size_t i = 0; i < measurements.size(); i++) {
    // take the position component of the measurement
    Eigen::Vector3d pos(measurements[i][1], measurements[i][2],
                        measurements[i][3]);
    // create homogeneous vector from the measurement
    Eigen::Vector4d pos_homogeneous(pos[0], pos[1], pos[2], 1);
    // transform the homogeneous vector using the query_pose_matrix
    Eigen::Vector4d pos_transformed = query_pose_matrix * pos_homogeneous;
    // normalize the homogeneous vector, take the first three elements
    Eigen::Vector3d pos_transformed_normalized =
        pos_transformed.head(3) / pos_transformed[3];
    // put the label and the transformed position into the vector as well as the
    // dimensions
    Eigen::Vector7d measurement_rotated;
    measurement_rotated << measurements[i][0], pos_transformed_normalized[0],
        pos_transformed_normalized[1], pos_transformed_normalized[2],
        measurements[i][4], measurements[i][5], measurements[i][6];
    measurements_transformed_to_map_frame.push_back(measurement_rotated);
  }

  // call the match_maps function
  std::vector<double> xyzYaw_out;
  Eigen::Matrix4d transform_out;

  // call the findTransformation function
  bool closure_found = findTransformation(
      submap, measurements_transformed_to_map_frame, xyzYaw_out, transform_out);
  // check if the closure is found
  if (!closure_found) {
    RCLCPP_INFO(node_->get_logger(),
        "[findLoopClosure] no loop closure found due to not enough inliers");
    return false;
  }

  double yaw_estimate_out = xyzYaw_out[3];
  Eigen::Vector3d position_estimate_out =
      Eigen::Vector3d(xyzYaw_out[0], xyzYaw_out[1], xyzYaw_out[2]);
  // compute the rotation matrix using the yaw_estimate_out and the query_odom
  // orientation
  Eigen::Matrix3d rotation_matrix_yaw;
  rotation_matrix_yaw << cos(yaw_estimate_out), -sin(yaw_estimate_out), 0,
      sin(yaw_estimate_out), cos(yaw_estimate_out), 0, 0, 0, 1;

  // put the z of position_estimate_out the same as the query_pose z
  position_estimate_out[2] = 0.0;  // query_pose.translation()[2];

  // homogeneous transformation matrix from drifted pose to corrected pose
  Eigen::Matrix4d loop_closure_transform = Eigen::Matrix4d::Identity();
  loop_closure_transform.block<3, 3>(0, 0) = rotation_matrix_yaw;
  loop_closure_transform.block<3, 1>(0, 3) = position_estimate_out;

  // // construct a SE3 using loop_closed_rotation_matrix and the
  // // position_estimate_out
  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: position_estimate_out is: " << position_estimate_out);
  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: yaw_estimate_out is: " << yaw_estimate_out);
  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: loop_closure_transform is: " << loop_closure_transform);

  SE3 tfFromQueryDrifted2CandidateSE3 = candidate_pose.inverse() * query_pose;

  // H_query(corrected)^candidate = H_query(drifted)^candidate *
  // loop_closure_transform.inverse(), loop_closure_transform is
  // H_query(drifted)^query(corrected)
  SE3 loop_closure_transform_SE3 = SE3(loop_closure_transform);
  SE3 tfFromQueryCorrect2CandidateSE3 =
      tfFromQueryDrifted2CandidateSE3 * loop_closure_transform_SE3;

  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: tfFromQueryCorrect2CandidateSE3 is: "
  //                  << tfFromQueryCorrect2CandidateSE3.matrix());
  // // candidate_pose
  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: candidate_pose is: " << candidate_pose.matrix());
  // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: query_pose is: " << query_pose.matrix());

  tfFromQuery2Candidate =
      tfFromQueryCorrect2CandidateSE3.matrix().cast<double>();
  return true;
}

bool PlaceRecognition::findInterLoopClosure(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects,
    Eigen::Matrix4d &tfFromQueryToRef) {
  // IMPORTANT: each object in reference_objects and query_objects is in the [label, x, y, z, dim1, dim2, dim3] format
  // call the match_maps function
  std::vector<double> xyzYaw_out;
  Eigen::Matrix4d transform_out;
  bool closure_found = false;
  // only call the findTransformation function if the number of objects in both maps exceed the slidematch_min_num_map_objects_to_start_
  if (reference_objects.size() < slidematch_min_num_map_objects_to_start_ ||
      query_objects.size() < slidematch_min_num_map_objects_to_start_) {
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: number of objects in reference_objects or query_objects is less than slidematch_min_num_map_objects_to_start_");
  } else {
    closure_found = findTransformation(reference_objects, query_objects,
                                            xyzYaw_out, transform_out);
  }
  if (!closure_found) {
    RCLCPP_ERROR(node_->get_logger(),
        "[findInterLoopClosure] no loop closure found due to not enough "
        "inliers");
    return false;
  } else {
    RCLCPP_INFO(node_->get_logger(),"[findInterLoopClosure] SUCCESS: loop closure found!");
    // compose a transformation matrix
    double x = xyzYaw_out[0];
    double y = xyzYaw_out[1];
    double z = xyzYaw_out[2];
    double yaw = xyzYaw_out[3];

    tfFromQueryToRef = Eigen::Matrix4d::Identity();
    tfFromQueryToRef(0, 3) = x;
    tfFromQueryToRef(1, 3) = y;
    tfFromQueryToRef(2, 3) = z;
    tfFromQueryToRef(0, 0) = cos(yaw);
    tfFromQueryToRef(0, 1) = -sin(yaw);
    tfFromQueryToRef(1, 0) = sin(yaw);
    tfFromQueryToRef(1, 1) = cos(yaw);
    return true;
  }
}

#if USE_CLIPPER
bool PlaceRecognition::findInterLoopClosureWithClipper(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects,
    Eigen::Matrix4d &tfFromQueryToRef) {

    // // save the input in a file for debugging
    // std::ofstream reference_objects_file;
    // // TODO(ankit): Remove hardcoding of the file path
    // std::string save_reference_objects_file_path = "/home/sam/reference_objects.txt";
    // // if not exist, create the file, if exist, truncate it
    // reference_objects_file.open(save_reference_objects_file_path);
    // for (int i = 0; i < reference_objects.size(); i++) {
    //   reference_objects_file << reference_objects[i][0] << " " << reference_objects[i][1] << " " << reference_objects[i][2] << " " << reference_objects[i][3] << " " << reference_objects[i][4] << " " << reference_objects[i][5] << " " << reference_objects[i][6] << std::endl;
    // }
    // reference_objects_file.close();

    // std::ofstream query_objects_file;
    // std::string save_query_objects_file_path = "/home/sam/query_objects.txt";
    // query_objects_file.open(save_query_objects_file_path);
    // for (int i = 0; i < query_objects.size(); i++) {
    //   query_objects_file << query_objects[i][0] << " " << query_objects[i][1] << " " << query_objects[i][2] << " " << query_objects[i][3] << " " << query_objects[i][4] << " " << query_objects[i][5] << " " << query_objects[i][6] << std::endl;
    // }
    // query_objects_file.close();

    
    double sigma = slidegraph_sigma_;
    double epsilon = slidegraph_epsilon_;
    int min_num_pairs = slidegraph_num_inliners_;
    double matching_threshold = slidegraph_matching_threshold_;

    // print the params in ROS_ERROR_STREAM
    // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: sigma is: " << sigma);
    // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: epsilon is: " << epsilon);
    // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: min_num_pairs is: " << min_num_pairs);
    // RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: matching_threshold is: " << matching_threshold);

    // convert the reference_objects and query_objects to semantic_clipper
    // argument format
    std::vector<std::vector<double>> reference_objects_vector;
    std::vector<std::vector<double>> query_objects_vector;
    for (int i = 0; i < reference_objects.size(); i++) {
      // check if the object has 0 in its coordinates, if so skip it since it is
      // not valid
      if (reference_objects[i][1] == 0.0 && reference_objects[i][2] == 0.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
            "reference object has 0 in its coordinates, skipping it");
        continue;
      }
      std::vector<double> object;
      object.push_back(reference_objects[i][0]);
      object.push_back(reference_objects[i][1]);
      object.push_back(reference_objects[i][2]);
      object.push_back(reference_objects[i][3]);
      object.push_back(reference_objects[i][4]);
      object.push_back(reference_objects[i][5]);
      object.push_back(reference_objects[i][6]);
      reference_objects_vector.push_back(object);
  }
  for (int i = 0; i < query_objects.size(); i++) {
    // check if the object has 0 in its coordinates, if so skip it since it is not valid
    if (query_objects[i][1] == 0.0 && query_objects[i][2] == 0.0) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"[PlaceRecognition]: query object has 0 in its coordinates, skipping it");
      continue;
    }
    std::vector<double> object;
    object.push_back(query_objects[i][0]);
    object.push_back(query_objects[i][1]);
    object.push_back(query_objects[i][2]);
    object.push_back(query_objects[i][3]);
    object.push_back(query_objects[i][4]);
    object.push_back(query_objects[i][5]);
    object.push_back(query_objects[i][6]);
    query_objects_vector.push_back(object);
  }
  
  bool found = false;
  // make sure we have at least slidegraph_min_num_map_objects_to_start_ objects to do the matching in both the reference and query
  if (reference_objects_vector.size() >= slidegraph_min_num_map_objects_to_start_ && query_objects_vector.size() >= slidegraph_min_num_map_objects_to_start_) {
    // RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: reference_objects_vector size is: " << reference_objects_vector.size() << " and query_objects_vector size is: " << query_objects_vector.size());
    RCLCPP_WARN(node_->get_logger(),"Calling CLIPPER for inter loop closure, if anything bad happens, look into that piece of code...");
    found = semantic_clipper::run_semantic_clipper(reference_objects_vector, query_objects_vector, tfFromQueryToRef, sigma, epsilon, min_num_pairs, matching_threshold);
    RCLCPP_DEBUG(node_->get_logger(),"EXIT CLIPPER SUCCESSFULLY");
    // get the inverse of the transformation matrix
    tfFromQueryToRef = tfFromQueryToRef.inverse();
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: Not enough objects to start the place recognition,  slidegraph_min_num_map_objects_to_start_ is set as " << slidegraph_min_num_map_objects_to_start_ << " but the reference_objects_vector size is: " << reference_objects_vector.size() << " and query_objects_vector size is: " << query_objects_vector.size());
  }  
  return found;
}
#endif

void PlaceRecognition::solveLSQ(
    const std::vector<Eigen::Vector3d> &map_objects_matched_out,
    const std::vector<Eigen::Vector3d> &detection_objects_matched_out,
    std::vector<double> &xyzyaw_out, Eigen::Matrix4d &transform_out) {
  Eigen::MatrixXd map_objects_matched_out_matrix =
      Eigen::MatrixXd::Zero(map_objects_matched_out.size(), 3); 
  Eigen::MatrixXd detection_objects_matched_out_matrix =
      Eigen::MatrixXd::Zero(detection_objects_matched_out.size(), 3);  
  for (int i = 0; i < map_objects_matched_out.size(); i++) {
    map_objects_matched_out_matrix(i, 0) = map_objects_matched_out[i][0];
    map_objects_matched_out_matrix(i, 1) = map_objects_matched_out[i][1];
    map_objects_matched_out_matrix(i, 2) = map_objects_matched_out[i][2];
  }
  for (int i = 0; i < detection_objects_matched_out.size(); i++) {
    detection_objects_matched_out_matrix(i, 0) =
        detection_objects_matched_out[i][0];
    detection_objects_matched_out_matrix(i, 1) =
        detection_objects_matched_out[i][1];
    detection_objects_matched_out_matrix(i, 2) =
        detection_objects_matched_out[i][2];
  }

  // Find the centroids of the source and target points
  Eigen::Vector3d centroidSource =
      detection_objects_matched_out_matrix.colwise().mean();
  Eigen::Vector3d centroidTarget =
      map_objects_matched_out_matrix.colwise().mean();

  // transpose the source and target points
  detection_objects_matched_out_matrix.transposeInPlace();
  map_objects_matched_out_matrix.transposeInPlace();

  // Center the source and target points
  Eigen::MatrixXd centeredSource =
      detection_objects_matched_out_matrix.colwise() - centroidSource;
  Eigen::MatrixXd centeredTarget =
      map_objects_matched_out_matrix.colwise() - centroidTarget;

  // Compute the cross-covariance matrix H
  Eigen::Matrix3d H = centeredSource * centeredTarget.transpose();

  // Perform SVD on H
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute the rotation matrix R
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

  if (R.determinant() < 0) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) = -V.col(2);
    R = V * svd.matrixU().transpose();
  }

  // Compute the translation vector t
  Eigen::Vector3d t = centroidTarget - R * centroidSource;
  RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: The translation vector in solveLSQ is: " << t.transpose());
  transform_out = Eigen::Matrix4d::Identity();
  transform_out.block<3, 3>(0, 0) = R;
  transform_out.block<3, 1>(0, 3) = t;
  getxyzYawfromTF(transform_out, xyzyaw_out);
}

void PlaceRecognition::getxyzYawfromTF(const Eigen::Matrix4d &tf,
                                       std::vector<double> &xyzYaw) {
  Eigen::Vector3d translation = tf.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = tf.block<3, 3>(0, 0);
  // get the yaw angle
  double yaw = atan2(rotation(1, 0), rotation(0, 0));
  double roll = atan2(rotation(2, 1), rotation(2, 2));
  double pitch = atan2(-rotation(2, 0), sqrt(rotation(2, 1) * rotation(2, 1) +
                                             rotation(2, 2) * rotation(2, 2)));
  // put the x y z yaw into the vector
  xyzYaw.push_back(translation[0]);
  xyzYaw.push_back(translation[1]);
  xyzYaw.push_back(translation[2]);
  xyzYaw.push_back(yaw);
}

Eigen::Vector2d PlaceRecognition::getCentroid(
    const std::vector<Eigen::Vector7d> &objects) {
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (int i = 0; i < objects.size(); i++) {
    centroid[0] += objects[i][1];
    centroid[1] += objects[i][2];
  }
  centroid /= objects.size();
  return centroid;
}

Eigen::Vector2d PlaceRecognition::getMapBoundaries(
    const std::vector<Eigen::Vector7d> &objects) {
  double max_x = 0;
  double max_y = 0;
  for (int i = 0; i < objects.size(); i++) {
    max_x = std::max(max_x, std::abs(objects[i][1]));
    max_y = std::max(max_y, std::abs(objects[i][2]));
  }
  Eigen::Vector2d boundaries(max_x, max_y);
  return boundaries;
}

bool PlaceRecognition::findTransformation(
    const std::vector<Eigen::Vector7d> &reference_objects_input,
    const std::vector<Eigen::Vector7d> &query_objects_input,
    std::vector<double> &xyz_yaw_out, Eigen::Matrix4d &transform_out) {
  // initialize the two to be zeros
  Eigen::Vector2d centroid_reference = Eigen::Vector2d::Zero();
  Eigen::Vector2d centroid_query = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector7d> reference_objects;
  std::vector<Eigen::Vector7d> query_objects;
  if (inter_loop_closure) {
    RCLCPP_INFO(node_->get_logger(),
        "[findTransformation] running INTER (multi-robot) loop closure, doing "
        "preprocessing to zero center the maps and reduce search region");
    // preprocess step to reduce search region by centering the
    // reference_objects and query_objects to make their centroid aligned on XY
    // plane 
    centroid_reference = getCentroid(reference_objects_input);
    centroid_query = getCentroid(query_objects_input);
    // shift the reference_objects to align the centroid with origin
    reference_objects = reference_objects_input;
    for (int i = 0; i < reference_objects_input.size(); i++) {
      reference_objects[i][1] -= centroid_reference[0];
      reference_objects[i][2] -= centroid_reference[1];
    }
    // shift the query_objects to align the centroid with origin
    query_objects = query_objects_input;
    for (int i = 0; i < query_objects_input.size(); i++) {
      query_objects[i][1] -= centroid_query[0];
      query_objects[i][2] -= centroid_query[1];
    }

    // get the max bounding region of the reference_objects and query_objects
    Eigen::Vector2d boundaries_reference = getMapBoundaries(reference_objects);
    Eigen::Vector2d boundaries_query = getMapBoundaries(query_objects);

    double max_x_region =
        std::max(boundaries_reference[0], boundaries_query[0]);
    double max_y_region =
        std::max(boundaries_reference[1], boundaries_query[1]);

    // check if yaw search is needed
    if (!disable_yaw_search_) {
      // since there may be yaw, we need to take the max of the two
      double max_region_size = std::max(max_x_region, max_y_region);
      max_x_region = max_region_size;
      max_y_region = max_region_size;
    }

    // instead of using hard-coded half range values, we use the auto computed
    // max_x_region and max_y_region dilate the map to consider partial overlap
    match_x_half_range_ = max_x_region * dilation_factor_;
    match_y_half_range_ = max_y_region * dilation_factor_;

    // print the centroid difference
    Eigen::Vector2d centroid_diff = centroid_reference - centroid_query;
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: centroid diff between the two maps in x and y is: "
                    << centroid_diff[0] << " " << centroid_diff[1]);

    RCLCPP_WARN_STREAM(node_->get_logger(),
        "Auto computed the bounding (candidate loop closure) region of the two "
        "maps, the half range in x and y is: "
        << match_x_half_range_ << " " << match_y_half_range_
        << " (with dilation factor: " << dilation_factor_ << ")");
  } else {
    RCLCPP_INFO(node_->get_logger(),
        "[findTransformation] running INTRA loop closure, no preprocessing "
        "needed");
    // if it is intra loop closure, we do not need to preprocess the data
    reference_objects = reference_objects_input;
    query_objects = query_objects_input;
    // set the match_x_half_range_ and match_y_half_range_ and
    // match_yaw_half_range to be the same as the intra params
    match_x_half_range_ = match_x_half_range_intra_;
    match_y_half_range_ = match_y_half_range_intra_;
    match_yaw_half_range_ = match_yaw_half_range_intra_;
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "Using the intra loop closure parameters, the half range in x, y, and "
        "yaw is: "
        << match_x_half_range_ << " " << match_y_half_range_ << " "
        << match_yaw_half_range_);
  }

  Eigen::Matrix3d R_t_out;
  int best_num_inliers_out = 0;
  std::vector<Eigen::Vector4d> map_objects_matched_out;
  std::vector<Eigen::Vector4d> detection_objects_matched_out;
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"[PlaceRecognition]: Ready to run MatchMaps, which costs time...");

  // the input of matchmaps is vector7d label x y z dim1 dim2 dim3
  // the output map_objects_matched_out from matchmaps is vector 3d label x y
  MatchMaps(reference_objects, query_objects, R_t_out, best_num_inliers_out,
            map_objects_matched_out, detection_objects_matched_out);

  RCLCPP_DEBUG_STREAM(node_->get_logger(),"[PlaceRecognition]: PRINT SOME RESULTS FROM MATCHESMAP");
  for (int i = 0; i < map_objects_matched_out.size(); i++) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),"[PlaceRecognition]: map_objects_matched_out: "
                     << map_objects_matched_out[i][0] << " "
                     << map_objects_matched_out[i][1] << " "
                     << map_objects_matched_out[i][2]);
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(),"[PlaceRecognition]: MatchMaps finished...");
  // check if the best_num_inliers_out is less than
  // min_num_inliers_for_valid_closure_
  // UPDATE: NO LONGER USING THE OVERLAP THRESHOLD SINCE IT IS NON INTUITIVE TO THE USER
  // double min_num_inliners =
  //     min_loop_closure_overlap_percentage_ *
  //     static_cast<double>(
  //         std::min(reference_objects.size(), query_objects.size()));
  int min_num_inliners = min_num_inliers_;

  // make min_num_inliners at least one to avoid division by zero
  // min_num_inliners = std::max(min_num_inliners, 1.0);
  if (best_num_inliers_out < min_num_inliners) {
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: Not enough inliers found, best_num_inliers_out is: "
                    << best_num_inliers_out);
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "min_num_inliners is: "
        << min_num_inliners
        << " YOU CAN TUNE THE PARAMETER IN THE YAML FILE (min_num_inliers)");
    return false;
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: Enough inliers found, best_num_inliers_out is: "
                    << best_num_inliers_out);
    RCLCPP_INFO_STREAM(node_->get_logger(),
        "actual overlap percentage is: "
        << static_cast<double>(best_num_inliers_out) /
               static_cast<double>(
                   std::min(reference_objects.size(), query_objects.size())));
  }

  if (visualize_matching_results) {
    RCLCPP_WARN_STREAM(node_->get_logger(),"[PlaceRecognition]: Visualizing the place recognition matching results...");
    // take the first 4 dims out of query_objects
    std::vector<Eigen::Vector4d> query_objects_vis;
    for (int i = 0; i < query_objects.size(); i++) {
      Eigen::Vector4d cur_query_object(query_objects[i][0], query_objects[i][1],
                                       query_objects[i][2],
                                       query_objects[i][3]);
      query_objects_vis.push_back(cur_query_object);
    }
    VisualizeMatchingResults(map_objects_matched_out,
                             detection_objects_matched_out, query_objects_vis,
                             R_t_out);
  }

  if (!use_lsq) {
    Eigen::Matrix4d transform_out_raw = Eigen::Matrix4d::Identity();
    // take the upper 2x2 of R_t which is the yaw rotation matrix
    Eigen::Matrix2d Rot_yaw = R_t_out.block<2, 2>(0, 0);
    transform_out_raw.block<2, 2>(0, 0) = Rot_yaw;
    transform_out_raw.block<3, 1>(0, 3) =
        Eigen::Vector3d(R_t_out(0, 2), R_t_out(1, 2), 0);

    if (inter_loop_closure) {
      // add centroid_diff back (only x and y since we don't estimate Z in this
      // case)
      transform_out = PlaceRecognition::revertCentroidShift(
          transform_out_raw, centroid_reference, centroid_query);
    } else {
      // no preprocessing done, so the transform_out is the same as
      // transform_out_raw
      transform_out = transform_out_raw;
    }

    // get the xyzYaw from the transform_out
    getxyzYawfromTF(transform_out, xyz_yaw_out);

    // print error message, to check if this pass test cases!
    return true;
  } else {
    std::vector<Eigen::Vector3d> map_objects_matched;
    std::vector<Eigen::Vector3d> detection_objects_matched;
    // remove the label from map_objects_matched_out and
    // detection_objects_matched_out and put them into map_objects_matched and
    // detection_objects_matched
    for (int i = 0; i < map_objects_matched_out.size(); i++) {
      Eigen::Vector3d cur_map_object(map_objects_matched_out[i][1],
                                     map_objects_matched_out[i][2],
                                     map_objects_matched_out[i][3]);
      map_objects_matched.push_back(cur_map_object);
    }
    for (int i = 0; i < detection_objects_matched_out.size(); i++) {
      Eigen::Vector3d cur_detection_object(detection_objects_matched_out[i][1],
                                           detection_objects_matched_out[i][2],
                                           detection_objects_matched_out[i][3]);
      detection_objects_matched.push_back(cur_detection_object);
    }

    if (inter_loop_closure) {
      // for inter loop closure instead of manually call revertCentroidShift, we
      // first shift the map_objects_matched and detection_objects_matched back
      // to their original position before least square fitting
      for (int i = 0; i < map_objects_matched.size(); i++) {
        map_objects_matched[i][0] += centroid_reference[0];
        map_objects_matched[i][1] += centroid_reference[1];
      }
      for (int i = 0; i < detection_objects_matched.size(); i++) {
        detection_objects_matched[i][0] += centroid_query[0];
        detection_objects_matched[i][1] += centroid_query[1];
      }
    }

    // no need to revert the centroid shift since the inputs of solveLSQ
    // function already take care of it 
    solveLSQ(map_objects_matched, detection_objects_matched, xyz_yaw_out,
             transform_out);
    return true;
  }
}

Eigen::Matrix4d PlaceRecognition::revertCentroidShift(
    const Eigen::Matrix4d &tf, const Eigen::Vector2d &centroid_reference,
    const Eigen::Vector2d &centroid_query) {
  // H_1^2 = H_1^1(shifted) * H_1(shifted)^2(shifted) * H_2(shifted)^2
  // we get H_1^1(shifted) based on the centroid_reference
  // we get H_1(shifted)^2(shifted) from tf
  // we get H_2(shifted)^2 from centroid_query
  Eigen::Matrix4d H_1_1_shifted = Eigen::Matrix4d::Identity();
  // translation part is centroid_reference
  H_1_1_shifted(0, 3) = centroid_reference[0];
  H_1_1_shifted(1, 3) = centroid_reference[1];
  // same for H_2shifted^2
  Eigen::Matrix4d H_2_shifted_2 = Eigen::Matrix4d::Identity();
  // need to invert since it is from shifted back to original, since rot is
  // identity, just need to negate the translation
  H_2_shifted_2(0, 3) = -centroid_query[0];
  H_2_shifted_2(1, 3) = -centroid_query[1];
  // H_1^2 = H_1^1(shifted) * H_1(shifted)^2(shifted) * H_2(shifted)^2
  Eigen::Matrix4d tf_out = H_1_1_shifted * tf * H_2_shifted_2;
  return tf_out;
}

// define the function to visualize the matching results
void PlaceRecognition::VisualizeMatchingResults(
    const std::vector<Eigen::Vector4d> &map_objects_matched,
    const std::vector<Eigen::Vector4d> &detection_objects_matched,
    const std::vector<Eigen::Vector4d> &all_detection_objects,
    Eigen::Matrix3d &R_t) {
  // create a marker array
  visualization_msgs::msg::MarkerArray matching_results;
  // create a marker for each map object
  for (int i = 0; i < map_objects_matched.size(); i++) {
    // create a marker
    visualization_msgs::msg::Marker cur_marker;
    // set the header
    cur_marker.header.frame_id = vis_ref_frame_;
    // set the namespace
    cur_marker.ns = "map_objects_matched";
    // set the id
    cur_marker.id = i;
    // set the type
    cur_marker.type = visualization_msgs::msg::Marker::CUBE;
    // set the action
    cur_marker.action = visualization_msgs::msg::Marker::ADD;
    // set the pose
    cur_marker.pose.position.x = map_objects_matched[i][1];
    cur_marker.pose.position.y = map_objects_matched[i][2];
    cur_marker.pose.position.z = 0;
    cur_marker.pose.orientation.x = 0;
    cur_marker.pose.orientation.y = 0;
    cur_marker.pose.orientation.z = 0;
    cur_marker.pose.orientation.w = 1;
    // set the scale
    cur_marker.scale.x = 1;
    cur_marker.scale.y = 1;
    cur_marker.scale.z = 1;
    // set the color
    cur_marker.color.r = 0;
    cur_marker.color.g = 0;
    cur_marker.color.b = 1;
    cur_marker.color.a = 1;
    matching_results.markers.push_back(cur_marker);
  }

  // draw the line between each pair of matched objects and detected_objects
  for (int i = 0; i < map_objects_matched.size(); i++) {
    // create a marker
    visualization_msgs::msg::Marker cur_marker;
    // set the header
    cur_marker.header.frame_id = vis_ref_frame_;
    // set the namespace
    cur_marker.ns = "matched_objects";
    // set the id
    cur_marker.id = i;
    // set the type
    cur_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // set the action
    cur_marker.action = visualization_msgs::msg::Marker::ADD;
    // set the pose
    cur_marker.pose.position.x = 0;
    cur_marker.pose.position.y = 0;
    cur_marker.pose.position.z = 0;
    cur_marker.pose.orientation.x = 0;
    cur_marker.pose.orientation.y = 0;
    cur_marker.pose.orientation.z = 0;
    cur_marker.pose.orientation.w = 1;
    // set the scale
    cur_marker.scale.x = 0.1;
    cur_marker.scale.y = 0.1;
    cur_marker.scale.z = 0.1;
    // set the color
    cur_marker.color.r = 1;
    cur_marker.color.g = 0;
    cur_marker.color.b = 0;
    cur_marker.color.a = 1;
    geometry_msgs::msg::Point cur_point;
    cur_point.x = map_objects_matched[i][1];
    cur_point.y = map_objects_matched[i][2];
    cur_point.z = 0;
    cur_marker.points.push_back(cur_point);
    // set the pose
    Eigen::Vector3d cur_object_temp = Eigen::Vector3d::Zero();
    cur_object_temp[0] = detection_objects_matched[i][1];
    cur_object_temp[1] = detection_objects_matched[i][2];
    // homogeneous coordinates
    cur_object_temp[2] = 1;
    Eigen::Vector3d cur_object_transformed_temp = cur_object_temp;
    cur_point.x = cur_object_transformed_temp[0];
    cur_point.y = cur_object_transformed_temp[1];
    cur_point.z = detection_objects_matched[i][3];
    cur_marker.points.push_back(cur_point);
    // add the marker to the marker array
    matching_results.markers.push_back(cur_marker);
  }

  // use R_t to transform the all_detection_objects and visualize them
  for (int i = 0; i < all_detection_objects.size(); i++) {
    // create a marker
    visualization_msgs::msg::Marker cur_marker;
    // set the header
    cur_marker.header.frame_id = vis_ref_frame_;
    // set the namespace
    cur_marker.ns = "all_detection_objects";
    // set the id
    cur_marker.id = i;
    // set the type to be a cube
    cur_marker.type = visualization_msgs::msg::Marker::CUBE;
    // set the action
    cur_marker.action = visualization_msgs::msg::Marker::ADD;
    // set the pose
    Eigen::Vector3d cur_object = Eigen::Vector3d::Zero();
    cur_object[0] = all_detection_objects[i][1];
    cur_object[1] = all_detection_objects[i][2];
    // homogeneous coordinates
    cur_object[2] = 1;
    Eigen::Vector3d cur_object_transformed = cur_object;
    cur_marker.pose.position.x = cur_object_transformed[0];
    cur_marker.pose.position.y = cur_object_transformed[1];
    cur_marker.pose.position.z = all_detection_objects[i][3];
    cur_marker.pose.orientation.x = 0;
    cur_marker.pose.orientation.y = 0;
    cur_marker.pose.orientation.z = 0;
    cur_marker.pose.orientation.w = 1;
    // set the scale
    double scale_factor = 1.0;
    cur_marker.scale.x = scale_factor;
    cur_marker.scale.y = scale_factor;
    cur_marker.scale.z = scale_factor;
    // set the color
    cur_marker.color.r = 0;
    cur_marker.color.g = 1;
    cur_marker.color.b = 0;
    cur_marker.color.a = 1;
    matching_results.markers.push_back(cur_marker);
  }

  // publish the marker array
  viz_pub_->publish(matching_results);
  // ROS info the marker, which topic and which ref frame
  RCLCPP_INFO_STREAM(node_->get_logger(),
      "Visualizing the place recognition matching results in the topic: "
      << viz_pub_->get_topic_name());
  RCLCPP_INFO_STREAM(node_->get_logger(),"[PlaceRecognition]: The reference frame is: "
                  << matching_results.markers[0].header.frame_id);
}
