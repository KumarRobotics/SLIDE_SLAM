/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <graph.h>
#include <rclcpp/rclcpp.hpp>

namespace {
inline rclcpp::Logger gfg_logger() {
  return rclcpp::get_logger("SemanticFactorGraph");
}
}  // namespace

SemanticFactorGraph::SemanticFactorGraph() {
  isam_params.factorization = ISAM2Params::CHOLESKY;
  isam_params.relinearizeSkip = 1;
  isam_params.relinearizeThreshold = 0.1;
  isam = new ISAM2(isam_params);

  // VERY IMPORTANT: the first three corresponds to Rotation and the last three
  // corresponds to Translation
  Vector6 noise_vec_prior_first_pose;
  noise_vec_prior_first_pose << 0.000001, 0.000001, 0.000001, 0.00001, 0.00001,
      0.00001;
  noise_model_prior_first_pose =
      noiseModel::Diagonal::Sigmas(noise_vec_prior_first_pose);

  // Landmark Noise model
  // TODO(xu:) update the cylinder measurement noise
  noise_model_cylinder =
      noiseModel::Diagonal::Sigmas(100 * Vector7::Ones() * 4);

  // TODO(xu:) update the cuboid measurement noise
  double stddev_angle = 5 * (3.14 / 180);  // 120.0 * (3.14 / 180);
  double stddev_pos = 1.0;                 // 3.0;
  Vector9 noise_vec;
  noise_vec << stddev_angle, stddev_angle, stddev_angle, stddev_pos, stddev_pos,
      stddev_pos, stddev_pos, stddev_pos, stddev_pos;
  noise_model_cube = noiseModel::Diagonal::Sigmas(noise_vec);

  // For range and bearing (ellipsoid objects) measurements
  double bearing_noise_std_temp = 1;
  noise_model_bearing = noiseModel::Isotropic::Sigma(3, bearing_noise_std_temp);

  Vector6 noise_model_pose_vec = Vector6::Ones() * 0.00001;
  noise_model_pose = noiseModel::Diagonal::Sigmas(noise_model_pose_vec);
  noise_model_closure =
      noiseModel::Diagonal::Sigmas(noise_model_pose_vec * 0.01);
      
  start_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
}

void SemanticFactorGraph::setPriors(const Pose3 &pose_prior,
                                    const int &robotID) {
  size_t idx = 0;
  fgraph.addPrior<Pose3>(getSymbol(robotID, idx), pose_prior,
                         noise_model_prior_first_pose);
  // here the insert also set the initial estimate for the optimization
  fvalues.insert(getSymbol(robotID, idx), pose_prior);
  RCLCPP_INFO_STREAM(gfg_logger(),
                     "Setting prior for robot " << robotID
                                                << " with pose: " << pose_prior);

  // only for active SLAM:
  // --------------------------------
  // "fake" factor graph for information gain evaluation only
  // fgraph_loop.addPrior<Pose3>(getSymbol(robotID, idx), pose_prior,
  //                        noise_model_prior_first_pose);
  // fvalues_loop.insert(getSymbol(robotID, idx), pose_prior);
  // current_pose_global_ = pose_prior;
  // --------------------------------
}

// No longer used - GPS factor
// void SemanticFactorGraph::addGPSFactor(const Point3 &gps_measurement) {
//   size_t pose_counter_GPS = 0;  // TODO: make this counter correct
//   auto current_pose_key = X(pose_counter_GPS);
//   GPSFactor gps_factor(current_pose_key,
//                        gps_measurement,  // N,E,D
//                        noise_model_gps);
//   fgraph.add(gps_factor);
// }

void SemanticFactorGraph::addKeyPoseAndBetween(
    const size_t prevIdx, const size_t curIdx, const Pose3 &relativeMotion,
    const Pose3 &poseEstimate, const int &robotID,
    boost::optional<boost::array<double, 36>> cov, const bool &loopClosureFound,
    const SE3 &loop_closure_relative_pose,
    const size_t &closure_matched_pose_idx) {
  // bool use_msckf_cov = false;
  // Pose3 poseRelative = poseFrom.between(poseEstimate);

  // No longer used, getting cov from VIO
  // --------------------------------
  ////if (cov) {
  ////  Matrix6 M =
  ////      Eigen::Map<Eigen::Matrix<double, 6, 6,
  ///Eigen::RowMajor>>(cov->data()); /  boost::shared_ptr<noiseModel::Gaussian>
  ///noise_model_odom = /      noiseModel::Gaussian::Covariance(M); /
  ///fgraph.add(BetweenFactor<Pose3>(getSymbol(robotID, prevIdx), /
  ///getSymbol(robotID, curIdx), poseRelative, / noise_model_odom));
  ////} else {
  ////  fgraph.add(BetweenFactor<Pose3>(getSymbol(robotID, prevIdx),
  ////                                  getSymbol(robotID, curIdx),
  ///poseRelative, /                                  noise_model_pose));
  ////}
  ////// here the insert also set the initial estimate for the optimization
  //// fvalues.insert(getSymbol(robotID, curIdx), poseTo);
  // double mins_elapsed;
  // if (start_timestamp == 0.0) {
  //   mins_elapsed = 0.0;
  // } else {
  //   mins_elapsed = (node_->now().seconds() - start_timestamp) / 60.0;
  // }
  // if (noise_model_pose_inflation > 0) {
  //   printf(
  //       "+++++++ inflating covariance with time, if you are playing a bag, "
  //       "make sure that /use_sim_time is set as true and play bag with "
  //       "--clock flag \n");

  //   printf("odom_factor_noise_std_inflation_per_min is set as: \n");
  //   std::cout << noise_model_pose_inflation << std::endl;
  //   printf(
  //       "number of mins since the first "
  //       "odometry factor: \n");
  //   std::cout << mins_elapsed << std::endl;
  // }
  // --------------------------------

  Vector6 cur_noise_vec;

  // disable the inflation of covariance
  // cur_noise_vec = noise_model_pose->sigmas() *
  //                 (1.0 + noise_model_pose_inflation * mins_elapsed);

  // RCLCPP_INFO_STREAM_THROTTLE(
  //     1,
  //     "Using predefined between-factor covariance instead of "
  //     "VIO estimated covariance (due to its inaccuracy)");

  // scale covariance by travel distance so that motion uncertainty is properly
  // accounted for
  cur_noise_vec = noise_model_pose->sigmas();
  double relative_dist = relativeMotion.translation().norm();
  // clip the relative dist to avoid numerical issues when optimizing
  if (relative_dist < 0.1) {
    relative_dist = 0.1;
  }
  Vector6 noise_vec_scaled_by_travel_dist = cur_noise_vec * relative_dist;

  // noiseModel::Diagonal::Sigmas() takes in standard deviation, not variance
  fgraph.add(BetweenFactor<Pose3>(
      getSymbol(robotID, prevIdx), getSymbol(robotID, curIdx), relativeMotion,
      noiseModel::Diagonal::Sigmas(noise_vec_scaled_by_travel_dist)));

  // Only for active SLAM
  // fgraph_loop.add(BetweenFactor<Pose3>(
  //     getSymbol(robotID,prevIdx), getSymbol(robotID,curIdx),
  //     relativeMotion, noiseModel::Diagonal::Sigmas(cur_noise_vec)));

  if (latest_to_idx_ < curIdx) {
    latest_to_idx_ = curIdx;
  }

  if (loopClosureFound) {
    // add loop_closure_relative_pose to X(closure_matched_pose_idx)
    gtsam::Pose3 est_loop_closure_pose;

    // get current estimate of X(closure_matched_pose_idx)
    gtsam::Pose3 est_from_pose;

    if (closure_matched_pose_idx == 0) {
      RCLCPP_ERROR(gfg_logger(),
          "ERROR: closure_matched_pose_idx is 0, this probably means this "
          "param is not correctly set, check!!");
    }
    bool cur_pose_valid =
        getPose(closure_matched_pose_idx, robotID, est_from_pose);
    if (!cur_pose_valid) {
      RCLCPP_ERROR_STREAM(gfg_logger(),
          "ERROR: get closure_matched_pose_idx fail to fetch pose for pose_idx "
          ": "
          << closure_matched_pose_idx);
      auto odom_pose_prior_factor_noise = cur_noise_vec * 1.0;
      // here the insert also set the initial estimate for the optimization
      fvalues.insert(getSymbol(robotID, curIdx), poseEstimate);
      // Only for active SLAM
      // fvalues_loop.insert(getSymbol(robotID,curIdx), poseEstimate);
      // current_pose_global_ = poseEstimate;
      return;
    }

    // TODO(xu:) check why est_from_pose is removed here, maybe because we are
    // no longer treating loop closure as a prior, instead we are using between
    // factor
    //  but we should still keep this since we use it as initial guess when
    //  adding the pose key to the factor graph

    // gtsam::Pose3 est_from_pose =
    // fvalues.at<gtsam::Pose3>(X(closure_matched_pose_idx)); multiply
    // est_from_pose.matrix and loop_closure_relative_pose.matrix
    // loop_closure_relative_pose will be used for adding loop closure between
    // factor
    auto est_pose_matrix =
        est_from_pose.matrix() * loop_closure_relative_pose.matrix();
    est_loop_closure_pose = gtsam::Pose3(est_pose_matrix);

    RCLCPP_INFO_STREAM(gfg_logger(), "++++++++++++CLOSURE FACTOR BEING ADDED++++++++++++");
    RCLCPP_INFO_STREAM(gfg_logger(), "loop_closure_relative_pose"
                    << loop_closure_relative_pose.matrix());
    RCLCPP_INFO_STREAM(gfg_logger(), "est_loop_closure_pose" << est_loop_closure_pose.matrix());

    // No longer treating loop closure as a prior. Instead, use it as a proper
    // between factor add loop closure as a between factor
    // loop_closure_relative_pose is from history loop closure key pose to
    // current pose The correct interface for BetweenFactor<Pose3> is (pose1,
    // pose2, H_pose2^pose1, noise)
    fgraph.add(
        BetweenFactor<Pose3>(getSymbol(robotID, closure_matched_pose_idx),
                             getSymbol(robotID, curIdx),
                             gtsam::Pose3(loop_closure_relative_pose.matrix()),
                             noise_model_closure));

    // Only for active SLAM:
    // --------------------------------
    // fgraph_loop.add(BetweenFactor<Pose3>(
    //     getSymbol(robotID, closure_matched_pose_idx),
    //     getSymbol(robotID,curIdx),
    //     gtsam::Pose3(loop_closure_relative_pose.matrix()),
    //     noise_model_closure));
    // --------------------------------

    // here the insert also set the initial estimate for the optimization
    fvalues.insert(getSymbol(robotID, curIdx), est_loop_closure_pose);
    // Only for active SLAM:
    // fvalues_loop.insert(getSymbol(robotID, curIdx),est_loop_closure_pose);
    RCLCPP_INFO_STREAM(gfg_logger(), "loop_closure_relative_pose being added");
    // current_pose_global_ = est_loop_closure_pose;
  } else {
    auto odom_pose_prior_factor_noise = cur_noise_vec * 1.0;
    // here the insert also set the initial estimate for the optimization
    fvalues.insert(getSymbol(robotID, curIdx), poseEstimate);
    // Only for active SLAM
    // fvalues_loop.insert(getSymbol(robotID, curIdx), poseEstimate);
    // current_pose_global_ = poseEstimate;
  }
}

void SemanticFactorGraph::addPointLandmarkKey(const size_t ugvIdx,
                                              const Point3 &ugv_position) {
  fvalues.insert(U(ugvIdx), ugv_position);
}

void SemanticFactorGraph::addRangeBearingFactor(
    const size_t poseIdx, const size_t ugvIdx,
    const Point3 &bearing_measurement, const double &range_measurement,
    const int &robotID) {
  // add bearing measurement with a covaraince
  Unit3 bearing_measurement3D = Pose3().bearing(bearing_measurement);
  // IMPORTANT: bearing vector should be expressed in body frame!
  // RCLCPP_INFO_STREAM_THROTTLE(1,
  //                          "bearing vector should be expressed in body
  //                          frame");

  fgraph.add(BearingRangeFactor3D(getSymbol(robotID, poseIdx), U(ugvIdx),
                                  bearing_measurement3D, range_measurement,
                                  noise_model_bearing));
  // Only for active SLAM:
  // fgraph_loop.add(BearingRangeFactor3D(X(poseIdx),
  //                                      U(ugvIdx), bearing_measurement3D,
  //                                      range_measurement,
  //                                      noise_model_bearing));
  if (latest_landmark_counter < ugvIdx) {
    latest_landmark_counter = ugvIdx;
  }
}

void SemanticFactorGraph::addCylinderFactor(const size_t poseIdx,
                                            const size_t cylIdx,
                                            const Pose3 &pose,
                                            const CylinderMeasurement &cylinder,
                                            bool alreadyExists,
                                            const int &robotID) {
  // pose here is tf_map_to_sensor, not tf_sensor_to_map, hence it needs to be
  // inverted
  CylinderMeasurement c_local = cylinder.project(pose.inverse());
  fgraph.add(CylinderFactor(getSymbol(robotID, poseIdx), L(cylIdx), c_local,
                            noise_model_cylinder));
  if (!alreadyExists) {
    fvalues.insert(L(cylIdx), cylinder);
  }
}

void SemanticFactorGraph::addCubeFactor(
    const size_t poseIdx, const size_t cubeIdx, const Pose3 &pose,
    const CubeMeasurement &cube_global_meas_raw, bool alreadyExists,
    const int &robotID) {
  // pose here is tf_map_to_sensor, not tf_sensor_to_map, hence it needs to be
  // inverted
  CubeMeasurement cube_global_meas = cube_global_meas_raw;

  // Temporarily handling the edge case of cuboid having yaw close to 0 or
  // pi will have yaw values flipping / jumping back and forth between 0 and
  // pi, which cause the optimization to break, hard coding it to be 0 for now
  // Update: removed this since our process node can handle this properly

  CubeMeasurement cube_local_meas = cube_global_meas.project(pose.inverse());
  fgraph.add(CubeFactor(getSymbol(robotID, poseIdx), C(cubeIdx),
                        cube_local_meas, noise_model_cube));

  if (cube_local_meas.pose.translation().norm() > 50) {
    RCLCPP_WARN_THROTTLE(gfg_logger(), *rclcpp::Clock::make_shared(), 1000,
                         "cube_local_meas.pose.translation().norm() is larger "
                         "than 25 meters, maybe it is due to the front end keeping "
                         "track of observations over a long time or maybe it is because "
                         " the robot is moving fast!!");
  }

  if (!alreadyExists) {
    // Initialize with first measurement of cube (in fixed world frame)
    fvalues.insert(C(cubeIdx), cube_global_meas);
  }
}

void SemanticFactorGraph::addLoopClosureFactor(const Pose3 poseRelative,
                                               const size_t prevIdx,
                                               const size_t robotID1,
                                               const size_t curIdx,
                                               const size_t robotID2) {
  // Correct interface for BetweenFactor<Pose3>(pose1, pose2, H_pose2^pose1,
  // noise) poseRelative is H_query^candidate, i.e., from query pose to
  // candidate (history) pose candidate is robotID1, prevIdx query is robotID2,
  // curIdx
  fgraph.add(BetweenFactor<Pose3>(getSymbol(robotID1, prevIdx),
                                  getSymbol(robotID2, curIdx), poseRelative,
                                  noise_model_closure));
}

void SemanticFactorGraph::solve() {
  // RCLCPP_INFO_STREAM("START SOLVING THE FACTOR GRAPH OPTIMIZATION PROBLEM");
  isam->update(fgraph, fvalues);
  // Only for active SLAM:
  // isam_loop->update(fgraph_loop, fvalues_loop);

  // Extract the result/current estimates
  currEstimate = isam->calculateEstimate();

  // Reset the newFactors and newValues list
  fgraph.resize(0);
  fvalues.clear();
}

CylinderMeasurement SemanticFactorGraph::getCylinder(const int idx) {
  return currEstimate.at<CylinderMeasurement>(L(idx));
}

CubeMeasurement SemanticFactorGraph::getCube(const int idx) {
  return currEstimate.at<CubeMeasurement>(C(idx));
}

Point3 SemanticFactorGraph::getCentroidLandmark(const int idx) {
  if (isam->valueExists(U(idx))) {
    return currEstimate.at<Point3>(U(idx));
  } else {
    return Point3();
  }
}

bool SemanticFactorGraph::getPose(const size_t idx, const int &robotID,
                                  Pose3 &poseOut) {
  if (robotID >= 0 || robotID < MAX_NUM_ROBOTS) {
    if (isam->valueExists(getSymbol(robotID, idx))) {
      poseOut = currEstimate.at<Pose3>(getSymbol(robotID, idx));
      return true;
    } else {
      printf(
          "############# Error!!! Node of interest does not exist in factor "
          "graph #############\n");
      std::cout << "the node is X(" << idx << ")" << std::endl;
      poseOut = Pose3();
      return false;
    }
    // return currEstimate.at<Pose3>(getSymbol(robotID, idx));
  } else {
    RCLCPP_ERROR_STREAM(gfg_logger(),
                        "############# Error: "
                            << robotID << " is an invalid robotID !!! #############");
    // printf("############# Error: invalid robotID!!! #############\n");
    poseOut = Pose3();
    return false;
  }
}

Eigen::MatrixXd SemanticFactorGraph::getPoseCovariance(const int idx,
                                                       const int &robotID) {
  if (robotID >= 0 || robotID < MAX_NUM_ROBOTS) {
    return isam->marginalCovariance(getSymbol(robotID, idx));
  } else {
    RCLCPP_ERROR_STREAM(gfg_logger(),
                        "############# Error: "
                            << robotID << " is an invalid robotID !!! #############");
    return isam->marginalCovariance(getSymbol(robotID, idx));
  }
}

Symbol SemanticFactorGraph::getSymbol(const int &robotID, const int idx) {
  Symbol returned_symbol;
  switch (robotID) {
    case 0:
      returned_symbol = X(idx);
      break;
    case 1:
      returned_symbol = Y(idx);
      break;
    case 2:
      returned_symbol = Z(idx);
      break;
    case 3:
      returned_symbol = M(idx);
      break;
    case 4:
      returned_symbol = N(idx);
      break;
    case 5:
      returned_symbol = O(idx);
      break;
    case 6:
      returned_symbol = P(idx);
      break;
    case 7:
      returned_symbol = Q(idx);
      break;
    case 8:
      returned_symbol = R(idx);
      break;
    case 9:
      returned_symbol = S(idx);
      break;
    case 10:
      returned_symbol = T(idx);
      break;
    case 11:
      returned_symbol = V(idx);
      break;
    case 12:
      returned_symbol = W(idx);
      break;
    default:
      break;
  }
  return returned_symbol;
}

Symbol SemanticFactorGraph::getSymbol(const int &robotID, const size_t idx) {
  Symbol returned_symbol;
  switch (robotID) {
    case 0:
      returned_symbol = X(idx);
      break;
    case 1:
      returned_symbol = Y(idx);
      break;
    case 2:
      returned_symbol = Z(idx);
      break;
    case 3:
      returned_symbol = M(idx);
      break;
    case 4:
      returned_symbol = N(idx);
      break;
    case 5:
      returned_symbol = O(idx);
      break;
    case 6:
      returned_symbol = P(idx);
      break;
    case 7:
      returned_symbol = Q(idx);
      break;
    case 8:
      returned_symbol = R(idx);
      break;
    case 9:
      returned_symbol = S(idx);
      break;
    case 10:
      returned_symbol = T(idx);
      break;
    case 11:
      returned_symbol = V(idx);
      break;
    case 12:
      returned_symbol = W(idx);
      break;
    default:
      break;
  }
  return returned_symbol;
}

// ONLY FOR ACTIVE SLAM, ALL FUNCTIONS BELOW
// -----------------------------------------------------
// void SemanticFactorGraph::logEntropy() {
//   // keep current covaraince matrix
//   double sum_entropy_pose = 0.0;
//   double sum_entropy_landmark = 0.0;
//   size_t num_valid_poses = 0;
//   size_t num_valid_landmarks = 0;
//   // iterate through all latest_landmark_counter and latest_to_idx_
//   for (size_t i = 0; i < latest_to_idx_; i++) {
//     // get the covariance matrix of the current pose
//     if (isam->valueExists(X(i)) && isam_loop->valueExists(X(i))) {
//     gtsam::Matrix cov = isam->marginalCovariance(X(i));
//     sum_entropy_pose += cov.trace();
//     num_valid_poses++;
//     }
//   }

//   // RCLCPP_ERROR_STREAM("landmark marginal cov printing, num of landark is "
//   //                  << latest_landmark_counter);
//   for (size_t i = 0; i < latest_landmark_counter; i++) {
//     // get the covariance matrix of the current pose
//     if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
//       // RCLCPP_ERROR_STREAM("getting landmark cov ");
//       gtsam::Matrix cov = isam->marginalCovariance(U(i));
//       // RCLCPP_ERROR_STREAM("landmark marginal cov: " << cov);
//       sum_entropy_landmark += cov.trace();
//       num_valid_landmarks++;
//     }
//   }

//   std::ofstream entropy_log_file("/home/yuezhan/bags/entropy_log.txt",
//   ios::app); if (entropy_log_file.fail()){
//     cout << "open file error!\n";
//   }
//   rclcpp::Time time_now = node_->now();
//   // write a header for entropy log file if it is empty, header is TIME,
//   ENTROPY_POSE, ENTROPY_LANDMARK, NUM_VALID_POSES if
//   (entropy_log_file.tellp() == 0) {
//     entropy_log_file << "TIME, ENTROPY_POSE, ENTROPY_LANDMARK,
//     NUM_VALID_POSES, NUM_VALID_LANDMARKS" << std::endl;
//   }
//   entropy_log_file << (time_now-start_time_).toSec() << ", " <<
//   sum_entropy_pose << ", " << sum_entropy_landmark << ", " << num_valid_poses
//   << ", " << num_valid_landmarks << std::endl; entropy_log_file.close();

// }

// double SemanticFactorGraph::estimateClosureInfoGain(
//     const std::vector<size_t> &candidateTrajPoseIndices,
//     const std::vector<double> &travel_distances) {
//   // candidateTrajPoseIndices is a vector of pose indices in the candidate
//   // trajecotry from current pose to key pose 1, key pose 2, ..., key pose n

//   // travel_distances is a vector of travel distances between each pair of
//   poses

//   // assert length of travel_distances is one less than length of
//   // candidateTrajPoseIndices
//   assert(travel_distances.size() == candidateTrajPoseIndices.size() - 1);

//   // safety check
//   for (size_t i = 0; i < candidateTrajPoseIndices.size(); i++) {
//     size_t thisPoseIdx = candidateTrajPoseIndices[i];
//     if (currEstimate.exists(X(thisPoseIdx)) == false) {
//       RCLCPP_ERROR_STREAM("current pose index is not in the graph, error!");
//       return -1;
//     }
//   }

//   NonlinearFactorGraph fgraph_loop_tmp;
//   // iterate through all poses in the candidate trajectory
//   for (size_t i = 0; i < candidateTrajPoseIndices.size() - 1; i++) {
//     size_t currentPoseIdx = candidateTrajPoseIndices[i];
//     size_t keyPoseIdx = candidateTrajPoseIndices[i + 1];
//     auto motion_noise = noiseModel::Diagonal::Sigmas(
//         noise_model_pose_vec_per_m * travel_distances[i]);

//     // add between factor between current pose and history pose
//     Pose3 pose = currEstimate.at<Pose3>(X(currentPoseIdx));
//     Pose3 pose_history = currEstimate.at<Pose3>(X(keyPoseIdx));
//     // TODO: flip order of pose_history and pose, either way should work
//     though Pose3 pose_relative = pose_history.between(pose);
//     fgraph_loop_tmp.add(BetweenFactor<Pose3>(X(keyPoseIdx),
//     X(currentPoseIdx),
//                                              pose_relative, motion_noise));
//   }

//   // add fake loop factor
//   ISAM2Result result = isam_loop->update(fgraph_loop_tmp);
//   FactorIndices factor_ids = result.newFactorsIndices;

//   result.print();

//   // keep current covaraince matrix
//   double sum_entropy_pose = 0.0;
//   double sum_entropy_fake_loop_pose = 0.0;
//   double sum_entropy_landmark = 0.0;
//   double sum_entropy_fake_loop_landmark = 0.0;
//   // iterate through all latest_landmark_counter and latest_to_idx_
//   for (size_t i = 0; i < latest_to_idx_; i++) {
//     // get the covariance matrix of the current pose
//     gtsam::Matrix cov = isam->marginalCovariance(X(i));
//     gtsam::Matrix cov_loop = isam_loop->marginalCovariance(X(i));
//     sum_entropy_pose += cov.trace();
//     sum_entropy_fake_loop_pose += cov_loop.trace();
//   }

//   // RCLCPP_ERROR_STREAM("landmark marginal cov printing, num of landark is "
//   //                  << latest_landmark_counter);
//   for (size_t i = 0; i < latest_landmark_counter; i++) {
//     // get the covariance matrix of the current pose
//     if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
//       // RCLCPP_ERROR_STREAM("getting landmark cov ");
//       gtsam::Matrix cov = isam->marginalCovariance(U(i));
//       // RCLCPP_ERROR_STREAM("landmark marginal cov: " << cov);
//       gtsam::Matrix cov_loop = isam_loop->marginalCovariance(U(i));
//       sum_entropy_landmark += cov.trace();
//       sum_entropy_fake_loop_landmark += cov_loop.trace();
//       // RCLCPP_ERROR_STREAM("getting landmark cov ends ");
//     }
//   }
//   // RCLCPP_ERROR_STREAM("landmark marginal cov printing ends");

//   double info_gain_pose = sum_entropy_pose - sum_entropy_fake_loop_pose;
//   double info_gain_landmark =
//       sum_entropy_landmark - sum_entropy_fake_loop_landmark;

//   RCLCPP_ERROR_STREAM("sum_entropy_pose: " << sum_entropy_pose);
//   RCLCPP_ERROR_STREAM(
//       "sum_entropy_fake_loop_pose: " << sum_entropy_fake_loop_pose);
//   RCLCPP_ERROR_STREAM("sum_entropy_landmark: " << sum_entropy_landmark);
//   RCLCPP_ERROR_STREAM(
//       "sum_entropy_fake_loop_landmark: " << sum_entropy_fake_loop_landmark);
//   RCLCPP_ERROR_STREAM("info_gain_pose: " << info_gain_pose);
//   RCLCPP_ERROR_STREAM("info_gain_landmark: " << info_gain_landmark);

//   // remove fake loop factor
//   ISAM2Result result_after_delete =
//       isam_loop->update(NonlinearFactorGraph(), Values(), factor_ids);

//   bool sanity_check = true;
//   // sanity check the covariance matrix
//   if (sanity_check) {
//     // keep current covaraince matrix
//     sum_entropy_pose = 0.0;
//     sum_entropy_fake_loop_pose = 0.0;
//     sum_entropy_landmark = 0.0;
//     sum_entropy_fake_loop_landmark = 0.0;
//     // iterate through all latest_landmark_counter and latest_to_idx_
//     for (size_t i = 0; i < latest_to_idx_; i++) {
//       // get the covariance matrix of the current pose
//       if (isam->valueExists(X(i)) && isam_loop->valueExists(X(i))) {
//         gtsam::Matrix cov = isam->marginalCovariance(X(i));
//         gtsam::Matrix cov_loop = isam_loop->marginalCovariance(X(i));
//       sum_entropy_pose += cov.trace();
//       sum_entropy_fake_loop_pose += cov_loop.trace();
//       }
//     }

//     for (size_t i = 0; i < latest_landmark_counter; i++) {
//       // get the covariance matrix of the current pose
//       if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
//         gtsam::Matrix cov = isam->marginalCovariance(U(i));
//         // RCLCPP_ERROR_STREAM("landmark marginal cov: " << cov);
//         gtsam::Matrix cov_loop = isam_loop->marginalCovariance(U(i));
//         sum_entropy_landmark += cov.trace();
//         sum_entropy_fake_loop_landmark += cov_loop.trace();
//       }
//       // else {
//       //   RCLCPP_ERROR_STREAM("landmark marginal cov for landmark # "
//       //                    << i << "not exist");
//       // }
//     }
//     double info_gain_pose = sum_entropy_pose - sum_entropy_fake_loop_pose;
//     double info_gain_landmark =
//         sum_entropy_landmark - sum_entropy_fake_loop_landmark;
//     RCLCPP_ERROR_STREAM(
//         "++++++++++++++++++++++++++++++++++++++++SANITY "
//         "CHECK+++++++++++++++++++++++");

//     // RCLCPP_ERROR_STREAM("sum_entropy_pose: " << sum_entropy_pose);
//     // RCLCPP_ERROR_STREAM(
//     //     "sum_entropy_fake_loop_pose: " << sum_entropy_fake_loop_pose);
//     // RCLCPP_ERROR_STREAM("sum_entropy_landmark: " << sum_entropy_landmark);
//     // RCLCPP_ERROR_STREAM(
//     //     "sum_entropy_fake_loop_landmark: " <<
//     //     sum_entropy_fake_loop_landmark);
//     RCLCPP_ERROR_STREAM(
//         "info_gain_pose difference (should be 0): " << info_gain_pose);
//     RCLCPP_ERROR_STREAM(
//         "info_gain_landmark difference (should be 0): " <<
//         info_gain_landmark);
//     bool is_equal = isam->equals(*isam_loop);
//     RCLCPP_ERROR_STREAM("two factor graphs should be equal, are they? >> "
//                      << is_equal);
//     RCLCPP_ERROR_STREAM(
//         "++++++++++++++++++++++++++++++++++++++++SANITY "
//         "CHECK+++++++++++++++++++++++");
//   }
//   // total weighted info gain
//   double total_info_gain = 10.0 * info_gain_pose + info_gain_landmark;
//   return total_info_gain;
// }
// -----------------------------------------------------------------------
