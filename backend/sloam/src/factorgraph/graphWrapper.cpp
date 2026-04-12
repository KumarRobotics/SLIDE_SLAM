/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <graphWrapper.h>
#include <rclcpp/rclcpp.hpp>

namespace {
inline rclcpp::Logger gw_logger() {
  return rclcpp::get_logger("SemanticFactorGraphWrapper");
}
}  // namespace

SemanticFactorGraphWrapper::SemanticFactorGraphWrapper(int num_of_robots)
    : numRobots(num_of_robots) {
  cyl_counter_ = 0;
  cube_counter_ = 0;
  point_landmark_counter_ = 0;
  for (int i = 0; i < numRobots; i++) {
    pose_counter_robot_.push_back(0);
    robot_prev_pose_.push_back(gtsam::Pose3());
  }
  // keep this because we need it for old indoor activeSlamInputNode
  // should remove this after we finish the new activeSlamInputNode
  pose_counter_robot1_ = 0;
}

// for active team localization
bool SemanticFactorGraphWrapper::addLoopClosureObservation(
    const SE3 &relativeMotionSE3, const SE3 &poseEstimateSE3,
    const boost::array<double, 36> &cov, const SE3 &loop_closure_relative_pose,
    const size_t &closure_matched_pose_idx) {
  // default only have 1 aerial robot for active team localization
  gtsam::Pose3 relativeMotion(relativeMotionSE3.matrix());
  gtsam::Pose3 poseEstimate(poseEstimateSE3.matrix());
  int robotID = 0;
  bool optimize = false;

  if (pose_counter_robot1_ == 0) {
    RCLCPP_ERROR_STREAM(gw_logger(),
        "ERROR: First pose should not be the loop closure pose!!!!!!!!!");
    return false;
  } else {
    // add BetweenFactor for two consecutive poses (initialize the new pose node
    // with the curr_pose too)
    bool loopClosureFound = true;

    addKeyPoseAndBetween(pose_counter_robot1_ - 1, pose_counter_robot1_,
                         relativeMotion, poseEstimate, robotID, cov,
                         loopClosureFound, loop_closure_relative_pose,
                         closure_matched_pose_idx);
    pose_counter_robot1_++;
    RCLCPP_INFO_STREAM(gw_logger(),
                       "Running factor-graph optimization, pose counter is: "
                           << pose_counter_robot1_);
    solve();
    return true;
  }
}

bool SemanticFactorGraphWrapper::addSLOAMObservation(
    const CylinderMapManager &semanticMap, const CubeMapManager &cube_semantic_map,
    const EllipsoidMapManager &ellipsoid_semantic_map,
    const std::vector<int> &cyl_matches, const std::vector<Cylinder> &cylinders,
    const std::vector<int> &cube_matches,
    const std::vector<Cube> &scan_cubes_world,
    const std::vector<int> &ellipse_matches,
    const std::vector<Ellipsoid> &ellipses, const SE3 &relativeMotionSE3,
    const SE3 &poseEstimateSE3, const int &robotID, bool opt) {
  size_t pose_counter;

  gtsam::Pose3 curr_pose(poseEstimateSE3.matrix());
  gtsam::Pose3 relativeMotion(relativeMotionSE3.matrix());

  pose_counter = pose_counter_robot_[robotID];
  // ROS_INFO_STREAM("################ ROBOT " << robotID << " pose counter: "
  //                                           << "################");

  if (pose_counter == 0) {
    // set priors for the first pose
    setPriors(curr_pose, robotID);
    RCLCPP_WARN_STREAM(gw_logger(),
        "EROOR: Factor graph optimization is done when adding the first "
        "pose prior, this may cause problems!!!");
  } else {
    // if cov is not specified, deafult covariance will be used, see
    // noise_model_pose param in graph.cpp
    addKeyPoseAndBetween(pose_counter - 1, pose_counter, relativeMotion,
                         curr_pose, robotID);
  }
  const auto matchesMap = semanticMap.getMatchesMap();
  const auto cubeMatchesMap = cube_semantic_map.getMatchesMap();

  ////////// CYLINDER LANDMARK FACTOR ADDING //////////
  for (auto i = 0; i < cyl_matches.size(); i++) {
    if (cyl_matches[i] == -1) {
      // No matches, add as new landmarks
      addCylinderFactor(pose_counter, cyl_counter_, curr_pose, cylinders[i],
                        false, robotID);
      cyl_counter_++;
    } else {
      // transform from observation to map index
      int mapIdx = matchesMap.at(cyl_matches[i]);

      addCylinderFactor(pose_counter, mapIdx, curr_pose, cylinders[i], true,
                        robotID);
    }
  }

  ////////// CUBE LANDMARK FACTOR ADDING //////////
  for (auto i = 0; i < cube_matches.size(); i++) {
    if (cube_matches[i] == -1) {
      // No matches, add as new landmarks
      addCubeFactor(pose_counter, cube_counter_, curr_pose, scan_cubes_world[i],
                    false, robotID);
      cube_counter_++;
    } else {
      // transform from observation to map index
      int mapIdx = cubeMatchesMap.at(cube_matches[i]);
      addCubeFactor(pose_counter, mapIdx, curr_pose, scan_cubes_world[i], true,
                    robotID);
    }
  }

  ////////// ELLIPSOID LANDMARK FACTOR ADDING //////////
  const auto ellipMatchesMap = ellipsoid_semantic_map.getMatchesMap();
  // calculate range and bearing factors based on the landmark_body_positions
  std::vector<Point3> bearing_factors;
  std::vector<double> range_factors;

  // project ellipsoid to body frame using the current pose
  // create a new ellipsoid object and update the pose
  std::vector<Ellipsoid> ellipses_body_frame;
  for (size_t i = 0; i < ellipses.size(); i++) {
    Ellipsoid ellipsoid_body(ellipses[i].model.pose, ellipses[i].model.scale,
                             ellipses[i].model.semantic_label);
    // curr_pose is from robot to world, so we need inverse of that, which is
    // pose_ellipse_body = H_world2robot @ pose_ellipse_world
    ellipsoid_body.model.pose =
        Pose3(curr_pose.matrix().inverse() * ellipses[i].model.pose.matrix());
    ellipses_body_frame.push_back(ellipsoid_body);
  }
  for (size_t i = 0; i < ellipses_body_frame.size(); i++) {
    // range is the norm of the landmark_body_positions
    double range_factor =
        ellipses_body_frame[i].model.pose.translation().norm();
    // bearing is the direction, i.e., normalized landmark_body_positions use
    // .normalized()
    Point3 bearing_factor =
        ellipses_body_frame[i].model.pose.translation().normalized();
    bearing_factors.push_back(bearing_factor);
    range_factors.push_back(range_factor);
  }

  for (auto i = 0; i < ellipse_matches.size(); i++) {
    if (ellipse_matches[i] == -1) {
      // No matches, add as new landmarks
      addPointLandmarkKey(point_landmark_counter_,
                          ellipses[i].model.pose.translation());
      addRangeBearingFactor(pose_counter, point_landmark_counter_,
                            bearing_factors[i], range_factors[i], robotID);
      point_landmark_labels_.push_back(ellipses[i].model.semantic_label);
      point_landmark_counter_++;
    } else {
      // transform from observation to map index
      int mapIdx = ellipMatchesMap.at(ellipse_matches[i]);
      addRangeBearingFactor(pose_counter, mapIdx, bearing_factors[i],
                            range_factors[i], robotID);
    }
  }

  // update the pose counter
  pose_counter++;
  pose_counter_robot_[robotID] = pose_counter;
  // ROS_INFO_STREAM("pose counter for robotID " << robotID
  //                                             << " is: " << pose_counter);

  bool optimize = true;
  // updated logic: always optimize unless opt is set to false
  if (optimize && opt) {
    // ROS_INFO_STREAM("Running factor graph optimization");
    size_t num_factors = fgraph.size();
    size_t num_keys = fgraph.keys().size();
    // ros::Time graph_optimization_start_time = ros::Time::now();
    solve();
    // -------------------------------
    // ONLY FOR BECHMARKING TIME PERFORMANCE
    // ros::Time graph_optimization_end_time = ros::Time::now();
    // double graph_optimization_time = (graph_optimization_end_time -
    // graph_optimization_start_time).toSec();
    // fg_optimization_time.push_back(graph_optimization_time);
    // ROS_INFO_STREAM("graph optimization time is: " <<
    // (graph_optimization_end_time - graph_optimization_start_time).toSec() <<
    // " seconds for a graph with " << num_factors << " factors and " <<
    // num_keys << " keys");
    // -------------------------------
    return true;
  }
  // ROS_WARN(
  //     "WARNING: addSLOAMObservation: not optimizing! either optimize is false, "
  //     "or "
  //     "opt is set false, may cause problems for single robot case! If "
  //     "using multi robot, this may be expected");
  return false;
}

void SemanticFactorGraphWrapper::updateCylinder(
    const CylinderMeasurement &measurement, Cylinder &cyl) {
  cyl.model.root = measurement.root;
  cyl.model.ray = measurement.ray;
  cyl.model.radius = measurement.radius;
}

void SemanticFactorGraphWrapper::updateCube(const CubeMeasurement &measurement,
                                            Cube &cube) {
  cube.model.scale = measurement.scale;
  cube.model.pose = measurement.pose;
}

void SemanticFactorGraphWrapper::updateEllipsoid(const Point3 &measurement,
                                                 Ellipsoid &ellipsoid) {
  // update position only
  ellipsoid.model.pose = Pose3(Rot3(), measurement);
}

// for generic SLOAM
void SemanticFactorGraphWrapper::updateFactorGraphMap(
    CylinderMapManager &semanticMap, CubeMapManager &cube_semantic_map,
    EllipsoidMapManager &ellipsoid_semantic_map) {
  auto &map = semanticMap.getRawMap();
  auto &cube_map = cube_semantic_map.getRawMap();
  auto &ellipsoid_map = ellipsoid_semantic_map.getRawMap();
  for (auto i = 0; i < cyl_counter_; ++i) {
    updateCylinder(getCylinder(i), map[i]);
  }
  for (auto i = 0; i < cube_counter_; ++i) {
    updateCube(getCube(i), cube_map[i]);
  }

  for (auto i = 0; i < point_landmark_counter_; ++i) {
    updateEllipsoid(getCentroidLandmark(i), ellipsoid_map[i]);
  }
}

void SemanticFactorGraphWrapper::getCurrPose(
    SE3 &curr_pose, const int &robotID,

    boost::optional<Eigen::MatrixXd &> cov) {
  size_t pose_counter;
  pose_counter = pose_counter_robot_[robotID];
  gtsam::Pose3 pose;

  bool pose_valid = getPose(pose_counter - 1, robotID, pose);
  curr_pose = SE3(pose.matrix());
  if (!pose_valid) {
    RCLCPP_ERROR_STREAM(gw_logger(),
        "getCurrPose fail to fetch pose for pose_idx : " << pose_counter - 1);
  }
  if (cov) {
    std::cout << "graphWarpper.cpp the getPoseCovarinace is probably not "
                 "implemented correctly!"
              << '\n';
    *cov = getPoseCovariance(pose_counter - 1, robotID);
  }
}

bool SemanticFactorGraphWrapper::getPoseByID(SE3 &curr_pose,
                                             const int &poseID) {
  gtsam::Pose3 pose;
  bool pose_valid = getPose(poseID, 0, pose);
  curr_pose = SE3(pose.matrix());
  if (!pose_valid) {
    RCLCPP_ERROR_STREAM(gw_logger(),
        "getPoseByID fail to fetch pose for pose_idx : " << poseID);
    return false;
  } else {
    return true;
  }
}

void SemanticFactorGraphWrapper::getAllPoses(std::vector<SE3> &optimized_poses,
                                             std::vector<size_t> &pose_inds,
                                             const int &robotID) {
  optimized_poses.clear();
  pose_inds.clear();
  for (size_t i = 0; i < pose_counter_robot_[robotID]; i++) {
    gtsam::Pose3 pose;
    bool pose_valid = getPose(i, robotID, pose);
    if (!pose_valid) {
      RCLCPP_ERROR_STREAM(gw_logger(),
          "MAJOR ERROR: getAllPoses fail to fetch pose for pose_idx: " << i);
      RCLCPP_ERROR(gw_logger(), "YOU MUST CORRECT THIS ERROR!!!");
    } else {
      optimized_poses.push_back(SE3(pose.matrix()));
      pose_inds.push_back(i);
    }
  }

  // sanity check
  if (pose_inds.size() != optimized_poses.size()) {
    RCLCPP_ERROR_STREAM(gw_logger(),
        "ERROR: getAllPoses fail to due to pose_inds and optimized_poses "
        "having "
        "different sizes!!!");
  }
}

void SemanticFactorGraphWrapper::getAllCentroidLandmarks(
    std::vector<SE3> &optimized_landmark_pos,
    std::vector<size_t> &landmark_inds) {
  for (auto i = 0; i < point_landmark_counter_; i++) {
    gtsam::Point3 landmark_position = getCentroidLandmark(i);
    if (landmark_position.x() == gtsam::Point3().x() &&
        landmark_position.y() == gtsam::Point3().y() &&
        landmark_position.z() == gtsam::Point3().z()) {
      // ROS_INFO_STREAM("fail to fetch landmark idx: " << i);
    } else {
      gtsam::Pose3 landmark_pose =
          gtsam::Pose3(gtsam::Rot3(), landmark_position);
      optimized_landmark_pos.push_back(SE3(landmark_pose.matrix()));
      landmark_inds.push_back(i);
    }
  }
}

void SemanticFactorGraphWrapper::getAllCentroidLandmarks(
    std::vector<SE3> &optimized_landmark_pos) {
  for (auto i = 0; i < point_landmark_counter_; i++) {
    gtsam::Point3 landmark_position = getCentroidLandmark(i);
    if (landmark_position.x() == gtsam::Point3().x() &&
        landmark_position.y() == gtsam::Point3().y() &&
        landmark_position.z() == gtsam::Point3().z()) {
      RCLCPP_INFO_STREAM(gw_logger(), "fail to fetch landmark idx: " << i);
    } else {
      gtsam::Pose3 landmark_pose =
          gtsam::Pose3(gtsam::Rot3(), landmark_position);
      optimized_landmark_pos.push_back(SE3(landmark_pose.matrix()));
    }
  }
}

void SemanticFactorGraphWrapper::getAllCentroidLandmarksAndLabels(
    std::vector<SE3> &optimized_landmark_pos,
    std::vector<int> &landmark_labels) {
  for (auto i = 0; i < point_landmark_counter_; i++) {
    gtsam::Point3 landmark_position = getCentroidLandmark(i);
    if (landmark_position.x() == gtsam::Point3().x() &&
        landmark_position.y() == gtsam::Point3().y() &&
        landmark_position.z() == gtsam::Point3().z()) {
      // ROS_INFO_STREAM("fail to fetch landmark idx: " << i);
    } else {
      gtsam::Pose3 landmark_pose =
          gtsam::Pose3(gtsam::Rot3(), landmark_position);
      optimized_landmark_pos.push_back(SE3(landmark_pose.matrix()));
      if (i < point_landmark_labels_.size()) {
        landmark_labels.push_back(point_landmark_labels_[i]);
      } else {
        landmark_labels.push_back(-1);
        RCLCPP_ERROR_STREAM(gw_logger(),
            "ERROR: point_landmark_labels_ is not the same size as "
            "point_landmark_counter_, which is abnormal!!!");
      }
    }
  }
}

size_t SemanticFactorGraphWrapper::getPoseCounterById(
    const int &robotID) const {
  return pose_counter_robot_[robotID];
}