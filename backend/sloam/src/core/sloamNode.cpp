/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#include <fstream>
#include <pcl/common/io.h>
#include <rclcpp/rclcpp.hpp>
#include <sloamNode.h>

#include <chrono>

namespace sloam {

namespace {
template <typename ParamT>
ParamT sn_declare_or_get(rclcpp::Node *node, const std::string &name,
                         const ParamT &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParamT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ParamT>();
}
}  // namespace

SLOAMNode::SLOAMNode(rclcpp::Node *node)
    : dbManager(node), inter_loopCloser_(node), intra_loopCloser_(node),
      node_(node) {

  // initialize intra loop closure and inter loop closure nodes
  intra_loopCloser_.inter_loop_closure = false;
  inter_loopCloser_.inter_loop_closure = true;

  debugMode_ = sn_declare_or_get<bool>(node_, "debug_mode", false);
  if (debugMode_) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "Running SLOAM in Debug Mode" << std::endl);
    auto ret = rcutils_logging_set_logger_level(
        node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    (void)ret;
  } else {
    auto ret = rcutils_logging_set_logger_level(
        node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    (void)ret;
  }

  std::string node_name = node_->get_name();
  std::string idName = node_name + "/hostRobotID";


  use_slidematch_ = sn_declare_or_get<bool>(node_, "use_slidematch", true);

  int num_of_robots = sn_declare_or_get<int>(node_, "number_of_robots", 1);
  inter_robot_place_recognition_frequency_ = sn_declare_or_get<double>(
      node_, "inter_robot_place_recognition_frequency", 0.1);
  intra_robot_place_recognition_frequency_ = sn_declare_or_get<double>(
      node_, "intra_robot_place_recognition_frequency", 0.1);
  // place_recognition_attempt_time_offset
  double place_recognition_attempt_time_offset = sn_declare_or_get<double>(
      node_, "place_recognition_attempt_time_offset", 1.5);
  for (int i = 0; i < num_of_robots; i++) {
    std::string topic_name =
        node_name + "/robot" + std::to_string(i) + "/trajectory";
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();
    pubRobotTrajectory_.push_back(
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            topic_name, latched_qos));
  }

  hostRobotID = sn_declare_or_get<int>(node_, idName, 0);


  // initialize last_intra_loop_closure_stamp_ as current time
  // add offset the timestamp a bit to avoid all robots calling loop closure at the same time when running on the same machine
  rclcpp::Time now_t = node_->now();
  last_intra_loop_closure_stamp_ =
      now_t + rclcpp::Duration::from_seconds(
                  place_recognition_attempt_time_offset * hostRobotID);
  last_inter_loop_closure_stamp_ =
      now_t + rclcpp::Duration::from_seconds(
                  place_recognition_attempt_time_offset * hostRobotID);

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  pubMapTreeModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "cylinders_map", latched_qos);
  pubSubmapTreeModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "submap_cylinder_models", latched_qos);
  pubObsTreeModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "debug/obs_tree_models", latched_qos);
  pubMapGroundModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "debug/map_ground_model", 1);
  pubObsGroundModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "debug/obs_ground_model", 1);

  // cuboids related publishers
  pubMapCubeModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "cubes_map", latched_qos);
  pubSubmapCubeModel_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "cubes_submap", latched_qos);

  pubAllPointLandmarks_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "optimized_point_landmarks", latched_qos);

  pubObs_ = node_->create_publisher<sloam_msgs::msg::ROSObservation>(
      "observation", 10);
  pubMapPose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "map_pose", 10);

  firstScan_ = true;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  worldTfBr_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  initParams_();

  // Loop Closure
  bool turn_off_intra_loop_closure = sn_declare_or_get<bool>(
      node_, node_name + "/turn_off_intra_loop_closure", true);
  if (turn_off_intra_loop_closure) {
    RCLCPP_WARN(node_->get_logger(), "Intra Loop closure is turned off");
  } else {
    intraLoopthread_ = std::thread(&SLOAMNode::intraLoopClosureThread_, this);
    RCLCPP_WARN(node_->get_logger(), "Intra Loop closure is turned on");
  }

  // Loop Closure
  bool turn_off_inter_loop_closure = sn_declare_or_get<bool>(
      node_, node_name + "/turn_off_inter_loop_closure", true);
  if (turn_off_inter_loop_closure) {
    RCLCPP_WARN(node_->get_logger(), "Inter Loop closure is turned off");
  } else {
    interLoopthread_ = std::thread(&SLOAMNode::interLoopClosureThread_, this);
    RCLCPP_WARN(node_->get_logger(), "Inter Loop closure is turned on");
  }
  lastLoopAttemptPose_ = -1;

  // initialize the runtime analysis variables
  runtime_analysis_file = save_runtime_analysis_dir_ + "/robot" +
                          std::to_string(hostRobotID) + "_runtime_analysis.txt";
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "THE RUNTIME ANALYSIS FILE IS: " << runtime_analysis_file);
}

SLOAMNode::~SLOAMNode() {
  if (intraLoopthread_.joinable())
    intraLoopthread_.join();
  if (interLoopthread_.joinable())
    interLoopthread_.join();
}

void SLOAMNode::initParams_() {
  // PARAMETERS


  numRobots = sn_declare_or_get<int>(node_, "number_of_robots", 1);
  semanticMap_ = CylinderMapManager(numRobots);
  cube_semantic_map_ = CubeMapManager();
  ellipsoid_semantic_map_ = EllipsoidMapManager();
  factorGraph_ = SemanticFactorGraphWrapper(numRobots);
  fmParams_.cylinderMatchThresh =
      sn_declare_or_get<double>(node_, "cylinder_match_thresh", 2.0);
  fmParams_.cuboidMatchThresh =
      sn_declare_or_get<double>(node_, "cuboid_match_thresh", 2.0);
  fmParams_.ellipsoidMatchThresh =
      sn_declare_or_get<double>(node_, "ellipsoid_match_thresh", 0.75);
  fmParams_.defaultCylinderRadius =
      sn_declare_or_get<double>(node_, "default_cylinder_radius", 0.2);

  fmParams_.scansPerSweep = 1;
  fmParams_.minGroundModels =
      sn_declare_or_get<double>(node_, "min_ground_models", 50.0);
  fmParams_.maxLidarDist =
      sn_declare_or_get<double>(node_, "max_lidar_dist", 20.0);
  fmParams_.maxGroundLidarDist =
      sn_declare_or_get<double>(node_, "max_ground_dist", 30.0);
  double beam_cluster_threshold =
      sn_declare_or_get<double>(node_, "beam_cluster_threshold", 0.1);
  (void)beam_cluster_threshold;
  fmParams_.minGroundLidarDist =
      sn_declare_or_get<double>(node_, "min_ground_dist", 0.0);

  fmParams_.twoStepOptim = sn_declare_or_get<bool>(node_, "two_step_optim", true);
  double min_landmark_height =
      sn_declare_or_get<double>(node_, "min_landmark_height", 1.0);
  (void)min_landmark_height;
  int min_landmark_size = sn_declare_or_get<int>(node_, "min_landmark_size", 3);
  (void)min_landmark_size;
  int min_vertex_size = sn_declare_or_get<int>(node_, "min_vertex_size", 2);
  (void)min_vertex_size;

  fmParams_.groundRadiiBins =
      sn_declare_or_get<int>(node_, "ground_radii_bins", 2);
  fmParams_.groundThetaBins =
      sn_declare_or_get<int>(node_, "ground_theta_bins", 10);
  fmParams_.groundMatchThresh =
      sn_declare_or_get<double>(node_, "ground_match_thresh", 1.0);
  fmParams_.groundRetainThresh =
      sn_declare_or_get<double>(node_, "ground_retain_thresh", 2.0);

  fmParams_.maxTreeRadius =
      sn_declare_or_get<double>(node_, "max_tree_radius", 10.0);
  fmParams_.maxAxisTheta =
      sn_declare_or_get<double>(node_, "max_axis_theta", 50.0);
  fmParams_.maxFocusOutlierDistance = 0.5;

  fmParams_.featuresPerTree =
      sn_declare_or_get<int>(node_, "features_per_tree", 50);
  fmParams_.numGroundFeatures =
      sn_declare_or_get<int>(node_, "num_ground_features", 5);


  setFmParams(fmParams_);

  // Frame Ids
  map_frame_id_ =
      sn_declare_or_get<std::string>(node_, "map_frame_id", "map");
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "MAP FRAME " << map_frame_id_);
}

// TODO: visualize transmitted objects in a different color
void SLOAMNode::publishMap_(const rclcpp::Time stamp) {
  sloam_msgs::msg::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  if (pubMapTreeModel_->get_subscription_count() > 0) {
    auto semantic_map = semanticMap_.getFinalMap();
    visualization_msgs::msg::MarkerArray mapTMarkerArray;
    size_t cid = 500000;
    vizTreeModels(semantic_map, mapTMarkerArray, cid);
    pubMapTreeModel_->publish(mapTMarkerArray);
  }
}

void SLOAMNode::publishCubeMaps_(const rclcpp::Time stamp) {
  sloam_msgs::msg::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  // get the cubes that are observed more than once
  auto semantic_map = cube_semantic_map_.getFinalMap();
  visualization_msgs::msg::MarkerArray cubeMapTMarkerArray;
  size_t cube_id = 0;
  // publish all cubes that have been observed more than once
  vizCubeModels(semantic_map, cubeMapTMarkerArray, cube_id, true);
  pubMapCubeModel_->publish(cubeMapTMarkerArray);
  visualization_msgs::msg::MarkerArray cubeSubMapTMarkerArray;
  // publish current scan cube map
  vizCubeModels(scan_cubes_world_, cubeSubMapTMarkerArray, cube_id, false);
  pubSubmapCubeModel_->publish(cubeSubMapTMarkerArray);
}

void SLOAMNode::publishResults_(const SloamInput &sloamIn,
                                const SloamOutput &sloamOut, rclcpp::Time stamp,
                                const int &robotID) {
  publishMap_(stamp);
  publishCubeMaps_(stamp);

  std::vector<SE3> allLandmarks;
  std::vector<int> allLabels;
  factorGraph_.getAllCentroidLandmarksAndLabels(allLandmarks, allLabels);

  visualization_msgs::msg::MarkerArray allLandmarksMarkers =
      vizAllCentroidLandmarks(allLandmarks, map_frame_id_, allLabels);

  pubAllPointLandmarks_->publish(allLandmarksMarkers);

  pubMapPose_->publish(makeROSPose(sloamOut.T_Map_Curr, map_frame_id_, stamp));

    std::vector<SE3> trajectory_viz;
    std::vector<size_t> pose_idx;

    // iterate through all robots till numRobots
    for (int temp_id = 0; temp_id < numRobots; temp_id++) {
      factorGraph_.getAllPoses(trajectory_viz, pose_idx, temp_id);
      visualization_msgs::msg::MarkerArray trajMarkers =
          vizTrajectory(trajectory_viz, map_frame_id_, temp_id);
      pubRobotTrajectory_[temp_id]->publish(trajMarkers);
    }

    std::vector<SE3> trajectory;
    factorGraph_.getAllPoses(trajectory, pose_idx, robotID);

    // sanity check: the size of the stamped raw odometry pose should be exactly
    // the same as pose_counter_robot_[robotID]
    if (KeyPoseTimeStamps.size() != trajectory.size()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Key pose time stamps and trajectory size do not match!!!");
    } else if (save_robot_trajectory_as_csv_) {
      // Save the trajectory of the robot and corresponding timestamps for
      // each pose as a csv file save rotation as quaternion
      std::ofstream myfile;
      std::string fname = save_results_dir_ + "/trajectory.csv";
      myfile.open(fname);
      myfile << "x,y,z,qx,qy,qz,qw,timestamp\n";
      for (size_t i = 0; i < trajectory.size(); i++) {
        myfile << trajectory[i].translation()[0] << ","
               << trajectory[i].translation()[1] << ","
               << trajectory[i].translation()[2] << ","
               << trajectory[i].so3().unit_quaternion().x() << ","
               << trajectory[i].so3().unit_quaternion().y() << ","
               << trajectory[i].so3().unit_quaternion().z() << ","
               << trajectory[i].so3().unit_quaternion().w() << ","
               << KeyPoseTimeStamps[i].nanoseconds() << "\n";
      }
    }

    visualization_msgs::msg::MarkerArray mapTMarkerArray;
    size_t cid = 200000;
    vizTreeModels(sloamIn.submapCylinders, mapTMarkerArray, cid);
    pubSubmapTreeModel_->publish(mapTMarkerArray);

    // Publish aligned observation
    size_t cylinderId = 100000;
    visualization_msgs::msg::MarkerArray obsTMarkerArray;
    vizTreeModels(sloamOut.scanCylindersWorld, obsTMarkerArray, cylinderId);
    pubObsTreeModel_->publish(obsTMarkerArray);
}


/**
 * @brief loop closure thread for intra-loop closure
 */
void SLOAMNode::intraLoopClosureThread_() {
  rclcpp::Rate rate(0.2);
  double desired_loop_closure_interval =
      1.0 / intra_robot_place_recognition_frequency_;

  while (rclcpp::ok()) {
    // if current stamp has passed more than 15 seconds, we continue to try loop
    // closure, otherwise we wait
    if ((node_->now() - last_intra_loop_closure_stamp_).seconds() <
        desired_loop_closure_interval) {
      RCLCPP_INFO_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 3000,
          "Just did intra loop closure or it's the beginning of the mission, waiting for "
              << desired_loop_closure_interval
              << " seconds before next loop closure");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      continue;
    }
    if (!isInLoopClosureRegion_) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;
    } else {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "isInLoopClosureRegion_ is true, attempting loop closure");
    }
    RCLCPP_DEBUG(node_->get_logger(), "****** Starting Intra Loop Closure Thread *****");
    // get latest pose and its observation to do loop closure
    semanticMapMtx_.lock();
    int latestPoseIdx = semanticMap_.getLatestPoseIdx(hostRobotID);
    semanticMapMtx_.unlock();
    if (latestPoseIdx < 20) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                           "latestPoseIdx < 20, skipping loop closure");
      continue;
    }
    dbMutex.lock();

    PoseMstPair latestPoseMstPair =
        dbManager.getHostRobotData().poseMstPacket[latestPoseIdx];
    dbMutex.unlock();
    // extract location of all observations
    std::vector<Eigen::Vector7d> measurements = prepareLCInput(
        latestPoseMstPair.cylinderMsts, latestPoseMstPair.cubeMsts,
        latestPoseMstPair.ellipsoidMsts);
    // get candidate historical pose and its submap to do loop closure
    if (latestPoseIdx == lastLoopAttemptPose_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "latestPoseIdx == lastLoopAttemptPose_, skipping loop closure");
    } else {
      num_attempts_intra_loop_closure++;
      lastLoopAttemptPose_ = latestPoseIdx;
      size_t candidatePoseIdx = 0;
      semanticMapMtx_.lock();
      double max_dist = 15;
      size_t at_least_num_of_poses_old = 50;
      bool candidate_key_pose_found = semanticMap_.getLoopCandidateIdx(
          max_dist, latestPoseIdx, candidatePoseIdx, hostRobotID,
          at_least_num_of_poses_old);
      semanticMapMtx_.unlock();
      if (candidate_key_pose_found) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "loop closure candidate history key pose found");
        SE3 query_pose = latestPoseMstPair.keyPose;
        SE3 cp = semanticMap_.getPose(candidatePoseIdx, hostRobotID);
        double submap_radius = 20;

        // get the submap at the candidate pose
        std::vector<Cylinder> candidateCylinderObs;
        semanticMapMtx_.lock();
        semanticMap_.getkeyPoseSubmap(cp, candidateCylinderObs, submap_radius,
                                      hostRobotID);
        semanticMapMtx_.unlock();

        std::vector<Cube> candidateCubeObs;
        cubeSemanticMapMtx_.lock();
        cube_semantic_map_.getkeyPoseSubmap(cp, candidateCubeObs, submap_radius,
                                            hostRobotID);
        cubeSemanticMapMtx_.unlock();

        std::vector<Ellipsoid> candidateEllipsoidObs;
        ellipsoidSemanticMapMtx_.lock();
        ellipsoid_semantic_map_.getkeyPoseSubmap(cp, candidateEllipsoidObs,
                                                 submap_radius, hostRobotID);
        ellipsoidSemanticMapMtx_.unlock();

        std::vector<Eigen::Vector7d> submaps = prepareLCInput(
            candidateCylinderObs, candidateCubeObs, candidateEllipsoidObs);

        Eigen::Matrix4d tfFromQuery2Candidate;
        int best_number_inliers = 0;
        (void)best_number_inliers;
        std::vector<Eigen::Vector3d> map_objects_matched_out;
        std::vector<Eigen::Vector3d> detection_objects_matched_out;
        RCLCPP_DEBUG(node_->get_logger(),
                     "number of measurements is: %zu", measurements.size());
        RCLCPP_DEBUG(node_->get_logger(),
                     "number of object in submaps is: %zu", submaps.size());

        // Main function for loop closure
        rclcpp::Time loop_closure_start = node_->now();
        if (intra_loopCloser_.findIntraLoopClosure(
                measurements, submaps, query_pose, cp, tfFromQuery2Candidate)) {
          num_successful_intra_loop_closure++;
          RCLCPP_INFO(node_->get_logger(), "Success: Intra Loop closure found");
          rclcpp::Time loop_closure_end = node_->now();
          RCLCPP_INFO_STREAM(
              node_->get_logger(),
              "Intra Loop Closure took "
                  << (loop_closure_end - loop_closure_start).seconds()
                  << " seconds"
                  << "between two submaps of size " << measurements.size()
                  << " and " << submaps.size());
          intra_loop_closure_time.push_back(
              (loop_closure_end - loop_closure_start).seconds());
          last_intra_loop_closure_stamp_ = node_->now();
          Eigen::Matrix3d rotation_matrix =
              tfFromQuery2Candidate.block<3, 3>(0, 0);
          Eigen::Vector3d translation_vector =
              tfFromQuery2Candidate.block<3, 1>(0, 3);

          gtsam::Pose3 relativePose = gtsam::Pose3(
              gtsam::Rot3(rotation_matrix), gtsam::Point3(translation_vector));

          // add loop closure factor to the factor graph
          factorGraphMtx_.lock();
          RCLCPP_DEBUG_STREAM(
              node_->get_logger(),
              "A Loop Closure Factor is added between Pose "
                  << latestPoseIdx << "and Pose " << candidatePoseIdx);
          factorGraph_.addLoopClosureFactor(relativePose, candidatePoseIdx,
                                            hostRobotID, latestPoseIdx,
                                            hostRobotID);
          factorGraphMtx_.unlock();
        } else{
          RCLCPP_DEBUG_STREAM(node_->get_logger(),
                              "Tried loop closure but was not succesfull");
        }
      } else {
        RCLCPP_DEBUG(node_->get_logger(),
                     "No loop closure candidate history key pose found");
      }
    }
    rate.sleep();
  }
}

void SLOAMNode::getCentroidSubmap(const std::vector<SE3> &allCentroidLandmarks,
                                  std::vector<SE3> &centroidSubmap,
                                  const SE3 &query_pose,
                                  const double &submap_radius) {
  for (auto centroid : allCentroidLandmarks) {
    if ((centroid.translation() - query_pose.translation()).norm() <
        submap_radius) {
      centroidSubmap.push_back(centroid);
    }
  }
}

std::vector<Eigen::Vector3d>
SLOAMNode::extractPosition(const std::vector<Cylinder> &candidateCylinderObs,
                           const std::vector<Cube> &candidateCubeObs,
                           const std::vector<SE3> &candidateCentroidObs) {
  std::vector<Eigen::Vector3d> lanmark_positions;
  for (auto cylinder : candidateCylinderObs) {
    lanmark_positions.emplace_back(cylinder.model.root.x(),
                                   cylinder.model.root.y(),
                                   cylinder.model.root.z());
  }
  for (auto cube : candidateCubeObs) {
    gtsam::Point3 cube_center = cube.model.pose.translation();
    lanmark_positions.emplace_back(cube_center.x(), cube_center.y(),
                                   cube_center.z());
  }
  for (auto centroid : candidateCentroidObs) {
    lanmark_positions.emplace_back(centroid.translation().x(),
                                   centroid.translation().y(),
                                   centroid.translation().z());
  }
  return lanmark_positions;
}

std::vector<Eigen::Vector3d> SLOAMNode::extractPosition(
    const std::vector<gtsam_cylinder::CylinderMeasurement>
        &candidateCylinderObs,
    const std::vector<gtsam_cube::CubeMeasurement> &candidateCubeObs,
    const std::vector<gtsam::Point3> &candidateCentroidObs) {
  std::vector<Eigen::Vector3d> lanmark_positions;
  for (auto cylinder : candidateCylinderObs) {
    lanmark_positions.emplace_back(cylinder.root.x(), cylinder.root.y(),
                                   cylinder.root.z());
  }
  for (auto cube : candidateCubeObs) {
    lanmark_positions.emplace_back(cube.pose.translation().x(),
                                   cube.pose.translation().y(),
                                   cube.pose.translation().z());
  }
  for (auto centroid : candidateCentroidObs) {
    lanmark_positions.emplace_back(centroid.x(), centroid.y(), centroid.z());
  }
  return lanmark_positions;
}

std::vector<Eigen::Vector7d>
SLOAMNode::prepareLCInput(const std::vector<Cylinder> &candidateCylinderObs,
                          const std::vector<Cube> &candidateCubeObs,
                          const std::vector<Ellipsoid> &candidateEllipsoidObs) {
  std::vector<Eigen::Vector7d> objects;
  for (auto cylinder : candidateCylinderObs) {
    Eigen::Vector7d temp_vec;
    temp_vec << cylinder.model.semantic_label, cylinder.model.root.x(),
        cylinder.model.root.y(), cylinder.model.root.z(),
        cylinder.model.radius, 0, 0;
    objects.emplace_back(temp_vec);
  }
  for (auto cube : candidateCubeObs) {
    Eigen::Vector7d temp_vec;
    temp_vec << cube.model.semantic_label, cube.model.pose.translation().x(),
        cube.model.pose.translation().y(), cube.model.pose.translation().z(),
        cube.model.scale.x(), cube.model.scale.y(), cube.model.scale.z();
    objects.emplace_back(temp_vec);
  }
  for (auto ellipsoid : candidateEllipsoidObs) {
    Eigen::Vector7d temp_vec;
    temp_vec << ellipsoid.model.semantic_label,
        ellipsoid.model.pose.translation().x(),
        ellipsoid.model.pose.translation().y(),
        ellipsoid.model.pose.translation().z(), ellipsoid.model.scale.x(),
        ellipsoid.model.scale.y(), ellipsoid.model.scale.z();
    objects.emplace_back(temp_vec);
  }
  return objects;
}

void SLOAMNode::interLoopClosureThread_() {
  rclcpp::Rate rate(1.0);
  double desired_loop_closure_interval =
      1.0 / inter_robot_place_recognition_frequency_;
  while (rclcpp::ok()) {
    if ((node_->now() - last_inter_loop_closure_stamp_).seconds() <
        desired_loop_closure_interval) {
      RCLCPP_INFO_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 3000,
          "just did inter loop closure, or in the beginning of the mission, wait for "
              << desired_loop_closure_interval
              << " seconds before trying again...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      continue;
    }
    std::vector<int> robotIDLoopClosureToFind;
    dbMutex.lock();
    for (auto iter = dbManager.getRobotDataDict().begin();
         iter != dbManager.getRobotDataDict().end(); iter++) {
      int curRobotID = iter->first;
      if (dbManager.loopClosureTf.find(curRobotID) ==
              dbManager.loopClosureTf.end() &&
          curRobotID != dbManager.getHostRobotID()) {
        robotIDLoopClosureToFind.push_back(curRobotID);
      }
    }
    dbMutex.unlock();
    num_attempts_inter_loop_closure++;
    for (auto query_robot_id : robotIDLoopClosureToFind) {
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "START TO FIND INTER LOOP CLOSURE BETWEEN ROBOTS: "
                             << query_robot_id << " AND "
                             << dbManager.getHostRobotID());
      dbMutex.lock();
      std::vector<Eigen::Vector7d> reference_map =
          dbManager.getRobotMap(dbManager.getHostRobotID());
      if (reference_map.size() == 0) {
        RCLCPP_ERROR(node_->get_logger(),
                     "current robot map is empty, skipping loop closure");
        dbMutex.unlock();
        break;
      }

      // get other Robots' map
      std::vector<Eigen::Vector7d> query_map =
          dbManager.getRobotMap(query_robot_id);
      dbMutex.unlock();
      Eigen::Matrix4d tfFromQuery2Ref;
      tfFromQuery2Ref.setIdentity();
      RCLCPP_INFO(node_->get_logger(), "Trying to do inter loop closure");
      rclcpp::Time inter_loop_closure_start = node_->now();
      bool found_inter_loop_closure = false;

      #if USE_CLIPPER
        if (use_slidematch_){
          RCLCPP_INFO(node_->get_logger(),
                      "Using SlideMatch instead of SlideGraph for inter loop closure");
          found_inter_loop_closure = inter_loopCloser_.findInterLoopClosure(
            reference_map, query_map, tfFromQuery2Ref);
        } else {
          RCLCPP_INFO(node_->get_logger(),
                      "Using SlideGraph instead of SlideMatch for inter loop closure");
          found_inter_loop_closure =
              inter_loopCloser_.findInterLoopClosureWithClipper(
                  reference_map, query_map, tfFromQuery2Ref);
        }
      #else
        if (use_slidematch_){
          RCLCPP_INFO(node_->get_logger(),
                      "Using SlideMatch instead of SlideGraph for inter loop closure");
          found_inter_loop_closure = inter_loopCloser_.findInterLoopClosure(
            reference_map, query_map, tfFromQuery2Ref);
        } else {
          RCLCPP_ERROR(node_->get_logger(),
                       "CRITICAL ERROR: Using SlideGraph is requested but build option USE_CLIPPER is not enabled, please enable USE_CLIPPER in CMakeLists.txt and recompile");
          found_inter_loop_closure = false;
        }
      #endif

      if (!found_inter_loop_closure) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "INTER LOOP CLOSURE NOT FOUND BETWEEN ROBOTS: "
                               << query_robot_id << " AND "
                               << dbManager.getHostRobotID());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "INTER LOOP CLOSURE FOUND BETWEEN ROBOTS: "
                               << query_robot_id << " AND "
                               << dbManager.getHostRobotID());
      }

      if (found_inter_loop_closure) {
        num_successful_inter_loop_closure++;
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "INTER LOOP CLOSURE FOUND BETWEEN ROBOTS: "
                               << query_robot_id << " AND "
                               << dbManager.getHostRobotID());
        rclcpp::Time inter_loop_closure_end = node_->now();
        inter_loop_closure_time.push_back(
            (inter_loop_closure_end - inter_loop_closure_start).seconds());
        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "Inter Loop Closure took "
                << (inter_loop_closure_end - inter_loop_closure_start).seconds()
                << " seconds"
                << "to match two maps with size " << reference_map.size()
                << " and " << query_map.size());
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "relatively transformation tfFromQuery2Ref is: "
                               << tfFromQuery2Ref);
        RCLCPP_WARN_STREAM(node_->get_logger(), "Saving inter robot TF results");
        if (save_inter_robot_closure_results_){
          std::ofstream myfile;
          std::string fname =
              save_results_dir_ + "/inter-robot-tf-" +
              std::to_string(query_robot_id) + "-" +
              std::to_string(dbManager.getHostRobotID()) + "-" +
              std::to_string(node_->now().seconds()) + ".txt";
          myfile.open(fname);
          for (int temp_i = 0; temp_i < 4; temp_i++) {
            for (int temp_j = 0; temp_j < 4; temp_j++) {
              myfile << tfFromQuery2Ref(temp_i, temp_j) << " ";
            }
            myfile << "\n";
          }
        }

        SE3 tfFromQuery2RefSE3;
        tfFromQuery2RefSE3 = SE3(tfFromQuery2Ref);
        dbMutex.lock();
        dbManager.loopClosureTf[query_robot_id] = tfFromQuery2RefSE3;
        dbMutex.unlock();
      }
    }
    rate.sleep();
  }
}

bool SLOAMNode::runSLOAMNode(const SE3 &relativeRawOdomMotion,
                             const SE3 &prevKeyPose,
                             const std::vector<Cylinder> &cylindersBodyIn,
                             const std::vector<Cube> &cubesBodyIn,
                             const std::vector<Ellipsoid> &ellipsoidBodyIn,
                             rclcpp::Time stamp, SE3 &outPose,
                             const int &robotID) {
  semanticMapMtx_.lock();
  cubeSemanticMapMtx_.lock();
  ellipsoidSemanticMapMtx_.lock();
  dbMutex.lock();
  SE3 poseEstimate = prevKeyPose * relativeRawOdomMotion;
  // compute translation and add to the trajectory length
  double translation = relativeRawOdomMotion.translation().norm();
  trajectory_length += translation;
  struct PoseMstPair pmp;
  pmp.keyPose = poseEstimate;
  pmp.cylinderMsts = cylindersBodyIn;
  pmp.relativeRawOdomMotion = relativeRawOdomMotion;
  pmp.cubeMsts = cubesBodyIn;
  pmp.ellipsoidMsts = ellipsoidBodyIn;
  dbManager.getHostRobotData().poseMstPacket.push_back(pmp);
  std::vector<Cylinder> cylindersBody;
  std::vector<Cube> cubesBody;
  std::vector<Ellipsoid> ellipsoidBody;
  if (isInLoopClosureRegion_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "isInLoopClosureRegion_ is true, clearing all measurements to avoid "
        "polluting the map");
    cylindersBody = std::vector<Cylinder>();
    cubesBody = std::vector<Cube>();
    ellipsoidBody = std::vector<Ellipsoid>();
  } else {
    cylindersBody = cylindersBodyIn;
    cubesBody = cubesBodyIn;
    ellipsoidBody = ellipsoidBodyIn;
  }

  SloamInput sloamIn = SloamInput();
  sloamIn.poseEstimate = poseEstimate;
  sloamIn.distance = relativeRawOdomMotion.translation().norm();
  sloamIn.relativeRawOdomMotion = relativeRawOdomMotion;
  sloamIn.scanCubesBody = cubesBody;
  sloamIn.scanCylindersBody = cylindersBody;
  sloamIn.scanEllipsoidsBody = ellipsoidBody;

  // get Submap at the current Pose
  semanticMap_.getSubmap(poseEstimate, submap_cylinders_);
  cube_semantic_map_.getSubmap(poseEstimate, submap_cubes_);
  ellipsoid_semantic_map_.getSubmap(poseEstimate, submap_ellipsoids_);
  sloamIn.submapCylinders = submap_cylinders_;
  sloamIn.submapCubes = submap_cubes_;
  sloamIn.submapEllipsoids = submap_ellipsoids_;

  SloamOutput sloamOut = SloamOutput();

  double da_start = node_->now().seconds();
  bool success = RunSloam(sloamIn, sloamOut);
  double da_end = node_->now().seconds();
  double da_time = da_end - da_start;
  data_association_time.push_back(da_time);

  if (!success) {
    semanticMapMtx_.unlock();
    cubeSemanticMapMtx_.unlock();
    ellipsoidSemanticMapMtx_.unlock();
    dbMutex.unlock();
    return false;
  }

  // Only update map if RunSloam is successful
  semanticMap_.updateMap(sloamOut.T_Map_Curr, sloamOut.scanCylindersWorld,
                         sloamOut.cylinderMatches, robotID);
  cube_semantic_map_.updateMap(sloamOut.T_Map_Curr, sloamOut.scanCubesWorld,
                               sloamOut.cubeMatches, robotID);
  ellipsoid_semantic_map_.updateMap(sloamOut.T_Map_Curr,
                                    sloamOut.scanEllipsoidsWorld,
                                    sloamOut.ellipsoidMatches, robotID);

  // sanity check
  if (sloamOut.cubeMatches.size() != sloamOut.scanCubesWorld.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "# cube matches does not matches # detected cubes!!");
  }
  if (sloamOut.cylinderMatches.size() != sloamOut.scanCylindersWorld.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "# cylinder matches does not matches # detected "
                        "cylinders!!");
  }
  if (sloamOut.ellipsoidMatches.size() != sloamOut.scanEllipsoidsWorld.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "# ellipsoid matches does not matches # detected "
                        "ellipsoids!!");
  }

  // This step will add odometry, cylinder and cuboid factors into factor
  // graph, centroid object perform its own DA here
  double fg_optimization_start = node_->now().seconds();
  bool optimized = factorGraph_.addSLOAMObservation(
      semanticMap_, cube_semantic_map_, ellipsoid_semantic_map_,
      sloamOut.cylinderMatches, sloamOut.scanCylindersWorld,
      sloamOut.cubeMatches, sloamOut.scanCubesWorld, sloamOut.ellipsoidMatches,
      sloamOut.scanEllipsoidsWorld, relativeRawOdomMotion, sloamOut.T_Map_Curr,
      robotID);
  double fg_optimization_end = node_->now().seconds();
  double time_diff = fg_optimization_end - fg_optimization_start;
  fg_optimization_time.push_back(time_diff);
  // sanity check
  KeyPoseTimeStamps.push_back(stamp);
  if (KeyPoseTimeStamps.size() != factorGraph_.getPoseCounterById(robotID)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
        "KeyPoseTimeStamps.size() is: " << KeyPoseTimeStamps.size());
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "factorGraph_.getPoseCounterById(robotID) is: "
                            << factorGraph_.getPoseCounterById(robotID));
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "KeyPoseTimeStamps.size() is not equal to "
                        "factorGraph_.getPoseCounterById(robotID)");
  }

  // PERFORM INTER-ROBOT LOOP CLOSURE, UPDATE MAP, ADD OBSERVATION, AND
  // PERFORM OPTIMIZATION
  for (auto iter = dbManager.getRobotDataDict().begin();
       iter != dbManager.getRobotDataDict().end(); iter++) {
    int curRobotID = iter->first;
    size_t curBmFG = iter->second.bookmarkFG;
    size_t curBmLC = iter->second.bookmarkLC;
    (void)curBmLC;
    size_t curSize = iter->second.poseMstPacket.size();

    // MULTI-ROBOT STUFF
    if (dbManager.loopClosureTf.find(curRobotID) !=
            dbManager.loopClosureTf.end() &&
        curRobotID != dbManager.getHostRobotID()) {
      for (size_t i = curBmFG; i < curSize; i++) {
        SE3 poseEstimateInRefFrame = dbManager.loopClosureTf[curRobotID] *
                                     iter->second.poseMstPacket[i].keyPose;
        SE3 relativeRawOdomMotionInRefFrame =
            iter->second.poseMstPacket[i].relativeRawOdomMotion;
        std::vector<Cylinder> CylinderMeasurement =
            iter->second.poseMstPacket[i].cylinderMsts;
        std::vector<Cube> CubeMeasurement =
            iter->second.poseMstPacket[i].cubeMsts;
        std::vector<Ellipsoid> EllipsoidMeasurement =
            iter->second.poseMstPacket[i].ellipsoidMsts;

        // project measurements from the other robot's body frame to current
        // robot's map frame
        projectModels(poseEstimateInRefFrame, CylinderMeasurement,
                      CubeMeasurement, EllipsoidMeasurement);
        std::vector<Cylinder> cylinderInRefFrame = CylinderMeasurement;
        std::vector<Cube> cubeInRefFrame = CubeMeasurement;
        std::vector<Ellipsoid> ellipsoidInRefFrame = EllipsoidMeasurement;

        // Update Map to get updated semanticMap
        std::vector<int> cubeMatchIndices(cubeInRefFrame.size(), -1);
        std::vector<int> cylinderMatchIndices(cylinderInRefFrame.size(), -1);
        std::vector<int> ellipsoidMatchIndices(ellipsoidInRefFrame.size(), -1);
        std::vector<Cylinder> cylinderSubMap;
        std::vector<Cube> cubeSubMap;
        std::vector<Ellipsoid> ellipsoidSubMap;
        semanticMap_.getSubmap(poseEstimateInRefFrame, cylinderSubMap);
        cube_semantic_map_.getSubmap(poseEstimateInRefFrame, cubeSubMap);
        ellipsoid_semantic_map_.getSubmap(poseEstimateInRefFrame,
                                          ellipsoidSubMap);

        matchModels(cylinderInRefFrame, cylinderSubMap, cylinderMatchIndices);
        matchCubeModels(cubeInRefFrame, cubeSubMap, cubeMatchIndices);
        matchEllipsoidModels(ellipsoidInRefFrame, ellipsoidSubMap,
                             ellipsoidMatchIndices);
        semanticMap_.updateMap(poseEstimateInRefFrame, cylinderInRefFrame,
                               cylinderMatchIndices, curRobotID);
        cube_semantic_map_.updateMap(poseEstimateInRefFrame, cubeInRefFrame,
                                     cubeMatchIndices, curRobotID);
        ellipsoid_semantic_map_.updateMap(poseEstimateInRefFrame,
                                          ellipsoidInRefFrame,
                                          ellipsoidMatchIndices, curRobotID);
        factorGraph_.addSLOAMObservation(
            semanticMap_, cube_semantic_map_, ellipsoid_semantic_map_,
            cylinderMatchIndices, cylinderInRefFrame, cubeMatchIndices,
            cubeInRefFrame, ellipsoidMatchIndices, ellipsoidInRefFrame,
            relativeRawOdomMotionInRefFrame, poseEstimateInRefFrame, curRobotID,
            false);
      }
      factorGraph_.solve();
      dbManager.updateFGBookmark(curSize, curRobotID);
    }
  }

  if (optimized) {
    factorGraph_.updateFactorGraphMap(semanticMap_, cube_semantic_map_,
                                      ellipsoid_semantic_map_);
    factorGraph_.getCurrPose(sloamOut.T_Map_Curr, robotID);
  }

  // update the map in databaseManager
  dbManager.updateRobotMap(semanticMap_.getFinalMap(),
                           cube_semantic_map_.getFinalMap(),
                           ellipsoid_semantic_map_.getFinalMap(), robotID);
  total_number_of_landmarks = semanticMap_.getFinalMap().size() +
                              cube_semantic_map_.getFinalMap().size() +
                              ellipsoid_semantic_map_.getFinalMap().size();

  outPose = sloamOut.T_Map_Curr;
  publishResults_(sloamIn, sloamOut, stamp, robotID);
  semanticMapMtx_.unlock();
  cubeSemanticMapMtx_.unlock();
  ellipsoidSemanticMapMtx_.unlock();
  dbMutex.unlock();

  return success;
}

} // namespace sloam
