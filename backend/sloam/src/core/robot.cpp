/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <robot.h>

namespace {
template <typename ParamT>
ParamT declare_or_get(rclcpp::Node *node, const std::string &name,
                      const ParamT &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParamT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ParamT>();
}
}  // namespace

Robot::Robot(rclcpp::Node *node) : node_(node) {

  // access ros parameter
  odomFreqFilter_ = declare_or_get<int>(node_, "odom_freq_filter", 10);
  robot_frame_id_ = declare_or_get<std::string>(node_, "robot_frame_id", "robot");
  minSLOAMAltitude_ = declare_or_get<double>(node_, "min_robot_altitude", 0.0);
  maxQueueSize_ = declare_or_get<int>(node_, "max_queue_size", 100);
  robot_ns_prefix_ = declare_or_get<std::string>(node_, "robot_ns_prefix", "robot");
  odom_topic_ = declare_or_get<std::string>(node_, "odom_topic", "odom");

  // initialization
  robotId_ = declare_or_get<int>(node_, "hostRobotID", 0);
  std::string cur_robot_odom_topic;
  int robot_actual_ID;
  robot_actual_ID = robotId_;
  robotFirstOdom_ = true;
  robotOdomCounter_ = 0;

  // initialize publisher and subscriber
  std::string RobotHighFreqSLOAMPose_topic =
      robot_ns_prefix_ + std::to_string(robot_actual_ID) + "/pose_high_freq";
  pubRobotHighFreqSLOAMPose_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          RobotHighFreqSLOAMPose_topic, 10);

  std::string RobotHighFreqSLOAMOdom_topic = robot_ns_prefix_ +
                                             std::to_string(robot_actual_ID) +
                                             "/sloam_odom_high_freq";
  pubRobotHighFreqSLOAMOdom_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(
          RobotHighFreqSLOAMOdom_topic, 20);

  std::string SloamToVioOdom_topic =
      robot_ns_prefix_ + std::to_string(robot_actual_ID) + "/sloam_to_vio_odom";
  pubSloamToVioOdom_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(SloamToVioOdom_topic, 20);

  std::string RobotHighFreqSyncOdom_topic = robot_ns_prefix_ +
                                            std::to_string(robot_actual_ID) +
                                            "/sync_odom_high_freq";
  pubRobotHighFreqSyncOdom_ =
      node_->create_publisher<sloam_msgs::msg::ROSSyncOdom>(
          RobotHighFreqSyncOdom_topic, 20);

  RobotOdomSub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&Robot::RobotOdomCb, this, std::placeholders::_1));
  std::string observationSub_topic = robot_ns_prefix_ +
                                     std::to_string(robot_actual_ID) +
                                     "/semantic_meas_sync_odom";

  RobotObservationSub_ =
      node_->create_subscription<sloam_msgs::msg::SemanticMeasSyncOdom>(
          observationSub_topic, 10,
          std::bind(&Robot::RobotObservationCb, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Robot Initialized! Robot # " << robot_actual_ID);
}

void Robot::RobotOdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  if (odomFreqFilter_ > 1) {
    robotOdomCounter_++;
    if (robotOdomCounter_ % odomFreqFilter_ != 0)
      return;
    robotOdomCounter_ = 0;
  }

  auto pose = odom_msg->pose.pose;
  rclcpp::Time odomStamp(odom_msg->header.stamp);
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;


  if (robotFirstOdom_ && pose.position.z < minSLOAMAltitude_) {
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Robot is too low, will not call sloam, height threshold is "
            << minSLOAMAltitude_);
    return;
  }

  // if either robotObservationQueue_.empty() or robotOdomQueue_.empty(), we need to add odom factor
  bool is_first_run = false;
  if (robotObservationQueue_.empty() || robotOdomQueue_.empty()) {
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Either robotObservationQueue_ or robotOdomQueue_ is empty, will add odom factor");
    is_first_run = true;
  }

  robotOdomQueue_.emplace_back(odom, odomStamp);
  if (robotOdomQueue_.size() > 10 * maxQueueSize_)
    robotOdomQueue_.pop_front();

  if(is_first_run){
    robotOdomUpdated_ = true;
    return;
  } else {
    // we set robotOdomUpdated_ to true only if the latest semantic measurements can be discard (i.e. its timestamp is at least semantic_meas_delay_tolerance_ seconds earlier than the latest odometry stamp)
    auto latest_observation_stamp = robotObservationQueue_.back().stampedPose.stamp;
    if ((odomStamp - latest_observation_stamp).seconds() > semantic_meas_delay_tolerance_) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Odometry delay is enabled, and semantic observation is old enough, setting robotOdomUpdated_ to true");
      robotOdomUpdated_ = true;
    } else {
      robotOdomUpdated_ = false;
    }
  }
}

void Robot::RobotObservationCb(
    const sloam_msgs::msg::SemanticMeasSyncOdom::ConstSharedPtr observation_msg) {
  // compared the stamp of the observation with the latest odometry stamp, if too old, discard this observation msg
  if (robotOdomQueue_.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "RobotOdomQueue is empty, cannot compare stamp");
    return;
  }
  auto latest_odom_stamp = robotOdomQueue_.back().stamp;
  rclcpp::Time obs_stamp(observation_msg->header.stamp);
  if ((latest_odom_stamp - obs_stamp).seconds() > semantic_meas_delay_tolerance_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Semantic observation arrived too late, discard this observation");
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "Semantic observation arrived "
            << (latest_odom_stamp - obs_stamp).seconds()
            << " seconds late, tolerance is " << semantic_meas_delay_tolerance_);
    return;
  }
  Observation cur_observation;
  cur_observation.stampedPose.pose =
      databaseManager::toSE3Pose(observation_msg->odometry.pose.pose);
  cur_observation.stampedPose.stamp = obs_stamp;
  std::vector<Cube> scan_cubes_body;
  int total_cuboids = observation_msg->cuboid_factors.size();
  for (const sloam_msgs::msg::ROSCube &cur_cuboid_marker :
       observation_msg->cuboid_factors) {
    double x = cur_cuboid_marker.pose.position.x;
    double y = cur_cuboid_marker.pose.position.y;
    double z = cur_cuboid_marker.pose.position.z;
    gtsam::Rot3 rot(cur_cuboid_marker.pose.orientation.w,
                    cur_cuboid_marker.pose.orientation.x,
                    cur_cuboid_marker.pose.orientation.y,
                    cur_cuboid_marker.pose.orientation.z);
    gtsam::Point3 position(x, y, z);
    gtsam::Pose3 pose(rot, position);
    const gtsam::Point3 scale(cur_cuboid_marker.dim[0],
                              cur_cuboid_marker.dim[1],
                              cur_cuboid_marker.dim[2]);
    scan_cubes_body.push_back(
        Cube(pose, scale, cur_cuboid_marker.semantic_label));
  }

  // Cylinder
  cur_observation.cubes = scan_cubes_body;
  cur_observation.cylinders =
      rosCylinder2CylinderObj(observation_msg->cylinder_factors);
  cur_observation.ellipsoids =
      rosEllipsoid2EllipObj(observation_msg->ellipsoid_factors);
  robotObservationQueue_.push(cur_observation);
  robotObservationUpdated_ = true;
}

std::vector<Cylinder> Robot::rosCylinder2CylinderObj(
    const std::vector<sloam_msgs::msg::ROSCylinder> &rosCylinders) {

  std::vector<Cylinder> cylinders;
  for (const auto &obj : rosCylinders) {
    double radius = obj.radius;
    gtsam::Point3 rayPoint3(obj.ray[0], obj.ray[1], obj.ray[2]);
    gtsam::Point3 rootPoint3(obj.root[0], obj.root[1], obj.root[2]);
    cylinders.emplace_back(rootPoint3, rayPoint3, radius, obj.semantic_label);
  }
  return cylinders;
}

std::vector<Ellipsoid> Robot::rosEllipsoid2EllipObj(
    const std::vector<sloam_msgs::msg::ROSEllipsoid> &msgs) {
  std::vector<Ellipsoid> ellipsoids;

  for (const auto m : msgs) {
    double x = m.pose.position.x;
    double y = m.pose.position.y;
    double z = m.pose.position.z;
    gtsam::Rot3 rot(m.pose.orientation.w, m.pose.orientation.x,
                    m.pose.orientation.y, m.pose.orientation.z);
    gtsam::Point3 position(x, y, z);
    gtsam::Pose3 pose(rot, position);
    int label = m.semantic_label;
    // scale of the cuboid
    const gtsam::Point3 scale(m.scale[0], m.scale[1], m.scale[2]);
    // assemble a pose from the
    ellipsoids.emplace_back(pose, scale, label);
  }

  return ellipsoids;
}
