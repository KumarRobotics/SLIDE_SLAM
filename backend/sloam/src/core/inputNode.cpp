/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <cube.h>
#include <definitions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <graphWrapper.h>
#include <gtsam/geometry/Point3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot.h>
#include <sloamNode.h>
#include <sloam_msgs/srv/evaluate_loop_closure.hpp>
#include <sloam_msgs/msg/ros_range_bearing.hpp>
#include <sloam_msgs/msg/semantic_loop_closure.hpp>
#include <sloam_msgs/msg/ros_range_bearing_sync_odom.hpp>
#include <sloam_msgs/msg/ros_sync_odom.hpp>
#include <sloam_msgs/msg/sync_pc_odom.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vizTools.h>

#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace {
template <typename ParamT>
ParamT in_declare_or_get(rclcpp::Node *node, const std::string &name,
                         const ParamT &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParamT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ParamT>();
}
}  // namespace

class InputManager : public rclcpp::Node {
 public:
  explicit InputManager();
  void RunInputNode();
  void saveRuntimeCommUsage();
  std::shared_ptr<Robot> robot;

 private:
  void resetAllFlags();
  double runInputNodeRate_;
  rclcpp::TimerBase::SharedPtr timer_;

  void updateLastPose(const StampedSE3 &odom, const int &robotID);

  double max_timestamp_offset_ = 0.01;

  bool callSLOAM(SE3 relativeRawOdomMotion, rclcpp::Time stamp,
                 std::deque<StampedSE3> &odomQueue, const int &robotID);
  void PublishAccumOdom_(const SE3 &relativeRawOdomMotion);
  void Odom2SlamTf();
  void PublishOdomAsTf(const nav_msgs::msg::Odometry &odom_msg,
                       const std::string &parent_frame_id,
                       const std::string &child_frame_id);

  SE3 computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                     const SE3 &vio_odom);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // params
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_ns_prefix_;
  int number_of_robots_;
  std::string odom_topic_;
  std::string robot_frame_id_;
  double minOdomDistance_;
  double minSLOAMAltitude_;

  // vars
  std::shared_ptr<sloam::SLOAMNode> sloam_ = nullptr;

  bool publishTf_;

  // robotID
  int hostRobotID_;

  bool turn_off_intra_loop_closure_;
};

InputManager::InputManager() : rclcpp::Node("sloam") {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  robot = std::make_shared<Robot>(this);

  runInputNodeRate_ =
      in_declare_or_get<double>(this, "main_node_rate", 5.0);
  minOdomDistance_ =
      in_declare_or_get<double>(this, "min_odom_distance", 0.5);
  minSLOAMAltitude_ =
      in_declare_or_get<double>(this, "min_robot_altitude", 0.0);
  publishTf_ = in_declare_or_get<bool>(this, "publish_tf", false);
  number_of_robots_ =
      in_declare_or_get<int>(this, "number_of_robots", 1);
  robot_ns_prefix_ =
      in_declare_or_get<std::string>(this, "robot_ns_prefix", "robot");
  odom_topic_ = in_declare_or_get<std::string>(this, "odom_topic", "odom");
  robot_frame_id_ =
      in_declare_or_get<std::string>(this, "robot_frame_id", "robot");
  odom_frame_id_ =
      in_declare_or_get<std::string>(this, "odom_frame_id", "odom");
  map_frame_id_ = in_declare_or_get<std::string>(this, "map_frame_id", "map");
  std::string node_name = this->get_name();
  std::string idName = node_name + "/hostRobotID";
  hostRobotID_ = in_declare_or_get<int>(this, idName, 0);
  turn_off_intra_loop_closure_ = in_declare_or_get<bool>(
      this, node_name + "/turn_off_intra_loop_closure", false);

  sloam_ = std::make_shared<sloam::SLOAMNode>(this);

  auto period =
      std::chrono::duration<double>(1.0 / runInputNodeRate_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InputManager::RunInputNode, this));
}

void InputManager::RunInputNode() {
  if (robot->robotOdomQueue_.size() == 0) {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Odom queue is not filled yet for robot "
                                    << robot->robotId_
                                    << ", waiting for odometry...");
    return;
  }

  SE3 highFreqSLOAMPose;
  rclcpp::Time odom_stamp;

  // Publishing high freq pose, should use the latest odom pose
  auto cur_vio_odom = robot->robotOdomQueue_.back();
  if (robot->robotOdomReceived_) {
    SE3 latestRelativeMotionFactorGraph =
        robot->robotLatestOdom_.pose.inverse() * cur_vio_odom.pose;
    highFreqSLOAMPose =
        robot->robotLastSLOAMKeyPose_ * latestRelativeMotionFactorGraph;
    odom_stamp = cur_vio_odom.stamp;
  } else {
    highFreqSLOAMPose = cur_vio_odom.pose;
    odom_stamp = cur_vio_odom.stamp;
  }

  // syncOdom is used for publishing the relative transform from semantic slam
  // reference frame to odometry reference frame for drift compensation in the
  // navigation stack
  sloam_msgs::msg::ROSSyncOdom syncOdom;
  syncOdom.header.stamp = odom_stamp;
  auto odom_msg =
      sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp);
  syncOdom.vio_odom =
      sloam::toRosOdom_(cur_vio_odom.pose, map_frame_id_, odom_stamp);
  syncOdom.sloam_odom = odom_msg;
  // calculate this for drift compensation
  SE3 sloam_to_vio_tf =
      computeSloamToVioOdomTransform(highFreqSLOAMPose, cur_vio_odom.pose);
  // publish sloam_to_vio_tf as a odometry message
  auto sloam_to_vio_msg =
      sloam::toRosOdom_(sloam_to_vio_tf, map_frame_id_, odom_stamp);
  robot->pubSloamToVioOdom_->publish(sloam_to_vio_msg);

  robot->pubRobotHighFreqSLOAMPose_->publish(
      sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));

  robot->pubRobotHighFreqSLOAMOdom_->publish(odom_msg);
  robot->pubRobotHighFreqSyncOdom_->publish(syncOdom);

  // ADDING FACTORS
  bool add_odom_factor = false;
  bool valid_pose_found = false;
  StampedSE3 validStampedPose;
  if (robot->robotOdomUpdated_) {
    for (int i = robot->robotOdomQueue_.size() - 1; i >= 0; i--) {
      if ((robot->robotOdomQueue_.back().stamp - robot->robotOdomQueue_[i].stamp)
              .seconds() > robot->semantic_meas_delay_tolerance_) {
        validStampedPose = robot->robotOdomQueue_[i];
        valid_pose_found = true;
        break;
      }
    }

    if (valid_pose_found) {
      SE3 currRelativeMotion;
      currRelativeMotion =
          robot->robotLatestOdom_.pose.inverse() * validStampedPose.pose;
      double accumMovement = currRelativeMotion.translation().norm();
      bool moved_enough = accumMovement > minOdomDistance_;
      if (moved_enough) {
        add_odom_factor = true;
      }
    }
  }

  if (add_odom_factor || robot->robotObservationUpdated_) {
    // add both odometry and object factor
    Observation latestObservation;
    if (robot->robotObservationUpdated_) {
      double max_dist_xy = 10;
      double max_dist_z = 2;
      size_t at_least_num_of_poses_old = 30;
      SE3 inputPose = robot->robotObservationQueue_.back().stampedPose.pose;
      if (turn_off_intra_loop_closure_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Intra Loop closure is turned off, the default of the variable is "
            "false");
      }

      if (sloam_->semanticMap_.InLoopClosureRegion(max_dist_xy, max_dist_z,
                                                   inputPose, hostRobotID_,
                                                   at_least_num_of_poses_old) &&
          !turn_off_intra_loop_closure_) {
        latestObservation = robot->robotObservationQueue_.back();
        robot->robotObservationUpdated_ = false;
        robot->robotOdomUpdated_ = false;
        sloam_->isInLoopClosureRegion_ = true;
      } else {
        sloam_->isInLoopClosureRegion_ = false;
        latestObservation = robot->robotObservationQueue_.back();
        robot->robotObservationUpdated_ = false;
        robot->robotOdomUpdated_ = false;
      }
    } else {
      robot->robotOdomUpdated_ = false;
      latestObservation = Observation();
      latestObservation.stampedPose = validStampedPose;
    }

    SE3 keyPose;
    SE3 relativeRawOdomMotion;
    StampedSE3 raw_vio_odom_used_for_sloam = latestObservation.stampedPose;
    relativeRawOdomMotion = robot->robotLatestOdom_.pose.inverse() *
                            raw_vio_odom_used_for_sloam.pose;

    SE3 prevKeyPose;
    if (robot->robotKeyPoses_.size() > 0) {
      prevKeyPose = robot->robotKeyPoses_[robot->robotKeyPoses_.size() - 1];
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No previous key pose. Use identity as the previous key pose.");
      prevKeyPose = SE3();
    }

    bool success = sloam_->runSLOAMNode(
        relativeRawOdomMotion, prevKeyPose, latestObservation.cylinders,
        latestObservation.cubes, latestObservation.ellipsoids,
        latestObservation.stampedPose.stamp, keyPose, hostRobotID_);
    if (success) {
      robot->robotKeyPoses_.push_back(keyPose);
      updateLastPose(raw_vio_odom_used_for_sloam, hostRobotID_);
    }
  } else {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Neither the odometry nor the observation is updated for robot "
            << hostRobotID_);
  }
  if (sloam_->save_runtime_analysis) {
    saveRuntimeCommUsage();
  }
}

void InputManager::updateLastPose(const StampedSE3 &odom, const int & /*robotID*/) {
  robot->robotOdomReceived_ = true;
  robot->robotLatestOdom_.pose = odom.pose;
  robot->robotLatestOdom_.stamp = odom.stamp;
  if (robot->robotKeyPoses_.size() == 0) {
    robot->robotLastSLOAMKeyPose_ = robot->robotLatestOdom_.pose;
  } else {
    robot->robotLastSLOAMKeyPose_ = robot->robotKeyPoses_.back();
  }
}

SE3 InputManager::computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                                 const SE3 &vio_odom) {
  SE3 sloam_to_vio_transform = vio_odom * sloam_odom.inverse();
  return sloam_to_vio_transform;
}

void InputManager::PublishOdomAsTf(const nav_msgs::msg::Odometry &odom_msg,
                                   const std::string &parent_frame_id,
                                   const std::string &child_frame_id) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom_msg.header;
  tf.header.frame_id = parent_frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = odom_msg.pose.pose.position.x;
  tf.transform.translation.y = odom_msg.pose.pose.position.y;
  tf.transform.translation.z = odom_msg.pose.pose.position.z;
  tf.transform.rotation = odom_msg.pose.pose.orientation;
  broadcaster_->sendTransform(tf);
}

void InputManager::saveRuntimeCommUsage() {
  RCLCPP_DEBUG(this->get_logger(),
               "Saving runtime communication usage to a txt file 1...");
  std::ofstream file(sloam_->runtime_analysis_file,
                     std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    std::cerr << "Failed to open the file." << std::endl;
    return;
  }

  file << "Total number of landmarks: " << sloam_->total_number_of_landmarks
       << std::endl;
  file << "Trajectory length: " << sloam_->trajectory_length << std::endl;
  file << "Number of attempts for intra loop closure: "
       << sloam_->num_attempts_intra_loop_closure << std::endl;
  file << "Number of successful intra loop closure: "
       << sloam_->num_successful_intra_loop_closure << std::endl;
  file << "Number of attempts for inter loop closure: "
       << sloam_->num_attempts_inter_loop_closure << std::endl;
  file << "Number of successful inter loop closure: "
       << sloam_->num_successful_inter_loop_closure << std::endl;

  if (!sloam_->fg_optimization_time.empty()) {
    file << "Average factor adding and graph optimization time [s]: "
         << std::accumulate(sloam_->fg_optimization_time.begin(),
                            sloam_->fg_optimization_time.end(), 0.0) /
                sloam_->fg_optimization_time.size()
         << std::endl;
  }
  if (!sloam_->data_association_time.empty()) {
    file << "Average data association time [s]: "
         << std::accumulate(sloam_->data_association_time.begin(),
                            sloam_->data_association_time.end(), 0.0) /
                sloam_->data_association_time.size()
         << std::endl;
  }
  if (!sloam_->intra_loop_closure_time.empty()) {
    file << "Average intra loop closure time [s]: "
         << std::accumulate(sloam_->intra_loop_closure_time.begin(),
                            sloam_->intra_loop_closure_time.end(), 0.0) /
                sloam_->intra_loop_closure_time.size()
         << std::endl;
  }
  if (!sloam_->inter_loop_closure_time.empty()) {
    file << "Average inter loop closure time [s]: "
         << std::accumulate(sloam_->inter_loop_closure_time.begin(),
                            sloam_->inter_loop_closure_time.end(), 0.0) /
                sloam_->inter_loop_closure_time.size()
         << std::endl;
  }

  // total communication usage
  file << "Total Publication Communication Usage [MB]: "
       << std::accumulate(sloam_->dbManager.publishMsgSizeMB.begin(),
                          sloam_->dbManager.publishMsgSizeMB.end(), 0.0)
       << std::endl;
  file << "Total Subscription Communication Usage [MB]: "
       << std::accumulate(sloam_->dbManager.receivedMsgSizeMB.begin(),
                          sloam_->dbManager.receivedMsgSizeMB.end(), 0.0)
       << std::endl;

  if (!sloam_->dbManager.publishMsgSizeMB.empty()) {
    file << "Average publish msg size MB: "
         << std::accumulate(sloam_->dbManager.publishMsgSizeMB.begin(),
                            sloam_->dbManager.publishMsgSizeMB.end(), 0.0) /
                sloam_->dbManager.publishMsgSizeMB.size()
         << std::endl;
  }
  if (!sloam_->dbManager.receivedMsgSizeMB.empty()) {
    file << "Average received msg size MB: "
         << std::accumulate(sloam_->dbManager.receivedMsgSizeMB.begin(),
                            sloam_->dbManager.receivedMsgSizeMB.end(), 0.0) /
                sloam_->dbManager.receivedMsgSizeMB.size()
         << std::endl;
  }

  auto maximal_instant_publish_msg_size_MB =
      std::max_element(sloam_->dbManager.publishMsgSizeMB.begin(),
                       sloam_->dbManager.publishMsgSizeMB.end());
  if (maximal_instant_publish_msg_size_MB !=
      sloam_->dbManager.publishMsgSizeMB.end())
    file << "Maximal Instant publish msg size MB: "
         << *maximal_instant_publish_msg_size_MB << std::endl;

  auto maximal_instant_received_msg_size_MB =
      std::max_element(sloam_->dbManager.receivedMsgSizeMB.begin(),
                       sloam_->dbManager.receivedMsgSizeMB.end());
  if (maximal_instant_received_msg_size_MB !=
      sloam_->dbManager.receivedMsgSizeMB.end())
    file << "Maximal Instant received msg size MB: "
         << *maximal_instant_received_msg_size_MB << std::endl;

  for (auto iter = sloam_->dbManager.loopClosureTf.begin();
       iter != sloam_->dbManager.loopClosureTf.end(); iter++) {
    file << "Loop closure tf between " << std::to_string(hostRobotID_)
         << " and " << std::to_string(iter->first) << ": " << std::endl;
    file << iter->second.matrix() << std::endl;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InputManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
