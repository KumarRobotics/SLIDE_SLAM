/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <cube.h>
#include <cylinder.h>
#include <ellipsoid.h>
#include <gtsam/geometry/Point3.h>
#include <definitions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sloam_msgs/msg/semantic_meas_sync_odom.hpp>
#include <sloam_msgs/msg/ros_cylinder.hpp>
#include <sloam_msgs/msg/ros_ellipsoid.hpp>
#include <sloam_msgs/msg/ros_sync_odom.hpp>
#include <databaseManager.h>

#include <deque>
#include <queue>
#include <vector>

struct StampedSE3 {
  StampedSE3(SE3 p, rclcpp::Time s) : pose(p), stamp(s){};
  StampedSE3() : pose(SE3()), stamp(rclcpp::Time(0, 0, RCL_ROS_TIME)){};
  SE3 pose;
  rclcpp::Time stamp;
};

struct Observation {
  StampedSE3 stampedPose;
  std::vector<Cube> cubes;
  std::vector<Cylinder> cylinders;
  std::vector<Ellipsoid> ellipsoids;
};

class Robot {
 private:
  size_t odomFreqFilter_;
  std::string robot_frame_id_;
  float minSLOAMAltitude_;
  size_t maxQueueSize_;
  rclcpp::Node *node_;

 public:
  // how many seconds max to wait for semantic measurements to arrive
  double semantic_meas_delay_tolerance_ = 3.0;
  std::deque<StampedSE3> robotOdomQueue_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> robotTreePcQueue_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> robotGroundPcQueue_;
  std::queue<std::pair<std::vector<Cube>, rclcpp::Time>> robotCubesQueue_;
  std::queue<Observation> robotObservationQueue_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubRobotHighFreqSLOAMPose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotHighFreqSLOAMOdom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubSloamToVioOdom_;
  rclcpp::Publisher<sloam_msgs::msg::ROSSyncOdom>::SharedPtr pubRobotHighFreqSyncOdom_;
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr RobotOdomSub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr RobotGroundPCSub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr RobotTreePCSub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr RobotCubeSub_;
  rclcpp::Subscription<sloam_msgs::msg::SemanticMeasSyncOdom>::SharedPtr RobotObservationSub_;

  std::string robot_odom_topic_;
  bool robotTreeCloudUpdated_ = false;
  bool robotGroundCloudUpdated_ = false;
  bool robotCubesUpdated_ = false;
  bool robotOdomReceived_ = false;
  bool robotOdomUpdated_ = false;
  bool robotObservationUpdated_ = false;
  // optimized key poses
  std::vector<SE3> robotKeyPoses_;
  StampedSE3 robotLatestOdom_;
  SE3 robotLastSLOAMKeyPose_ = SE3();
  bool robotFirstOdom_;
  size_t robotOdomCounter_;
  int robotId_;
  std::string robot_ns_prefix_;
  std::string odom_topic_;

  void RobotOdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void RobotTreePCCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg);
  void RobotGroundPCCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg);
  void RobotCubeCb(const visualization_msgs::msg::MarkerArray::ConstSharedPtr cuboid_msg);
  void RobotObservationCb(const sloam_msgs::msg::SemanticMeasSyncOdom::ConstSharedPtr observation_msg);
  std::vector<Ellipsoid> rosEllipsoid2EllipObj(const std::vector<sloam_msgs::msg::ROSEllipsoid>& msgs);
  std::vector<Cylinder> rosCylinder2CylinderObj(const std::vector<sloam_msgs::msg::ROSCylinder>& rosCylinders);
  Robot(rclcpp::Node *node);
};
