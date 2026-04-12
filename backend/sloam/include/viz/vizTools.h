/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/


#pragma once

// ROS
#include <cube.h>
#include <cv_bridge/cv_bridge.hpp>
#include <cylinder.h>
#include <definitions.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <plane.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sloam {

// functions adapted from ros tf2/tf2_eigen since
// the original implementation needs double...
geometry_msgs::msg::Quaternion toMsg_(const Quaternionf &in);
geometry_msgs::msg::Point toMsg_(const Vector3 &in);
geometry_msgs::msg::Quaternion toRosQuat_(const Sophus::SO3d &R);
geometry_msgs::msg::Pose toRosPose_(const SE3 &T);
nav_msgs::msg::Odometry toRosOdom_(const SE3 &pose, const std::string slam_ref_frame,
                              const rclcpp::Time stamp);
geometry_msgs::msg::PoseStamped makeROSPose(const SE3 &tf, std::string frame_id,
                                       const rclcpp::Time stamp);
SE3 toSE3(geometry_msgs::msg::PoseStamped pose);
visualization_msgs::msg::MarkerArray vizTrajectory(const std::vector<SE3> &poses, const std::string &frame_id, const int &robot_id);
visualization_msgs::msg::MarkerArray vizTrajectoryAndPoseInds(
    const std::vector<SE3> &poses, const std::vector<size_t> &pose_inds, const std::string &frame_id);
visualization_msgs::msg::MarkerArray vizAllCentroidLandmarks(const std::vector<SE3> &allLandmarks, const std::string &frame_id, const std::vector<int> &allLabels);
visualization_msgs::msg::MarkerArray vizGroundModel(
    const std::vector<Plane> &gplanes, const std::string &frame_id, int idx);
void vizTreeModels(const std::vector<Cylinder> &scanTm,
                   visualization_msgs::msg::MarkerArray &tMarkerArray,
                   size_t &cylinderId);
void vizCubeModels(const std::vector<Cube> &cubeModels,
                   visualization_msgs::msg::MarkerArray &tMarkerArray,
                   size_t &cubeId, const bool &is_global_map);
visualization_msgs::msg::Marker vizGroundModel(const Plane &gplane,
                                          const std::string &frame_id,
                                          const int idx);
void landmarksToCloud(const std::vector<std::vector<TreeVertex>> &landmarks,
                      CloudT::Ptr &cloud);
cv::Mat DecodeImage(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
}  // namespace sloam
