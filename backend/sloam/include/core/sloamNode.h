/**
 * This file is part of SlideSLAM
 *
 * Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu,
 * Yuezhan Tao
 *
 * TODO: License information
 *
 */

#pragma once

// ROS
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// pcl
#include <cube.h>
#include <cubeMapManager.h>
#include <cylinderMapManager.h>
#include <databaseManager.h>
#include <definitions.h>
#include <ellipsoid.h>
#include <ellipsoidMapManager.h>
#include <graphWrapper.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <place_recognition.h>
#include <sloam.h>
#include <sloam_msgs/msg/ros_observation.hpp>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <utils.h>
#include <vizTools.h>

#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <thread>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

namespace sloam {
class SLOAMNode : public sloam {
 public:
  explicit SLOAMNode(rclcpp::Node *node);
  ~SLOAMNode();
  void destaggerCloud(const Cloud::Ptr cloud, Cloud::Ptr &outCloud);
  SLOAMNode(const SLOAMNode &) = delete;
  SLOAMNode operator=(const SLOAMNode &) = delete;
  using Ptr = std::shared_ptr<SLOAMNode>;
  using ConstPtr = std::shared_ptr<const SLOAMNode>;

  // timestamp is used for visualization
  bool runSLOAMNode(const SE3 &relativeRawOdomMotion, const SE3 &prevKeyPose,
                    const std::vector<Cylinder> &cylindersBody,
                    const std::vector<Cube> &cubesBody,
                    const std::vector<Ellipsoid> &ellipsoidBody,
                    rclcpp::Time stamp, SE3 &outPose, const int &robotID);
  bool isInLoopClosureRegion_ = false;
  SemanticFactorGraphWrapper factorGraph_;
  databaseManager dbManager;
  std::mutex dbMutex;
  int hostRobotID;
  CylinderMapManager semanticMap_;
  // results analysis related variables
  std::vector<double> fg_optimization_time;
  int total_number_of_landmarks = 0;
  double trajectory_length = 0;
  std::vector<double> data_association_time;
  std::vector<double> intra_loop_closure_time;
  int num_attempts_intra_loop_closure = 0;
  int num_successful_intra_loop_closure = 0;
  std::vector<double> inter_loop_closure_time;
  int num_attempts_inter_loop_closure = 0;
  int num_successful_inter_loop_closure = 0;
  bool save_runtime_analysis = false;
  std::string runtime_analysis_file;

 private:
  // TODO(xu): load the following four params from rosparam
  bool save_inter_robot_closure_results_ = true;
  std::string save_results_dir_ = "/home/sam";
  bool save_robot_trajectory_as_csv_ = false;
  std::string save_runtime_analysis_dir_ = "/home/sam";


  double inter_robot_place_recognition_frequency_;
  double intra_robot_place_recognition_frequency_;

  std::vector<rclcpp::Time> KeyPoseTimeStamps;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundPub_;

  void initParams_();
  Cloud::Ptr trellisCloud(
      const std::vector<std::vector<TreeVertex>> &landmarks);
  void publishMap_(const rclcpp::Time stamp);
  void publishCubeMaps_(const rclcpp::Time stamp);

  bool prepareInputs_(const SE3 relativeMotion, const SE3 prevKeyPose,
                      CloudT::Ptr tree_cloud, CloudT::Ptr ground_cloud,
                      SloamInput &sloamIn);
  void publishResults_(const SloamInput &sloamIn, const SloamOutput &sloamOut,
                       rclcpp::Time stamp, const int &robotID);
  void intraLoopClosureThread_();
  void interLoopClosureThread_();

  std::vector<Eigen::Vector3d> extractPosition(
      const std::vector<Cylinder> &candidateCylinderObs,
      const std::vector<Cube> &candidateCubeObs,
      const std::vector<SE3> &candidateCentroidObs);

  std::vector<Eigen::Vector3d> extractPosition(
      const std::vector<gtsam_cylinder::CylinderMeasurement>
          &candidateCylinderObs,
      const std::vector<gtsam_cube::CubeMeasurement> &candidateCubeObs,
      const std::vector<gtsam::Point3> &candidateCentroidObs);

  std::vector<Eigen::Vector7d> prepareLCInput(
      const std::vector<Cylinder> &candidateCylinderObs,
      const std::vector<Cube> &candidateCubeObs,
      const std::vector<Ellipsoid> &candidateCentroidObs);

  rclcpp::Node *node_;

  bool use_slidematch_; // whether to use the slidematch algorithm, if false, use
                        // the slidegraph place recognition algorithm

  std::unique_ptr<tf2_ros::TransformBroadcaster> worldTfBr_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubMapPose_;
  rclcpp::Publisher<sloam_msgs::msg::ROSObservation>::SharedPtr pubObs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubAllPointLandmarks_;

  std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pubRobotTrajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMapGroundFeatures_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubObsTreeFeatures_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubObsGroundFeatures_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMapTreeModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSubmapTreeModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubObsTreeModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMapGroundModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubObsGroundModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMapCubeModel_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSubmapCubeModel_;

  // Transform
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_id_;

  // Instance graphDetector_;
  std::vector<Cylinder> submap_cylinders_;
  std::vector<int> cylinder_matches_;
  FeatureModelParams fmParams_;

  // loop closure related
  PlaceRecognition intra_loopCloser_;
  rclcpp::Time last_intra_loop_closure_stamp_;
  rclcpp::Time last_inter_loop_closure_stamp_;
  PlaceRecognition inter_loopCloser_;
  std::thread intraLoopthread_;
  std::thread interLoopthread_;
  int lastLoopAttemptPose_;
  std::mutex semanticMapMtx_;
  std::mutex cubeSemanticMapMtx_;
  std::mutex ellipsoidSemanticMapMtx_;
  std::mutex factorGraphMtx_;

  // For cuboid semantic landmarks
  CubeMapManager cube_semantic_map_;
  std::vector<int> cube_matches_;
  std::vector<Cube> submap_cubes_;
  std::vector<Cube> scan_cubes_world_;

  // For ellipsoid semantic landmarks
  EllipsoidMapManager ellipsoid_semantic_map_;
  std::vector<int> ellipsoid_matches_;
  std::vector<Ellipsoid> submap_ellipsoids_;
  std::vector<Ellipsoid> scan_ellipsoids_world_;

  // flags
  bool cube_map_initialized_ = false;
  int counter_for_noise = 0;
  bool firstScan_;
  bool debugMode_;

  int numRobots;

  void getCentroidSubmap(const std::vector<SE3> &allCentroids,
                         std::vector<SE3> &submapCentroids, const SE3 &pose,
                         const double &radius);
};
}  // namespace sloam
