/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once
#define MAX_NUM_ROBOTS 13
#include <cubeFactor.h>
#include <cylinderFactor.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <rclcpp/rclcpp.hpp>
#include <string.h>

#include <boost/array.hpp>
#include <fstream>

// using namespace std;
using namespace gtsam;
using namespace gtsam_cylinder;
using namespace gtsam_cube;

// using gtsam_cube::CubeFactor;
// using gtsam_cube::CubeMeasurement;

// define a symbol for each type of nodes in the graph
// using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::C;  // C()  Cube Landmark
using symbol_shorthand::L;  // L() Cylinder Landmark
// using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
// pose means tf_sensor_to_map, instead of tf_map_to_sensor
using symbol_shorthand::M;
using symbol_shorthand::N;
using symbol_shorthand::O;
using symbol_shorthand::P;
using symbol_shorthand::Q;
using symbol_shorthand::R;
using symbol_shorthand::S;
using symbol_shorthand::T;
using symbol_shorthand::U;  // U() Centroid (or UGV) Landmark
using symbol_shorthand::V;
using symbol_shorthand::W;
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::Y;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::Z;

// bearing factor 
typedef BearingFactor<Pose3, Point3> BearingFactor3D;
// bearing-range factor
typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;

// This class wraps the graph optimization back-end.
// To add application specific knowledge, such as "add odom measurement every x
// meters" it is recommended that you inherit this class (e.g. GSLAMNode)
class SemanticFactorGraph {
 public:
  explicit SemanticFactorGraph();

  void setPriors(const Pose3& pose_prior, const int& robotID);
  void solve();
  // factors
  void addGPSFactor(const Point3& gps_measurement);
  void addCylinderFactor(const size_t poseIdx, const size_t cylIdx,
                         const Pose3& pose, const CylinderMeasurement& cylinder,
                         bool alreadyExists, const int& robotID);
  void addRangeBearingFactor(const size_t poseIdx, const size_t ugvIdx,
                             const Point3& bearing_measurement,
                             const double& range_measurement,
                             const int& robotID = 0);
  void addCubeFactor(const size_t poseIdx, const size_t cubeIdx,
                     const Pose3& pose, const CubeMeasurement& cube_global_meas,
                     bool alreadyExists, const int& robotID);
  void addKeyPoseAndBetween(
      const size_t fromIdx, const size_t toIdx, const Pose3& relativeMotion,
      const Pose3& poseEstimate, const int& robotID,
      boost::optional<boost::array<double, 36>> cov = boost::none,
      const bool& loopClosureFound = false,
      const SE3& loop_closure_pose = SE3(),
      const size_t& closure_matched_pose_idx = 0);
  void addLoopClosureFactor(const Pose3 poseRelative, const size_t fromIdx,
                            const size_t fromRobotID, const size_t toIdx,
                            const size_t toRobotID);
  void addPointLandmarkKey(const size_t ugvIdx, const Point3& ugv_position);

  // getters and setters
  bool getPose(const size_t idx, const int& robotID, Pose3& poseOut);
  Eigen::MatrixXd getPoseCovariance(const int idx, const int& robotID);
  CylinderMeasurement getCylinder(const int idx);
  CubeMeasurement getCube(const int idx);
  Point3 getCentroidLandmark(const int idx);
  double estimateClosureInfoGain(
      const std::vector<size_t>& candidateTrajPoseIndices,
      const std::vector<double>& travel_distances);
  int addFakeClosureFactor(const size_t& currentPoseIdx,
                           const size_t& histroyPoseIdx,
                           const double& travel_distance);
  std::vector<int> point_landmark_labels_;
  void logEntropy();

  // these two noise models will be passed as parameters from graphWarpper
  boost::shared_ptr<noiseModel::Diagonal> noise_model_pose;
  Vector6 noise_model_pose_vec_per_m;
  SharedNoiseModel noise_model_bearing;
  double noise_model_pose_inflation = 0.0;
  double start_timestamp = 0.0;

  boost::shared_ptr<noiseModel::Diagonal> noise_model_closure;

  // TODO(xu:) unify the following two functions
  Symbol getSymbol(const int& robotID, const int idx);
  Symbol getSymbol(const int& robotID, const size_t idx);

 protected:
  // Noise models
  boost::shared_ptr<noiseModel::Diagonal> noise_model_prior_first_pose;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_gps;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_cylinder;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_cube;

  // Aux attributes
  Pose3 odomToGPS_;

  // Graph factors and values
  NonlinearFactorGraph fgraph;
  Values fvalues;
  Values currEstimate;
  // ISAM Parameters
  ISAM2Params isam_params;  
  ISAM2* isam;             
  size_t latest_to_idx_ = 0;
  size_t latest_landmark_counter = 0;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
};
