/**
 * This file is part of SlideSLAM
 *
 * Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu
 *
 * TODO: License information
 *
 */

#pragma once
#include <definitions.h>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/bind/bind.hpp>
#include <cmath>
#include <unsupported/Eigen/NonLinearOptimization>

#if USE_CLIPPER
#include "semantic_clipper.h"
#endif

namespace Eigen {
typedef Matrix<double, 7, 1> Vector7d;
}

class PlaceRecognition {
 public:
  // flag to visualize the matching results
  bool visualize_matching_results;

  // min overlap percentage to determine if a loop closure is valid
  double min_loop_closure_overlap_percentage_;

  // flag to use least square optimization
  bool use_lsq;

  // specify if this is inter-loop closure or intra-loop closure
  bool inter_loop_closure = true;

  // constructor takes in rclcpp Node pointer (used for parameters and pub/sub)
  PlaceRecognition(rclcpp::Node *node);

  /**
   * @brief MatchMaps, match two maps (set of objects) and output the
   * transformation
   * @param reference_objects: a vector of Eigen::Vector7d, where each element
   * is a 7D vector, where the first element is the label, the next three
   * elements are the XYZ coordinates, and the last three elements are the
   * dimensions
   * @param query_objects: a vector of Eigen::Vector7d, where each element is a
   * 7D vector, where the first element is the label, the next three elements
   * are the XYZ coordinates, and the last three elements are the dimensions
   * @param R_t_out: a 3x3 matrix, where the upper left 2x2 is the rotation
   * matrix, and the last column is the translation vector
   * @param best_num_inliers_out: the best number of inliers
   * @param map_objects_matched_out: a vector of Eigen::Vector4d, where each
   * element is a 4D vector, where the first element is the label, and the next
   * three elements are the XYZ coordinates
   * @param detection_objects_matched_out: a vector of Eigen::Vector4d, where
   * each element is a 4D vector, where the first element is the label, and the
   * next three elements are the XYZ coordinates
   * @return void
   *
   */
  void MatchMaps(const std::vector<Eigen::Vector7d> &reference_objects,
                 const std::vector<Eigen::Vector7d> &query_objects,
                 Eigen::Matrix3d &R_t_out, int &best_num_inliers_out,
                 std::vector<Eigen::Vector4d> &map_objects_matched_out,
                 std::vector<Eigen::Vector4d> &detection_objects_matched_out);

  /**
   * @brief findIntraLoopClosure, wrapper function for finding intra robot
   * closure
   *
   * @param reference_objects
   * @param query_objects
   * @param query_pose
   * @param reference_pose
   * @param tfFromQueryToCandidate
   * @return true
   * @return false
   */
  bool findIntraLoopClosure(
      const std::vector<Eigen::Vector7d> &reference_objects,
      const std::vector<Eigen::Vector7d> &query_objects, const SE3 &query_pose,
      const SE3 &reference_pose, Eigen::Matrix4d &tfFromQueryToCandidate);

  /**
   * @brief findInterLoopClosure, wrapper function for finding inter robot
   * closure
   *
   * @param reference_objects
   * @param query_objects
   * @param tfFromQueryToCandidate
   * @return true
   * @return false
   */
  bool findInterLoopClosure(
      const std::vector<Eigen::Vector7d> &reference_objects,
      const std::vector<Eigen::Vector7d> &query_objects,
      Eigen::Matrix4d &tfFromQueryToCandidate);

#if USE_CLIPPER
  bool findInterLoopClosureWithClipper(
      const std::vector<Eigen::Vector7d> &reference_objects,
      const std::vector<Eigen::Vector7d> &query_objects,
      Eigen::Matrix4d &tfFromQueryToRef);

#endif

  /**
   * @brief solveLSQ, solve min ||RA+t-B||, the rotation from A to B is R, the
   * translation is t
   *
   * @param map_objects_matched_out, a vector of Eigen::Vector3d, each element
   * is [x, y, z], z is set to be zero here
   * @param detection_objects_matched_out a vector of Eigen::Vector3d, each
   * element is [x, y, z]
   * @param xyzyaw_out
   * @param transform_out
   */
  void solveLSQ(
      const std::vector<Eigen::Vector3d> &map_objects_matched_out,
      const std::vector<Eigen::Vector3d> &detection_objects_matched_out,
      std::vector<double> &xyzyaw_out, Eigen::Matrix4d &transform_out);

  /**
   * @brief getxyzYawfromTF, extract x y z yaw from the transformation matrix
   *
   * @param tf
   * @param xyzYaw
   */
  void getxyzYawfromTF(const Eigen::Matrix4d &tf, std::vector<double> &xyzYaw);

  /**
   * @brief findTransformation, wrapper function for finding transformation
   *
   * @param reference_objects
   * @param query_objects
   * @param xyzYaw
   * @param transform_out
   * @return true
   * @return false
   */
  bool findTransformation(const std::vector<Eigen::Vector7d> &reference_objects,
                          const std::vector<Eigen::Vector7d> &query_objects,
                          std::vector<double> &xyzYaw,
                          Eigen::Matrix4d &transform_out);

  void printParams();

 private:
  void ParamInit();
  rclcpp::Node *node_;
  void VisualizeMatchingResults(
      const std::vector<Eigen::Vector4d> &map_objects_matched,
      const std::vector<Eigen::Vector4d> &detection_objects_matched,
      const std::vector<Eigen::Vector4d> &all_detection_objects,
      Eigen::Matrix3d &R_t);

  Eigen::Vector2d getCentroid(const std::vector<Eigen::Vector7d> &objects);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  std::string ns_prefix_;

  // parameters
  
  // min number of objects to consider a match
  int min_num_inliers_;

  int slidegraph_min_num_map_objects_to_start_;
  int slidematch_min_num_map_objects_to_start_;
  int slidegraph_num_inliners_;
  double slidegraph_matching_threshold_;
  double slidegraph_sigma_;
  double slidegraph_epsilon_;
  bool ignore_dimension_;   // whether to ignore the dimension of the objects
  double match_threshold_;  // threshold for valid object matching in position
  double match_threshold_dimension_;  // threshold for valid object matching in
                                      // dimension
  double match_x_half_range_;  // search range for x position (half range, total
                               // range is 2 * half range)
  double match_y_half_range_;  // search range for y position (half range, total
                               // range is 2 * half range)
  double match_x_half_range_intra_;  // only for intra-robot loop closure:
                                     // search range for x position (half range,
                                     // total range is 2 * half range)
  double match_y_half_range_intra_;  // only for intra-robot loop closure:
                                     // search range for y position (half range,
                                     // total range is 2 * half range)
  double match_yaw_half_range_intra_;  // only for intra-robot loop closure:
                                       // search range for yaw angle (half
                                       // range, total range is 2 * half range)
  double dilation_factor_;  // place recognition search region along XY will be
                            // dilation_factor_ * max(map_1, map_2)
  double
      match_xy_step_size_;  // data association search step size for XY position
  double match_yaw_half_range_;       // search range for yaw angle (half range,
                                      // total range is 2 * half range)
  bool disable_yaw_search_;           // disable yaw search or not
  double match_yaw_angle_step_size_;  // data association search step size for
                                      // yaw angle
  double compute_budget_sec_;         // compute budget in seconds for place
                               // recognition algorithm, it will return the best
                               // estimate within this time

  std::string vis_ref_frame_ =
      "quadrotor/map";  // reference frame for visualization

  /**
   * @brief revertCentroidShift, this function convert the transformation matrix
   * from centroid-shifted coordinates back to the original coordinates
   *
   * @param tf
   * @param centroid_reference
   * @param centroid_query
   * @return Eigen::Matrix4d
   */
  Eigen::Matrix4d revertCentroidShift(const Eigen::Matrix4d &tf,
                                      const Eigen::Vector2d &centroid_reference,
                                      const Eigen::Vector2d &centroid_query);

  /**
   * @brief
   * Take in a vector of objects and return the max absolute x and y coordinates
   * of the objects
   * @param objects
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d getMapBoundaries(const std::vector<Eigen::Vector7d> &objects);

};  // end of the class GraphMatchNode