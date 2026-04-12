/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <cylinder.h>
#include <definitions.h>
#include <plane.h>
#include <rclcpp/rclcpp.hpp>

#include <set>

struct KeyFrames {
  std::vector<SE3> poses;
  std::vector<std::vector<size_t>> landmark_idxs;
};

class CylinderMapManager {
 public:
  explicit CylinderMapManager(int num_of_robots = 1,
                              const float searchRadius = 50);
  std::vector<Cylinder> getFinalMap(const int &num_min_observations = 3);
  const std::vector<Cylinder> &getConstMap() const;
  std::vector<Cylinder> &getRawMap();
  std::map<int, int> getMatchesMap() const;
  void getSubmap(const SE3 &pose, std::vector<Cylinder> &submap);
  const std::vector<SE3> &getTrajectory(const int &robotID) const;
  void updateMap(const SE3 &pose, std::vector<Cylinder> &obs_tms,
                 const std::vector<int> &matches, const int &robotID);
  int getLatestPoseIdx(const int robotID);
  SE3 getPose(const size_t idx, const int robotID);
  void getkeyPoseSubmap(const SE3 &pose, std::vector<Cylinder> &submap,
                        double submap_radius, const int robotID);
  // TODO: merge these two functions
  bool getLoopCandidateIdx(const double max_dist, const size_t poseIdx,
                           size_t &candidateIdx, const int robotID,
                           const size_t &at_least_num_of_poses_old = 50);
  bool InLoopClosureRegion(const double &max_dist_xy, const double &max_dist_z,
                           const SE3 &inputPose, const int robotID,
                           const size_t &at_least_num_of_poses_old);

 private:
  int numRobots;
  float sqSearchRadius;
  CloudT::Ptr landmarks_;
  std::vector<CloudT::Ptr> robotPoseCloud_;

  std::vector<Cylinder> treeModels_;
  std::map<int, int> matchesMap_;
  std::vector<size_t> treeHits_;
  std::vector<KeyFrames> robotKeyFrames_;
};