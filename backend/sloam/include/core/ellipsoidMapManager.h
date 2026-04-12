/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <definitions.h>
#include <ellipsoid.h>
#include <rclcpp/rclcpp.hpp>

#include <set>
class EllipsoidMapManager {
 public:
  explicit EllipsoidMapManager();
  std::vector<Ellipsoid> getFinalMap(const int &num_min_observations = 3);
  const std::vector<Ellipsoid> &getConstMap() const;
  std::vector<Ellipsoid> &getRawMap();
  std::map<int, int> getMatchesMap() const;
  void getSubmap(const SE3 &pose, std::vector<Ellipsoid> &submap);
  void updateMap(const SE3 &pose, std::vector<Ellipsoid> &obs_tms,
                 const std::vector<int> &matches, const int &robotID);
  void getkeyPoseSubmap(const SE3 &pose, std::vector<Ellipsoid> &submap,
                        double submap_radius, const int robotID);

 private:
  // landmarks_ variable only records the 3D positions of semantic landmarks for
  // purposes such as get sub map
  CloudT::Ptr ellipsoid_landmarks_;
  std::vector<size_t> ellipsoid_hits_;
  std::vector<Ellipsoid> ellipsoid_models_;
  std::map<int, int> ellipsoidMatchesMap_;
};