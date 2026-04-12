/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <ellipsoidMapManager.h>

EllipsoidMapManager::EllipsoidMapManager() {
  ellipsoid_landmarks_.reset(new CloudT);
}

std::vector<Ellipsoid> EllipsoidMapManager::getFinalMap(
    const int &num_min_observations) {
  std::vector<Ellipsoid> map;
  for (auto i = 0; i < ellipsoid_models_.size(); ++i) {
    // at least observed for how many times for the cuboid to be visualized
    if (ellipsoid_hits_[i] >= num_min_observations) {
      map.push_back(ellipsoid_models_[i]);
    }
  }
  return map;
}

std::vector<Ellipsoid> &EllipsoidMapManager::getRawMap() {
  return ellipsoid_models_;
}

const std::vector<Ellipsoid> &EllipsoidMapManager::getConstMap() const {
  return ellipsoid_models_;
}

std::map<int, int> EllipsoidMapManager::getMatchesMap() const {
  return ellipsoidMatchesMap_;
}

void EllipsoidMapManager::getSubmap(const SE3 &pose,
                                    std::vector<Ellipsoid> &ellipsoid_submap) {
  // very important: must clear sub map before adding new ellipsoids
  ellipsoid_submap.clear();

  if (ellipsoid_landmarks_->size() == 0) {
    // std::cout
    //     << "INFO: No ellipsoid model yet, probably update map function has not been "
    //        "called "
    //     << '\n';
    return;
  }

  ellipsoidMatchesMap_.clear();
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(ellipsoid_landmarks_);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;
  PointT searchPoint;

  // Search for nearby ellipsoids
  searchPoint.x = pose.translation()[0];
  searchPoint.y = pose.translation()[1];
  searchPoint.z = pose.translation()[2];

  int ellipsoid_match_sub_map_distance_threshold = 1000;
  if (kdtree.nearestKSearch(searchPoint,
                            ellipsoid_match_sub_map_distance_threshold,
                            pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
    int idx_count = 0;
    auto map_size = ellipsoid_models_.size();
    for (auto map_idx : pointIdxKNNSearch) {
      ellipsoidMatchesMap_.insert(std::pair<int, int>(idx_count, map_idx));
      ellipsoid_submap.push_back(ellipsoid_models_[map_idx]);
      idx_count++;
    }
  } else {
    // RCLCPP_INFO(logger, "Not enough landmarks around pose: Total: %ld",
    //          pointIdxKNNSearch.size());
  }
}

void EllipsoidMapManager::getkeyPoseSubmap(const SE3 &pose,
                                           std::vector<Ellipsoid> &submap,
                                           double submap_radius,
                                           const int robotID) {
  submap.clear();
  PointT p;
  p.x = pose.translation()[0];
  p.y = pose.translation()[1];
  p.z = pose.translation()[2];
  size_t counter = 0;
  for (auto model : ellipsoid_models_) {
    if (model.distance(p) <= submap_radius) {
      double model_z = model.model.pose.translation()[2];
      double pose_z = pose.translation()[2];
      double diff = std::fabs(model_z - pose_z);
      if (diff < 1.5) {
        // print a warning that we hard code submap z height threshold
        std::cout
            << "TODO: submap landmark Z position difference (wrt current pose) "
               "threshold is hard coded to 1.5. This is to avoid including "
               "landmarks at different floors in a building in the submap. "
               "Change this to a param later"
            << '\n';
        submap.push_back(model);
      }
    }
  }
}

void EllipsoidMapManager::updateMap(const SE3 &pose,
                                    std::vector<Ellipsoid> &obs_tms,
                                    const std::vector<int> &ellipsoid_matches,
                                    const int &robotID) {
  if (obs_tms.size() == 0) {
    // std::cout << "No ellipsoids are detected!" << '\n';
    return;
  } else {
    // std::cout << "Ellipsoids are detected!" << obs_tms.size() << '\n';
  }
  size_t i = 0;
  for (auto const &ellipsoid : obs_tms) {
    PointT pt;
    pt.x = static_cast<float>(ellipsoid.model.pose.translation()[0]);
    pt.y = static_cast<float>(ellipsoid.model.pose.translation()[1]);
    pt.z = static_cast<float>(ellipsoid.model.pose.translation()[2]);
    if (ellipsoid_matches[i] == -1) {
      ellipsoid_landmarks_->push_back(pt);
      ellipsoid_models_.push_back(ellipsoid);
      ellipsoid_hits_.push_back(1);
    } else {
      // transform from observation to map index
      int matchIdx = ellipsoidMatchesMap_.at(ellipsoid_matches[i]);
      ellipsoid_hits_[matchIdx] += 1;
      // update the ellipsoid model
      // update the dimensions of the ellipsoid, making it a moving average
      // moving average factor, how much to trust the new measurement
      double alpha = 0.2;
      ellipsoid_models_[matchIdx].model.scale =
          (1. - alpha) * ellipsoid_models_[matchIdx].model.scale +
          alpha * ellipsoid.model.scale;
    }
    i++;
  }
}
