/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <cylinderMapManager.h>

CylinderMapManager::CylinderMapManager(int num_of_robots,
                                       const float searchRadius) {
  numRobots = num_of_robots;
  sqSearchRadius = searchRadius * searchRadius;
  landmarks_.reset(new CloudT);

  robotPoseCloud_.resize(numRobots);
  robotKeyFrames_.resize(numRobots);
  for (int i = 0; i < numRobots; i++) {
    robotPoseCloud_[i].reset(new CloudT);
  }
}

/**
 * @brief Given the current pose, currently detected objects, corrsponding
 * matches, update the cylinderMapManager class members like landmarks_,
 * treeModels, treeHits
 *
 * @param pose current Pose
 * @param obs_tms detected cube object
 * @param matches matched object indice
 * @param robotID
 */
void CylinderMapManager::updateMap(const SE3 &pose,
                                   std::vector<Cylinder> &obs_tms,
                                   const std::vector<int> &matches,
                                   const int &robotID) {
  size_t i = 0;
  std::vector<size_t> lidxs;
  for (auto const &tree : obs_tms) {
    PointT pt;
    pt.x = tree.model.root[0];
    pt.y = tree.model.root[1];
    pt.z = tree.model.root[2];
    if (matches[i] == -1) {
      // use treeModels.size() as its index since it is a new landmark
      lidxs.push_back(treeModels_.size());
      landmarks_->push_back(pt);
      treeModels_.push_back(tree);
      treeHits_.push_back(1);
    } else {
      // transform from observation to map index
      int matchIdx = matchesMap_.at(matches[i]);
      treeHits_[matchIdx] += 1;
      lidxs.push_back(matchIdx);
    }
    i++;
  }

  PointT posePt;
  posePt.x = float(pose.translation()[0]);
  posePt.y = float(pose.translation()[1]);
  posePt.z = float(pose.translation()[2]);

  robotPoseCloud_[robotID]->points.push_back(posePt);
  robotKeyFrames_[robotID].poses.push_back(pose);
}

std::vector<Cylinder> CylinderMapManager::getFinalMap(const int& num_min_observations) {
  std::vector<Cylinder> map;
  for (auto i = 0; i < treeModels_.size(); ++i) {
    if (treeHits_[i] >= num_min_observations)
      map.push_back(treeModels_[i]);
  }
  return map;
}

std::vector<Cylinder> &CylinderMapManager::getRawMap() { return treeModels_; }

const std::vector<Cylinder> &CylinderMapManager::getConstMap() const {
  return treeModels_;
}

std::map<int, int> CylinderMapManager::getMatchesMap() const {
  return matchesMap_;
}

const std::vector<SE3> &CylinderMapManager::getTrajectory(
    const int &robotID) const {
  if (robotID >= numRobots || robotID < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("CylinderMapManager"),
                        "############# Error: "
                            << robotID << " is an invalid robotID !!! #############");

    return robotKeyFrames_[0].poses;
  } else {
    return robotKeyFrames_[robotID].poses;
  }
}

int CylinderMapManager::getLatestPoseIdx(const int robotID) {
  return robotPoseCloud_[robotID]->points.size() - 1;
}

SE3 CylinderMapManager::getPose(const size_t idx, const int robotID) {
  return robotKeyFrames_[robotID].poses[idx];
}

bool CylinderMapManager::InLoopClosureRegion(
    const double &max_dist_xy, const double &max_dist_z, const SE3 &inputPose,
    const int robotID, const size_t &at_least_num_of_poses_old) {
  // RCLCPP_INFO_STREAM(logger, "Checking whether in loop closure region");
  if (robotPoseCloud_[robotID]->points.size() < at_least_num_of_poses_old) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CylinderMapManager"),
                         *rclcpp::Clock::make_shared(), 3000,
                         "Not enough poses to check loop closure");
    return false;
  }
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(robotPoseCloud_[robotID]);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;

  PointT searchPoint;
  searchPoint.x = inputPose.translation()[0];
  searchPoint.y = inputPose.translation()[1];
  searchPoint.z = inputPose.translation()[2];
  double max_dist_3d =
      std::sqrt(max_dist_xy * max_dist_xy + max_dist_z * max_dist_z);

  // Search for nearby trees
  if (kdtree.radiusSearch(searchPoint, max_dist_3d, pointIdxKNNSearch,
                          pointKNNSquaredDistance, 0) > 0) {
    for (const auto nnIdx : pointIdxKNNSearch) {
      // check if x y and z all within the threshold
      double diff_xy = std::sqrt(
          std::pow(robotPoseCloud_[robotID]->points[nnIdx].x - searchPoint.x,
                   2) +
          std::pow(robotPoseCloud_[robotID]->points[nnIdx].y - searchPoint.y,
                   2));
      double diff_z =
          std::fabs(robotPoseCloud_[robotID]->points[nnIdx].z - searchPoint.z);
      // if any of the three is not within the threshold, then continue
      if (diff_xy > max_dist_xy || diff_z > max_dist_z) {
        continue;
      } else if ((robotPoseCloud_[robotID]->points.size() - 1) - nnIdx >
                 at_least_num_of_poses_old) {
        return true;
      }
    }
  }
  // RCLCPP_INFO_STREAM(logger, "No candidate loop closure pose found");

  return false;
}

bool CylinderMapManager::getLoopCandidateIdx(
    const double max_dist, const size_t poseIdx, size_t &candidateIdx,
    const int robotID, const size_t &at_least_num_of_poses_old) {
  if (robotPoseCloud_[robotID]->points.size() < 50) return false;

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(robotPoseCloud_[robotID]);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;

  PointT searchPoint = robotPoseCloud_[robotID]->points[poseIdx];
  // Search for nearby trees
  if (kdtree.radiusSearch(searchPoint, max_dist, pointIdxKNNSearch,
                          pointKNNSquaredDistance, 0) > 0) {
    for (const auto nnIdx : pointIdxKNNSearch) {
      // RCLCPP_DEBUG(logger, "Cheking candidates around current pose");
      if (nnIdx != poseIdx && poseIdx - nnIdx > at_least_num_of_poses_old) {
        candidateIdx = nnIdx;
        return true;
      }
    }
  }

  return false;
}

void CylinderMapManager::getkeyPoseSubmap(const SE3 &pose,
                                          std::vector<Cylinder> &submap,
                                          double submap_radius,
                                          const int robotID) {
  submap.clear();
  PointT p;
  p.x = pose.translation()[0];
  p.y = pose.translation()[1];
  p.z = pose.translation()[2];
  for (auto model : treeModels_) {
    if (model.distance(p) <= submap_radius) {
      double model_z = model.model.root[2];
      double pose_z = pose.translation()[2];
      double diff = std::fabs(model_z - pose_z);
      if (diff < 1.5) {
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

void CylinderMapManager::getSubmap(const SE3 &pose,
                                   std::vector<Cylinder> &submap) {
  if (landmarks_->size() == 0) return;

  matchesMap_.clear();
  // should already be empty but just to make sure
  submap.clear();
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(landmarks_);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;
  PointT searchPoint;

  // Search for nearby trees
  searchPoint.x = pose.translation()[0];
  searchPoint.y = pose.translation()[1];
  searchPoint.z = pose.translation()[2];
  if (kdtree.nearestKSearch(searchPoint, 50, pointIdxKNNSearch,
                            pointKNNSquaredDistance) > 0) {
    int idx_count = 0;
    auto map_size = treeModels_.size();
    for (auto map_idx : pointIdxKNNSearch) {
      matchesMap_.insert(std::pair<int, int>(idx_count, map_idx));
      submap.push_back(treeModels_[map_idx]);
      idx_count++;
    }
  } else {
    // RCLCPP_INFO(logger, "Not enough landmarks around pose: Total: %ld",
    //          pointIdxKNNSearch.size());
  }
}