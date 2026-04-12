/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Jiuzhou Lei, Xu Liu, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <databaseManager.h>

namespace {
template <typename ParamT>
ParamT declare_or_get(rclcpp::Node *node, const std::string &name,
                      const ParamT &default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParamT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ParamT>();
}
}  // namespace

databaseManager::databaseManager(rclcpp::Node *node) {
  node_ = node;
  startTime_ = rclcpp::Clock(RCL_ROS_TIME).now();
  std::chrono::milliseconds period{200};
  timer_ = node_->create_wall_timer(
      period, std::bind(&databaseManager::runCommunication_, this));
  hostRobotID_ = declare_or_get<int>(node_, "hostRobotID", 0);
  priorTFKnown_ = declare_or_get<bool>(node_, "priorTFKnown", false);

  if (priorTFKnown_) {
    double priorTF_x = declare_or_get<double>(node_, "priorTF_x", 0.0);
    double priorTF_y = declare_or_get<double>(node_, "priorTF_y", 0.0);
    double priorTF_z = declare_or_get<double>(node_, "priorTF_z", 0.0);

    int numRobots = declare_or_get<int>(node_, "number_of_robots", 0);

    Eigen::Vector3d priorTF_xyz(priorTF_x, priorTF_y, priorTF_z);
    priorTF2World_ = SE3(SO3(), priorTF_xyz);
    SE3 tfWorld2Robot = priorTF2World_.inverse();
    for (int i = 0; i < numRobots; i++) {
      loopClosureTf[i] = tfWorld2Robot;
    }
  }

  std::string msgName = "/robot" + std::to_string(hostRobotID_) + "/PoseMstPairFromOthers";
  std::string robot_ns_prefix_local =
      declare_or_get<std::string>(node_, "robot_ns_prefix", std::string("robot"));
  commWaitTime_ = declare_or_get<double>(node_, "communication_wait_time", 30.0);
  std::string communicationTriggerTopic =
      robot_ns_prefix_local + std::to_string(hostRobotID_) + "/pose_high_freq";

  // create different subscribers for different robots
  int numRobots = declare_or_get<int>(node_, "number_of_robots", 0);
  for (int i = 0; i < numRobots; i++) {
    poseMstVectorSub_.emplace_back(
        node_->create_subscription<sloam_msgs::msg::PoseMstBundle>(
            "/robot" + std::to_string(i) + "/PoseMstPairFromOthers", 10,
            std::bind(&databaseManager::poseMstCb_, this,
                      std::placeholders::_1)));
  }
  poseMstPub_ =
      node_->create_publisher<sloam_msgs::msg::PoseMstBundle>(msgName, 10);
  robotDataDict_.emplace(std::make_pair(hostRobotID_, robotData()));
}

void databaseManager::updateRobotMap(const std::vector<Cylinder> &cylMap,
                                     const std::vector<Cube> &cubeMap,
                                     const std::vector<Ellipsoid> &ellipMap,
                                     int robotID) {
  std::vector<Eigen::Vector7d> newMap;
  for (const auto &cyl : cylMap) {
    Eigen::Vector7d cylVec;
    // 7 dimension vector: semantic_label, root_x, root_y, root_z, radius, 0, 0
    cylVec << cyl.model.semantic_label, cyl.model.root.x(), cyl.model.root.y(),
        cyl.model.root.z(), cyl.model.radius, 0, 0;
    newMap.push_back(cylVec);
  }
  for (const auto &cube : cubeMap) {
    Eigen::Vector7d cubeVec;
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    cubeVec << cube.model.semantic_label, cube.model.pose.translation().x(),
        cube.model.pose.translation().y(), cube.model.pose.translation().z(),
        cube.model.scale.x(), cube.model.scale.y(), cube.model.scale.z();
    newMap.push_back(cubeVec);
  }
  for (const auto &ellip : ellipMap) {
    Eigen::Vector7d ellipVec;
    // 7 dimension vector: semantic_label, x, y, z, scale_x, scale_y, scale_z
    ellipVec << ellip.model.semantic_label, ellip.model.pose.translation().x(),
        ellip.model.pose.translation().y(), ellip.model.pose.translation().z(),
        ellip.model.scale.x(), ellip.model.scale.y(), ellip.model.scale.z();
    newMap.push_back(ellipVec);
  }
  robotMapDict_[robotID] = newMap;
}

void databaseManager::publishPoseMsts(int robotID) {
  if (robotDataDict_.find(robotID) == robotDataDict_.end()) {
    printf("the robot data doesn't exist in the database! No available data to "
           "publish!\n");
  } else {
    std::deque<PoseMstPair> poseMsts = robotDataDict_[robotID].poseMstPacket;
    sloam_msgs::msg::PoseMstBundle bundleMsg;
    bundleMsg.robot_id = robotID;
    for (size_t i = 0; i < poseMsts.size(); i++) {
      sloam_msgs::msg::PoseMst singleMsg;
      geometry_msgs::msg::Pose poseMsg = ToRosPoseMsg(poseMsts[i].keyPose);
      geometry_msgs::msg::Pose relativeOdom =
          ToRosPoseMsg(poseMsts[i].relativeRawOdomMotion);
      singleMsg.pose = poseMsg;
      singleMsg.relative_raw_odom = relativeOdom;
      for (size_t mstIdx = 0; mstIdx < poseMsts[i].cubeMsts.size(); mstIdx++) {
        sloam_msgs::msg::ROSCube rosCubeMsg;
        Cube curCube = poseMsts[i].cubeMsts[mstIdx];
        for (int j = 0; j < 3; j++) {
          rosCubeMsg.dim[j] = curCube.model.scale[j];
        }
        rosCubeMsg.pose = gtsamPoseToRosPose(curCube.model.pose);
        rosCubeMsg.semantic_label = curCube.model.semantic_label;
        singleMsg.cubes.push_back(rosCubeMsg);
      }
      for (size_t mstIdx = 0; mstIdx < poseMsts[i].cylinderMsts.size(); mstIdx++) {
        sloam_msgs::msg::ROSCylinder rosCylinderMsg;
        Cylinder curCylinder = poseMsts[i].cylinderMsts[mstIdx];
        for (int j = 0; j < 3; j++) {
          rosCylinderMsg.ray[j] = curCylinder.model.ray[j];
          rosCylinderMsg.root[j] = curCylinder.model.root[j];
        }
        rosCylinderMsg.radius = curCylinder.model.radius;
        rosCylinderMsg.semantic_label = curCylinder.model.semantic_label;
        singleMsg.cylinders.push_back(rosCylinderMsg);
      }
      for (size_t mstIdx = 0; mstIdx < poseMsts[i].ellipsoidMsts.size();
           mstIdx++) {
        sloam_msgs::msg::ROSEllipsoid rosEllipsoidMsg;
        Ellipsoid curEllipsoid = poseMsts[i].ellipsoidMsts[mstIdx];
        for (int j = 0; j < 3; j++) {
          rosEllipsoidMsg.scale[j] = curEllipsoid.model.scale[j];
        }
        rosEllipsoidMsg.pose = gtsamPoseToRosPose(curEllipsoid.model.pose);
        rosEllipsoidMsg.semantic_label = curEllipsoid.model.semantic_label;
        singleMsg.ellipsoids.push_back(rosEllipsoidMsg);
      }
      bundleMsg.pose_mst_pair.push_back(singleMsg);
    }
    for (size_t i = 0; i < robotMapDict_[robotID].size(); i++) {
      Eigen::Vector7d curPoint = robotMapDict_[robotID][i];
      sloam_msgs::msg::Vector7d labelXYZ;
      labelXYZ.label_xyz[0] = curPoint[0];
      labelXYZ.label_xyz[1] = curPoint[1];
      labelXYZ.label_xyz[2] = curPoint[2];
      labelXYZ.label_xyz[3] = curPoint[3];
      labelXYZ.label_xyz[4] = curPoint[4];
      labelXYZ.label_xyz[5] = curPoint[5];
      labelXYZ.label_xyz[6] = curPoint[6];
      bundleMsg.map_of_label_xyz.push_back(labelXYZ);
    }
    poseMstPub_->publish(bundleMsg);
  }
}

void databaseManager::poseMstCb_(
    const sloam_msgs::msg::PoseMstBundle::ConstSharedPtr msgs) {
  int robotID = msgs->robot_id;
  if (robotDataDict_.find(robotID) == robotDataDict_.end()) {
    robotDataDict_.emplace(std::make_pair(robotID, robotData()));
  }
  size_t bundleSize = msgs->pose_mst_pair.size();
  size_t poolSize = robotDataDict_[robotID].poseMstPacket.size();
  if (bundleSize > poolSize && robotID != this->hostRobotID_) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "New robot data received from robot:"
                            << robotID << "by robot:" << this->hostRobotID_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "New robot data received ");
    size_t startIdx = poolSize;
    for (size_t i = startIdx; i < bundleSize; i++) {
      struct PoseMstPair poseMst;
      sloam_msgs::msg::PoseMst singleMsg = msgs->pose_mst_pair[i];
      SE3 pose = toSE3Pose(singleMsg.pose);
      SE3 relativeOdom = toSE3Pose(singleMsg.relative_raw_odom);
      // construct a PoseMstPair struct
      poseMst.keyPose = pose;
      poseMst.relativeRawOdomMotion = relativeOdom;
      // convert msg to object
      for (size_t j = 0; j < singleMsg.cylinders.size(); j++) {
        sloam_msgs::msg::ROSCylinder curObjMsg = singleMsg.cylinders[j];
        gtsam::Point3 root(curObjMsg.root[0], curObjMsg.root[1],
                           curObjMsg.root[2]);
        gtsam::Point3 ray(curObjMsg.ray[0], curObjMsg.ray[1], curObjMsg.ray[2]);
        double radius = curObjMsg.radius;
        Cylinder tempCylinder(root, ray, radius, curObjMsg.semantic_label);
        poseMst.cylinderMsts.push_back(tempCylinder);
      }
      for (size_t j = 0; j < singleMsg.cubes.size(); j++) {
        sloam_msgs::msg::ROSCube curObjMsg = singleMsg.cubes[j];
        gtsam::Point3 scale(curObjMsg.dim[0], curObjMsg.dim[1],
                            curObjMsg.dim[2]);
        gtsam::Pose3 gtsamPose = ToGtsamPose3(curObjMsg.pose);
        Cube tempCube(gtsamPose, scale, curObjMsg.semantic_label);
        poseMst.cubeMsts.push_back(tempCube);
      }
      for (size_t j = 0; j < singleMsg.ellipsoids.size(); j++) {
        sloam_msgs::msg::ROSEllipsoid curObjMsg = singleMsg.ellipsoids[j];
        gtsam::Point3 scale(curObjMsg.scale[0], curObjMsg.scale[1],
                            curObjMsg.scale[2]);
        gtsam::Pose3 gtsamPose = ToGtsamPose3(curObjMsg.pose);
        Ellipsoid tempEp(gtsamPose, scale, curObjMsg.semantic_label);
        poseMst.ellipsoidMsts.push_back(tempEp);
      }
      robotDataDict_[robotID].poseMstPacket.push_back(poseMst);
    }
    std::vector<Eigen::Vector7d> newRobotMap;
    for (size_t i = 0; i < msgs->map_of_label_xyz.size(); i++) {
      // construct vector7d first, store it in temp_vector
      Eigen::Vector7d temp_vector;
      temp_vector << msgs->map_of_label_xyz[i].label_xyz[0],
          msgs->map_of_label_xyz[i].label_xyz[1],
          msgs->map_of_label_xyz[i].label_xyz[2],
          msgs->map_of_label_xyz[i].label_xyz[3],
          msgs->map_of_label_xyz[i].label_xyz[4],
          msgs->map_of_label_xyz[i].label_xyz[5],
          msgs->map_of_label_xyz[i].label_xyz[6];
      newRobotMap.emplace_back(temp_vector);
    }
    robotMapDict_[robotID] = newRobotMap;
    // transmit the loop closure tf
    for (size_t i = 0; i < msgs->inter_robot_tfs.size(); i++) {
      // if the tf received contain the tf relevant to the host robot
      if (msgs->inter_robot_tfs[i].target_robot_id == hostRobotID_) {
        SE3 tfReceivedTarget2ReceivedHost =
            toSE3Pose(msgs->inter_robot_tfs[i].tf_from_target_to_host);
        SE3 tfReceivedHost2Host = tfReceivedTarget2ReceivedHost.inverse();
        loopClosureTf[msgs->inter_robot_tfs[i].host_robot_id] = tfReceivedHost2Host;
      }
      // if the tf received doesn't contain the tf relevant to the host robot
      else {
        // but the tf can be inferred using existing tf
        int robot_a = msgs->inter_robot_tfs[i].host_robot_id;
        int robot_b = msgs->inter_robot_tfs[i].target_robot_id;
        bool robot_a_found = loopClosureTf.find(robot_a) != loopClosureTf.end();
        bool robot_b_found = loopClosureTf.find(robot_b) != loopClosureTf.end();
        SE3 tfB2A = toSE3Pose(msgs->inter_robot_tfs[i].tf_from_target_to_host);
        SE3 tfA2B = tfB2A.inverse();
        if (!robot_a_found && robot_b_found) {
          SE3 tfB2host = loopClosureTf[robot_b];
          SE3 tfA2host = tfA2B * tfB2host;
          loopClosureTf[robot_a] = tfA2host;
        } else if (robot_a_found && !robot_b_found) {
          SE3 tfA2host = loopClosureTf[robot_a];
          SE3 tfB2host = tfB2A * tfA2host;
          loopClosureTf[robot_b] = tfB2host;
        }
      }
    }
    measureReceivedCommMsgSize(*msgs);
  }
}

void databaseManager::measureReceivedCommMsgSize(
    const sloam_msgs::msg::PoseMstBundle &msgs) {
  double cur_received_msg_size_bytes = 0;
  cur_received_msg_size_bytes += 1;
  for (size_t i = 0; i < msgs.pose_mst_pair.size(); i++) {
    cur_received_msg_size_bytes += 56;
    cur_received_msg_size_bytes += 56;
    cur_received_msg_size_bytes += 69 * msgs.pose_mst_pair[i].ellipsoids.size();
    cur_received_msg_size_bytes += 69 * msgs.pose_mst_pair[i].cubes.size();
    cur_received_msg_size_bytes += 37 * msgs.pose_mst_pair[i].cylinders.size();
    cur_received_msg_size_bytes += 58 * msgs.inter_robot_tfs.size();
  }
  cur_received_msg_size_bytes += msgs.map_of_label_xyz.size() * 32;
  receivedMsgSizeMB.push_back(cur_received_msg_size_bytes / 1000000);
}

void databaseManager::runCommunication_() {
  rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  rclcpp::Duration time_diff = now - startTime_;
  if (time_diff.seconds() > commWaitTime_) {
    startTime_ = now;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "TRIGGERING COMMUNICATION!");
    double cur_publish_msg_size_bytes = 0;
    for (auto iter = getRobotDataDict().begin();
         iter != getRobotDataDict().end(); iter++) {
      int curRobotID = iter->first;
      sloam_msgs::msg::PoseMstBundle bundleMsg;
      bundleMsg.robot_id = curRobotID;
      for (size_t i = 0; i < iter->second.poseMstPacket.size(); i++) {
        struct PoseMstPair pmp = iter->second.poseMstPacket[i];
        sloam_msgs::msg::PoseMst pmMsg;
        pmMsg.pose = ToRosPoseMsg(pmp.keyPose);
        pmMsg.relative_raw_odom = ToRosPoseMsg(pmp.relativeRawOdomMotion);
        pmMsg.cylinders = obj2RosObjMsg(pmp.cylinderMsts);
        pmMsg.cubes = obj2RosObjMsg(pmp.cubeMsts);
        pmMsg.ellipsoids = obj2RosObjMsg(pmp.ellipsoidMsts);
        bundleMsg.pose_mst_pair.push_back(pmMsg);
        cur_publish_msg_size_bytes += 69 * pmp.ellipsoidMsts.size();
        cur_publish_msg_size_bytes += 69 * pmp.cubeMsts.size();
        cur_publish_msg_size_bytes += 37 * pmp.cylinderMsts.size();
        cur_publish_msg_size_bytes += 56;
        cur_publish_msg_size_bytes += 56;
      }
      for (size_t i = 0; i < robotMapDict_[curRobotID].size(); i++) {
        Eigen::Vector7d curPoint = robotMapDict_[curRobotID][i];
        sloam_msgs::msg::Vector7d labelXYZ;
        labelXYZ.label_xyz[0] = curPoint[0];
        labelXYZ.label_xyz[1] = curPoint[1];
        labelXYZ.label_xyz[2] = curPoint[2];
        labelXYZ.label_xyz[3] = curPoint[3];
        labelXYZ.label_xyz[4] = curPoint[4];
        labelXYZ.label_xyz[5] = curPoint[5];
        labelXYZ.label_xyz[6] = curPoint[6];
        bundleMsg.map_of_label_xyz.push_back(labelXYZ);
        cur_publish_msg_size_bytes += 56;
      }
      for (auto iter = loopClosureTf.begin(); iter != loopClosureTf.end();
           iter++) {
        sloam_msgs::msg::InterRobotTf interRobotTFMsg;
        interRobotTFMsg.host_robot_id = hostRobotID_;
        interRobotTFMsg.target_robot_id = iter->first;
        interRobotTFMsg.tf_from_target_to_host = ToRosPoseMsg(iter->second);
        bundleMsg.inter_robot_tfs.push_back(interRobotTFMsg);
        cur_publish_msg_size_bytes += 58;
      }
      poseMstPub_->publish(bundleMsg);
    }
    publishMsgSizeMB.push_back(cur_publish_msg_size_bytes / 1000000);
  }
}
