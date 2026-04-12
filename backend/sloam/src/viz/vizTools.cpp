/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#include <gtsam/geometry/Quaternion.h>
#include <vizTools.h>
#include <yaml-cpp/yaml.h>
#include <vector>

namespace sloam {

namespace {
inline rclcpp::Logger viz_logger() {
  return rclcpp::get_logger("sloam_viz");
}
inline rclcpp::Time now_ros_time() {
  return rclcpp::Clock(RCL_ROS_TIME).now();
}
}  // namespace

// functions adapted from ros tf2/tf2_eigen
geometry_msgs::msg::Quaternion toMsg_(const Quat &in) {
  geometry_msgs::msg::Quaternion msg;
  msg.w = in.w();
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

geometry_msgs::msg::Point toMsg_(const Vector3 &in) {
  geometry_msgs::msg::Point msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

geometry_msgs::msg::Quaternion toRosQuat_(const Sophus::SO3d &R) {
  return toMsg_(R.unit_quaternion());
}

geometry_msgs::msg::Pose toRosPose_(const SE3 &T) {
  geometry_msgs::msg::Pose pose;
  pose.position = toMsg_(T.translation());
  pose.orientation = toRosQuat_(T.so3());
  return pose;
}

nav_msgs::msg::Odometry toRosOdom_(const SE3 &pose, const std::string slam_ref_frame,
                              const rclcpp::Time stamp) {
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = slam_ref_frame;
  odom.header.stamp = stamp;

  geometry_msgs::msg::Pose rosPose;
  rosPose.position.x = pose.translation()[0];
  rosPose.position.y = pose.translation()[1];
  rosPose.position.z = pose.translation()[2];
  auto quat = pose.unit_quaternion();
  rosPose.orientation.w = quat.w();
  rosPose.orientation.x = quat.x();
  rosPose.orientation.y = quat.y();
  rosPose.orientation.z = quat.z();
  odom.pose.pose = rosPose;
  std::array<double, 36> cov;
  for (int i = 0; i < 6; i++) {
    double var = 0.0;
    if (i < 3) {
      var = 0.01;
    } else {
      var = 0.01;
    }
    for (int j = 0; j < 6; j++) {
      if (j == i) {
        cov[6 * i + j] = var;
      } else {
        cov[6 * i + j] = 1e-5;
      }
    }
  }
  odom.pose.covariance = cov;
  return odom;
}

visualization_msgs::msg::MarkerArray
vizAllCentroidLandmarks(const std::vector<SE3> &allLandmarks,
                        const std::string &frame_id,
                        const std::vector<int> &allLabels) {
  visualization_msgs::msg::MarkerArray tMarkerArray;
  visualization_msgs::msg::Marker points;
  int cylinderId = 0;

  // TODO(ankit): Add a flag to read yaml file from open_vocab or closed_vocab
  YAML::Node cls_yaml_data = YAML::LoadFile(
      ament_index_cpp::get_package_share_directory("object_modeller") +
      "/config/open_vocab_cls_all.yaml");



  std::map<int, std::string> label_to_cls_name;
  std::map<int, std::vector<double>> label_to_cls_color;
  std::map<int, std::string> label_to_cls_mesh_path;
  std::map<int, double> label_to_mesh_scale_factor;
  double fixed_dim = 0.6;

  for (YAML::const_iterator it = cls_yaml_data.begin(); it != cls_yaml_data.end(); ++it) {

    int cls_id = it->second["id"].as<int>();
    std::string cls_name = it->first.as<std::string>();
    std::vector<double> cls_color = it->second["color"].as<std::vector<double>>();

    label_to_cls_name[cls_id] = cls_name;
    label_to_cls_color[cls_id] = cls_color;

    if (it->second["mesh_model_path"].IsNull()){
    continue;
    }
    else{
      std::string cls_mesh_path = it->second["mesh_model_path"].as<std::string>();
      double cls_mesh_scale_factor = it->second["mesh_model_scale"].as<double>();
      label_to_cls_mesh_path[cls_id] = cls_mesh_path;
      label_to_mesh_scale_factor[cls_id] = cls_mesh_scale_factor;
    }

  }


  // visualize poses as chairs
  for (size_t i = 0; i < allLandmarks.size(); i++) {
    auto o = allLandmarks[i];
    int cur_label = allLabels[i];
    geometry_msgs::msg::Point pt;
    auto obs_posit = o.translation();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now_ros_time();
    marker.id = cylinderId;
    if (label_to_cls_mesh_path.find(cur_label) == label_to_cls_mesh_path.end()) {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 0.7;
      marker.color.r = label_to_cls_color[cur_label][0];
      marker.color.g = label_to_cls_color[cur_label][1];
      marker.color.b = label_to_cls_color[cur_label][2];

    } else {
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.mesh_resource = label_to_cls_mesh_path[cur_label];

      marker.scale.x = label_to_mesh_scale_factor[cur_label] * fixed_dim;
      marker.scale.y = label_to_mesh_scale_factor[cur_label] * fixed_dim;
      marker.scale.z = label_to_mesh_scale_factor[cur_label] * fixed_dim;

      marker.color.r = label_to_cls_color[cur_label][0];
      marker.color.g = label_to_cls_color[cur_label][1];
      marker.color.b = label_to_cls_color[cur_label][2];
      marker.color.a = 1.0;
    }

    // height of cylinder
    double cyl_height = 1.0;
    double cyl_radius = 0.3;

    // Center of cylinder
    marker.pose.position.x = obs_posit[0];
    marker.pose.position.y = obs_posit[1];
    marker.pose.position.z = obs_posit[2];

    // yaw
    double yaw = obs_posit[0] * 0.34906585;
    // limit yaw to be within 0-2pi
    yaw = fmod(yaw, 6.28318531);
    // get quaternion from yaw using tf2
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];

    // add text to show the label
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = now_ros_time();
    text_marker.ns = "text";
    text_marker.id = cylinderId;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = obs_posit[0];
    text_marker.pose.position.y = obs_posit[1];
    text_marker.pose.position.z = obs_posit[2] + 0.75;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;
    text_marker.color.a = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 1.0;

    // given the mapping above, we can directly use the label as the text
    text_marker.text = label_to_cls_name[cur_label];
    tMarkerArray.markers.push_back(text_marker);

    cylinderId++;
    tMarkerArray.markers.push_back(marker);
  }
  return tMarkerArray;
}

visualization_msgs::msg::MarkerArray vizTrajectory(const std::vector<SE3> &poses,
                                              const std::string &frame_id,
                                              const int &robot_id) {
  visualization_msgs::msg::MarkerArray tMarkerArray;
  visualization_msgs::msg::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp = now_ros_time();
  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 1000;
  line_strip.id = 1001;

  points.type = visualization_msgs::msg::Marker::POINTS;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.05;
  points.scale.y = 0.05;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  line_strip.scale.x = 0.15;

  if (robot_id == 0) {
    // Line strip is orange
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.5;
    line_strip.color.a = 0.7;
    // Points are orange
    points.color.r = 1.0;
    points.color.g = 0.5;
    points.color.a = 1.0;
  } else if (robot_id == 1) {
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.7;
    // Points are blue
    points.color.b = 1.0;
    points.color.a = 1.0;
  } else if (robot_id == 2) {
    // Line strip is red
    line_strip.color.r = 1.0;
    line_strip.color.a = 0.7;
    // Points are red
    points.color.r = 1.0;
    points.color.a = 1.0;
  } else if (robot_id == 3) {
    // Line strip is cyan
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.7;
    // Points are cyan
    points.color.g = 1.0;
    points.color.b = 1.0;
    points.color.a = 1.0;
  } else if (robot_id == 4) {
    // Line strip is green
    line_strip.color.g = 1.0;
    line_strip.color.a = 0.7;
    // Points are green
    points.color.g = 1.0;
    points.color.a = 1.0;
  } else if (robot_id == 5) {
    // Line strip is purple
    line_strip.color.r = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.7;
    // Points are purple
    points.color.r = 1.0;
    points.color.b = 1.0;
    points.color.a = 1.0;
  } else {
    // Line strip is default (yellow)
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.a = 0.7;
    // Points are yellow
    points.color.r = 1.0;
    points.color.g = 1.0;
    points.color.a = 1.0;
  }

  // RCLCPP_DEBUG_STREAM(viz_logger(), "Number of poses to be visualized is:" << poses.size());
  double trajectory_length = 0;
  SE3 last_pose = SE3();
  for (auto o : poses) {
    // Between odom line strips
    geometry_msgs::msg::Point pt;
    auto obs_posit = o.translation();
    pt.x = obs_posit[0];
    pt.y = obs_posit[1];
    pt.z = obs_posit[2];
    points.points.push_back(pt);
    line_strip.points.push_back(pt);
    // calculate trajectory length
    double cur_displacement =
        (o.translation() - last_pose.translation()).norm();
    trajectory_length += cur_displacement;
    last_pose = o;
  }

  tMarkerArray.markers.push_back(line_strip);

  if (false) {
    // visualize the first pose as a sphere
    if (poses.size() > 0) {
      visualization_msgs::msg::Marker first_pose_marker;
      first_pose_marker.header.frame_id = frame_id;
      first_pose_marker.header.stamp = now_ros_time();
      first_pose_marker.ns = "points_and_lines";
      first_pose_marker.id = 0;
      first_pose_marker.type = visualization_msgs::msg::Marker::SPHERE;
      first_pose_marker.action = visualization_msgs::msg::Marker::ADD;

      first_pose_marker.pose.position.x = poses[0].translation()[0];
      first_pose_marker.pose.position.y = poses[0].translation()[1];
      first_pose_marker.pose.position.z = poses[0].translation()[2];
      first_pose_marker.pose.orientation.x = 0.0;
      first_pose_marker.pose.orientation.y = 0.0;
      first_pose_marker.pose.orientation.z = 0.0;
      first_pose_marker.pose.orientation.w = 1.0;
      first_pose_marker.scale.x = 0.5;
      first_pose_marker.scale.y = 0.5;
      first_pose_marker.scale.z = 0.5;
      if(robot_id == 0){
        // orange
        first_pose_marker.color.a = 1.0;
        first_pose_marker.color.r = 1.0;
        first_pose_marker.color.g = 0.5;
        first_pose_marker.color.b = 0.0;
      }
      else if(robot_id == 1){
        // blue
        first_pose_marker.color.a = 1.0;
        first_pose_marker.color.r = 0.0;
        first_pose_marker.color.g = 0.0;
        first_pose_marker.color.b = 1.0;
      }
      else if(robot_id == 2){
        // red
        first_pose_marker.color.a = 1.0;
        first_pose_marker.color.r = 1.0;
        first_pose_marker.color.g = 0.0;
        first_pose_marker.color.b = 0.0;
      }


      tMarkerArray.markers.push_back(first_pose_marker);
    }
  }
  return tMarkerArray;
}

visualization_msgs::msg::MarkerArray
vizTrajectoryAndPoseInds(const std::vector<SE3> &poses,
                         const std::vector<size_t> &pose_inds,
                         const std::string &frame_id) {
  visualization_msgs::msg::MarkerArray tMarkerArray;
  // sanity check
  if (pose_inds.size() != poses.size()) {
    RCLCPP_ERROR_STREAM(viz_logger(),
        "vizTrajectoryAndPoseInds fail to due to pose_inds and poses having "
        "different sizes!!!");
    return tMarkerArray;
  }

  for (size_t i = 0; i < poses.size(); i++) {
    auto cur_pose = poses[i];
    auto cur_pose_ind = pose_inds[i];


    // Between odom line strips
    geometry_msgs::msg::Point pt;
    auto obs_posit = cur_pose.translation();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now_ros_time();
    marker.ns = "traj_and_pose_inds";
    marker.id = cur_pose_ind;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obs_posit[0];
    marker.pose.position.y = obs_posit[1];
    marker.pose.position.z = obs_posit[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    tMarkerArray.markers.push_back(marker);
  }

  return tMarkerArray;
}

geometry_msgs::msg::PoseStamped makeROSPose(const SE3 &tf, std::string frame_id,
                                       const rclcpp::Time stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose = toRosPose_(tf);
  return pose;
}

SE3 toSE3(geometry_msgs::msg::PoseStamped pose) {
  Vector3d pos;
  Quaterniond quat;
  tf2::fromMsg(pose.pose.position, pos);
  tf2::fromMsg(pose.pose.orientation, quat);

  SE3 tf;
  tf.translation() = pos;
  tf.setQuaternion(quat);

  return tf;
}
void vizTreeModels(const std::vector<Cylinder> &scanTm,
                   visualization_msgs::msg::MarkerArray &tMarkerArray,
                   size_t &cylinderId) {
  for (const auto &tree : scanTm) {
    Scalar maxTreeRadius = 0.25;
    Scalar maxAxisTheta = 45;
    visualization_msgs::msg::Marker marker;
    // TODO(ankit): The frame_id should be passed as an argument
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = now_ros_time();
    marker.id = cylinderId;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Center of cylinder
    // Shift the center of the cylinder to the center of the ray
    double shift = 5.0;
    marker.pose.position.x = tree.model.root[0] + shift * tree.model.ray[0];
    marker.pose.position.y = tree.model.root[1] + shift * tree.model.ray[1];
    marker.pose.position.z = tree.model.root[2] + shift * tree.model.ray[2];


    // Orientation of cylidner
    Vector3 src_vec(0, 0, 1);
    Quat q_rot = Quat::FromTwoVectors(src_vec, tree.model.ray);
    marker.pose.orientation.x = q_rot.x();
    marker.pose.orientation.y = q_rot.y();
    marker.pose.orientation.z = q_rot.z();
    marker.pose.orientation.w = q_rot.w();

    marker.scale.x = 2 * tree.model.radius;
    marker.scale.y = 2 * tree.model.radius;
    marker.scale.z = 2 * shift * 0.65;

    if (cylinderId < 200000) {
      // current scan observations
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      // accumulated map
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }

    tMarkerArray.markers.push_back(marker);
    cylinderId++;
  }

  // delete extras
  for (auto i = cylinderId; i < cylinderId + 50; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = now_ros_time();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tMarkerArray.markers.push_back(marker);
  }
}

void vizCubeModels(const std::vector<Cube> &cubeModels,
                   visualization_msgs::msg::MarkerArray &tMarkerArray,
                   size_t &cubeId, const bool &is_global_map) {
  float scan_map_alpha = 0.6;
  float global_map_alpha = 0.6;
  float alpha;
  float red, blue, green;
  rclcpp::Time stamp;
  // To differentiate from local map, global map will (1) be transparent, (2)
  // display permanently and (3) use different color
  if (is_global_map) {
    alpha = global_map_alpha;
    stamp = now_ros_time();
    red = 0.0;
    blue = 1.0;
    green = 0.0;
  } else {
    alpha = scan_map_alpha;
    stamp = now_ros_time();
    red = 0.5;
    blue = 0.5;
    green = 0.5;
  }

  for (const auto &cur_cube : cubeModels) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = stamp;
    marker.id = cubeId;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Center of cube
    marker.pose.position.x = cur_cube.model.pose.x();
    marker.pose.position.y = cur_cube.model.pose.y();
    marker.pose.position.z = cur_cube.model.pose.z();

    // Orientation of cube
    Vector3 src_vec(0, 0, 1);
    gtsam::Quaternion q_rot = cur_cube.model.pose.rotation().toQuaternion();
    marker.pose.orientation.x = q_rot.x();
    marker.pose.orientation.y = q_rot.y();
    marker.pose.orientation.z = q_rot.z();
    marker.pose.orientation.w = q_rot.w();

    marker.scale.x = cur_cube.model.scale(0);
    marker.scale.y = cur_cube.model.scale(1);
    marker.scale.z = cur_cube.model.scale(2);

    marker.color.a = alpha;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;

    tMarkerArray.markers.push_back(marker);
    cubeId++;
  }

  for (auto i = cubeId; i < cubeId + 100; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = now_ros_time();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tMarkerArray.markers.push_back(marker);
  }
}

visualization_msgs::msg::MarkerArray
vizGroundModel(const std::vector<Plane> &gplanes, const std::string &frame_id,
               int idx) {
  visualization_msgs::msg::MarkerArray groundModels;
  for (const auto &g : gplanes) {
    auto gmodel = vizGroundModel(g, frame_id, idx);
    groundModels.markers.push_back(gmodel);
    idx++;
  }

  for (auto i = idx; i < idx + 300; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = now_ros_time();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.ns = "plane";
    groundModels.markers.push_back(marker);
  }
  return groundModels;
}
visualization_msgs::msg::Marker vizGroundModel(const Plane &gplane,
                                          const std::string &frame_id,
                                          const int idx) {
  visualization_msgs::msg::Marker cube;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.id = idx;
  cube.ns = "plane";
  cube.scale.x = 2.5;
  cube.scale.y = 2.5;
  cube.scale.z = 0.1;
  if (idx < 200) {
    cube.color.r = 1.0;
    cube.color.g = 1.0;
    cube.color.b = 0.0;
    cube.color.a = 0.5;
  } else {
    cube.color.r = 1.0;
    cube.color.g = 1.0;
    cube.color.b = 1.0;
    cube.color.a = 0.5;
  }

  cube.header.frame_id = frame_id;
  cube.header.stamp = now_ros_time();
  cube.pose.position.x = gplane.model.centroid(0);
  cube.pose.position.y = gplane.model.centroid(1);
  cube.pose.position.z = gplane.model.centroid(2);

  Vector3 src_vec(0, 0, 1);
  Quat q_rot = Quat::FromTwoVectors(src_vec, gplane.model.plane.segment(0, 3));
  cube.pose.orientation.x = q_rot.x();
  cube.pose.orientation.y = q_rot.y();
  cube.pose.orientation.z = q_rot.z();
  cube.pose.orientation.w = q_rot.w();
  return cube;
}

void landmarksToCloud(const std::vector<std::vector<TreeVertex>> &landmarks,
                      CloudT::Ptr &cloud) {
  size_t numPoints = 0;
  size_t color_id = 0;
  std::vector<float> color_values((int)landmarks.size());

  std::iota(std::begin(color_values), std::end(color_values), 1);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(color_values.begin(), color_values.end(), gen);

  RCLCPP_DEBUG_STREAM(viz_logger(), "Color values size: " << color_values.size());
  for (const auto &tree : landmarks) {
    if (tree[0].treeId == -1)
      continue;
    for (auto vtx : tree) {
      for (const auto &point : vtx.points) {
        PointT vp = point;
        vp.intensity = color_values[color_id];
        cloud->push_back(vp);
        numPoints++;
      }
    }
    color_id++;
  }
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->is_dense = false;
}

cv::Mat DecodeImage(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    image = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    RCLCPP_ERROR(viz_logger(), "Failed to convert depth image");
  }
  return image;
}

} // namespace sloam
