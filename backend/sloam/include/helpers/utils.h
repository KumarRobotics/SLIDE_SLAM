#pragma once

#include <definitions.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

inline float pow_2(const Scalar &x) { return x * x; }

inline float euclideanDist2D(const PointT &vecA, const PointT &vecB) {
  return std::sqrt(pow_2(vecA.x - vecB.x) + pow_2(vecA.y - vecB.y));
}

inline PointT computeCentroid(const VectorType &features) {
  PointT centroid;
  for (const auto &p : features) {
    centroid.x += p.x;
    centroid.y += p.y;
    centroid.z += p.z;
  }

  centroid.x /= (Scalar)features.size();
  centroid.y /= (Scalar)features.size();
  centroid.z /= (Scalar)features.size();
  return centroid;
}

inline void quatMsg2SE3(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &quatMsg,
                        SE3 &pose) {
  Quat q;
  q.w() = quatMsg->quaternion.w;
  q.x() = quatMsg->quaternion.x;
  q.y() = quatMsg->quaternion.y;
  q.z() = quatMsg->quaternion.z;
  pose.setQuaternion(q);
}

// Find intersection between ray (cylinder axis) and plane
inline Vector3 rayPlaneIntersection(const Vector3 &planeCentroid,
                                    const Vector3 &planeNormal,
                                    const Vector3 &rayOrigin,
                                    const Vector3 &rayDirection) {
  float denom = planeNormal.dot(rayDirection);
  if (abs(denom) > 0.001f) {
    float t = (planeCentroid - rayOrigin).dot(planeNormal) / denom;
    if (t >= 0.001f) return rayOrigin + t * rayDirection;
  }
  return rayOrigin;
}


inline SE3 ToSE3(const geometry_msgs::msg::Pose &transform_msg) {
  // Extract the translation components from the geometry_msgs::Transform message
  Eigen::Vector3d translation_vector;
  translation_vector.x() = transform_msg.position.x;
  translation_vector.y() = transform_msg.position.y;
  translation_vector.z() = transform_msg.position.z;

  // Extract the quaternion components from the geometry_msgs::Transform message
  Eigen::Quaterniond quaternion;
  quaternion.x() = transform_msg.orientation.x;
  quaternion.y() = transform_msg.orientation.y;
  quaternion.z() = transform_msg.orientation.z;
  quaternion.w() = transform_msg.orientation.w;

  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rotation_matrix = quaternion.normalized().toRotationMatrix();

  // Create the Sophus::SE3d transformation using rotation matrix and translation vector
  Sophus::SE3d se3_transform(rotation_matrix, translation_vector);

  return se3_transform;
}

inline geometry_msgs::msg::Pose ToRosPoseMsg(const SE3 &T){
  // Create a geometry_msgs::Transform message
  geometry_msgs::msg::Pose transform_msg;

  // Extract the rotation matrix and translation vector from Sophus::SE3d
  Eigen::Matrix3d rotation_matrix = T.rotationMatrix();
  Eigen::Vector3d translation_vector = T.translation();

  // Convert rotation matrix to quaternion
  Eigen::Quaterniond quaternion(rotation_matrix);

  // Fill in the fields of the geometry_msgs::Transform message
  transform_msg.position.x = translation_vector.x();
  transform_msg.position.y = translation_vector.y();
  transform_msg.position.z = translation_vector.z();

  transform_msg.orientation.x = quaternion.x();
  transform_msg.orientation.y = quaternion.y();
  transform_msg.orientation.z = quaternion.z();
  transform_msg.orientation.w = quaternion.w();
  return transform_msg;
}
