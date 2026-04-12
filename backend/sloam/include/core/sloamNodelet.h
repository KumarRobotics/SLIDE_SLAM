/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sloamNode.h>

#include <memory>

namespace sloam {
class SLOAMNodelet : public rclcpp::Node {
 public:
  explicit SLOAMNodelet(const rclcpp::NodeOptions &options);
  ~SLOAMNodelet() = default;

 private:
  SLOAMNode::Ptr sloamNode;
};
}  // namespace sloam
