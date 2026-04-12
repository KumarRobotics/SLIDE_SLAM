/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sloamNodelet.h>

namespace sloam {

SLOAMNodelet::SLOAMNodelet(const rclcpp::NodeOptions &options)
    : rclcpp::Node("sloam", options) {
  sloamNode = std::make_shared<SLOAMNode>(this);
  RCLCPP_INFO(this->get_logger(), "Created Sloam Component");
}

}  // namespace sloam

RCLCPP_COMPONENTS_REGISTER_NODE(sloam::SLOAMNodelet)
