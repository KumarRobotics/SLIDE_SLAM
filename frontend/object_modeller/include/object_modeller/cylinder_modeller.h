#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// include ROS 2 stuff
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>


// start a ROS2 node
// subscribe to the point cloud topic
// run the cylinder segmentation algorithm




using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

// start a class called CylinderModeller - a rclcpp::Node subclass
class CylinderModeller : public rclcpp::Node {
public:
    // default constructor
    CylinderModeller();
    // function for modelling
    void modelCylinder();
private:
    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_tree_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_cloud_;
    // publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cylinder_model_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_model_;
    // callback functions
    void treeCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
    void groundCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
    // cloud pointers for latest ground and tree cloud
    CloudT::Ptr tree_cloud_ptr_;
    CloudT::Ptr ground_cloud_ptr_;
    int cylinder_counter_;
};
