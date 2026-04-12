
// include cylinder_modeller.h
#include <object_modeller/cylinder_modeller.h>

// start a ROS2 node
// subscribe to the point cloud topic
// run the cylinder segmentation algorithm
// publish the cylinder model
// publish the cylinder model as a point cloud
// publish the cylinder model as a mesh

// implement the default constructor
CylinderModeller::CylinderModeller() : rclcpp::Node("cylinder_modeller") {
  // initialise the cylinder counter
  cylinder_counter_ = 0;

  // subscribe to the point cloud topic
  sub_tree_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/tree_cloud", 1,
      std::bind(&CylinderModeller::treeCloudCallback, this,
                std::placeholders::_1));
  sub_ground_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ground_cloud", 1,
      std::bind(&CylinderModeller::groundCloudCallback, this,
                std::placeholders::_1));

  // publish the cylinder model
  pub_cylinder_model_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/cylinder_model", 1);

  // publish the plane model
  pub_plane_model_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_model", 1);

  // initialise the cloud pointers as nullptr
  tree_cloud_ptr_ = nullptr;
  ground_cloud_ptr_ = nullptr;
}

// implement the tree cloud callback
void CylinderModeller::treeCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
  // print
  RCLCPP_INFO(this->get_logger(), "Tree cloud received");
  // convert the message to a pcl point cloud
  // initialize tree_cloud_ptr_ as a new cloud
  tree_cloud_ptr_ = CloudT::Ptr(new CloudT);
  pcl::fromROSMsg(*cloud_msg, *tree_cloud_ptr_);
}

// implement the ground cloud callback
void CylinderModeller::groundCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
  // print
  RCLCPP_INFO(this->get_logger(), "Ground cloud received");
  // convert the message to a pcl point cloud
  ground_cloud_ptr_ = CloudT::Ptr(new CloudT);
  pcl::fromROSMsg(*cloud_msg, *ground_cloud_ptr_);
}

// implement the modelling function that extract cylinder models from tree cloud
// based on the commented code above
void CylinderModeller::modelCylinder() {
  // sanity check if cloud is nullptr
  RCLCPP_INFO(this->get_logger(), "checking nullptr");
  if (tree_cloud_ptr_ == nullptr) {
    std::cerr << "Tree cloud is nullptr" << std::endl;
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Tree cloud is not nullptr");
  }
  // sanity check if ground cloud is nullptr
  if (ground_cloud_ptr_ == nullptr) {
    std::cerr << "Ground cloud is nullptr" << std::endl;
    return;
  }


  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());


  // Datasets
  CloudT::Ptr cloud(new CloudT);
  // assign tree_cloud to cloud
  cloud = tree_cloud_ptr_;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(10);
  ne.compute(*cloud_normals);


  // Create the segmentation object for cylinder segmentation and set all the
  // parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(10);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0, 0.3);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);


  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Obtain the cylinder inliers and coefficients
  RCLCPP_INFO(this->get_logger(), "starting segmentation...");
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  CloudT::Ptr cloud_cylinder(new CloudT());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty())
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else {
    std::cerr << "PointCloud representing the cylindrical component: "
              << cloud_cylinder->size() << " data points." << std::endl;
    RCLCPP_WARN(this->get_logger(), "FOUND MODEL");
  }
}

// start a ROS2 node
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto cylinder_modeller = std::make_shared<CylinderModeller>();

  rclcpp::Rate r(10);  // 10 hz
  while (rclcpp::ok()) {
    rclcpp::spin_some(cylinder_modeller);
    cylinder_modeller->modelCylinder();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
