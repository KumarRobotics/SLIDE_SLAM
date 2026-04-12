/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#include <cylinder.h>
#include <rclcpp/rclcpp.hpp>

Cylinder::Cylinder(const std::vector<TreeVertex> vertices, const Plane &gplane,
                   const FeatureModelParams &fmParams) {
  // The plane centroid approximates the center of the tf cloud
  PointT centroid;
  centroid.x = gplane.model.centroid(0);
  centroid.y = gplane.model.centroid(1);
  centroid.z = gplane.model.centroid(2);

  TreeVertex firstVtx = vertices[1];
  bool withinMaxDist =
      euclideanDist2D(firstVtx.coords, centroid) < fmParams.maxLidarDist;

  isValid = false;
  if (withinMaxDist) {
    // int numNewFeatures = 0;
    computeModel(vertices, fmParams.defaultCylinderRadius,
                 fmParams.featuresPerTree);
    // projects the tree into the ground plane
    bool validZ = groundBasedRoot(gplane);
    bool validNorm = model.root.norm() > 0.01;
    bool validRadius = model.radius > 0.0;
    bool validTree =
        filter(fmParams.maxTreeRadius, fmParams.maxAxisTheta, gplane);
    isValid = (validZ && validRadius && validTree && validNorm);
  }
}

Cylinder::Cylinder(const Vector3 &root, const Vector3 &ray,
                   const double &radius, const int &label) {
  model.root = root;
  model.ray = ray;
  model.radius = radius;
  model.semantic_label = label;
}

PointT Cylinder::asPoint(const Scalar height) {
  Scalar t = (height - model.root[2]) / model.ray[2];
  Vector3 eigenPoint = model.root + t * model.ray;
  PointT point;
  point.x = eigenPoint[0];
  point.y = eigenPoint[1];
  point.z = 0;
  return point;
}

bool Cylinder::groundBasedRoot(const Plane &gplane) {
  const auto &ground = gplane.model.plane;
  float dist = (abs(ground[0] * model.root(0) + ground[1] * model.root(1) +
                    ground[2] * model.root(2) + ground[3]) /
                ground.segment(0, 3).norm());

  if (dist < 2.0) {
    const auto normal = gplane.model.plane.segment(0, 3);
    model.root = rayPlaneIntersection(gplane.model.centroid, normal, model.root,
                                      model.ray);
    return true;
  }
  return false;
}

bool Cylinder::filter(const Scalar &maxTreeRadius, const Scalar &maxAxisTheta,
                      const Plane &gplane) {
  if (model.radius == -1) return false;

  // use estimated ground plane to check if the ray direction is reasonable
  Vector3 up(gplane.model.plane[0], gplane.model.plane[1],
             gplane.model.plane[2]);
  Scalar theta = (180 / PIDEF) *
                 acos(((model.ray).dot(up)) / ((model.ray).norm() * up.norm()));
  // Remove if radius is too large or angle to up is too large
  if ((model.radius < maxTreeRadius) &&
      ((theta <= maxAxisTheta) || (theta >= 180 - maxAxisTheta))) {
    return true;
  }
  return false;
}

void Cylinder::computeModel(const std::vector<TreeVertex> &landmarkVtxs,
                            const Scalar defaultCylinderRadius,
                            const Scalar featuresPerTree) {
  TreeVertex firstVtx = landmarkVtxs[2];
  std::vector<float> validRadii;
  CloudT::Ptr tree(new CloudT);

  for (size_t i = 0; i < landmarkVtxs.size(); ++i) {
    TreeVertex vtx = landmarkVtxs[i];
    tree->push_back(vtx.coords);

    for (auto &vtxP : vtx.points) {
      vtxP.intensity = (float)firstVtx.treeId;
      features.push_back(vtxP);
    }

    model.radii.push_back(vtx.radius);
    if (vtx.points.size() > 3) {
      validRadii.push_back(vtx.radius);
    }
  }

  id = firstVtx.treeId;
  auto bottomPt = landmarkVtxs[1].coords;
  auto topPt = landmarkVtxs[landmarkVtxs.size() - 2].coords;

  model.root(0) = bottomPt.x;
  model.root(1) = bottomPt.y;
  model.root(2) = bottomPt.z;

  // if tree is too short, we don't use it
  if (pcl::geometry::squaredDistance(bottomPt, topPt) < 1.5 ||
      tree->size() == 0) {
    model.radius = -1;
    return;
  }

  // Create a model parameter object to record the result
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // inliers indicate the point where the error can be tolerated, and the
  // sequence number of the point cloud is recorded
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create a splitter
  pcl::SACSegmentation<PointT> seg;
  // Optional, this setting can choose whether the points displayed in the
  // result plane are split points or the remaining points.
  seg.setOptimizeCoefficients(true);
  // Mandatory-set the target geometry
  seg.setModelType(pcl::SACMODEL_LINE);
  // Segmentation method: random sampling method
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the error tolerance range, which is the threshold
  seg.setDistanceThreshold(0.25);
  // Input point cloud
  seg.setInputCloud(tree);
  // Split point cloud, get plane and normal vector
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("cylinder"), "RANSAC Line Fitting failed");
    model.radius = -1;
    return;
  }

  // Force tree to be straight up
  model.ray(0) = coefficients->values[3];
  model.ray(1) = coefficients->values[4];
  model.ray(2) = coefficients->values[5];

  if (validRadii.size() > 0) {
    std::sort(validRadii.begin(), validRadii.end());
    // Find first non-zero element
    int d =
        std::distance(std::begin(validRadii),
                      std::find_if(std::begin(validRadii), std::end(validRadii),
                                   [](float x) { return x > 0; }));

    int middle = std::min((int)(validRadii.size() - 1),
                          (d + 1) + (int)((validRadii.size() - d) / 2));

    model.radius = validRadii[middle];
    if (model.radius == 0)
      model.radius = -1;
    else if (model.radius < defaultCylinderRadius)
      model.radius = defaultCylinderRadius;
  } else {
    model.radius = -1;
  }

  for (const auto &vtx : landmarkVtxs) {
    model.vertices.push_back(vtx);
  }

  model.lambda = 1.0;

  features.resize(featuresPerTree);
}

Scalar Cylinder::distance(const CylinderParameters &tgt) const {
  // sample trees at pre-defined heights

  if (tgt.semantic_label != model.semantic_label) {
    return 1000;
  }

  // at those sampled heights, as long as the cylinder model is close to each other at any point, we treat them as a match
  std::vector<Scalar> heights;
  heights.push_back(0.0);
  heights.push_back(3.0);
  heights.push_back(6.0);
  Scalar distance = 10000.0;
  // get the smallest X and Y distance at the two heights
  for (const auto &height : heights) {
    Scalar src_t = (height - model.root[2]) / model.ray[2];
    Vector3 modelPoint = model.root + src_t * model.ray;

    Scalar tgt_t = (height - tgt.root[2]) / tgt.ray[2];
    Vector3 tgtPoint = tgt.root + tgt_t * tgt.ray;

    // if distance is smaller than the current distance, update the distance
    Scalar dist = (modelPoint - tgtPoint).norm();
    if (dist < distance) {
      distance = dist;
    }
  }

  // print the distance
  bool use_position_only = true;
  if (use_position_only) {
    return distance;
  } else {
    Scalar distance_weight = 0.8;
    Scalar radius_diff = abs(model.radius - tgt.radius);
    return distance_weight * distance + (1 - distance_weight) * radius_diff;
  }
};

Scalar Cylinder::distance(const PointT &point) const {
  Vector3 eigenPoint;
  eigenPoint << point.x, point.y, point.z;
  Vector3 projectedPoint =
      model.root + (((eigenPoint - model.root).dot(model.ray)) /
                    ((model.ray).dot(model.ray))) *
                       model.ray;
  return (eigenPoint - projectedPoint).norm() - model.radius;
}

void Cylinder::project(const SE3 &tf) {
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  Vector3 otherPoint = model.root + model.ray;
  model.root = tf * model.root;
  otherPoint = tf * otherPoint;
  model.ray = otherPoint - model.root;
}

// project and return as a new object
Cylinder Cylinder::projectAndReturn(const SE3 &tf) {
  Vector3 otherPoint = model.root + model.ray;
  Vector3 newRoot = tf * model.root;
  otherPoint = tf * otherPoint;
  Vector3 newRay = otherPoint - newRoot;
  return Cylinder(newRoot, newRay, model.radius, model.semantic_label);
}