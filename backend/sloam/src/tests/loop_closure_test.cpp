#include <cylinder.h>
#include <definitions.h>
#include <plane.h>
#include <rcutils/logging.h>
#include <serialization.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <loopclosure.hpp>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
using ::testing::EmptyTestEventListener;
using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

class LoopClosureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    setupParams();
    PointT pta, ptb, ptc, ptd;
    pta.x = 0;
    pta.y = 0;
    pta.z = 0;
    ptb.x = 0;
    ptb.y = 1;
    ptb.z = 0;
    ptc.x = 1;
    ptc.y = 0;
    ptc.z = 0;
    ptd.x = 0.5;
    ptd.y = 0;
    ptd.z = 0;
    VectorType gfeatures{pta, ptb, ptc, ptd};
    plane = boost::make_shared<Plane>(gfeatures, params);

    std::ifstream ifs(
        "/opt/sloam_ws/src/sloam/src/tests/aux/still_landmarks_t0");
    boost::archive::text_iarchive ia(ifs);
    // read class state from archive
    ia >> landmarks;

    // ROS2 port: use rcutils_logging API instead of ros::console.
    rcutils_logging_set_logger_level("loop_closure_test",
                                     RCUTILS_LOG_SEVERITY_DEBUG);
  }

  void setupParams() {
    params.scansPerSweep = 1;
    params.maxLidarDist = 30.0;
    params.maxGroundLidarDist = 30.0;
    params.minGroundLidarDist = 0.0;

    params.groundRadiiBins = 1;
    params.groundThetaBins = 1;
    params.groundMatchThresh = 2.0;
    params.groundRetainThresh = 0.1;

    params.maxTreeRadius = 0.3;
    params.maxAxisTheta = 10;
    params.maxFocusOutlierDistance = 0.5;
    params.roughcylinderMatchThresh = 3.0;
    params.cylinderMatchThresh = 1.0;

    params.AddNewTreeThreshDist = 2.0;

    params.featuresPerTree = 100;
    params.numGroundFeatures = 60;
    params.defaultCylinderRadius = 0.1;
  }

  PointVector readFile(std::string path) {
    PointVector landmarks;
    std::ifstream infile(path);

    while (infile) {
      std::string s;
      if (!std::getline(infile, s)) break;

      std::istringstream ss(s);
      vecPtT record;

      while (ss) {
        std::string s;
        if (!std::getline(ss, s, ',')) break;
        record.push_back(stod(s));
      }

      landmarks.push_back(record);
    }
    if (!infile.eof()) {
      std::cerr << "Fooey!\n";
    }
    return landmarks;
  }

  std::vector<std::vector<TreeVertex>> landmarks;
  FeatureModelParams params;
  boost::shared_ptr<Plane> plane;
};

TEST_F(LoopClosureTest, makePtVector) {
  SE3 tfA = SE3(Sophus::SO3d(), Eigen::Vector3d(1.0, -1, 0));
  SE3 tfB = SE3(Sophus::SO3d(), Eigen::Vector3d(2.0, 0, 0));

  std::vector<Cylinder> obsA;
  std::vector<Cylinder> obsB;
  for (auto l : landmarks) {
    auto cA = Cylinder(l, *plane, params);
    auto cB = Cylinder(l, *plane, params);
    if (cA.model.radius > 0) {
      cA.project(tfA);
      cB.project(tfB);
      obsA.push_back(cA);
      obsB.push_back(cB);
    }
  }

  auto loop_closure = loop::UrquhartLoopCloser(10, 100.5);
  SE3 tf = SE3();
  auto ptv = loop_closure.tryLoopClosure(obsA, obsB, tf);
  std::cout << tf.matrix() << std::endl;
}

// TEST_F(LoopClosureTest, PoseEstimation)
// {

//   auto pA = readFile();
//   auto pB = readFile();

//   CloudT ptA;
//   for(const auto d : pA)
//   {
//     PointT pt;
//     pt.x = d[0];
//     pt.y = d[1];
//     pt.z = d[2];
//     ptA.points.push_back(pt);
//   }
//   ptA.width    = ptA.points.size();
//   ptA.height   = 1;

//   CloudT ptB;
//   for(const auto d : pB)
//   {
//     PointT pt;
//     pt.x = d[0];
//     pt.y = d[1];
//     pt.z = d[2];
//     ptB.points.push_back(pt);
//   }

//   ptB.width    = ptB.points.size();
//   ptB.height   = 1;
// }