#include <cylinder.h>
#include <definitions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <plane.h>
#include <rcutils/logging.h>
#include <serialization.h>
#include <sloam.h>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>

#include "gtest/gtest.h"
using ::testing::EmptyTestEventListener;
using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

void filterPC(CloudT::Ptr pc, CloudT::Ptr outPc, float quadZ) {
  if (pc->size() == 0) return;

  pcl::VoxelGrid<PointT> vox;
  CloudT voxCloud;
  vox.setInputCloud(pc);
  vox.setLeafSize(0.1f, 0.1f, 0.1f);
  vox.filter(voxCloud);

  size_t numPoints = 0;
  for (size_t i = 0; i < voxCloud.height; i++) {
    for (size_t j = 0; j < voxCloud.width; j++) {
      auto p = voxCloud.points[i * voxCloud.width + j];
      if (pcl::isFinite(p) && (p.z < quadZ)) {
        outPc->push_back(p);
      }
    }
  }

  outPc->width = outPc->points.size();
  outPc->height = 1;
  std::cout << "INPUT GROUND CLOUD POINTS: " << outPc->points.size()
            << std::endl;
}

class SLOAMTest : public ::testing::Test {
 protected:
  void SetUp() override {
    setupParams();

    // t0
    readInputData(t0Input, "still", "t0");
    t0Input.poseEstimate = SE3();

    // t1
    readInputData(t1Input, "still", "t1");
    t1Input.poseEstimate = SE3();

    // ROS2 port: use rcutils_logging API instead of ros::console.
    rcutils_logging_set_logger_level("sloam_test",
                                     RCUTILS_LOG_SEVERITY_DEBUG);
  }

  void readInputData(SloamInput& inp, std::string prefix, std::string stamp) {
    std::string base_dir = "/opt/sloam_ws/src/sloam/src/tests/aux/";
    std::string landmarks_path = base_dir + prefix + "_landmarks_" + stamp;
    std::string ground_path =
        base_dir + prefix + "_ground_" + stamp + std::string(".pcd");

    // read class state from archive
    std::ifstream ifs(landmarks_path);
    boost::archive::text_iarchive ia(ifs);
    ia >> inp.landmarks;

    for (auto& tree : inp.landmarks) {
      for (auto& vtx : tree) {
        vtx.points.resize(5);
      }
    }

    // CloudT::Ptr rawGround(new CloudT);
    // pcl::io::loadPCDFile<PointT>(ground_path, *rawGround);
    // filterPC(rawGround, inp.groundCloud, 1);
    pcl::io::loadPCDFile<PointT>(ground_path, *inp.groundCloud);
  }

  void setupParams() {
    params.scansPerSweep = 1;
    params.maxLidarDist = 15.0;
    params.maxGroundLidarDist = 30.0;
    params.minGroundLidarDist = 0.0;

    params.groundRadiiBins = 1;
    params.groundThetaBins = 18;
    params.groundMatchThresh = 2.0;
    params.groundRetainThresh = 0.05;

    params.maxTreeRadius = 0.3;
    params.maxAxisTheta = 10;
    params.maxFocusOutlierDistance = 0.5;
    params.roughcylinderMatchThresh = 3.0;
    params.cylinderMatchThresh = 1.0;

    params.AddNewTreeThreshDist = 2.0;

    params.featuresPerTree = 2;
    params.numGroundFeatures = 60;
    params.defaultCylinderRadius = 0.1;
  }

  SloamInput t0Input;
  SloamInput t1Input;
  SloamInput t0MovingInput;
  SloamInput t1MovingInput;
  FeatureModelParams params;
};

TEST_F(SLOAMTest, FirstScan) {
  sloam::sloam sloam = sloam::sloam();
  SloamOutput out;
  sloam.setFmParams(params);
  const std::vector<Cube> detected_cubes_odom;
  std::vector<Cube> detected_cubes_world;
  const std::vector<Cube> submap_for_cubes;
  std::vector<int> cube_match_indices;
  sloam.RunSloam(t0Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);
  EXPECT_TRUE(out.tm.size() > 0);
  EXPECT_TRUE(sloam.getPrevGroundModel().size() > 0);
}

TEST_F(SLOAMTest, SecondScan) {
  sloam::sloam sloam = sloam::sloam();
  SloamOutput out;
  sloam.setFmParams(params);
  const std::vector<Cube> detected_cubes_odom;
  std::vector<Cube> detected_cubes_world;
  const std::vector<Cube> submap_for_cubes;
  std::vector<int> cube_match_indices;
  sloam.RunSloam(t1Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);
  EXPECT_TRUE(out.tm.size() > 0);
  EXPECT_TRUE(sloam.getPrevGroundModel().size() > 0);
}

TEST_F(SLOAMTest, SLOAMSucess) {
  sloam::sloam sloam = sloam::sloam();
  SloamOutput out;
  sloam.setFmParams(params);
  const std::vector<Cube> detected_cubes_odom;
  std::vector<Cube> detected_cubes_world;
  const std::vector<Cube> submap_for_cubes;
  std::vector<int> cube_match_indices;
  sloam.RunSloam(t0Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);
  // first scan output is the initial map
  t1Input.mapModels = out.tm;
  bool success =
      sloam.RunSloam(t1Input, detected_cubes_odom, detected_cubes_world,
                     submap_for_cubes, cube_match_indices, out);
  EXPECT_TRUE(success);
}

TEST_F(SLOAMTest, PoseOptimization) {
  sloam::sloam sloam = sloam::sloam();
  SloamOutput out;
  sloam.setFmParams(params);
  const clock_t t0 = clock();
  const std::vector<Cube> detected_cubes_odom;
  std::vector<Cube> detected_cubes_world;
  const std::vector<Cube> submap_for_cubes;
  std::vector<int> cube_match_indices;
  sloam.RunSloam(t0Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);
  // first scan output is the initial map
  t1Input.mapModels = out.tm;
  sloam.RunSloam(t1Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);

  const clock_t t1 = clock();
  const double elapsedSec = (t1 - t0) / (double)CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << 1000 * elapsedSec << std::endl;

  float translation = out.T_Delta.translation().norm();
  std::cout << "TRANSLATION: " << translation << std::endl;
  EXPECT_NEAR(0.0, translation, 0.1);
}

TEST_F(SLOAMTest, ObjectAssociation) {
  sloam::sloam sloam = sloam::sloam();
  SloamOutput out;
  sloam.setFmParams(params);
  const std::vector<Cube> detected_cubes_odom;
  std::vector<Cube> detected_cubes_world;
  const std::vector<Cube> submap_for_cubes;
  std::vector<int> cube_match_indices;
  sloam.RunSloam(t0Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);
  // first scan output is the initial map
  t1Input.mapModels = out.tm;
  sloam.RunSloam(t1Input, detected_cubes_odom, detected_cubes_world,
                 submap_for_cubes, cube_match_indices, out);

  bool matched = false;
  for (auto m : out.matches) matched = matched || m != -1;
  EXPECT_TRUE(matched);
}