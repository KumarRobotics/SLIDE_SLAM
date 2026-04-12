cmake_minimum_required(VERSION 3.16)
project(sloam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_TESTING "Build tests" OFF)
option(USE_CLIPPER "Use clipper" ON)

set(CC_TARGET_PREFIX sloam)
include(CMakeHelpers)

# ament / rclcpp packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sloam_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)

# system / non-ROS packages
find_package(Boost REQUIRED system serialization)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

if(NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "Yaml-cpp is not found
  Please install yaml-cpp using:

  sudo apt-get install libyaml-cpp-dev")
endif()

set(CLIPPER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/clipper_semantic_object")
set(CLIPPER_VALUE 0)
if(USE_CLIPPER)
  add_subdirectory(${CLIPPER_DIR})
  set(CLIPPER_VALUE 1)
endif()

# Aggregate of ament dependencies that almost every sloam library uses.
set(SLOAM_AMENT_DEPS
  rclcpp
  rclcpp_components
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sloam_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  message_filters
  ament_index_cpp
)

# ============================================================================
# Libraries (using cc_library helper from cmake/CMakeHelpers.cmake)
# ============================================================================

cc_library(
  NAME log
  DEPS fmt::fmt-header-only
  INTERFACE
)

cc_library(
  NAME base
  DEPS ${Boost_LIBRARIES} ${OpenCV_LIBS}
  INCS include/helpers ${Boost_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  INTERFACE
)
ament_target_dependencies(${CC_TARGET_PREFIX}_base INTERFACE ${SLOAM_AMENT_DEPS})

cc_library(
  NAME objects
  DEPS sloam_base gtsam
  INCS include/objects
  SRCS "src/objects/plane.cpp" "src/objects/cylinder.cpp" "src/objects/cube.cpp" "src/objects/ellipsoid.cpp"
)
ament_target_dependencies(${CC_TARGET_PREFIX}_objects PUBLIC ${SLOAM_AMENT_DEPS})

cc_library(
  NAME viz
  DEPS sloam_log sloam_base sloam_objects yaml-cpp
  INCS include/viz
  SRCS "src/viz/vizTools.cpp"
)
ament_target_dependencies(${CC_TARGET_PREFIX}_viz PUBLIC ${SLOAM_AMENT_DEPS})

cc_library(
  NAME map
  INCS include/core include/objects
  SRCS "src/core/cylinderMapManager.cpp" "src/core/cubeMapManager.cpp" "src/core/ellipsoidMapManager.cpp"
  DEPS sloam_log sloam_base sloam_objects
)
ament_target_dependencies(${CC_TARGET_PREFIX}_map PUBLIC ${SLOAM_AMENT_DEPS})

cc_library(
  NAME factorgraph
  DEPS Eigen3::Eigen gtsam ${Boost_LIBRARIES} sloam_base sloam_objects sloam_map
  INCS ${GTSAM_LIBRARIES} ${Boost_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} include/factorgraph include/core
  SRCS "src/factorgraph/cylinderFactor.cpp" "src/factorgraph/cubeFactor.cpp" "src/factorgraph/graph.cpp" "src/factorgraph/graphWrapper.cpp"
)
ament_target_dependencies(${CC_TARGET_PREFIX}_factorgraph PUBLIC ${SLOAM_AMENT_DEPS})

cc_library(
  NAME core
  INCS include/core ${CLIPPER_DIR}/include
  SRCS "src/core/sloam.cpp" "src/core/robot.cpp" "src/core/databaseManager.cpp" "src/core/place_recognition.cpp" "src/core/sloamNode.cpp"
  DEPS sloam_log sloam_base sloam_objects sloam_viz sloam_map sloam_factorgraph Eigen3::Eigen gtsam
  DEFINES USE_CLIPPER=${CLIPPER_VALUE}
)
ament_target_dependencies(${CC_TARGET_PREFIX}_core PUBLIC ${SLOAM_AMENT_DEPS})

if(USE_CLIPPER)
  target_link_libraries(${CC_TARGET_PREFIX}_core PRIVATE semantic_clipper)
endif()

# ============================================================================
# Executable: sloam node
# ============================================================================

add_executable(sloam_node
  src/core/inputNode.cpp
)
target_link_libraries(sloam_node
  ${CC_TARGET_PREFIX}_log
  ${CC_TARGET_PREFIX}_base
  ${CC_TARGET_PREFIX}_objects
  ${CC_TARGET_PREFIX}_core
  ${CC_TARGET_PREFIX}_viz
  ${CC_TARGET_PREFIX}_factorgraph
  gtsam
)
ament_target_dependencies(sloam_node ${SLOAM_AMENT_DEPS})

# ============================================================================
# rclcpp_components: SLOAM node component (replaces ROS1 nodelet)
# ============================================================================

add_library(sloam_component SHARED
  src/core/sloamNodelet.cpp
)
target_link_libraries(sloam_component
  ${CC_TARGET_PREFIX}_log
  ${CC_TARGET_PREFIX}_base
  ${CC_TARGET_PREFIX}_objects
  ${CC_TARGET_PREFIX}_core
  ${CC_TARGET_PREFIX}_viz
  ${CC_TARGET_PREFIX}_factorgraph
  gtsam
)
ament_target_dependencies(sloam_component ${SLOAM_AMENT_DEPS})
rclcpp_components_register_nodes(sloam_component "sloam::SLOAMNodelet")

# ============================================================================
# Executable: place recognition test
# ============================================================================

add_executable(sloam_place_recognition_test
  src/tests/place_recognition_test.cpp
)
target_link_libraries(sloam_place_recognition_test
  ${CC_TARGET_PREFIX}_base
  ${CC_TARGET_PREFIX}_core
)
ament_target_dependencies(sloam_place_recognition_test ${SLOAM_AMENT_DEPS})

# ============================================================================
# Install
# ============================================================================

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
    ${CC_TARGET_PREFIX}_log
    ${CC_TARGET_PREFIX}_base
    ${CC_TARGET_PREFIX}_objects
    ${CC_TARGET_PREFIX}_viz
    ${CC_TARGET_PREFIX}_map
    ${CC_TARGET_PREFIX}_factorgraph
    ${CC_TARGET_PREFIX}_core
    sloam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
    sloam_node
    sloam_place_recognition_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/params")
  install(DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
