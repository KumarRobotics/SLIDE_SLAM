cmake_minimum_required(VERSION 3.16)
project(object_modeller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sloam_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---- C++ executables ----

add_executable(${PROJECT_NAME}_cylinder src/cylinder_modeller.cpp)
ament_target_dependencies(${PROJECT_NAME}_cylinder
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(${PROJECT_NAME}_cylinder
  ${PCL_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_cylinder PUBLIC include ${PCL_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}_cylinder
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Python nodes ----

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/cylinder_plane_modeller.py
  script/merge_synced_measurements.py
  script/sync_centroid_odom.py
  script/sync_cuboid_odom.py
  script/sync_cylinder_odom.py
  object_detector_utils/detect.py
  object_detector_utils/detect_open_vocab.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Install headers, launch, config, models, rviz ----

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
