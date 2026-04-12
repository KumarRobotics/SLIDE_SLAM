cmake_minimum_required(VERSION 3.16)
project(scan2shape_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install the Python ROS nodes that live in ../script/ as executables.
# Only the four top-level scripts that contain a __main__ + rclpy.init are
# real ROS nodes; everything else under ../script/ is a support module that
# is imported from one of these nodes.
install(PROGRAMS
  ../script/infer_node.py
  ../script/process_cloud_node.py
  ../script/process_cloud_node_outdoor.py
  ../script/process_cloud_node_lidar_indoor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the support Python modules so they can be imported at runtime by
# the executables above. They live next to the nodes in ../script/.
install(PROGRAMS
  ../script/CRF.py
  ../script/assignment.py
  ../script/cuboid_utils_indoor.py
  ../script/cuboid_utils_outdoor.py
  ../script/laserscan.py
  ../script/load_model.py
  ../script/object_tracker.py
  ../script/object_tracker_utils.py
  ../script/segmentator.py
  ../script/sklearn_matching_utils.py
  ../script/utils.py
  ../script/utils_outdoor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install support module directories so they're importable at runtime.
# These are loaded via sys.path manipulation from the nodes above.
install(DIRECTORY ../script/backbone
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py" PATTERN "*.pth" PATTERN "*.yaml")

install(DIRECTORY ../script/decoder
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py" PATTERN "*.pth")

ament_package()
