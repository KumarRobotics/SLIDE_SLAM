cmake_minimum_required(VERSION 3.16)
project(sloam_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(msg_files
  "msg/CubeMap.msg"
  "msg/CylinderMap.msg"
  "msg/InterRobotTf.msg"
  "msg/KeyPoses.msg"
  "msg/LoopClosure.msg"
  "msg/MultiArrayPoseObjectEdges.msg"
  "msg/ObservationPair.msg"
  "msg/PoseMst.msg"
  "msg/PoseMstBundle.msg"
  "msg/PoseObjectEdges.msg"
  "msg/ROSCube.msg"
  "msg/ROSCylinder.msg"
  "msg/ROSCylinderArray.msg"
  "msg/ROSEllipsoid.msg"
  "msg/ROSGround.msg"
  "msg/ROSObservation.msg"
  "msg/ROSRangeBearing.msg"
  "msg/ROSRangeBearingSyncOdom.msg"
  "msg/ROSScan.msg"
  "msg/ROSSubMap.msg"
  "msg/ROSSweep.msg"
  "msg/ROSSyncOdom.msg"
  "msg/SemanticLoopClosure.msg"
  "msg/SemanticMeasSyncOdom.msg"
  "msg/StampedRvizMarkerArray.msg"
  "msg/SyncPcOdom.msg"
  "msg/Vector4d.msg"
  "msg/Vector7d.msg"
)

set(srv_files
  "srv/EvaluateLoopClosure.srv"
  "srv/GraphTansmission.srv"
)

set(action_files
  "action/ActiveLoopClosure.action"
  "action/DetectLoopClosure.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    action_msgs
    builtin_interfaces
    unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
