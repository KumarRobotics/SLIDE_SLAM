# SlideSLAM ROS2 Migration & Testing Status

This document is the detailed companion to the experimental banner at the
top of the `ros2_dev` [README](README.md). It describes exactly what was
migrated from ROS1 Noetic to ROS2 Jazzy Jalisco, the mechanical rules
applied to each class of change, which checks the branch currently
enforces, which runtime behaviors have **not** been verified, and the
known follow-up items still waiting for a real machine with a full ROS2
Jazzy + GTSAM + PCL build.

It is intended for anyone who:

- wants to evaluate whether `ros2_dev` is ready to build on their setup,
- wants to understand *how* the port was done before they trust it,
- wants a punch list of things a future contributor should investigate,
- or is landing on the branch after a CI failure and wants to know
  which checks are enforced and why.

- **Source branch:** [`ros2_dev`](https://github.com/KumarRobotics/SLIDE_SLAM/tree/ros2_dev)
- **Base branch:** [`master`](https://github.com/KumarRobotics/SLIDE_SLAM/tree/master) (ROS1 Noetic, Ubuntu 20.04)
- **Target platform:** ROS2 Jazzy Jalisco on Ubuntu 24.04
- **Static test suite state:** `51 checks, 51 passed, 0 failed, 0 skipped`
  (see [§5](#5-what-the-static-suite-enforces))

---

## Table of contents

1. [Migration scope](#1-migration-scope)
2. [Per-package changes](#2-per-package-changes)
3. [Cross-cutting rules applied](#3-cross-cutting-rules-applied)
4. [Infrastructure changes](#4-infrastructure-changes)
5. [What the static suite enforces](#5-what-the-static-suite-enforces)
6. [What has NOT been verified](#6-what-has-not-been-verified)
7. [Known follow-ups and pre-existing caveats](#7-known-follow-ups-and-pre-existing-caveats)
8. [Commit history on `ros2_dev`](#8-commit-history-on-ros2_dev)

---

## 1. Migration scope

Five ROS packages were ported from catkin + ROS1 to ament + ROS2:

| Package | Type | Contents |
|---|---|---|
| `backend/sloam_msgs` | Interface | 28 `.msg`, 2 `.srv`, 2 `.action` definitions |
| `backend/sloam` | C++ core | Metric-semantic SLAM backend (graph, place recognition, loop closure); was a nodelet, now a composable `rclcpp::Node` |
| `backend/multi_robot_utils_launch` | Launch + scripts | 15 launch files + 12 tmux orchestration scripts for multi-robot experiments |
| `frontend/object_modeller` | C++ + Python | Semantic object modeling (cylinders, ellipsoids, cuboids) — 2 C++ files + 7 Python nodes + 4 launch files |
| `frontend/scan2shape` | Python-heavy | Perception pipeline (point cloud processing, YOLOv8, RangeNet++, place recognition) — 12 Python nodes + 22 launch files + 9 yaml param files |

Plus one vendored third-party library, `backend/sloam/clipper_semantic_object/`,
where only the top-level `CMakeLists.txt` was touched (catkin dependency
stripped, `ROS_*_STREAM` macros shimmed to no-ops) so the library still
builds as a plain CMake subdirectory. No clipper `.cpp`/`.h` sources were
modified.

Repo-level additions:

- New `tools/convert_ros1_bags.sh` — a rosbags-convert wrapper for the
  existing demo bags (all of which were recorded under ROS1 Noetic and
  must be converted before `ros2 bag play` can replay them).
- New `tests/` directory — see [§5](#5-what-the-static-suite-enforces).
- Updated top-level `README.md` and `run_slide_slam_docker.sh` for the
  Jazzy + colcon + Ubuntu 24.04 workflow.

---

## 2. Per-package changes

### `backend/sloam_msgs`

- `package.xml`: format 2 + catkin + `message_generation`/`message_runtime`
  → format 3 + `ament_cmake` + `rosidl_default_generators` /
  `rosidl_default_runtime`, `<member_of_group>rosidl_interface_packages</member_of_group>`.
- `CMakeLists.txt`: rewritten around `rosidl_generate_interfaces(...)`.
- Interface filename renames (ROS2 requires UpperCamelCase):
  `cubeMap` → `CubeMap`, `cylinderMap` → `CylinderMap`,
  `interRobotTF` → `InterRobotTf`, `keyPoses` → `KeyPoses`,
  `poseObjectEdges` → `PoseObjectEdges`, `syncPcOdom` → `SyncPcOdom`,
  `vector4d` → `Vector4d`, `vector7d` → `Vector7d`,
  `graphTansmission.srv` → `GraphTansmission.srv` (typo preserved
  intentionally to avoid silently renaming the service contract).
- Field renames (camelCase → snake_case; 25 fields):
  - `CubeMap.robot_id`, `CylinderMap.robot_id`, `KeyPoses.robot_id`,
    `MultiArrayPoseObjectEdges.{robot_id, object_type, multi_array_edges}`,
    `PoseMstBundle.{robot_id, pose_mst_pair, map_of_label_xyz, inter_robot_tfs}`,
    `InterRobotTf.{host_robot_id, target_robot_id, tf_from_target_to_host}`,
    `PoseMst.relative_raw_odom`,
    `ROSObservation.{initial_guess, tree_models}`,
    `ROSScan.{tree_models, tree_features, ground_features}`,
    `ROSSubMap.{tree_models, tree_features, ground_features}`,
    `Vector4d.label_xyz`, `Vector7d.label_xyz`,
    `ActiveLoopClosure (result).relative_motion`.
- `Header` field type prefixed to `std_msgs/Header` (bare `Header` is
  rejected by `rosidl` in ROS2).
- `duration` keyword in `.action` feedback replaced with
  `builtin_interfaces/Duration` (`rosidl` does not accept bare `duration`).
- `srv/GraphTansmission.srv` was an empty file in master; populated with
  an empty request / empty response separator so `rosidl` can parse it.
- Any downstream code accessing these fields was rewritten to use the
  new names in the same commit that ported the respective package.

### `backend/sloam`

- `package.xml`: catkin → `ament_cmake`; depends rewritten to the ROS2
  set (`rclcpp`, `rclcpp_components`, `rclcpp_action`, `sloam_msgs`,
  `tf2_{ros,eigen,geometry_msgs}`, `pcl_{ros,conversions}`, `cv_bridge`,
  `image_transport`, `message_filters`, `ament_index_cpp`, …).
- `CMakeLists.txt`: rewritten for ament. `cmake/CMakeHelpers.cmake`
  (`cc_library` helper) was kept verbatim — it was already catkin-free
  — and each `cc_library` call is followed by `ament_target_dependencies(...)`
  to wire up the ROS2 exports. Libraries: `sloam_log` (interface),
  `sloam_base` (interface), `sloam_objects`, `sloam_viz`, `sloam_map`,
  `sloam_factorgraph`, `sloam_core`, `sloam_component` (shared component
  library). Executables: `sloam_node` (entry in `src/core/inputNode.cpp`)
  and `sloam_place_recognition_test` (from `src/tests/place_recognition_test.cpp`).
- **Nodelet → component**: `sloam::SLOAMNodelet` is now a `rclcpp::Node`
  subclass registered via `RCLCPP_COMPONENTS_REGISTER_NODE(sloam::SLOAMNodelet)`
  in `src/core/sloamNodelet.cpp`. The old `nodelet_plugins.xml` has been
  deleted. `sloam_component` can be loaded into a
  `ComposableNodeContainer` or run standalone via
  `ros2 component standalone sloam sloam::SLOAMNodelet`.
- **Node ownership model**: only `InputManager` (the executable's main
  class) and `SLOAMNodelet` (the component) inherit from `rclcpp::Node`.
  `SLOAMNode`, `Robot`, `databaseManager`, and `PlaceRecognition` store a
  non-owning `rclcpp::Node *` pointer set from whichever top-level Node
  is constructed, avoiding the need to instantiate multiple Nodes per
  subsystem.
- **Parameter subsystem** (commit `f97c1c1`): the ROS1 code computed
  `std::string node_name = node_->get_name();` and declared parameters
  as `node_name + "/hostRobotID"`, which is illegal in ROS2 (`/` is not
  allowed in parameter names — rclcpp throws
  `InvalidParametersException`). Every such call in `databaseManager.cpp`,
  `robot.cpp`, `inputNode.cpp`, and `sloamNode.cpp` was fixed by
  dropping the prefix (11 calls). In `place_recognition.cpp`, 17
  parameters under `place_recognition/` and `place_recognition_slidegraph/`
  sub-namespaces were renamed from slash-separated to dot-separated
  (`place_recognition.dilation_factor`, `place_recognition_slidegraph.num_inliners_threshold`,
  etc.) — dots are the idiomatic ROS2 sub-namespace separator.
- **YAML restructure**: `params/sloam.yaml` and `params/sloam-forest-parking-lot.yaml`
  were ROS1 flat-dict format (`sloam:` / `place_recognition:` /
  `place_recognition_slidegraph:` nested blocks). Both were rewritten
  under the `/**: ros__parameters:` node-agnostic wildcard envelope,
  with the nested blocks flattened to dot-namespaced keys so the C++
  readers match. Two ints that the C++ reads as `double`
  (`min_robot_altitude`, `match_yaw_half_range_intra`) were promoted
  to float literals so rclcpp's parameter type-checker accepts them.
  The dead `enable_rviz: true` key (not declared by any sloam node —
  it was a ROS1 global roslaunch arg) was removed; every sloam launch
  file uses a local `DeclareLaunchArgument('enable_rviz', ...)` +
  `IfCondition` for the rviz toggle.
- **C++ API migration**: every `ros::NodeHandle`, `ros::Publisher`,
  `ros::Subscriber`, `ros::Time`, `ros::Duration`, `ros::Rate`,
  `ros::init`/`spin`/`ok`, `ROS_INFO`/`WARN`/`ERROR`/`DEBUG`/`FATAL`,
  `nodelet::Nodelet`, `PLUGINLIB_EXPORT_CLASS`, `actionlib::`, and
  old-style message include (`<pkg/Type.h>`) was converted to its
  rclcpp equivalent. `tf/`→ `tf2_ros/`, `tf2/*.h` → `tf2/*.hpp` in the
  Jazzy-renamed headers (commit `01a80cc` swept up the handful the
  initial port missed).
- **Latched publishers** → `rclcpp::QoS(1).transient_local()` on
  `pubMapTreeModel_`, `pubMapCubeModel_`, `pubAllPointLandmarks_`,
  `pubRobotTrajectory_[i]`, etc., preserving the original ROS1
  `latch=true` semantics.
- `vizTools.cpp`: `ros::package::getPath("object_modeller")` →
  `ament_index_cpp::get_package_share_directory("object_modeller")`.
- `KeyPoseTimeStamps` CSV serialization: `<< ros::Time` →
  `<< .nanoseconds()` (int64).
- **Bug fix discovered during port**: `Robot::pubRobotHighFreqSyncOdom_`
  was declared as `nav_msgs::Odometry` in ROS1 but always published
  `sloam_msgs::ROSSyncOdom` to it. The typed declaration was corrected
  to `rclcpp::Publisher<sloam_msgs::msg::ROSSyncOdom>` so the publish
  call in `inputNode.cpp` compiles (ROS2 publishers are strictly typed).
- **22 `.launch` XML files** converted to `.launch.py`. `rosbag play`
  references become `ExecuteProcess(['ros2', 'bag', 'play', ...])`.
- **Dead test files** (`src/tests/core_test.cpp`, `plane_test.cpp`,
  `loop_closure_test.cpp`) were ported off `ros::console` to use
  `rcutils_logging_set_logger_level` so static grep sweeps don't
  flag them. They remain out of the build (only
  `place_recognition_test.cpp` is built).
- **`actionlib` use**: the old `inputNode.cpp` `#include`d actionlib
  headers but never actually instantiated any server or client. The
  includes were dropped; no `rclcpp_action::Server` was needed.
- `clipper_semantic_object/CMakeLists.txt`: stripped
  `find_package(catkin REQUIRED COMPONENTS roscpp)` and the
  `${catkin_INCLUDE_DIRS}` / `${catkin_LIBRARIES}` references. The
  two `ROS_*_STREAM` calls inside the vendored source are shimmed to
  no-ops via `target_compile_definitions` so the file still compiles.

### `backend/multi_robot_utils_launch`

- Pure launch + script package. `package.xml`: format 3 + `ament_cmake`.
- `CMakeLists.txt`: minimal, installs `launch/` and `script/` to
  `share/${PROJECT_NAME}`.
- **15 XML `.launch` files** converted to `.launch.py`. Topic-list-composing
  launches (`record_bag_multi_robot`, `record_bag_vems_slam_ground_station`,
  `relay_topics_*`) use `OpaqueFunction` to resolve `LaunchConfiguration`
  values at launch time and splice them into positional command
  arguments (`ros2 bag record --compression-mode file --compression-format zstd ...`),
  which `LaunchConfiguration` substitutions alone can't do cleanly.
- **12 tmux orchestration scripts** rewritten: `roslaunch` → `ros2 launch`,
  `rosbag {play,record}` → `ros2 bag {play,record}`, `rosrun` → `ros2 run`,
  `rostopic/rosnode/rosservice/rosparam` → their `ros2 *` equivalents,
  `roscd X` → `cd $(ros2 pkg prefix --share X)`, `devel/setup` →
  `install/setup`, `/opt/ros/noetic` → `/opt/ros/jazzy`, `catkin build` /
  `catkin_make` → `colcon build --symlink-install`. `roscore` invocations
  became no-op echos (ROS2 has no master). ROS1-only `rosbag -b512`
  buffer flag was dropped; LZ4 compression translated to zstd.

### `frontend/object_modeller`

- `package.xml` + `CMakeLists.txt`: rewritten for `ament_cmake` +
  `ament_cmake_python`. Builds one C++ executable
  (`object_modeller_cylinder` from `src/cylinder_modeller.cpp`) and
  installs seven Python nodes via `install(PROGRAMS ...)`:
  `cylinder_plane_modeller.py`, `merge_synced_measurements.py`,
  `sync_centroid_odom.py`, `sync_cuboid_odom.py`, `sync_cylinder_odom.py`,
  `detect.py`, `detect_open_vocab.py`.
- **2 C++ files** (`include/object_modeller/cylinder_modeller.h`,
  `src/cylinder_modeller.cpp`) ported to rclcpp (class inherits from
  `rclcpp::Node`; `ros::NodeHandle` removed; publishers / subscribers
  become `SharedPtr`s; `ROS_*` → `RCLCPP_*`).
- **7 Python nodes** ported from `rospy` to `rclpy`: node class pattern
  with `main(args=None)`, `create_publisher` / `create_subscription`,
  `declare_parameter` + `get_parameter`, `sensor_msgs_py.point_cloud2`,
  `ament_index_python.packages.get_package_share_directory`,
  `message_filters.Subscriber(node, T, topic)` (new ROS2 API signature).
- **Bug fix discovered during cleanup**: `cylinder_plane_modeller.py`
  had `super().__init__("cylidner_plane_modeller")` (typo'd node name)
  matching the yaml top-level key `cylidner_plane_modeller:`. The
  launch file overrode the node name to the correct spelling, breaking
  yaml parameter loading at runtime. Fixed by correcting the source
  typo *and* moving the yaml key to the `/**:` wildcard so future
  renames don't re-break it.
- **4 `.launch` files** converted to `.launch.py`.
- **4 tmux shell scripts** under `script/` ported (same substitution
  set as the backend tmux scripts).
- `config/cylinder_plane_modeller_params.yaml`: restructured into the
  ROS2 `<node_name>/ros__parameters:` two-level envelope.
- `ament_python_install_package(${PROJECT_NAME})` was initially present
  but the corresponding `object_modeller/object_modeller/` directory
  does not exist; the call was removed (commit `edbf07f`) so
  `colcon build` no longer fails at the configure step. The Python
  nodes are standalone scripts, not an importable package.

### `frontend/scan2shape`

- `scan2shape_launch` is the ament package; the Python nodes live
  one directory up at `frontend/scan2shape/script/` and are installed
  via `install(PROGRAMS ../script/foo.py DESTINATION lib/${PROJECT_NAME})`.
  Support modules (`script/backbone/`, `script/decoder/`, etc.) are
  installed via `install(DIRECTORY ...)` so they remain importable at
  runtime.
- **4 ROS node Python files** rewritten to inherit from
  `rclpy.node.Node` with a `main(args=None)` entry point:
  `infer_node.py`, `process_cloud_node.py`, `process_cloud_node_outdoor.py`,
  `process_cloud_node_lidar_indoor.py`.
- **8 support modules** (`utils.py`, `utils_outdoor.py`,
  `cuboid_utils_{indoor,outdoor}.py`, `object_tracker.py`,
  `object_tracker_utils.py`, `segmentator.py`) cleaned up:
  `rospy` → `rclpy`, `tf` → `tf2_ros`, `rospkg` → `ament_index_python`,
  `imp.load_source` → `importlib.util` (Python 3.12 compat).
- **`ros_numpy` replacement**: not available in Jazzy. Each
  `process_cloud_node*.py` got a local `_pointcloud2_to_structured(msg)`
  helper that wraps `sensor_msgs_py.point_cloud2.read_points` and
  produces the same structured numpy array layout the old
  `ros_numpy.numpify` returned.
- **tf2 migration**: every node that had a `tf.TransformListener` /
  `tf.TransformBroadcaster` now creates
  `self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)`
  and `self.odom_broadcaster = TransformBroadcaster(self)`. `sendTransform`
  callers use a local `_make_transform_stamped(...)` helper that packs
  a `geometry_msgs/TransformStamped` (the old ROS1 API took
  `translation, rotation, stamp, child, parent` as separate args).
- **22 XML `.launch` files** under `scan2shape_launch/launch/` converted
  to `.launch.py`. `rosbag` calls become `ExecuteProcess` invocations.
- **9 config yaml files** restructured into the
  `/**: ros__parameters:` wildcard format. The class-info yamls
  (`process_cloud_node_indoor_cls_info.yaml`, etc.) are loaded
  directly via Python `yaml.load()` rather than via the parameter
  server and were deliberately left as plain dicts.
- **2 stub scripts** added later to unblock launches that referenced
  files that were never checked in on master:
  `pub_loop_closure_retrigger.py` (`closure_retrigger.launch.py` target)
  and `add_noise_to_ground_truth_odom.py` (`sim_perturb_odom.launch.py`
  target). Both log a prominent warning at startup noting they are
  stubs and not the full original ROS1 implementation.

### `tools/convert_ros1_bags.sh` (new)

A wrapper around [`rosbags-convert`](https://gitlab.com/ternaris/rosbags)
(installable via `pip install rosbags` with no ROS1 runtime). Accepts a
single `.bag` file or a directory of `.bag` files; writes ROS2 bag
directories next to the originals by default; skips existing outputs.
Required because every published SlideSLAM demo / benchmark bag is in
ROS1 format and must be converted before `ros2 bag play` can replay it.

---

## 3. Cross-cutting rules applied

The mechanical substitutions below were applied to every file that matched.
The static suite (see [§5](#5-what-the-static-suite-enforces)) enforces
them as regression gates.

### C++

| ROS1 | ROS2 Jazzy |
|---|---|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <sensor_msgs/PointCloud2.h>` | `#include <sensor_msgs/msg/point_cloud2.hpp>` |
| `#include <geometry_msgs/PoseStamped.h>` | `#include <geometry_msgs/msg/pose_stamped.hpp>` |
| `#include <std_msgs/Header.h>` | `#include <std_msgs/msg/header.hpp>` |
| `#include <visualization_msgs/MarkerArray.h>` | `#include <visualization_msgs/msg/marker_array.hpp>` |
| `#include <nav_msgs/Odometry.h>` | `#include <nav_msgs/msg/odometry.hpp>` |
| `#include <sloam_msgs/ROSCylinder.h>` | `#include <sloam_msgs/msg/ros_cylinder.hpp>` |
| `#include <tf/transform_listener.h>` | `#include <tf2_ros/transform_listener.h>` + `<tf2_ros/buffer.h>` |
| `#include <tf2/convert.h>` | `#include <tf2/convert.hpp>` |
| `#include <tf2_eigen/tf2_eigen.h>` | `#include <tf2_eigen/tf2_eigen.hpp>` |
| `#include <cv_bridge/cv_bridge.h>` | `#include <cv_bridge/cv_bridge.hpp>` |
| `#include <nodelet/nodelet.h>` | `#include <rclcpp_components/register_node_macro.hpp>` |
| `#include <actionlib/server/simple_action_server.h>` | `#include <rclcpp_action/rclcpp_action.hpp>` |
| `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |
| `sloam_msgs::ROSCylinder` | `sloam_msgs::msg::ROSCylinder` |
| `ros::NodeHandle nh;` | inherit class from `rclcpp::Node` |
| `nh.advertise<T>("t", 10)` | `this->create_publisher<T>("t", 10)` |
| `nh.subscribe("t", 10, &C::cb, this)` | `this->create_subscription<T>("t", 10, std::bind(&C::cb, this, std::placeholders::_1))` |
| `ros::Publisher` / `ros::Subscriber` | `rclcpp::Publisher<T>::SharedPtr` / `rclcpp::Subscription<T>::SharedPtr` |
| `const T::ConstPtr& msg` | `const typename T::ConstSharedPtr msg` |
| `ros::Time::now()` | `this->now()` (inside Node) |
| `ros::Duration(1.0)` | `rclcpp::Duration::from_seconds(1.0)` |
| `ros::Rate r(10); r.sleep();` | `rclcpp::Rate r(10); r.sleep();` (API same) |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv);` + node name in class ctor |
| `ROS_INFO(fmt, …)` | `RCLCPP_INFO(this->get_logger(), fmt, …)` (similarly `_WARN/_ERROR/_DEBUG/_FATAL`, `_STREAM` variants, `_THROTTLE` with ms) |
| `nh.getParam("x", v)` / `nh.param<T>("x", v, def)` | `this->declare_parameter("x", def); this->get_parameter("x", v);` |
| `nodelet::Nodelet` + `PLUGINLIB_EXPORT_CLASS` | `rclcpp::Node` subclass + `RCLCPP_COMPONENTS_REGISTER_NODE(ns::Class)` |
| `actionlib::SimpleActionServer<T>` | `rclcpp_action::Server<T>::SharedPtr` via `rclcpp_action::create_server<T>(...)` |
| `ros::package::getPath("pkg")` | `ament_index_cpp::get_package_share_directory("pkg")` |
| latched publisher (`latch=true`) | `rclcpp::QoS(1).transient_local()` |

### Python

| ROS1 | ROS2 Jazzy |
|---|---|
| `import rospy` | `import rclpy` + `from rclpy.node import Node` |
| `rospy.init_node("name")` | refactor into class; `super().__init__("name")`; `rclpy.init(args=args)` in `main()` |
| `rospy.Publisher("t", T, queue_size=10)` | `self.create_publisher(T, "t", 10)` |
| `rospy.Subscriber("t", T, cb)` | `self.create_subscription(T, "t", cb, 10)` |
| `rospy.spin()` | `rclpy.spin(node)` in `main(args=None)` |
| `rospy.Rate(10)` | prefer `self.create_timer(0.1, self.timer_cb)`; simple loops use `time.sleep` |
| `rospy.loginfo(x)` | `self.get_logger().info(str(x))` (similarly warn/error/debug; throttle via `throttle_duration_sec=period`) |
| `rospy.get_param("x", def)` | inside `__init__`: `self.declare_parameter("x", def); self.get_parameter("x").value` |
| `rospy.get_param("~x", def)` | drop the tilde — ROS2 has no private namespace |
| `rospy.Time.now()` | `self.get_clock().now()` |
| `rospy.is_shutdown()` | `not rclpy.ok()` |
| `tf.TransformListener()` | `from tf2_ros import Buffer, TransformListener` — stored on self |
| `tf.TransformBroadcaster()` | `from tf2_ros import TransformBroadcaster` — stored on self |
| `message_filters.Subscriber("t", T)` | `message_filters.Subscriber(self, T, "t")` |
| `from sensor_msgs import point_cloud2` | `from sensor_msgs_py import point_cloud2` |
| `rospkg.RosPack().get_path("pkg")` | `ament_index_python.packages.get_package_share_directory("pkg")` |
| `ros_numpy.numpify(pc_msg)` | local `_pointcloud2_to_structured` helper around `point_cloud2.read_points` |
| `imp.load_source(...)` (Python 3.12 removed) | `importlib.util.spec_from_file_location(...)` |

### package.xml

| ROS1 (format 2 + catkin) | ROS2 (format 3 + ament) |
|---|---|
| `<package format="2">` | `<package format="3">` |
| `<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` |
| `<build_depend>message_generation</build_depend>` | `<buildtool_depend>rosidl_default_generators</buildtool_depend>` (interface pkgs only) |
| `<exec_depend>message_runtime</exec_depend>` | `<exec_depend>rosidl_default_runtime</exec_depend>` |
| `<depend>roscpp</depend>` | `<depend>rclcpp</depend>` |
| `<depend>rospy</depend>` | `<depend>rclpy</depend>` |
| `<depend>nodelet</depend>` | `<depend>rclcpp_components</depend>` |
| `<depend>actionlib</depend>` | `<depend>rclcpp_action</depend>` |
| `<depend>actionlib_msgs</depend>` | `<depend>action_msgs</depend>` |
| `<depend>tf</depend>` | dropped; use `tf2_ros` only |
| `<export><nodelet plugin="…"/></export>` | `<export><build_type>ament_cmake</build_type></export>` |
| — | `<member_of_group>rosidl_interface_packages</member_of_group>` (interface pkgs only) |

### CMakeLists.txt

- `find_package(catkin REQUIRED COMPONENTS …)` → `find_package(ament_cmake REQUIRED)` + one `find_package(X REQUIRED)` per ROS dep.
- `catkin_package(INCLUDE_DIRS … CATKIN_DEPENDS … DEPENDS …)` → deleted.
- `${catkin_INCLUDE_DIRS}` / `${catkin_LIBRARIES}` → deleted; use `ament_target_dependencies(...)` and `target_link_libraries(...)` per-target.
- `add_message_files` / `add_service_files` / `add_action_files` / `generate_messages` → `rosidl_generate_interfaces(${PROJECT_NAME} …)`.
- Every `add_library` / `add_executable` is followed by `ament_target_dependencies(target dep1 dep2 …)`.
- Every CMakeLists ends with `ament_package()`.
- Nodelet libraries → shared libraries + `rclcpp_components_register_nodes(lib "ns::Class")`.

### Launch files

- XML `*.launch` → Python `*.launch.py` (one-to-one, 63 files total).
- `<arg name="x" default="y"/>` → `DeclareLaunchArgument('x', default_value='y')`.
- `<node pkg="P" type="T" name="N"/>` → `Node(package='P', executable='T', name='N')` (note: ROS1 `type=` becomes ROS2 `executable=`).
- `<param name="x" value="y"/>` inside a node → `parameters=[{'x': 'y'}]`.
- `<rosparam file="…"/>` → `parameters=[PathJoinSubstitution([FindPackageShare('pkg'), 'config', 'x.yaml'])]`.
- `<remap from="a" to="b"/>` → `remappings=[('a', 'b')]`.
- `<include file="…"/>` → `IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare('pkg'), '/launch/X.launch.py']))`.
- `<group ns="x">` → `GroupAction([PushRosNamespace('x'), …])`.
- `$(arg x)` → `LaunchConfiguration('x')`.
- `$(find pkg)` → `FindPackageShare('pkg')`.
- `<node pkg="rosbag" type="play|record" …>` → `ExecuteProcess(cmd=['ros2', 'bag', 'play|record', …])`.
- `<env name="K" value="V"/>` → `SetEnvironmentVariable('K', 'V')`.

### YAML parameter files

- ROS1 flat-dict format (`param_a: 1\nparam_b: 2`) → ROS2 two-level envelope (`<node_name>: ros__parameters:` or `/**: ros__parameters:`).
- Nested ROS1 sub-namespaces (`place_recognition: {x: 1, y: 2}`) → flat dot-separated keys under `ros__parameters:` (`place_recognition.x: 1`, `place_recognition.y: 2`).
- Integer literals where C++ reads `double` were promoted to float literals so rclcpp's parameter type-checker accepts them.

---

## 4. Infrastructure changes

- **`run_slide_slam_docker.sh`**: container name `slideslam_ros` →
  `slideslam_ros2`; image `xurobotics/slide-slam:latest` →
  `xurobotics/slide-slam:ros2-jazzy` (placeholder tag — not yet
  published on Docker Hub); added a top-of-file comment noting this
  is the ROS2 Jazzy workflow; appended an in-container setup block
  showing `source /opt/ros/jazzy/setup.bash` and
  `colcon build --symlink-install`.
- **Top-level `README.md`**:
  - New `[!NOTE]` banner at the top marking the branch as experimental
    ROS2 Jazzy support and pointing to `master` for paper-reproducible
    results.
  - "Use docker" section: `catkin build` →
    `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`;
    `devel/setup.bash` → `install/setup.bash`; container name updated.
  - "Build from source" section: Ubuntu 20.04 + Noetic → Ubuntu 24.04
    + Jazzy; `ros_numpy` apt dep replaced with a note that the branch
    uses a local `sensor_msgs_py.point_cloud2` helper instead; pip
    deps updated for Python 3.12 (`numpy>=1.24,<2.0`, `pypcd4`
    replacing the broken upstream `pypcd`, added `tf_transformations`
    plus `sudo apt install ros-jazzy-tf-transformations`); build/source
    commands rewritten around colcon + `install/setup.bash`.
  - Every `roslaunch <pkg> <file>.launch` in the demo / raw-sensor /
    KITTI sections rewritten as `ros2 launch <pkg> <file>.launch.py`.
  - `source devel/setup.bash && roscd multi_robot_utils_launch/script`
    replaced with `source install/setup.bash` +
    `cd ~/slideslam_ws/src/SLIDE_SLAM/backend/multi_robot_utils_launch/script`
    (source-tree path is more reliable than `ros2 pkg prefix` for
    development).
  - New "Converting ROS1 bags to ROS2" section pointing at
    `tools/convert_ros1_bags.sh`, cross-referenced from both the
    "Run our demos" and "Run on raw sensor data" section intros.
  - New "Testing the ROS2 port" section near the bottom listing every
    static and runtime check (see [§5](#5-what-the-static-suite-enforces)).
- **Identity**: commits on this branch are authored as
  `Xu Liu <liuxushawn@gmail.com>` (repo-local git config only; global
  config is left untouched).

---

## 5. What the static suite enforces

The branch ships [`tests/`](tests/) — a permanent regression suite with
three layers. Full per-layer usage notes live in
[`tests/README.md`](tests/README.md); the summary is:

### Static bash runner (`tests/static/check_ros2_port.sh`)

Pure bash + [ripgrep](https://github.com/BurntSushi/ripgrep) + `awk`.
Zero Python or ROS2 dependencies — runs on any Unix environment.
**51 checks across 14 sections (A–N)**. Current state:
`51 checks, 51 passed, 0 failed, 0 skipped`.

| Section | Checks | What it enforces |
|---|---|---|
| **A** | 21 | No ROS1 C++ idioms in active source (headers, macros, types, node lifecycle) |
| **B** | 6  | No ROS1 Python idioms (`rospy.*`, bare `import tf`, `ros_numpy`, `rospkg`) |
| **C** | 5  | Every `package.xml` is format 3 + ament + no `catkin`/`message_generation`/`message_runtime` |
| **D** | 5  | Every `CMakeLists.txt` has no catkin leftovers and calls `ament_package()` |
| **E** | 4  | Launch file structure: no XML `.launch`, every `.launch.py` has `LaunchDescription` import + `generate_launch_description` def + no literal `<launch>` tag |
| **F** | 1  | No camelCase `sloam_msgs` field accessors remain (25 rename targets) |
| **G** | 2  | Every `<sloam_msgs/…>` include and Python import resolves to a real `.msg`/`.srv`/`.action` file |
| **H** | 1  | Every `install(PROGRAMS …)` target exists on disk (handles multi-line install blocks) |
| **I** | 1  | No `nodelet_plugins.xml` remains |
| **J** | 1  | No XML `.launch` leftovers anywhere outside `tools/` and `tests/` |
| **K** | 1  | Within-file launch-argument consistency — every `LaunchConfiguration('x')` has a matching `DeclareLaunchArgument('x', …)` in the same file (slurp-whole-file awk so multi-line declarations are matched) |
| **L** | 1  | No hardcoded user-specific absolute paths (`/home/<user>/`, `/opt/slideslam_docker_ws`, `/opt/bags/`, `/root/`) in `.cpp`/`.h`/`.hpp`/`.py` |
| **M** | 1  | Every `Node(package='<local_pkg>', executable='<y>')` resolves to an `add_executable` target or `install(PROGRAMS)` entry in `<local_pkg>`'s CMakeLists (external packages skipped) |
| **N** | 1  | No raw `declare_parameter("key", …)` in 2+ source files of the same package (safe `*_declare_or_get<T>(node, …)` wrapper family explicitly exempted — it guards on `has_parameter`) |

Exempted from every section: `backend/sloam/clipper_semantic_object/`
(vendored third-party CMake library) and `frontend/scan2shape/rviz/`.

### pytest mirror (`tests/python/`)

Parallel encoding of the same checks as real pytest unit tests, for CI
ergonomics. Requires Python 3.10+ and `pytest`; no ROS.

### Launch-graph smoke test (`tests/integration/launch_smoke_test.sh`)

Runtime test requiring a working ROS2 Jazzy environment (`source
/opt/ros/jazzy/setup.bash`). For every `*.launch.py` under `backend/`
and `frontend/`, runs `ros2 launch --print-description <abs_path>` with
a per-file timeout (default 20s) and reports `OK` / `FAIL` / `TIMEOUT`
per file. Uses the direct-file-path form so the workspace need not be
built — only `ros2` on PATH. Skips with exit code 77 if `ros2` is not
installed.

---

## 6. What has NOT been verified

The following have not been verified end-to-end on `ros2_dev`:

- `colcon build --symlink-install` success on Ubuntu 24.04 + ROS2 Jazzy.
- Runtime pub/sub, QoS, and message serialization.
- SLAM correctness on converted ROS2 bags.
- TF chain correctness across the multi-robot pipeline.
- Action handshakes for `ActiveLoopClosure` / `DetectLoopClosure`.
- End-to-end demo runs (forest, parking lot, indoor RGBD, KITTI).

See [`tests/integration/README.md`](tests/integration/README.md) for the
concrete checklist of runtime tests future contributors should add.

---

## 7. Known follow-ups and pre-existing caveats

### Pre-existing on master, carried over unchanged

- Two launches reference scripts that were never checked in on master:
  `closure_retrigger.launch.py` → `pub_loop_closure_retrigger.py` and
  `sim_perturb_odom.launch.py` → `add_noise_to_ground_truth_odom.py`.
  Minimal rclpy **stubs** have been added under
  `frontend/scan2shape/script/` so the launches resolve and the
  `check_ros2_port.sh` section M passes, but the stubs log a warning
  at startup and are NOT the full original implementations.
- Three yaml files referenced by sloam launches do not exist on master
  and still do not on this branch: `sloam_sim.yaml`, `sim.yaml`,
  `sloam_active_slam_real_robot.yaml`. The affected launches
  (`run_indoor_large_scale_exploration.launch.py`,
  `segmentation.launch.py`) were ported one-to-one with NOTE comments.
- Four legacy `process_cloud_node_*.yaml` files in
  `scan2shape_launch/config/` contain ~13 orphan keys each
  (`depth_percentile_uppper` (sic), `expected_segmentation_rate`,
  `epsilon_scan`, `min_samples_scan`, etc.) that are loaded by active
  launches but silently skipped by the current `process_cloud_node.py`
  source. Either the yamls should be rewritten against the current
  source, or the legacy launches that load them should be removed.
  Not auto-fixed.
- `rgb_segmentation_open_vocab.launch.py` passes topic parameters
  (`rgb_topic`, `depth_topic`, `aligned_depth_topic`,
  `sync_odom_measurements`, `sync_pc_odom_topic`, `pc_topic`) that
  `detect_open_vocab.py` builds from the hardcoded `robot_name`
  internally — the launch-dict entries are silently ignored at
  runtime. Proper fix is a refactor of `detect_open_vocab.py` to
  parameterize topic names.

### ROS2-specific caveats

- Every published SlideSLAM demo / benchmark bag is in ROS1 `.bag`
  format. Run `tools/convert_ros1_bags.sh` on them before any demo
  will replay under `ros2 bag play`.
- The `xurobotics/slide-slam:ros2-jazzy` Docker tag is a **placeholder**
  — the image has not been published to Docker Hub. Building from
  source on a Jazzy host is required until someone builds and pushes
  the image.
- The `cmake/CMakeHelpers.cmake` module used by `backend/sloam` was
  kept verbatim (it was already catkin-free). `cc_library` targets
  carry their ROS2 metadata via explicit `ament_target_dependencies(...)`
  calls placed right after each `cc_library(...)`.
- `number_of_robots` is declared via the `*_declare_or_get<T>(...)`
  wrapper family in THREE source files (`databaseManager.cpp`,
  `inputNode.cpp`, `sloamNode.cpp`) with three different default
  values (0, 1, 1). Section N of the static suite does not flag this
  because the wrapper is guarded, but the "first-declarer wins"
  semantic is a footgun worth consolidating when someone has the
  context to pick the canonical owner.

### Disabled / dead code

- `backend/sloam/src/tests/{core_test,plane_test,loop_closure_test}.cpp`
  have had their ROS1 console calls ported to `rcutils_logging_*` but
  remain out of the build (only `place_recognition_test.cpp` is listed
  in `add_executable`). They can be added back to the build when the
  SLOAM team has the bandwidth.

---

## 8. Commit history on `ros2_dev`

All commits are authored as `Xu Liu <liuxushawn@gmail.com>` with
`Co-Authored-By: Claude Opus 4.6`. Chronological, oldest first (master
is one commit behind this list, at the branch point):

1. `a9b26ac` — docs: add ros2_dev branch experimental banner
2. `f2c86ab` — sloam_msgs: port to ROS2 Jazzy (ament_cmake + rosidl)
3. `a193351` — multi_robot_utils_launch: port to ROS2 Jazzy
4. `1f6d68f` — object_modeller: port to ROS2 Jazzy
5. `ada2984` — scan2shape: port to ROS2 Jazzy (rclpy + ament_cmake)
6. `f09c9be` — sloam: port core SLAM backend to ROS2 Jazzy
7. `bd1416a` — docs: update README and docker script for ROS2 Jazzy workflow
8. `01a80cc` — sloam: use .hpp tf2 headers for ROS2 Jazzy compatibility
9. `734c3a5` — tools: add ROS1 → ROS2 bag conversion script
10. `bebcd9f` — tests: add static/python test suite for ros2_dev
11. `f97c1c1` — sloam: fix ROS2 parameter subsystem + port dead tests + clean comments
12. `45fa132` — cleanup: parameter consistency, tmux, comments, README links, smoke-test
13. `edbf07f` — fix: drop dead enable_rviz yaml key and missing Python package call
14. `2097cd1` — fix: use correct sloam launch in `tmux_single_indoor_robot.sh` (#10)
15. `eda25a5` — sloam: fix misleading `EROOR:` prefix on first-pose warning
16. `2c937f7` — tests: add 4 new static checks (K, L, M, N)
17. `155ae24` — fix: remove hardcoded `/home/sam/` paths (section L)
18. `c6fc552` — scan2shape: add stub `pub_loop_closure_retrigger` + `add_noise_to_ground_truth_odom`
19. `2b0f8fd` — tests: fix false positives in sections K and N of `check_ros2_port.sh`
20. `a621313` — docs: describe the ros2_dev test suite in README + point to it from the banner
21. `b78614e` — docs: shorten "What has NOT been tested" section in README

Plus this report.
