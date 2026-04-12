# `tests/integration/` — placeholder for runtime tests

This directory is intentionally empty. It is reserved for **runtime**
integration tests that require a working ROS2 Jazzy environment:

* `/opt/ros/jazzy/setup.bash` sourced
* a built workspace (`colcon build` succeeds)
* GTSAM, Sophus, PCL, OpenCV, cv_bridge, message_filters, image_transport,
  rosbag2, etc. installed
* (optionally) a recorded ROS2 bag of a SlideSLAM run

The static checks in `tests/static/` and `tests/python/` give us roughly 80%
of the regression-detection value at zero infrastructure cost. The remaining
20% is what should live here. None of these are implemented yet — feel free
to grab any of the items below.

## Suggested test cases

1. **`test_sloam_msgs_round_trip` (C++ gtest under ament_cmake_gtest)**
   For each of the 32 interfaces (28 msgs + 2 srvs + 2 actions), construct
   an instance, publish on a topic, subscribe to the same topic, compare the
   round-tripped value to the original. Verifies that `rosidl_generate_interfaces`
   actually emitted code for every interface and that there are no ABI gaps.

2. **`test_launch_smoke` (pytest with `launch_testing`)**
   For every `*.launch.py` in the workspace, run `ros2 launch <pkg> <file>.launch.py`
   with a 2-second timeout. The test passes if the launch description parses
   and every referenced executable resolves. A non-zero exit code from an
   unrecognized executable counts as FAIL. Use `launch_testing.markers.keep_alive`
   so the launch returns control after the timeout.

3. **`test_component_loads` (C++ gtest)**
   Load `sloam::SLOAMNodelet` (or whatever the rclcpp_components class is
   called after the port) into a `ComponentContainer`, wait for the node to
   appear in `rcl::get_node_names()`, and assert no exceptions were thrown
   during construction.

4. **`test_tf2_buffer` (C++ gtest)**
   Instantiate a `sloam::vizTools` object, feed it a synthetic TF tree via
   `tf2_ros::Buffer::setTransform`, call its tf-dependent helper methods
   (`worldToBody`, `publishMarker`, etc.), assert no exceptions and that
   the published Marker frame_ids are correct.

5. **`test_active_loop_closure_action` (Python pytest with rclpy)**
   Create an `ActionClient` for `sloam_msgs/action/ActiveLoopClosure`, send
   a goal with mock data, verify the server acknowledges (even if it
   immediately rejects — the handshake itself is what we're testing). Catches
   action-server registration regressions.

6. **`test_end_to_end_demo` (pytest, slow)**
   Build the workspace, play a converted ROS2 bag for 10 seconds, assert
   that the map publisher emits at least one message on `/sloam/map` (or
   whichever topic is canonical after the rename pass). Run this in
   nightly CI only — it's expensive.

## Suggested layout once implemented

```
tests/integration/
├── README.md                       (this file)
├── cpp/
│   ├── CMakeLists.txt              (ament_cmake_gtest target_link_libraries)
│   ├── test_sloam_msgs_round_trip.cpp
│   ├── test_component_loads.cpp
│   └── test_tf2_buffer.cpp
└── python/
    ├── conftest.py                 (rclpy fixture, executor fixture)
    ├── test_launch_smoke.py
    ├── test_active_loop_closure_action.py
    └── test_end_to_end_demo.py     (slow, marked with @pytest.mark.slow)
```

The static layer must remain dependency-free — do **not** add anything that
imports `rclpy` or links against `rclcpp` outside this directory.
