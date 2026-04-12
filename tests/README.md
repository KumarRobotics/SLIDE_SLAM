# SlideSLAM `ros2_dev` test suite

This directory contains a permanent regression suite for the SlideSLAM
**ros2_dev** branch — the fresh port from ROS1 Noetic to ROS2 Jazzy Jalisco.
It exists so future contributors can verify the port stays clean **without
needing a working ROS2 build environment**.

The suite is organised into three layers, each progressively heavier:

```
tests/
├── static/        bash + ripgrep + find         (zero deps, runs anywhere)
├── python/        pytest + stdlib only          (CI-friendly, Python 3.10+)
└── integration/   gtest + rclpy + a real bag    (requires Jazzy + workspace)
```

Only the static and Python layers are implemented today; the integration
layer is a placeholder describing what to add once a Jazzy build is available.

## Layer 1 — Static (`tests/static/`)

The most important layer. Everything is bash + ripgrep + POSIX userland; no
Python, no ROS, no compilers. Run it with:

```bash
bash tests/static/check_ros2_port.sh             # quiet
bash tests/static/check_ros2_port.sh --verbose   # show grep hits on FAIL
NO_COLOR=1 bash tests/static/check_ros2_port.sh  # disable ANSI colors
```

It exits 0 if everything passes, 1 otherwise. See `tests/static/README.md` for
details and the list of checks.

## Layer 2 — Python (`tests/python/`)

The same checks reimplemented as parameterised pytest tests, for use in CI on
any machine with Python 3.10+. Run with:

```bash
pip install pytest
pytest tests/python -v
```

The Python layer exposes one test ID per check × per file (e.g. one ID per
launch file, one per package.xml, one per CMakeLists), which makes failures
easier to triage in CI dashboards. See `tests/python/README.md`.

## Layer 3 — Integration (`tests/integration/`)

Placeholder. Describes the gtest + rclpy tests a future contributor should
write once they have `/opt/ros/jazzy/setup.bash` sourced and a built workspace
(GTSAM, Sophus, PCL, OpenCV, cv_bridge, etc. installed). See
`tests/integration/README.md` for the list of suggested test cases.

## What this suite **can** catch

* residual ROS1 C++ idioms (`ros::NodeHandle`, `ROS_INFO_STREAM`, `tf/`,
  `nodelet/`, `actionlib/`, `pluginlib/class_list_macros.h`, …)
* residual ROS1 Python idioms (`rospy`, bare `import tf`, `ros_numpy`,
  `rospkg`, …)
* `package.xml` left in format-2 / catkin / message_generation state
* `CMakeLists.txt` catkin leftovers (`catkin_package`, `add_message_files`, …)
  and missing `ament_package()`
* `.launch` XML files that should have been converted to `.launch.py`
* `.launch.py` files that don't actually define `generate_launch_description`
* `nodelet_plugins.xml` files left over from ROS1
* the 16 `sloam_msgs` camelCase field accessors that the rename pass was
  supposed to convert to `snake_case`
* `#include <sloam_msgs/...>` and `from sloam_msgs.* import …` that point at
  interfaces that don't exist on disk
* `install(PROGRAMS …)` blocks in CMakeLists that reference missing files

## What this suite **cannot** catch

* actual compilation or linking errors
* runtime topic / service / action behavior
* QoS, parameter, or TF correctness
* missing dependencies in `target_link_libraries` or `ament_target_dependencies`
* GTSAM / Sophus / PCL ABI mismatches
* whether SLAM produces the right map on a given bag

For those, build the workspace with `colcon build` and run the integration
tests once they exist.

## Adding a new check

* Add the regex / file walker to `tests/static/check_ros2_port.sh` AND to the
  matching `tests/python/test_*.py` file. **Keep them in sync** — the bash
  layer is the canonical source of truth, the Python layer is for CI ergonomics.
* Update the relevant section comment in `tests/static/check_ros2_port.sh`.
* Re-run both layers to confirm no regressions.
