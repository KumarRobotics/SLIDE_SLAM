# SlideSLAM `ros2_dev` test suite

This directory contains a permanent regression suite for the SlideSLAM
**ros2_dev** branch — the fresh port from ROS1 Noetic to ROS2 Jazzy Jalisco.
It exists so future contributors can verify the port stays clean **without
needing a working ROS2 build environment**.

The suite is organised into three layers, each progressively heavier:

```
tests/
├── static/        bash + ripgrep + awk + find       (zero deps, runs anywhere)
├── python/        pytest + stdlib only              (CI-friendly, Python 3.10+)
└── integration/   launch-graph smoke test + gtest   (requires Jazzy + optionally a built workspace)
```

The static layer is the authoritative regression gate and currently enforces
**51 checks across 14 sections (A–N)**. The Python layer mirrors the same
checks for CI ergonomics. The integration layer ships one runnable smoke test
(`launch_smoke_test.sh`) and a README describing the larger runtime tests a
future contributor should add.

**Current state:** `bash tests/static/check_ros2_port.sh` reports
`51 checks, 51 passed, 0 failed, 0 skipped`.

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

Two things live here:

1. **`launch_smoke_test.sh`** — a runnable bash script that does NOT require
   a built workspace, only `ros2` on `PATH`. For every `*.launch.py` file
   under `backend/` and `frontend/` it runs
   `ros2 launch --print-description <abs_path>` with a per-file timeout and
   reports `OK` / `FAIL` / `TIMEOUT`. Using the direct-file-path form means
   the test works even on a fresh Jazzy install with no `colcon build`
   yet. Skips with exit code 77 (autotools convention) if `ros2` is not
   installed. Usage:

   ```bash
   bash tests/integration/launch_smoke_test.sh
   VERBOSE=1  bash tests/integration/launch_smoke_test.sh
   LAUNCH_TIMEOUT=60 bash tests/integration/launch_smoke_test.sh
   ```

2. **`tests/integration/README.md`** — a placeholder describing the heavier
   gtest + rclpy tests a future contributor should add once they have a
   fully built workspace (GTSAM, Sophus, PCL, OpenCV, cv_bridge, etc.).
   The concrete checklist includes: sloam_msgs round-trip, sloam component
   load test, tf2 buffer test, action-server handshake test, and an
   end-to-end demo on a 10-second bag slice. See that file for details.

## What this suite **can** catch

Section letters refer to the corresponding block in
`tests/static/check_ros2_port.sh`.

* **A** — residual ROS1 C++ idioms (`ros::NodeHandle`, `ros::Publisher`,
  `ros::Subscriber`, `ros::Time::now()`, `ros::Duration`, `ros::Rate`,
  `ros::init`, `ros::spin`, `ros::ok`, `ROS_INFO_STREAM` and friends,
  `<tf/…>` headers, `<nodelet/…>`, `<pluginlib/class_list_macros.h>`,
  `<actionlib/…>`, old-style message includes `<pkg/Type.h>`, …)
* **B** — residual ROS1 Python idioms (`import rospy`, `rospy.*`, bare
  `import tf`, `from tf.<sub>`, `ros_numpy`, `rospkg`)
* **C** — `package.xml` files left in format-2 or with `catkin`,
  `message_generation`, or `message_runtime` leftovers; missing
  `<build_type>` in the export section
* **D** — `CMakeLists.txt` catkin leftovers (`find_package(catkin…)`,
  `catkin_package`, `${catkin_INCLUDE_DIRS}`, `add_message_files`,
  `generate_messages`, …) and missing `ament_package()`
* **E** — `.launch` XML files that should have been converted to
  `.launch.py`; `.launch.py` files missing `from launch import
  LaunchDescription` or `def generate_launch_description`; files
  containing a literal `<launch>` tag (half-converted)
* **F** — the 25 `sloam_msgs` camelCase field accessors that the rename
  pass was supposed to convert to `snake_case` (`robotID` →
  `robot_id`, `treeModels` → `tree_models`, `labelXYZ` → `label_xyz`,
  etc.). The internal C++ struct field `PoseMstPair::relativeRawOdomMotion`
  is correctly preserved via a word-boundary regex carve-out.
* **G** — `#include <sloam_msgs/msg|srv|action/*.hpp>` and `from
  sloam_msgs.msg|srv|action import …` that point at interfaces that
  don't exist on disk, using the known snake_case ↔ UpperCamelCase
  mapping.
* **H** — `install(PROGRAMS …)` blocks in CMakeLists that reference
  missing files (handles multi-line install blocks correctly).
* **I** — `nodelet_plugins.xml` files left over from ROS1.
* **J** — XML `.launch` files anywhere outside `tools/` and `tests/`.
* **K** — within-file launch-argument consistency: every
  `LaunchConfiguration('x')` in a `*.launch.py` has a matching
  `DeclareLaunchArgument('x', …)` in the same file. The awk slurps
  each launch file as a single record so multi-line
  `DeclareLaunchArgument(` … `)` forms are matched correctly. Exempts
  the ROS2-injected launch builtins (`log_level`, `launch_prefix`,
  `use_sim_time`, …).
* **L** — hardcoded user-specific absolute paths (`/home/<user>/`,
  `/opt/slideslam_docker_ws`, `/opt/bags/`, `/root/`) in `.cpp`/`.h`/
  `.hpp`/`.py` source. Strips C and Python comments before matching.
* **M** — every `Node(package='<local_pkg>', executable='<y>')` in
  every launch file resolves to either an `add_executable(<y> …)`
  target or an `install(PROGRAMS …/<y>)` entry in the target
  package's `CMakeLists.txt`. External packages (`tf2_ros`,
  `topic_tools`, `rviz2`, third-party drivers) are skipped. This is
  the check that would have caught issue #10 (a launch file pointing
  at the non-existent `single_robot_sloam_test_f250.launch.py`).
* **N** — raw `declare_parameter("key", …)` calls with the same key
  in 2+ source files of the same package. ROS2 throws
  `ParameterAlreadyDeclared` at runtime if the same parameter is
  declared twice on the same node. The safe
  `*_declare_or_get<T>(node, "key", default)` wrapper family used
  throughout `backend/sloam` is explicitly exempted — those wrappers
  guard with `has_parameter()` before declaring and are idiomatic.

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
