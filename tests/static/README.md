# `tests/static/` — bash static-check runner

A single-file bash script that runs seventeen classes of static checks
(sections A-Q) against the SlideSLAM `ros2_dev` branch. Zero Python, zero
ROS dependencies; only `bash`, `find`, `awk`, and either `ripgrep`
(preferred) or `grep -r` (fallback).

## Usage

```bash
bash tests/static/check_ros2_port.sh             # quiet, default repo root
bash tests/static/check_ros2_port.sh --verbose   # show grep hits on FAIL
bash tests/static/check_ros2_port.sh /path/to/SLIDE_SLAM
NO_COLOR=1 bash tests/static/check_ros2_port.sh  # disable ANSI colors
```

The runner:

* takes the repo root as positional arg 1 (defaults to `$(dirname $0)/../..`)
* prints one line per check: `[PASS]` / `[FAIL]` / `[SKIP]`
* counts totals at the end
* exits 0 if all checks pass, 1 otherwise

## Sections

| Section | Coverage |
|---------|----------|
| **A**   | ROS1 C++ idioms in active source (`backend/sloam/{src,include}`, `frontend/object_modeller/{src,include}`) — 21 separate patterns |
| **B**   | ROS1 Python idioms in `frontend/object_modeller/` and `frontend/scan2shape/` — 6 patterns |
| **C**   | `package.xml` format=3, ament buildtool, no message_generation — once per package.xml |
| **D**   | `CMakeLists.txt` catkin leftovers + presence of `ament_package()` — once per package CMakeLists |
| **E**   | Launch files: zero `.launch` XML; every `.launch.py` has `LaunchDescription` import + `generate_launch_description` + no literal `<launch>` tag |
| **F**   | The 16 `sloam_msgs` camelCase field accessors from the rename table |
| **G**   | Every `#include <sloam_msgs/…>` and `from sloam_msgs.* import …` resolves to a real interface file |
| **H**   | Every `install(PROGRAMS ...)` token in any CMakeLists points at a file that exists on disk |
| **I**   | No `nodelet_plugins.xml` files anywhere |
| **J**   | No `*.launch` XML files anywhere outside `tools/` or `tests/` |
| **K**   | Within each `*.launch.py`, every `LaunchConfiguration('x')` has a matching `DeclareLaunchArgument('x', ...)` in the same file |
| **L**   | No hardcoded `/home/<user>/`, `/root/`, `/opt/slideslam_docker_ws`, `/opt/bags/` paths in source |
| **M**   | Every `Node(package='<managed>', executable='<y>')` resolves to an `add_executable` target or an `install(PROGRAMS)` entry in `<managed>`'s `CMakeLists.txt` |
| **N**   | No duplicate raw `declare_parameter("key", ...)` calls across files within the same package (`declare_or_get<T>(...)` wrapper calls are ignored) |
| **O**   | Every `#include <pkg/...>` in a managed package's C/C++ sources resolves to an entry in that package's `package.xml`. Owning-package is resolved by walking up the directory tree. System libs (`Eigen`, `boost`, `pcl`, `gtsam`, `sophus`, `opencv2`, `yaml-cpp`, `fmt`, `glog`, `tbb`, `gtest`, `benchmark`, POSIX headers) are exempt because they're pulled via `find_package` + `target_link_libraries`, not `<depend>`. `backend/sloam/clipper_semantic_object/` is exempt entirely (vendored third-party `add_subdirectory()`, not a ROS package). |
| **P**   | For every `Node(package='<managed>', executable='<y>', parameters=[{'k': v, ...}])` in a `*.launch.py`, every literal dict key `k` is declared in the target package's source as `declare_parameter("k", ...)` (template or plain form), the `declare_or_get<T>(node, "k", ...)` wrapper, `get_param_or(node, "k", ...)`, or `declare_parameter_if_not_declared(node, "k", ...)`. Catches the classic ROS2 bug where a launch file passes a parameter that the target node silently ignores because it never calls `declare_parameter`. Only dict-literal `parameters=[{...}]` is analyzed; yaml-file-path parameters, `ComposableNode`s, and dynamically-built `params` variables are skipped. `scan2shape_launch` additionally walks `frontend/scan2shape/script/` because it installs scripts from that sibling directory. |
| **Q**   | Every `*.sh` / `*.bash` under `backend/`, `frontend/`, `tools/`, and `tests/` passes `bash -n`. Skipped (with a clear reason) when `bash -n` isn't usable in the runtime sandbox. |

## Carve-outs

Areas exempt from every check:

* `backend/sloam/clipper_semantic_object/` — vendored third-party library;
  catkin has been stripped from its top-level CMakeLists, but its `.cpp`/`.h`
  sources still contain `ROS_INFO_STREAM` calls that get macro-shimmed at
  build time.
* All `*.rviz` files and `**/rviz/` directories.
* The "Noetic" banner in the repo-root `README.md`.
* `tests/` and `tools/` themselves.

## False-positive guards

* C++ checks strip `//` line comments and inline `/* … */` comments before
  re-applying the pattern. Python checks strip `#` line comments. The
  comment strippers are not string-literal aware, so a forbidden token
  inside a string literal (e.g. `"ros::NodeHandle"`) would still flag —
  in practice no such cases exist in this repo.
* The camelCase field check uses `\b` boundaries so the forbidden token
  `relativeRawOdom` does NOT match the legitimate internal struct field
  `relativeRawOdomMotion` in `databaseManager.h` — `M` is a word character,
  so the boundary doesn't fire.
* The launch-file check only looks under `backend/` and `frontend/` to
  avoid flagging launch files that may live under `tools/` for tooling.
* The install(PROGRAMS) check skips any token containing `${…}` because we
  can't expand CMake variables from bash.

## Adding a new check

1. Pick the right section letter or add a new one with a `section "…"` call.
2. Use the `cpp_search` / `py_search` helpers if you need a glob-aware
   search. They handle the rg-vs-grep fallback.
3. For "no match expected" checks, prefer `check_cpp_no_match` /
   `check_py_no_match` — they handle comment stripping for you.
4. Mirror the new check in the matching `tests/python/test_*.py` file so
   the two layers stay in sync.

## Expected pass state

On a clean `ros2_dev` checkout, all sections except possibly H should be
green. Any FAIL is a real port bug — fix the source, not the check.
