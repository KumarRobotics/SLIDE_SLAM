# `tests/python/` — pytest static-check suite

Python re-implementation of `tests/static/check_ros2_port.sh`. Encodes the
same regression checks as parameterised pytest tests so they can run in CI on
any machine that has Python 3.10+ installed. **No ROS, rclpy, or rclcpp
imports** — stdlib only.

## Why two layers?

* The bash layer (`tests/static/`) runs on minimal images, in containers, on
  developer laptops without Python — perfect for a quick local sanity check
  and as the canonical source of truth for what's forbidden.
* The pytest layer (this directory) gives one test ID per check × per file,
  which CI dashboards (GitHub Actions, GitLab CI, Jenkins) render nicely as
  individual rows. It also makes failures easier to triage with `pytest -k`.

## Setup

```bash
pip install pytest
```

That's it. No other dependencies.

## Run

```bash
pytest tests/python -v                                  # all checks
pytest tests/python -v -k launch                        # only launch checks
pytest tests/python/test_no_ros1_leftovers.py -v        # only ROS1 leftovers
pytest tests/python -v --tb=short                       # short tracebacks
```

Test IDs follow the pattern `test_file.py::test_name[parameter_id]`. For
example:

```
tests/python/test_no_ros1_leftovers.py::test_cpp_no_ros1_idioms[ros::NodeHandle]
tests/python/test_package_xml.py::test_package_xml_format_is_3[backend/sloam/package.xml]
tests/python/test_launch_files.py::test_launch_py_imports_launchdescription[backend/sloam/launch/sloam.launch.py]
```

## Files

| File | Mirrors bash section | Notes |
|------|----------------------|-------|
| `conftest.py`                       | (fixtures) | repo_root, exempt_paths, active_cpp_files, active_py_files, comment-stripping helpers |
| `test_no_ros1_leftovers.py`         | A, B       | one parameterised test per forbidden pattern |
| `test_package_xml.py`               | C          | one parameterised test per package.xml × per check |
| `test_cmakelists_consistency.py`    | D, H       | catkin leftovers, ament_package, install(PROGRAMS) |
| `test_launch_files.py`              | E, I, J    | XML leftovers, .launch.py structure, nodelet_plugins.xml |
| `test_sloam_msgs_consistency.py`    | F, G       | camelCase field accessors + #include / import resolution |

## What if `pip install pytest` is not available?

Use the bash layer instead — it has zero Python dependencies:

```bash
bash tests/static/check_ros2_port.sh
```

Both layers should produce equivalent PASS/FAIL counts on a clean repo.
