# SlideSLAM

> [!NOTE]
> ## You are on the `ros2_dev` branch — ROS2 support for SlideSLAM.
>
> This branch ports the entire SlideSLAM stack from **ROS1 Noetic** to **ROS2 Jazzy Jalisco** (Ubuntu 24.04) — the latest ROS2 LTS. It is provided to help developers who want to use SlideSLAM with ROS2.
>
> **Note:** this ROS2 port has **not been as extensively tested or experimented with** as the ROS1 version. If you want the version used to produce the results in our paper, or the most battle-tested setup, please use the [`master`](https://github.com/KumarRobotics/SLIDE_SLAM/tree/master) branch (ROS1 Noetic, Ubuntu 20.04). See [Testing the ROS2 port](#testing-the-ros2-port) at the end of this README for the full list of static and runtime checks we **do** run on this branch, and which runtime behaviors have **not** been verified end-to-end.
>
> Issues and pull requests that improve the ROS2 port are very welcome.

This repository contains the source code for the project SlideSLAM: Sparse, Lightweight, Decentralized Metric-Semantic SLAM for Multi-Robot Navigation. 
- More details can be found on the [project website](https://xurobotics.github.io/slideslam/).
- Our paper is available on arXiv [here](https://arxiv.org/abs/2406.17249). 

# Table of contents
- [SlideSLAM](#slideslam)
- [Table of contents](#table-of-contents)
- [Use docker (recommended)](#use-docker-recommended)
- [Build from source (only if you do not want to use docker)](#build-from-source-only-if-you-do-not-want-to-use-docker)
- [Converting ROS1 bags to ROS2 (required before running demos)](#converting-ros1-bags-to-ros2-required-before-running-demos)
- [Run our demos (with processed data)](#run-our-demos-with-processed-data)
  - [Download example data](#download-example-data)
  - [What these demos will do](#what-these-demos-will-do)
  - [Run multi-robot demo (based on LiDAR data)](#run-multi-robot-demo-based-on-lidar-data)
- [Run on raw sensor data (RGBD or LiDAR bags)](#run-on-raw-sensor-data-rgbd-or-lidar-bags)
  - [Download example data](#download-example-data-1)
  - [Run our RGBD data experiments](#run-our-rgbd-data-experiments)
  - [Run our LiDAR Data experiments](#run-our-lidar-data-experiments)
- [Troubleshoot](#troubleshoot)
- [Testing the ROS2 port](#testing-the-ros2-port)
  - [Static checks (`tests/static/check_ros2_port.sh`)](#static-checks-teststaticcheck_ros2_portsh)
  - [pytest mirror (`tests/python/`)](#pytest-mirror-testspython)
  - [Launch-graph smoke test (`tests/integration/launch_smoke_test.sh`)](#launch-graph-smoke-test-testsintegrationlaunch_smoke_testsh)
  - [What has NOT been tested](#what-has-not-been-tested)
- [Acknowledgement](#acknowledgement)
- [Citation](#citation)




# Use docker (recommended)

**Install docker**: https://docs.docker.com/desktop/install/linux/ubuntu/#install-docker-desktop

**Pull the docker image**: 
```
docker pull xurobotics/slide-slam:ros2-jazzy
```
_Note: the `ros2-jazzy` tag is a placeholder and may not yet be published on Docker Hub. Build it locally or update the tag once it is available._

**Create the workspace (important)**
```
mkdir -p ~/slideslam_docker_ws/src
cd ~/slideslam_docker_ws/src
```
_Creating the workspace outside the docker helps you keep your files and changes within the workspace even if you delete the un-committed docker container._

**Clone the repo**: 
```
git clone https://github.com/XuRobotics/SLIDE_SLAM.git
cd ~/slideslam_docker_ws/src/SLIDE_SLAM
chmod +x run_slide_slam_docker.sh
```

**(Optional) Only if you need to run on LiDAR data, install Faster-LIO and LiDAR drivers**: 
```
cd ~/slideslam_docker_ws/src
git clone git@github.com:ouster-lidar/ouster_example.git && cd ouster_example && git checkout 43107a1 && cd ..
git clone git@github.com:XuRobotics/faster-lio
git clone git@github.com:KumarRobotics/ouster_decoder.git && cd ouster_decoder && git checkout d66b52d  && cd ..
```
*Find the ```CMakeLists.txt``` in ```ouster_decoder``` and comment out the last three lines (the ```ouster_viz```) to avoid fmt issue*

**Run the docker image**: 

**Important:** Go to `./run_slide_slam_docker.sh`, make sure the following three directories are correct
```
SlideSlamWs="/home/sam/slideslam_docker_ws"
```
should point to your workspace directory
```
SlideSlamCodeDir="/home/sam/slideslam_docker_ws/src/SLIDE_SLAM"
``` 
should point to your code directory where you cloned the repository
```
BAGS_DIR="/home/sam/bags"
```
should point to your bags (data) directory

Then run:
```
./run_slide_slam_docker.sh
```

**Build the workspace**: 
```
source /opt/ros/jazzy/setup.bash
cd /opt/slideslam_docker_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Run the demos**
```
source /opt/slideslam_docker_ws/install/setup.bash
```
Follow the instructions below to run the demos. Remember to commit your changes inside docker envirnoment to keep them (e.g. newly installed pkgs). 

Type `exit` to exit the container.

You can re-enter the container, or enter the container from a new terminal by either 
```
docker start slideslam_ros2 && docker exec -it slideslam_ros2 /bin/bash
``` 
or remove your docker container using the command 
```
docker rm slideslam_ros2
``` 
before you run the docker image again.

**Troubleshoot**:
- If you do not see your code or bags inside docker, double check `run_slide_slam_docker.sh` file to make sure you have your workspace and BAG folders mapped properly. 


# Build from source (only if you do not want to use docker)

**Install ROS2 Jazzy on Ubuntu 24.04**

Please refer to this [link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Jazzy Jalisco.

**Create your workspace under your preferred directory** (e.g., we name this directory as `~/slideslam_ws`):
```
cd ~
mkdir slideslam_ws
cd slideslam_ws 
mkdir src
cd src
```

**Then, pull the slideslam github repo**:
```
git clone https://github.com/XuRobotics/SLIDE_SLAM.git
```

**Install qhull 8.0.2**: 
  
*Download from [this link](http://www.qhull.org/download/qhull-2020-src-8.0.2.tgz), extract (unzip) the file, then:*
```
cd build
cmake ..
make install
```
*If ```make install``` gives a permission error then try ```sudo make install```*

**Install gtsam 4.0.3**:

```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0 
sudo apt update  
sudo apt install libgtsam-dev libgtsam-unstable-dev
sudo apt-get install libdw-dev
```

**Install Sophus**: 
```
git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && git checkout 49a7e1286910019f74fb4f0bb3e213c909f8e1b7 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make
sudo make install
```

**Install fmt 8.0.0**:
```
git clone https://github.com/fmtlib/fmt.git && \
    cd fmt && git checkout 8.0.0 && \
    mkdir build && cd build && \
    cmake .. && make  
sudo make install
```

**ros_numpy replacement**:

`ros_numpy` is not available in ROS2; the `ros2_dev` branch provides local helpers that wrap `sensor_msgs_py.point_cloud2` instead. No extra package needs to be installed.

**(Optional) Only if you need to run on LiDAR data, install Faster-LIO and LiDAR drivers**: 
```
sudo apt update
sudo apt-get install -y libgoogle-glog-dev
cd ~/slideslam_ws/src
git clone http://github.com/ouster-lidar/ouster_example.git && cd ouster_example && git checkout 43107a1 && cd ..
git clone https://github.com/XuRobotics/faster-lio.git
git clone https://github.com/KumarRobotics/ouster_decoder.git && cd ouster_decoder && git checkout d66b52d  && cd ..
```
*Find the ```CMakeLists.txt``` in ```ouster_decoder``` and comment out the last three lines (the ```ouster_viz```) to avoid fmt issue*

**(Optional) Only if you need to run on RGBD data with YOLOv8, install the following**:
```
pip install ultralytics==8.0.59
```

**Install pip dependencies**:
```
pip install "numpy>=1.24,<2.0"
pip install scikit-learn
pip install scipy
pip install open3d
pip install matplotlib
pip install pypcd4
pip install tf_transformations
```
_Note: Ubuntu 24.04 ships Python 3.12, which is incompatible with the old `numpy==1.22.3` pin used on the ROS1 branch — `numpy>=1.24,<2.0` is the supported range. The original `pypcd` from `dimatura/pypcd` is also broken on Python 3.12, so we use the maintained `pypcd4` fork instead. The Python nodes ported in this branch import `tf_transformations`, which is not part of the standard apt set._

Also install the apt-distributed `tf_transformations` package alongside the pip version (Jazzy ships its own python package via apt):
```
sudo apt install ros-jazzy-tf-transformations
```

- Install `tmux` for running our demo experiments
```
sudo apt update
sudo apt install tmux
```

**Build in release mode**
```
source /opt/ros/jazzy/setup.bash
cd ~/slideslam_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
**Source your workspace using**
```
source ~/slideslam_ws/install/setup.bash
```



**Troubleshoot:**
- If you have built GTSAM from source before, you need to remove everything related to gtsam/GTSAM in /usr/local by doing:
```
sudo rm -rf /usr/local/lib/cmake/*GTSAM*
sudo rm -rf /usr/local/include/gtsam
```
- If you have installed GTSAM using apt-get, remove them first, use this command `sudo apt remove --purge libgtsam*

# Converting ROS1 bags to ROS2 (required before running demos)

All of our published demo / benchmark bags — the processed multi-robot bags used under _Run our demos_ and the raw sensor bags used under _Run on raw sensor data_ — were recorded under **ROS1 Noetic** and are distributed as legacy `.bag` files. ROS2 Jazzy's `ros2 bag play` cannot read ROS1 bags directly, so **you must convert them once before running any of the demos** on this branch.

We provide a small wrapper script that drives the standalone [`rosbags-convert`](https://gitlab.com/ternaris/rosbags) tool:

```
# one-time install (no ROS1 required — the `rosbags` package is self-contained):
pip install rosbags

# convert a single .bag file:
./tools/convert_ros1_bags.sh /path/to/forest_robot1.bag

# or convert every .bag in a directory at once:
./tools/convert_ros1_bags.sh /path/to/bags/
```

By default the converted ROS2 bag is written next to the original as a directory (containing `metadata.yaml` + a `.db3` sqlite3 file) with the same base name. Existing output directories are skipped, so the script is safe to re-run. See `tools/convert_ros1_bags.sh` for full options.

After conversion, point `BAG_DIR` in the tmux scripts (e.g. `tmux_multi_robot_with_bags_forest.sh`) at the directory containing the **converted** ROS2 bags, not the original `.bag` files.

# Run our demos (with processed data)
Note: if the access to any of the links is lost, please contact the authors, and we will provide the data from our lab's NAS.

This section will guide you through running our demos with processed data. We provide processed data as legacy ROS1 `.bag` files that contain only the odometry and semantic measurements (i.e. object observations); **you must convert them to ROS2 bag format first** using `tools/convert_ros1_bags.sh` (see the _Converting ROS1 bags to ROS2_ section above). Running the entire pipeline containing object detection and the rest of SLAM for multiple robots simultaneously onboard one computer is computationally and memory intensive. 

**Note:** Such tests can to a large degree replicate what would happen onboard the robot since when you run real world multi-robot experiment, each robot will only be responsible for processing its own data, and the processed data shared by the other robots in the form provided by here. 

## Download example data

Please download the processed data bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). This containes compact processed bags for forest and urban outdoor environments. Please use the right data with the right scripts as specified below.


## What these demos will do
- Intermittent communication between robot nodes at a fixed time interval. 
- Multiple robots running on the same computer, and therefore, the computational load is going to be (num_robots multiplied by the computation load of each robot during actual experiment).

## Run multi-robot demo (based on LiDAR data)
**First, please refer to the section above and make sure you have everything built.**

**Option 1:** Use our tmux script (recommended)

Source your workspace and go to the `script` folder inside the `multi_robot_utils_launch` package (using the source-tree path is more reliable than `ros2 pkg prefix`):
```
source ~/slideslam_ws/install/setup.bash
cd ~/slideslam_ws/src/SLIDE_SLAM/backend/multi_robot_utils_launch/script
```

Modify `tmux_multi_robot_with_bags_forest.sh` to set the `BAG_DIR` to where you downloaded the bags

Modify `BAG_PLAY_RATE` to your desired play rate (lower than 1.0 if you have a low-specification CPU)

Then make it executable if needed
```
chmod +x tmux_multi_robot_with_bags_forest.sh
```

Finally, execute this script
```
./tmux_multi_robot_with_bags_forest.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `ros2 launch` commands inside this tmux script and execute those commands by yourself.

**To run the same above example with urban outdoor data, use the `tmux_multi_robot_with_bags_parking_lot.sh` script and repeat the above steps.**

# Run on raw sensor data (RGBD or LiDAR bags)
This section will guide you through running our code stack with raw sensor data. Our distributed raw bags are legacy ROS1 `.bag` files (10-100 GB each) — **run `tools/convert_ros1_bags.sh` on them first** (see the _Converting ROS1 bags to ROS2_ section above) so that `ros2 bag play` can replay them under ROS2 Jazzy.

## Download example data

Please download the LiDAR demo bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `outdoor` folder.

Please download the RGBD demo bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `indoor` folder.

Please download the KITTI benchmark processed bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `kitti_bags` folder.

Please download our trained RangeNet++ model from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is currently named `penn_smallest.zip`. Follow the instructions in the `Run our LiDAR data experiments`  section below on how to use this model.

## Run our RGBD data experiments

**Option 1:** Use our tmux script (recommended)

Source your workspace and go to the `script` folder inside the `multi_robot_utils_launch` package (using the source-tree path is more reliable than `ros2 pkg prefix`):
```
source ~/slideslam_ws/install/setup.bash
cd ~/slideslam_ws/src/SLIDE_SLAM/backend/multi_robot_utils_launch/script
```

Modify `tmux_single_indoor_robot.sh` to set the `BAG_DIR` to where you downloaded the bags

Modify `BAG_PLAY_RATE` to your desired play rate (lower than 1.0 if you have a low-specification CPU)

Then make it executable if needed
```
chmod +x tmux_single_indoor_robot.sh
```

Finally, if you want to use Yolo-v8, execute this script
```
./tmux_single_indoor_robot.sh
```


**IMPORTANT**: If it is your first time to run this script, the front-end instance segmentation network will download the weights from the internet. This may take a while depending on your internet speed. Once this is finished, kill all the tmux sessions (see below) and re-run the script.

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `ros2 launch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-rgbd-raw-bags-detailed-instructions).

## Run our LiDAR Data experiments

**Download the LiDAR semantic segmentation RangeNet++ model**
```
(1) Download the model from the above link.
(2) Unzip the file and place the model in a location of your choice.
(3) Open the extracted model folder and make sure that there are no files inside having a .zip extension. If there are, then rename ALL OF THEM to remove the .zip extension. For example backbone.zip should be renamed to backbone
```

**Option 1:** Use our tmux script (recommended)

Make sure you edit the ```infer_node_params.yaml``` file present inside the ```scan2shape_launch/config``` folder and set the value of ```model_dir``` param to point to the path to the RangeNet++ model you downloaded in the previous step. Make sure to compelte the path with the ```/``` at the end.

Source your workspace and go to the `script` folder inside the `multi_robot_utils_launch` package (using the source-tree path is more reliable than `ros2 pkg prefix`):
```
source ~/slideslam_ws/install/setup.bash
cd ~/slideslam_ws/src/SLIDE_SLAM/backend/multi_robot_utils_launch/script
```

Modify `tmux_single_outdoor_robot.sh` to set the `BAG_DIR` to where you downloaded the bags

Modify `BAG_PLAY_RATE` to your desired play rate (lower than 1.0 if you have a low-specification CPU)

Then make it executable if needed
```
chmod +x tmux_single_outdoor_robot.sh
```

Finally, execute this script
```
./tmux_single_outdoor_robot.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

**Option 2:** If you prefer not to use this tmux script, please refer to the `ros2 launch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-lidar-raw-bags-detailed-instructions).

## Run KITTI Benchmark experiments

**Option 1:** Use our tmux script

Source your workspace and go to the `script` folder inside the `multi_robot_utils_launch` package (using the source-tree path is more reliable than `ros2 pkg prefix`):
```
source ~/slideslam_ws/install/setup.bash
cd ~/slideslam_ws/src/SLIDE_SLAM/backend/multi_robot_utils_launch/script
```

Modify `tmux_single_outdoor_kitti.sh` to set the `BAG_DIR` to where you downloaded the bags

Then make it executable if needed
```
chmod +x tmux_single_outdoor_kitti.sh
```

Finally, execute this script
```
./tmux_single_outdoor_kitti.sh
```

If you want to terminate this program, go to the last terminal window and press `Enter` to kill all the tmux sessions.

# Troubleshoot
**Rate of segmentation:**
- When running on your own data, we recommend to throttle the segmentation topic (segmented point cloud or images) rate to 2-4 Hz to avoid computation delay in the front end, especially if you’re experiencing performance issues at higher rates. Please also update the `expected_segmentation_frequency` parameter in the corresponding `process_cloud_node_*_params.yaml` file as well as the `desired_frequency` in the `infer_node_params.yaml` to the actual rate of the topic. 

# Testing the ROS2 port

The `ros2_dev` branch ships a static test suite under [`tests/`](tests/) that future contributors can run to regression-check the port without needing a full ROS2 Jazzy build environment. The suite has three layers — a pure-bash static checker, a pytest mirror, and a runtime launch-graph smoke test. See [`tests/README.md`](tests/README.md) for per-layer usage notes.

**Current state (after the port + the review passes done on this branch):** `bash tests/static/check_ros2_port.sh` reports **51 checks, 51 passed, 0 failed, 0 skipped**.

## Static checks (`tests/static/check_ros2_port.sh`)

Pure bash + [ripgrep](https://github.com/BurntSushi/ripgrep) + `awk`. Runs anywhere with zero Python or ROS2 dependencies. **51 checks across 14 sections (A–N)** verifying:

- **A** — no ROS1 C++ idioms in active source under `backend/sloam/` and `frontend/object_modeller/` (21 sub-checks). Verifies zero occurrences of: `ros/ros.h`, `ros/package.h`, old-style message includes (`<pkg/Type.h>`), `tf/` headers, `nodelet/`, `pluginlib/`, `actionlib/`, `ros::NodeHandle`, `ros::Publisher`, `ros::Subscriber`, `ros::Time::now()`, `ros::Duration`, `ros::Rate`, `ros::init`, `ros::spin`/`spinOnce`, `ros::ok`, `ROS_INFO`/`WARN`/`ERROR`/`DEBUG`/`FATAL`, `nodelet::Nodelet`, `PLUGINLIB_EXPORT_CLASS`, `actionlib::`.
- **B** — no ROS1 Python idioms under `frontend/object_modeller/` and `frontend/scan2shape/` (6 sub-checks): no `import rospy` / `from rospy`, no `rospy.*` attribute access, no bare `import tf` / `from tf.*`, no `ros_numpy`, no `rospkg`.
- **C** — every `package.xml` across all 5 ROS packages is format 3, declares `ament_cmake`/`ament_python`/`rosidl_default_generators` as buildtool, declares `<build_type>` in `<export>`, contains no `catkin` or `message_generation`/`message_runtime`.
- **D** — every `CMakeLists.txt` across all 5 packages has no `find_package(catkin...)`, no `catkin_package(...)`, no `${catkin_INCLUDE_DIRS}`/`${catkin_LIBRARIES}`, no `add_message_files`/`add_service_files`/`add_action_files`/`generate_messages`, and calls `ament_package()` at the end.
- **E** — launch file structure (4 sub-checks): no XML `*.launch` files under `backend/` or `frontend/`, every `*.launch.py` imports `LaunchDescription`, every `*.launch.py` defines `generate_launch_description`, no `.launch.py` contains a literal `<launch>` XML tag (half-converted file).
- **F** — no camelCase `sloam_msgs` field accessors (`msg.robotID`, `msg.treeModels`, `msg.labelXYZ`, etc.) remain in any `.cpp`/`.h`/`.hpp`/`.py` file. The port renamed 25 fields from camelCase to `snake_case` to satisfy `rosidl`'s naming rules; this check catches any accessor we forgot to update. The internal C++ struct `PoseMstPair::relativeRawOdomMotion` is correctly preserved (not a ROS message field) via a word-boundary regex carve-out.
- **G** — every `#include <sloam_msgs/msg/*.hpp>` / `<sloam_msgs/srv/*.hpp>` / `<sloam_msgs/action/*.hpp>` and every `from sloam_msgs.msg import X` / `from sloam_msgs.srv import X` / `from sloam_msgs.action import X` resolves to a real `.msg`/`.srv`/`.action` file in the `sloam_msgs` package, using the known snake_case ↔ UpperCamelCase mapping.
- **H** — every file listed in every `install(PROGRAMS ...)` block in every `CMakeLists.txt` exists on disk relative to that CMakeLists.
- **I** — no `nodelet_plugins.xml` remains anywhere in the repo. The ROS1 nodelet machinery was replaced by `rclcpp_components::RCLCPP_COMPONENTS_REGISTER_NODE`.
- **J** — no XML `*.launch` files remain anywhere outside `tools/` and `tests/`.
- **K** — within each `*.launch.py` file, every `LaunchConfiguration('x')` reference has a matching `DeclareLaunchArgument('x', ...)` in the same file. Within-file check (does not follow `IncludeLaunchDescription` chains). Exempts the ROS2-injected launch builtins (`log_level`, `launch_prefix`, `use_sim_time`, etc.). The awk reads each launch file as a single record so multi-line `DeclareLaunchArgument(\n 'name',\n ...)` forms are matched correctly.
- **L** — no hardcoded user-specific absolute paths (`/home/<user>/`, `/opt/slideslam_docker_ws`, `/opt/bags/`, `/root/`) in any `.cpp`/`.h`/`.hpp`/`.py` source file. Strips C and Python comments before matching.
- **M** — every `Node(package='<local_pkg>', executable='<y>')` call in every `*.launch.py` resolves to either an `add_executable(<y> ...)` target or an `install(PROGRAMS .../<y>)` entry in the target package's `CMakeLists.txt`. Local packages are `sloam`, `sloam_msgs`, `multi_robot_utils_launch`, `object_modeller`, `scan2shape_launch`; external packages (`tf2_ros`, `topic_tools`, `rviz2`, third-party drivers, etc.) are skipped.
- **N** — no raw `declare_parameter("key", ...)` call appears in 2+ source files within the same package. ROS2 throws `rclcpp::exceptions::ParameterAlreadyDeclared` at runtime if the same parameter is declared twice on the same node, so this is a real hazard. The safe `*_declare_or_get<T>(node, "key", default)` wrapper family used throughout `backend/sloam` (`in_declare_or_get`, `sn_declare_or_get`, `pr_declare_or_get`, plain `declare_or_get`) is explicitly exempted — those wrappers guard with `node->has_parameter()` before declaring, so multiple callers on the same key are safe.

Run it with:

```
bash tests/static/check_ros2_port.sh            # summary only
bash tests/static/check_ros2_port.sh --verbose  # dump hit details on failures
```

Exits 0 if all 51 pass, non-zero otherwise. Exempted from every section: `backend/sloam/clipper_semantic_object/` (vendored third-party CMake library) and `frontend/scan2shape/rviz/`.

## pytest mirror (`tests/python/`)

Parallel encoding of the same checks as real `pytest` unit tests, for use in a future CI that has Python 3.10+ available. Uses only the standard library plus `pytest` — no ROS imports. The bash runner above is the authoritative layer; the pytest layer is useful when a build system needs structured pass/fail output.

```
pip install pytest
pytest tests/python -v
```

## Launch-graph smoke test (`tests/integration/launch_smoke_test.sh`)

Runtime test that requires a **working ROS2 Jazzy environment** (`source /opt/ros/jazzy/setup.bash`). For every `*.launch.py` file under `backend/` and `frontend/`, runs `ros2 launch --print-description <abs_path>` with a configurable per-file timeout (default 20s) and reports `OK` / `FAIL` / `TIMEOUT` per file. Using the direct-file-path form of `ros2 launch` means this test does NOT require the workspace to be built — only that `ros2` itself is on `PATH`. Skips with exit code 77 (autotools-style) if `ros2` is not installed, so CI runners without a ROS2 environment treat it as a skipped test rather than a failed one.

```
bash tests/integration/launch_smoke_test.sh
VERBOSE=1 bash tests/integration/launch_smoke_test.sh     # dump per-file error output
LAUNCH_TIMEOUT=60 bash tests/integration/launch_smoke_test.sh
```

## What has NOT been tested

Because no ROS2 Jazzy + GTSAM + PCL + GPU machine was available while this branch was put together, the following have **not** been verified end-to-end on `ros2_dev`. Contributions that exercise any of the below on a real ROS2 Jazzy machine — and file an issue or PR with the findings — are very welcome:

- Actual `colcon build --symlink-install` success of any package on Ubuntu 24.04 + ROS2 Jazzy (the static checks catch CMake/package.xml _consistency_, but they cannot confirm that the C++ actually _compiles_ against the installed ROS2 + GTSAM + Sophus + PCL + OpenCV headers).
- Runtime publisher / subscriber behaviour of any node — topic flow, QoS compatibility, message serialization round-trips.
- SLAM correctness on converted ROS2 bags (see the _Converting ROS1 bags to ROS2_ section for the conversion tool).
- TF chain correctness across the multi-robot pipeline.
- Action server / client handshakes for `sloam_msgs::action::ActiveLoopClosure` and `sloam_msgs::action::DetectLoopClosure`.
- End-to-end demo runs on the forest, parking-lot, indoor RGBD, and KITTI datasets.

A concrete checklist of runtime integration tests that a future contributor should implement lives in [`tests/integration/README.md`](tests/integration/README.md).

# Acknowledgement
We use GTSAM as the backend. We thank [Guilherme Nardari](https://linkedin.com/in/guilherme-nardari-23ba91a8) for his contributions to this repository. 

# Citation
If you find our system or any of its modules useful for your academic work, we would appreciate it if you could cite our work as follows:
```
@article{liu2024slideslam,
  title={Slideslam: Sparse, lightweight, decentralized metric-semantic slam for multi-robot navigation},
  author={Liu, Xu and Lei, Jiuzhou and Prabhu, Ankit and Tao, Yuezhan and Spasojevic, Igor and Chaudhari, Pratik and Atanasov, Nikolay and Kumar, Vijay},
  journal={arXiv preprint arXiv:2406.17249},
  year={2024}
}
```
