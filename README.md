# SlideSLAM

This repository contains the source code for the project SlideSLAM: Sparse, Lightweight, Decentralized Metric-Semantic SLAM for Multi-Robot Navigation. 
- More details can be found on the [project website](https://xurobotics.github.io/slideslam/).
- Our paper is available on arXiv [here](https://arxiv.org/abs/2406.17249). 

# Table of contents
- [SlideSLAM](#slideslam)
- [Table of contents](#table-of-contents)
- [Use docker (recommended)](#use-docker-recommended)
- [Build from source (only if you do not want to use docker)](#build-from-source-only-if-you-do-not-want-to-use-docker)
- [Run our demos (with processed data)](#run-our-demos-with-processed-data)
  - [Download example data](#download-example-data)
  - [What these demos will do](#what-these-demos-will-do)
  - [Run multi-robot demo (based on LiDAR data)](#run-multi-robot-demo-based-on-lidar-data)
- [Run on raw sensor data (RGBD or LiDAR bags)](#run-on-raw-sensor-data-rgbd-or-lidar-bags)
  - [Download example data](#download-example-data-1)
  - [Run our RGBD data experiments](#run-our-rgbd-data-experiments)
  - [Run our LiDAR Data experiments](#run-our-lidar-data-experiments)
- [Troubleshoot](#troubleshoot)
- [Acknowledgement](#acknowledgement)
- [Citation](#citation)




# Use docker (recommended)

**Install docker**: https://docs.docker.com/desktop/install/linux/ubuntu/#install-docker-desktop

**Pull the docker image**: 
```
docker pull xurobotics/slide-slam:latest
```

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
cd /opt/slideslam_docker_ws
catkin build -DCMAKE_BUILD_TYPE=Release
```

**Run the demos**
```
source /opt/slideslam_docker_ws/devel/setup.bash
```
Follow the instructions below to run the demos. Remember to commit your changes inside docker envirnoment to keep them (e.g. newly installed pkgs). 

Type `exit` to exit the container.

You can re-enter the container, or enter the container from a new terminal by either 
```
docker start slideslam_ros && docker exec -it slideslam_ros /bin/bash
``` 
or remove your docker container using the command 
```
docker rm slideslam_ros
``` 
before you run the docker image again.

**Troubleshoot**:
- If you do not see your code or bags inside docker, double check `run_slide_slam_docker.sh` file to make sure you have your workspace and BAG folders mapped properly. 


# Build from source (only if you do not want to use docker)

**Install ROS** (code currently only tested on Ubuntu 20.04 + ROS Noetic)

Please refer to this [link](https://wiki.ros.org/noetic/Installation/Ubuntu) for installing ROS Noetic

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

**Install ros_numpy**:
```
sudo apt install ros-noetic-ros-numpy
```

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
pip install numpy==1.22.3
pip install scikit-learn
pip install scipy
pip install open3d
pip install matplotlib
pip install git+https://github.com/dimatura/pypcd.git
```

- Install `tmux` for running our demo experiments
```
sudo apt update
sudo apt install tmux
```

**Build in release mode**
```
source /opt/ros/noetic/setup.bash
cd ~/slideslam_ws
catkin build -DCMAKE_BUILD_TYPE=Release
```
**Source your workspace using**
```
source ~/slideslam_ws/devel/setup.bash
```



**Troubleshoot:**
- If you have built GTSAM from source before, you need to remove everything related to gtsam/GTSAM in /usr/local by doing:
```
sudo rm -rf /usr/local/lib/cmake/*GTSAM*
sudo rm -rf /usr/local/include/gtsam
```
- If you have installed GTSAM using apt-get, remove them first, use this command `sudo apt remove --purge libgtsam*

# Run our demos (with processed data)
Note: if the access to any of the links is lost, please contact the authors, and we will provide the data from our lab's NAS.

This section will guide you through running our demos with processed data. We provide processed data in the form of rosbags that contains only the odometry and semantic measurements (i.e. object observations). Running the entire pipeline containing object detection and the rest of SLAM for multiple robots simultaneously onboard one computer is computationally and memory intensive. 

**Note:** Such tests can to a large degree replicate what would happen onboard the robot since when you run real world multi-robot experiment, each robot will only be responsible for processing its own data, and the processed data shared by the other robots in the form provided by here. 

## Download example data

Please download the processed data bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). This containes compact processed bags for forest and urban outdoor environments. Please use the right data with the right scripts as specified below.


## What these demos will do
- Intermittent communication between robot nodes at a fixed time interval. 
- Multiple robots running on the same computer, and therefore, the computational load is going to be (num_robots multiplied by the computation load of each robot during actual experiment).

## Run multi-robot demo (based on LiDAR data)
**First, please refer to the section above and make sure you have everything built.**

**Option 1:** Use our tmux script (recommended)

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
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

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself.

**To run the same above example with urban outdoor data, use the `tmux_multi_robot_with_bags_parking_lot.sh` script and repeat the above steps.**

# Run on raw sensor data (RGBD or LiDAR bags)
This section will guide you through running our code stack with raw sensor data, which is rosbags containing LiDAR-based or RGBD-based data. Note: size of these raw bags are usually anywhere from 10-100 GB.

## Download example data

Please download the LiDAR demo bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `outdoor` folder.

Please download the RGBD demo bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `indoor` folder.

Please download the KITTI benchmark processed bags from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is present inside the `kitti_bags` folder.

Please download our trained RangeNet++ model from [this link](https://drive.google.com/drive/folders/0B5oeNvHFA8WDfjVMa29HSldseF81SUxFMTA4d3F6VlN2YW9hdlZ0RzNMM1V3TGdJZmRMb1E?resourcekey=0-E7OTjCMZq2C58dps4Gibqw&usp=sharing). It is currently named `penn_smallest.zip`. Follow the instructions in the `Run our LiDAR data experiments`  section below on how to use this model.

## Run our RGBD data experiments

**Option 1:** Use our tmux script (recommended)

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
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

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-rgbd-raw-bags-detailed-instructions).

## Run our LiDAR Data experiments

**Download the LiDAR semantic segmentation RangeNet++ model**
```
(1) Download the model from the above link.
(2) Unzip the file and place the model in a location of your choice.
(3) Open the extracted model folder and make sure that there are no files inside having a .zip extension. If there are, then rename ALL OF THEM to remove the .zip extension. For example backbone.zip should be renamed to backbone
```

**Option 1:** Use our tmux script (recommended)

Make sure you edit the ```infer_node_params.yaml``` file present inside the ```scan2shape_launch/config``` folder and set the value of ```model_dir``` param to point to the path to the RangeNet++ model you downloaded in the previous step. Make sure to compelte the path with the ```/``` at the end.

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
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

**Option 2:** If you prefer not to use this tmux script, please refer to the `roslaunch` commands inside this tmux script and execute those commands by yourself, or using the detailed instructions found [here](https://github.com/XuRobotics/SLIDE_SLAM/wiki#run-lidar-raw-bags-detailed-instructions).

## Run KITTI Benchmark experiments

**Option 1:** Use our tmux script

Source and go to the ' folder inside `multi_robot_utils_launch` package:
```
source ~/slideslam_ws/devel/setup.bash
roscd multi_robot_utils_launch/script
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

# Acknowledgement
We use GTSAM as the backend. We thank [Guilherme Nardari](linkedin.com/in/guilherme-nardari-23ba91a8) for his contributions to this repository. 

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
