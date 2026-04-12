#!/usr/bin/env bash
# ROS2 Jazzy Jalisco workflow (ubuntu 24.04). For the stable ROS1 Noetic version, see the master branch.
# NOTE: the docker tag `xurobotics/slide-slam:ros2-jazzy` referenced below is a placeholder;
# the image does not exist on Docker Hub yet. Build it locally or update the tag once published.
# Inside the container, /opt/slideslam_docker_ws is the colcon workspace (formerly catkin_ws on ROS1).

SlideSlamWs="/home/sam/slideslam_docker_ws" # point to your workspace directory
SlideSlamCodeDir="/home/sam/slideslam_docker_ws/src/SLIDE_SLAM" # point to your code directory where you cloned the repository
BAGS_DIR='/home/sam/bags' # point to your bags / data directory

xhost +local:root # for the lazy and reckless
docker run -it \
    --name="slideslam_ros2" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/opt/slideslam_docker_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
    --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
    --volume="$BAGS_DIR:/opt/bags" \
    --volume="/home/$USER/.bash_aliases:/root/.bash_aliases" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/repos:/home/$USER/repos" \
    xurobotics/slide-slam:ros2-jazzy \
    bash

# Once inside the container, set up the environment with:
#   source /opt/ros/jazzy/setup.bash
#   cd /opt/slideslam_docker_ws && colcon build --symlink-install
#   source /opt/slideslam_docker_ws/install/setup.bash
