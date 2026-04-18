#!/bin/bash
#
# ⚠️  ROS1 LEGACY — NOT PORTED ⚠️
#
# This tmux script uses ROS1 CLI commands (roscore, rosparam, rosbag play,
# roslaunch, rosrun, roscd). It will NOT work in a ROS2 Jazzy environment
# as-is. See tmux_no_docker.sh in the same directory for a rough ROS1 ->
# ROS2 CLI cheat sheet. Also: /home/sam/bags/... is a hardcoded path that
# must be updated for your system before running.
#

SESSION_NAME=tmux_pipeline

## Aliases are not expanded in non-intereactive bash
shopt -s expand_aliases
source ~/.bash_aliases

## Check alias set
if [ "$(type -t ssloam)" = 'alias' ]; then
    echo 'ssloam is an alias'
else
    echo 'ssloam is not an alias, please set it first!'
    exit
fi
if [ "$(type -t runinfernode)" = 'alias' ]; then
    echo 'runinfernode is an alias'
else
    echo 'runinfernode is not an alias, please set it first!'
    exit
fi
if [ "$(type -t runprocesscloudnode)" = 'alias' ]; then
    echo 'runprocesscloudnode is an alias'
else
    echo 'runprocesscloudnode is not an alias, please set it first!'
    exit
fi

if [ -z ${TMUX} ];
then
  tmux has-session -t $SESSION_NAME 2>/dev/null
  if [ "$?" -eq 1 ] ; then
    # Set up session
    TMUX= tmux new-session -s $SESSION_NAME -d
    echo "Starting new session."
  else
    echo "Session exist, kill it first."
  fi
else
  echo "Already in tmux, leave it first."
  exit
fi

# runyuezhansloamdocker = run_sloam_node_xu.sh
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "roscore" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; rosparam set /use_sim_time True" Enter

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "rosbag play /home/sam/bags/forest-map-merging-starting-same-location-left-direction-loop-falcon_pennovation_2022-11-29-13-55-01.bag --clock --topics /os_node/lidar_packets /os_node/imu_packets /os_node/metadata -s 20"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; ssloam; roslaunch sloam run_pennovation.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; ssloam; runinfernode" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; ssloam; runprocesscloudnode" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Faster LIO"
tmux send-keys -t $SESSION_NAME "sleep 1; ssloam; roslaunch scan2shape_launch run_flio_with_driver.launch" Enter

tmux new-window -t $SESSION_NAME -n "Relay Topics"
tmux send-keys -t $SESSION_NAME "sleep 1; ssloam; roslaunch scan2shape_launch relay_topics_multi_robot.launch" Enter

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_pipeline"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear