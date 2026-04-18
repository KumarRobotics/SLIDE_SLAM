#!/bin/bash
#
# ⚠️  ROS1 LEGACY — NOT PORTED ⚠️
#
# This tmux script uses ROS1 CLI commands (roscore, rosparam, rosbag play,
# roslaunch, rosrun, roscd). It will NOT work in a ROS2 Jazzy environment
# as-is. See tmux_no_docker.sh in the same directory for a rough ROS1 ->
# ROS2 CLI cheat sheet. Also: ~/bag/pennovation/ assumes a specific layout
# on the original developer's machine.
#

SESSION_NAME=tmux_pipeline

## Aliases are not expanded in non-intereactive bash
shopt -s expand_aliases
source ~/.bash_aliases

## Check alias set
if [ "$(type -t runyuezhansloamdocker)" = 'alias' ]; then
    echo 'runyuezhansloamdocker is an alias'
else
    echo 'runyuezhansloamdocker is not an alias, please set it first!'
    exit
fi
if [ "$(type -t rundetectnode)" = 'alias' ]; then
    echo 'rundetectnode is an alias'
else
    echo 'rundetectnode is not an alias, please set it first!'
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
tmux send-keys -t $SESSION_NAME "rosbag play ~/bag/pennovation/ --clock -r 2"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; runyuezhansloamdocker" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; rundetectnode" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; runprocesscloudnode" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Closure"
tmux send-keys -t $SESSION_NAME "docker exec -it sloam_ros bash -c /opt/generic-sloam_ws/src/docker/run_map_merging_node.sh"

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "docker stop sloam_ros -t 1; tmux kill-session -t tmux_pipeline"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear