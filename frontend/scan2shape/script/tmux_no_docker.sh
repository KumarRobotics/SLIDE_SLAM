#!/bin/bash
#
# ⚠️  ROS1 LEGACY — NOT PORTED ⚠️
#
# This tmux script uses ROS1 CLI commands (roscore, rosparam, rosbag play,
# roslaunch, rosrun, roscd). It will NOT work in a ROS2 Jazzy environment
# as-is. The ROS1 source on master used this script; the ROS2 port preserved
# it verbatim as a reference for what the original pipeline did, but the
# commands must be ported before running on ROS2.
#
# Rough ROS1 -> ROS2 cheat sheet:
#   roscore                       -> (no equivalent; ROS2 uses DDS discovery)
#   rosparam set /use_sim_time X  -> launch with use_sim_time:=X per node,
#                                    or 'ros2 param set /<node> use_sim_time X'
#   rosbag play <bag.bag>         -> ros2 bag play <bag_dir>
#                                    (convert bags first via rosbags-convert)
#   roslaunch <pkg> <f>.launch    -> ros2 launch <pkg> <f>.launch.py
#   rosrun <pkg> <exe>            -> ros2 run <pkg> <exe>
#   roscd <pkg>                   -> cd "$(ros2 pkg prefix <pkg>)/share/<pkg>"
#
# Also: the hardcoded /home/sam/bags/... path must be updated for your
# system or made configurable via $SLIDE_SLAM_BAG_BASE or similar.
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
tmux send-keys -t $SESSION_NAME "rosbag play /home/sam/bags/yuezhan-bags/first_floor_handcarry_2023-04-04-14-11-02.bag --clock -r 2"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roslaunch sloam run_indoor_large_scale_exploration.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roslaunch rgb_sem_segmentation rgb_segmentation_bag.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roscd scan2shape_launch; python ../script/process_cloud_node.py" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Closure"
tmux send-keys -t $SESSION_NAME "sleep 1; rosrun loop_closure loop_closure_server_node" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "roscd scan2shape_launch; python ../script/pub_loop_closure_trigger.py"
tmux select-layout -t $SESSION_NAME tiled


tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_pipeline"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear