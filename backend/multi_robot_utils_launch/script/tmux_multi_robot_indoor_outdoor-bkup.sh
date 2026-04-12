#!/bin/bash


SESSION_NAME=multi_robot_nodes

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

SETUP_ROS_STRING="source ~/xmas_slam_ws/install/setup.bash; export ROS_MASTER_URI=http://localhost:11311"

# Python args
# ODOM_TOPIC="/dragonfly67/quadrotor_ukf/control_odom"
ODOM_TOPIC="/Odometry"
# ODOM_TOPIC="/quadrotor1/lidar_odom"
# ODOM_TOPIC="/quadrotor1/lidar_odom"
# POINT_CLOUD_NS="/quadrotor1/"
POINT_CLOUD_NS="/"


# Make mouse useful in copy mode
tmux setw -g mouse on


tmux rename-window -t $SESSION_NAME "Core"
# ROS2 has no roscore; left as a no-op for compatibility with legacy pane layout.
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; echo 'ROS2: no roscore needed'" Enter
tmux split-window -t $SESSION_NAME
# use_sim_time is applied per-node in ROS2 via launch parameters; left as a no-op here.
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; echo 'ROS2: set use_sim_time via node parameters'" Enter


# tmux new-window -t $SESSION_NAME -n "Main"
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot0.launch.py > log0.txt 2>&1" Enter
# tmux split-window -v -t $SESSION_NAME:1.0
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot1.launch.py > log1.txt 2>&1" Enter
# tmux select-pane -t $SESSION_NAME:1.1
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot2.launch.py > log2.txt 2>&1" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot3.launch.py > log3.txt 2>&1" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot4.launch.py > log4.txt 2>&1" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/for-multi-robot-experiement-outdoor-3rd-parking-lot-around-building-falcon-xmas-slam-pennovation_2023-10-20-13-22-40.bag --clock /quadrotor/lidar_odom:=/robot_4/lidar_odom /semantic_meas_sync_odom:=/robot_4/semantic_meas_sync_odom"
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/for-multi-robot-experiement-outdoor-1st-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-07-35.bag /quadrotor/lidar_odom:=/robot_2/lidar_odom /semantic_meas_sync_odom:=/robot_2/semantic_meas_sync_odom"
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/for-multi-robot-experiement-outdoor-2nd-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-00-01.bag /quadrotor/lidar_odom:=/robot_3/lidar_odom /semantic_meas_sync_odom:=/robot_3/semantic_meas_sync_odom"
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/test-map-merging-robot1ANDrobot2_2023-10-29-10-35-50.bag -u 200"
# tmux select-layout -t $SESSION_NAME tiled
tmux new-window -t $SESSION_NAME -n "Main"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.3
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.3
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.2
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.6
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.6
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot0.launch.py > log0.txt 2>&1" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot1.launch.py > log1.txt 2>&1" Enter
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot2.launch.py > log2.txt 2>&1" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot3.launch.py > log3.txt 2>&1" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $(ros2 pkg prefix --share sloam) && ros2 launch sloam decentralized_sloam_robot4.launch.py > log4.txt 2>&1" Enter
tmux select-pane -t $SESSION_NAME:1.5
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/xmas-slam-bags/for-multi-robot-experiement-outdoor-3rd-parking-lot-around-building-falcon-xmas-slam-pennovation_2023-10-20-13-22-40.bag --clock /quadrotor/lidar_odom:=/robot_4/lidar_odom /semantic_meas_sync_odom:=/robot_4/semantic_meas_sync_odom" Enter
tmux select-pane -t $SESSION_NAME:1.6
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/xmas-slam-bags/for-multi-robot-experiement-outdoor-1st-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-07-35.bag /quadrotor/lidar_odom:=/robot_2/lidar_odom /semantic_meas_sync_odom:=/robot_2/semantic_meas_sync_odom" Enter
tmux select-pane -t $SESSION_NAME:1.7
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/xmas-slam-bags/for-multi-robot-experiement-outdoor-2nd-parking-lot-falcon-xmas-slam-pennovation_2023-10-20-13-00-01.bag /quadrotor/lidar_odom:=/robot_3/lidar_odom /semantic_meas_sync_odom:=/robot_3/semantic_meas_sync_odom" Enter
tmux select-pane -t $SESSION_NAME:1.8
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 bag play ~/bags/xmas-slam-bags/test-map-merging-robot1ANDrobot2_2023-10-29-10-35-50.bag -u 200" Enter
tmux select-layout -t $SESSION_NAME tiled

# Adjust the layout to evenly distribute the panes
# tmux select-layout -t $SESSION_NAME even-vertical


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"


tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
