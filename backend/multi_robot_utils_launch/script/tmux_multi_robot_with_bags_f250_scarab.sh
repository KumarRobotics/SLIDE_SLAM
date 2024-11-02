#!/bin/bash


SESSION_NAME=multi_robot_nodes
BAG_PLAY_RATE=3.0
BAG_DIR='/home/sam/bags/vems-slam-bags/all_slide_slam_public_demos/f250_scarab_autonomy'

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

SETUP_ROS_STRING="export ROS_MASTER_URI=http://localhost:11311"

# Make mouse useful in copy mode
tmux setw -g mouse on

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
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot0*.bag -r $BAG_PLAY_RATE --topics /dragonfly67/quadrotor_ukf/control_odom /robot0/semantic_meas_sync_odom /dragonfly67/quadrotor_ukf/control_odom:=/robot0/odom /robot0/semantic_meas_sync_odom:=/robot0/semantic_meas_sync_odom" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot1*.bag -r $BAG_PLAY_RATE --topics /scarab45/odom_laser /robot0/semantic_meas_sync_odom /scarab45/odom_laser:=/robot1/odom /robot0/semantic_meas_sync_odom:=/robot1/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.2
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot2*.bag -r $BAG_PLAY_RATE --topics /dragonfly67/quadrotor_ukf/control_odom /robot0/semantic_meas_sync_odom /dragonfly67/quadrotor_ukf/control_odom:=/robot2/odom /robot0/semantic_meas_sync_odom:=/robot2/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.3
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot3*.bag -r $BAG_PLAY_RATE --topics /dragonfly67/quadrotor_ukf/control_odom /robot0/semantic_meas_sync_odom /dragonfly67/quadrotor_ukf/control_odom:=/robot3/odom /robot0/semantic_meas_sync_odom:=/robot3/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.4
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot4*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot4/odom /robot0/semantic_meas_sync_odom:=/robot4/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.5
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot5*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot5/odom /robot0/semantic_meas_sync_odom:=/robot5/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.6
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot6*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot6/odom /robot0/semantic_meas_sync_odom:=/robot6/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.7
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && rosbag play robot7*.bag --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot7/odom /robot0/semantic_meas_sync_odom:=/robot7/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.8
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING;" Enter
tmux select-layout -t $SESSION_NAME tiled

# Adjust the layout to evenly distribute the panes
# tmux select-layout -t $SESSION_NAME even-vertical



# Add window for sloam
tmux new-window -t $SESSION_NAME -n "Sloam"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roscd sloam && roslaunch sloam decentralized_sloam_multi_robot_f250_scarab.launch enable_rviz:=true" Enter

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"


tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
