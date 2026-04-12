#!/bin/bash


SESSION_NAME=slide_slam_nodes
BAG_PLAY_RATE=0.5
#BAG_DIR='/home/sam/bags/vems-slam-bags/all_slide_slam_public_demos/forests'
# BAG_DIR='/opt/bags/vems-slam-bags/all_slide_slam_public_demos/forests'
# BAG_DIR='/home/sam/bags/vems-slam-bags/all_slide_slam_public_demos/indoor'
BAG_DIR='/opt/bags/vems-slam-bags/all_slide_slam_public_demos/indoor'

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
# tmux select-pane -t $SESSION_NAME:1.3
# tmux split-window -h -t $SESSION_NAME
# tmux select-pane -t $SESSION_NAME:1.2
# tmux split-window -h -t $SESSION_NAME
# tmux select-pane -t $SESSION_NAME:1.6
# tmux split-window -h -t $SESSION_NAME
# tmux select-pane -t $SESSION_NAME:1.6
# tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 launch object_modeller rgb_segmentation_f250.launch.py" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 launch object_modeller sync_semantic_measurements.launch.py robot_name:=robot0 odom_topic:=/dragonfly67/quadrotor_ukf/control_odom" Enter
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 launch sloam single_robot_sloam_test_f250.launch.py enable_rviz:=true" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; ros2 launch scan2shape_launch process_cloud_node_rgbd_indoor_with_ns.launch.py odom_topic:=/dragonfly67/quadrotor_ukf/control_odom robot_name:=robot0" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && ros2 bag play indoor-f250-*.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /dragonfly67/quadrotor_ukf/control_odom /camera/aligned_depth_to_color/image_raw /camera/color/image_raw /camera/depth/image_rect_raw /camera/aligned_depth_to_color/image_raw:=/robot0/camera/aligned_depth_to_color/image_raw /camera/color/image_raw:=/robot0/camera/color/image_raw /camera/depth/image_rect_raw:=/robot0/camera/depth/image_rect_raw" Enter
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && ros2 bag play robot4*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot4/odom /robot0/semantic_meas_sync_odom:=/robot4/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.5
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && ros2 bag play robot5*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot5/odom /robot0/semantic_meas_sync_odom:=/robot5/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.6
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && ros2 bag play robot6*.bag -r $BAG_PLAY_RATE --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot6/odom /robot0/semantic_meas_sync_odom:=/robot6/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.7
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; cd $BAG_DIR && ros2 bag play robot7*.bag --topics /Odometry /robot0/semantic_meas_sync_odom /Odometry:=/robot7/odom /robot0/semantic_meas_sync_odom:=/robot7/semantic_meas_sync_odom" Enter
# tmux select-pane -t $SESSION_NAME:1.8
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING;" Enter
tmux select-layout -t $SESSION_NAME tiled


# Add window for roscore
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
# ROS2 has no roscore; left as a no-op for compatibility with legacy pane layout.
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; echo 'ROS2: no roscore needed'" Enter
tmux select-pane -t $SESSION_NAME:2.1
# use_sim_time is applied per-node in ROS2 via launch parameters; left as a no-op here.
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; echo 'ROS2: set use_sim_time via node parameters'" Enter


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"


tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
