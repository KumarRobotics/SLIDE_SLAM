#!/bin/sh

TOPICS="
/tf
/tf_static
/${AGENT}/camera/depth/image_rect_raw
/${AGENT}/camera/depth/camera_info
/${AGENT}/camera/color/image_raw
/${AGENT}/camera/infra1/image_rect_raw
/${AGENT}/camera/infra2/image_rect_raw
/${AGENT}/camera/imu
/${AGENT}/camera/aligned_depth_to_color/image_raw
/${AGENT}/camera/aligned_depth_to_color/camera_info
/scarab40/odom_laser
/scarab41/odom_laser
/scarab42/odom_laser
/scarab43/odom_laser
/scarab44/odom_laser
/scarab45/odom_laser
/scarab46/odom_laser
/robot0/semantic_meas_sync_odom 
/robot1/semantic_meas_sync_odom 
/robot2/semantic_meas_sync_odom 
/robot3/semantic_meas_sync_odom 
/robot4/semantic_meas_sync_odom 
"

ALL_TOPICS=$TOPICS

BAG_STAMP=$(date +%F-%H-%M-%S-%Z)
CURR_TIMEZONE=$(date +%Z)

BAG_PREFIX=VEMS-SLAM-${AGENT}-${CURR_TIMEZONE}

eval ros2 bag record $ALL_TOPICS -o $BAG_PREFIX