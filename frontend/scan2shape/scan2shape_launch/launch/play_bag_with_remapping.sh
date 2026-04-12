#!/usr/bin/env bash
# Replay a single-robot rosbag2 and remap its odometry / measurement
# topics into a namespaced stream used by downstream nodes.
ros2 bag play robot1-2023-09-28-10-35-14 \
    --topics /quadrotor1/lidar_odom /semantic_meas_sync_odom \
    --remap /quadrotor1/lidar_odom:=/quadrotor1/lidar_odom \
            /semantic_meas_sync_odom:=/quadrotor1/semantic_meas_sync_odom
