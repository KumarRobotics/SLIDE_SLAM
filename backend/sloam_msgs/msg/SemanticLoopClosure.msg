std_msgs/Header header
# VIO raw odometry
nav_msgs/Odometry vio_odom
# factor graph estimated pose
nav_msgs/Odometry corrected_odom
# history key pose index for loop closure
std_msgs/UInt64 key_pose_idx
# history key pose estimated by factor graph for loop closure
nav_msgs/Odometry optimized_key_pose
# output of map merging: relative transform from optimized_key_pose to current pose upon loop closure
nav_msgs/Odometry optimized_relative_loop_closure_pose
# output of map merging: relative transform from optimized_key_pose to current pose upon loop closure
nav_msgs/Odometry vio_to_sloam_pose