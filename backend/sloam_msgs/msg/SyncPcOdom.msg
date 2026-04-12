Header header
sensor_msgs/PointCloud2 cloud
nav_msgs/Odometry odom
# This message contains the synced odometry with the point cloud
# The point cloud contains data in the following order: 
# points (x, y, z), 
# label (classification), 
# id (an unique id for different instance. Note: not global ID),
# confidence (the confidence of the classification)
# see process cloud node in sloam to parse the point cloud
