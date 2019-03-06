# point_cloud_filter

Point Cloud Filter Node

ROS:
-subscribes to sensor_msgs/PointCloud2 msg produced by RGBD Camera
-publishes sensor_msgs/PointCloud2 msg

Point Cloud Filter Node filters a point cloud. It firstly filters Y (in camera frame), then Z(in camera frame), then finally a voxel grid filter
All parameters can be inputted into the node
