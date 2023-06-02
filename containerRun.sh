#!/bin/bash

# Launch container and enable visual odometry
docker run -it --rm \
    --user $UID \
    -e ROS_HOME=/tmp/.ros \
    --network host \
    -v ~/.ros:/tmp/.ros \
    introlab3it/rtabmap_ros:melodic-latest \
    roslaunch rtabmap_launch rtabmap.launch rtabmap_viz:=false database_path:=/tmp/.ros/rtabmap.db rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info  approx_sync:=false