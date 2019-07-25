#!/bin/bash

source "/root/catkin_ws/devel/setup.bash"

export ROS_IP=$(echo $(hostname -I) | cut -d' ' -f 2)
export ROS_MASTER_URI=http://192.168.12.$1:11311

roslaunch realsense2_camera multi_robot_slam_realsense_camera.launch robot_id:=$2
