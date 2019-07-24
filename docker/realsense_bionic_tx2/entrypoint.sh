#!/bin/bash

source "/root/catkin_ws/devel/setup.bash"

export ROS_IP=192.168.12.10$1
export ROS_MASTER_URI=http://192.168.12.100:11311

roslaunch realsense2_camera multi_robot_slam_realsense_camera.launch robot_id:=$1
