#!/bin/bash

export ROS_IP=192.168.12.10$1
export ROS_MASTER_URI=http://192.168.12.100:11311

source "/root/rdpgo_ws/devel/setup.bash"

case "$3" in
        optimization)
            roslaunch robust_distributed_slam_module generic_robot_slam.launch local_robot_id:=$1 other_robot_id:=$2 port:=2458$1 --screen
            ;;
         
        separators)
            roslaunch multi_robot_separators multi_robot_slam_example.launch local_robot_id:=$1 other_robot_id:=$2 --screen
            ;;
         
        *)
            echo $"Usage: $3 {optimization|separators}"
            exit 1
 
esac

