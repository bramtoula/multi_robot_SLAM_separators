#!/bin/bash

export ROS_IP=192.168.$2.$4
export ROS_MASTER_URI=http://192.168.$2.$3:11311

source "/root/rdpgo_ws/devel/setup.bash"

case "$1" in
        camera)
            roslaunch realsense2_camera multi_robot_slam_realsense_camera.launch robot_id:=$5
            ;;

        optimization)
            roslaunch robust_distributed_slam_module generic_robot_slam.launch local_robot_id:=$5 other_robot_id:=$6 port:=2458$5 --screen
            ;;

        blabbermouth3)
            cd blabbermouth/build &&\
	    ./blabbermouth -s $5 1:tcp:1:192.168.$2.$6:24580 2:tcp:1:192.168.$2.$7:24581 3:tcp:1:192.168.$2.$8:24582
            ;;

        blabbermouth2)
            cd blabbermouth/build &&\
            ./blabbermouth -s $5 1:tcp:1:192.168.$2.$6:24580 2:tcp:1:192.168.$2.$7:24581
            ;;

        trigger)
            rosservice call /robot_0/start_optimization
            rosservice call /robot_1/start_optimization
            rosservice call /robot_2/start_optimization
            ;;

        record)
            	rosbag record -o /root/rdpgo_ws/src/robust_distributed_slam_module/scripts/log/ /robot_$5/camera/color/camera_info /robot_$5/camera/color/image_raw \
                          /robot_$5/camera/infra1/camera_info /robot_$5/camera/infra1/image_rect_raw \
                          /robot_$5/camera/infra2/camera_info /robot_$5/camera/infra2/image_rect_raw \
                          /tf /tf_static &
            ;;

        separators)
            roslaunch multi_robot_separators multi_robot_slam_example.launch local_robot_id:=$5 other_robot_id:=$6 --screen
            ;;

        bag)
            roslaunch multi_robot_separators realsense_bag_example.launch local_robot_id:=$5 other_robot_id:=$6 bag:=$7 recorded_id:=$8 --screen
            ;;

        bash)
            /bin/bash
            ;;
        *)
            echo $"Usage: $1 {optimization|separators|bash}"
            exit 1
 
esac
