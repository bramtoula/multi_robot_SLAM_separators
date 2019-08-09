#!/usr/bin/env python
import rospy
import rosservice
import sys
import cv2
import numpy as np
from multi_robot_separators.srv import *
from sensor_msgs.msg import Image
from data_handler import DataHandler
from rtabmap_ros.msg import OdomInfo
import collections
import random


def find_separators():
    rospy.init_node('find_separators', anonymous=False)
    rate = rospy.Rate(0.3)

    dataHandler = DataHandler()

    # Callbacks to save images and keyframes
    rospy.Subscriber("left/image_rect",
                     Image, dataHandler.save_image_l)
    rospy.Subscriber("right/image_rect", Image, dataHandler.save_image_r)

    rospy.Subscriber("rgb/image_rect", Image, dataHandler.save_image_rgb)

    rospy.Subscriber("odom_info", OdomInfo, dataHandler.get_keyframes)

    # Initialize services
    s_find_matches = rospy.Service(
        'find_matches_compute', FindMatches, dataHandler.find_matches_service)
    s_receive_separators = rospy.Service(
        'receive_separators_py', ReceiveSeparators, dataHandler.receive_separators_service)

    # Find services that need to be called
    s_ans_est_transform = rospy.ServiceProxy(
        'estimate_transformation', EstTransform)
    s_find_matches_query = rospy.ServiceProxy(
        'find_matches_query', FindMatches)
    s_ans_rec_sep = rospy.ServiceProxy(
        'found_separators_query', ReceiveSeparators)

    i = 0
    while not rospy.is_shutdown():
        # main loop
        i += 1
        
        # Compute descriptors
        dataHandler.compute_descriptors()
        service_list = rosservice.get_service_list()
        
        # Check if other robot is visible
        if '/robot_'+str(dataHandler.other_robot_id)+'/find_matches_query' in service_list:
            if i % 20 and (len(dataHandler.local_descriptors) > 0):
                # Call service to find matches and corresponding keypoints and geometric descriptors
                try:
                    # Only send descriptors which havent been sent yet
                    descriptors_to_send = dataHandler.local_descriptors[dataHandler.nb_descriptors_already_sent:]

                    flatten_desc = [
                        item for sublist in list(descriptors_to_send) for item in sublist]
                    res_matches = s_find_matches_query(flatten_desc)
                except rospy.ServiceException, e:
                    print "Service call to find matches query failed: %s" % e
                    break

                dataHandler.nb_descriptors_already_sent += len(descriptors_to_send)

                # If no matches are found, do nothing
                if not res_matches.kf_ids_computing_robot:
                    continue
                
                # If matches are found, compute the separators, send them back, and add them to the pose graph

                frames_kept_ids_from_kept = collections.deque()
                frames_kept_ids_to_kept = collections.deque()
                kf_ids_to_kept = collections.deque()
                transform_est_success = collections.deque()
                separators_found = collections.deque()
                pose_estimates_to_kept = collections.deque()

                for i in range(len(res_matches.kf_ids_computing_robot)):
                    # Compute geometric features of the local frames that were matched. Use matched_ids_other since it is from the point of view of the other robot.
                    local_features_and_desc = dataHandler.get_geom_features(
                        res_matches.frames_kept_ids_querying_robot[i])
                    
                    try:
                        # Transform computed FROM local_features_and_desc TO res_matches (other robot)
                        res_transform = s_ans_est_transform(
                            local_features_and_desc.descriptors, res_matches.descriptors_vec[i], local_features_and_desc.kpts3D, res_matches.kpts3D_vec[i], local_features_and_desc.kpts, res_matches.kpts_vec[i])
                        
                    except rospy.ServiceException, e:
                        print "Service call estimate_transformation failed: %s" % e
                        continue
                    
                    # Check if transform was successfully computed
                    if res_transform.success:
                        transform_est_success.append(True)
                    else:
                        transform_est_success.append(False)
                    
                    frames_kept_ids_from_kept.append(
                        res_matches.frames_kept_ids_querying_robot[i])
                    frames_kept_ids_to_kept.append(res_matches.frames_kept_ids_computing_robot[i])
                    
                    kf_ids_to_kept.append(
                        res_matches.kf_ids_computing_robot[i])
                    
                    separators_found.append(res_transform.poseWithCov)
                    
                    if dataHandler.send_estimates_of_poses:
                        pose_estimates_to_kept.append(res_matches.pose_estimates[i])
                    
                kf_ids_from_kept = dataHandler.get_kf_ids_from_frames_kept_ids(
                    frames_kept_ids_from_kept)
                
                if dataHandler.send_estimates_of_poses:
                    try:
                        pose_estimates_from_kept = dataHandler.s_get_pose_estimates(
                            kf_ids_from_kept).pose_estimates
                    except rospy.ServiceException, e:
                        print "Service call pose_estimates failed: %s" % e
                else:
                    pose_estimates_from_kept = collections.deque()
                
                # Add the separator to the factor graph and save it
                dataHandler.found_separators_local(kf_ids_from_kept, kf_ids_to_kept, frames_kept_ids_from_kept,  frames_kept_ids_to_kept, pose_estimates_from_kept, pose_estimates_to_kept, transform_est_success, separators_found)
                
                # Send the separator found to the other robot concerned
                try:
                    s_ans_rec_sep(dataHandler.local_robot_id, dataHandler.other_robot_id,kf_ids_from_kept,
                                  kf_ids_to_kept, frames_kept_ids_from_kept, frames_kept_ids_to_kept, pose_estimates_from_kept, pose_estimates_to_kept, transform_est_success, separators_found)
                except rospy.ServiceException, e:
                    print "Service call found_separators_query failed: %s" % e
                
        rate.sleep()


if __name__ == '__main__':
    try:
        find_separators()
    except rospy.ROSInterruptException:
        pass
