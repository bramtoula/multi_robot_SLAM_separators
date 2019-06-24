#!/usr/bin/env python
import rospy
import rosservice
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
# import tensorflow as tf
# import glob
# import scipy
# import netvlad_tf.net_from_mat as nfm
# import netvlad_tf.nets as nets
# from std_msgs.msg import String
from multi_robot_separators.srv import *
from sensor_msgs.msg import Image
from data_handler import DataHandler
from rtabmap_ros.msg import OdomInfo

import random


def find_separators():
    rospy.init_node('find_separators', anonymous=False)
    rate = rospy.Rate(0.3)

    dataHandler = DataHandler()

    # Callbacks to save images and keyframes
    rospy.Subscriber("left/image_rect",
                     Image, dataHandler.save_image_l)
    rospy.Subscriber("right/image_rect", Image, dataHandler.save_image_r)

    rospy.Subscriber("odom_info", OdomInfo, dataHandler.get_keyframes)

    # Initialize services
    s_find_matches = rospy.Service(
        'find_matches_compute', FindMatches, dataHandler.find_matches_service)
    s_receive_separators = rospy.Service(
        'receive_separators_py', ReceiveSeparators, dataHandler.receive_separators_service)
    i = 0
    while not rospy.is_shutdown():
        # main loop
        i += 1
        rospy.loginfo("i = "+str(i))
        # Compute descriptors
        dataHandler.compute_descriptors()
        service_list = rosservice.get_service_list()

        if i % 20 and (len(dataHandler.descriptors) > 0):
            # resp_matches = dataHandler.call_find_matches_serv()
            matched_ids_local_kept = []
            matched_ids_other_kept = []
            separators_found = []

            # Call service to find matches and corresponding keypoints and geometric descriptors
            try:
                s_find_matches_query = rospy.ServiceProxy(
                    'find_matches_query', FindMatches)
                flatten_desc = [
                    item for sublist in list(dataHandler.descriptors) for item in sublist]
                res_matches = s_find_matches_query(flatten_desc)
            except rospy.ServiceException, e:
                print "Service call to find matches query failed: %s" % e
                break

            # If matches are found, compute the separators, send them back, and add them to the pose graph
            for i in range(len(res_matches.matched_id_other)):
                # Compute geometric features of the local frames that were matched. Use matched_id_other since it is from the point of view of the other robot.
                local_features_and_desc = dataHandler.get_features(res_matches.matched_id_other[i])

                # Get the transformation
                try:
                    s_ans_est_transform = rospy.ServiceProxy(
                        'estimate_transformation', EstTransform)
                    # Transform computed FROM local_features_and_desc TO res_matches (other robot)
                    res_transform = s_ans_est_transform(
                        local_features_and_desc.descriptors, res_matches.descriptors_vec[i], local_features_and_desc.kpts3D, res_matches.kpts3D_vec[i], local_features_and_desc.kpts, res_matches.kpts_vec[i])
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                    continue
                matched_ids_local_kept.append(res_matches.matched_id_other[i])
                matched_ids_other_kept.append(res_matches.matched_id_local[i])
                separators_found.append(res_transform.poseWithCov)
                
                # Add the separator to the factor graph
                try:
                    s_add_seps_pose_graph = rospy.ServiceProxy(
                        'add_separators_pose_graph', ReceiveSeparators)
                    s_add_seps_pose_graph(matched_ids_local_kept,
                                        matched_ids_other_kept, separators_found)
                except rospy.ServiceException, e:
                    print "Service call add sep to pose graph failed: %s" % e

                # Send the separator back to the other robot
                try:
                    s_ans_rec_sep = rospy.ServiceProxy(
                        'found_separators_send', ReceiveSeparators)
                    s_ans_rec_sep(matched_ids_local_kept,
                                matched_ids_other_kept, separators_found)
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
        rate.sleep()


if __name__ == '__main__':
    try:
        find_separators()
    except rospy.ROSInterruptException:
        pass
