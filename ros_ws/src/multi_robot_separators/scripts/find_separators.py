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
    rate = rospy.Rate(0.3)  # 10hz

    dataHandler = DataHandler()

    # Callbacks to save images and keyframes
    rospy.Subscriber("left/image_rect",
                     Image, dataHandler.save_image_l)
    rospy.Subscriber("right/image_rect", Image, dataHandler.save_image_r)

    rospy.Subscriber("odom_info", OdomInfo, dataHandler.get_keyframes)

    # Initialize services
    s_find_matches = rospy.Service(
        'find_matches', FindMatches, dataHandler.find_matches_service)
    s_receive_separators = rospy.Service(
        'receive_separators', ReceiveSeparators, dataHandler.receive_separators_service)
    i = 0
    while not rospy.is_shutdown():
        # main loop
        i += 1
        rospy.loginfo("i = "+str(i))
        # Compute descriptors
        dataHandler.compute_descriptors()
        service_list = rosservice.get_service_list()

        if i == 5:
            resp_matches = dataHandler.call_find_matches_serv()
            if resp_matches:
                dataHandler.call_receive_transform(
                    resp_matches.matched_id_other, resp_matches.matched_id_other, resp_matches.descriptors_vec, resp_matches.kpts3D_vec, resp_matches.kpts_vec)

        rate.sleep()


if __name__ == '__main__':
    try:
        find_separators()
    except rospy.ROSInterruptException:
        pass
