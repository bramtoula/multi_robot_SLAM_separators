!/usr/bin/env python3
import rospy

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rosservice
# import cv2
# import numpy as np
# import tensorflow as tf
# import glob
# import scipy
# import netvlad_tf.net_from_mat as nfm
# import netvlad_tf.nets as nets
# from std_msgs.msg import String
from multi_robot_separators.srv import *
from sensor_msgs.msg import Image
from data_handler import DataHandler

def find_separators():
        rospy.init_node('find_separators', anonymous=False)
        rate = rospy.Rate(10) # 10hz

        dataHandler = DataHandler()

        # Callbacks to save images
        rospy.Subscriber("image_left", Image, dataHandler.save_image_l)
        rospy.Subscriber("image_right", Image, dataHandler.save_image_r)

        # Initialize services
        s_find_matches = rospy.Service('find_matches', FindMatches, dataHandler.find_matches)
        s_send_separators = rospy.Service('receive_separators', ReceiveSeparators, dataHandler.receive_separators)

        while not rospy.is_shutdown():
            # main loop
            service_list = rosservice.get_service_list()
            # If other robot founds (through advertised service)
                # If have a lower id
                    # Find matches (will get descriptors and match ids).
                        # Service in: all descriptors, return: list of IDs, list of keypoints and visual descriptors
                        rospy.wait_for_service('/robot_2/find_matches')
                        try:
                            find_matches_serv = rospy.ServiceProxy(
                                '/robot_X/find_matches', FindMatches)
                            resp_matches = find_matches_serv(dataHandler.get_descriptors())
                        except rospy.ServiceException, e:
                            print "Service call failed: %s"%e

                    # Compute transform
                        # Get visual stuff for matched IDs (using service)
                        # Call service to compute transform
                        # Send transform to other robot via service (In: transform, return: nothing)

            rate.sleep()



if __name__ == '__main__':
    try:
        find_separators()
    except rospy.ROSInterruptException:
        pass
