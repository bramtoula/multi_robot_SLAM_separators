from cv_bridge import CvBridge, CvBridgeError
import rospy
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import constants
import tensorflow as tf
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets
import numpy as np
from multi_robot_separators.srv import *
from sensor_msgs.msg import Image
class DataHandler:
    def __init__(self):
        self.images_l = []
        self.images_r = []
        self.timestamps_l = []
        self.timestamps_r = []
        self.descriptors = []
        self.separators_found = []

        tf.reset_default_graph()
        self.image_batch = tf.placeholder(
            dtype=tf.float32, shape=[None, None, None, 3])

        self.net_out = nets.vgg16NetvladPca(self.image_batch)

        saver = tf.train.Saver()

        self.sess = tf.Session()
        saver.restore(self.sess, nets.defaultCheckpoint())

        self.bridge = CvBridge()

    def save_image_l(self,image_l):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_l, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_l.append(cv_image)

        # Compute descriptors (only done if enough images ready)
        self.compute_descriptors()

    def save_image_r(self, image_r):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_r, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_r.append(cv_image)

    def compute_descriptors(self):
        # Check if enough data to fill a batch
        rospy.loginfo("Should I compute a descriptor ?")
        if len(self.images_l) - len(self.descriptors) >= constants.BATCH_SIZE:
            # If so, compute and store descriptors
            batch = self.images_l[len(self.descriptors):]
            rospy.loginfo("I'm going to do it!")

            descriptors = self.sess.run(self.net_out, feed_dict={self.image_batch: batch})

            rospy.loginfo("I did iiit!")

            rospy.loginfo(descriptors)

            # Add descriptors to list

    def find_matches(self, descriptors_to_comp):
        # find closest matches between self.descriptors and descriptors_to_comp, use scipy.spatial.distance.cdist
        matches = []
        return matches

    def get_images(self, image_id):
        return self.image_l[image_id], self.image_r[image_id]

    def save_separator(self, transform, local_frame_id, other_robot_id, other_robot_frame_id):
        self.separators_found.append(
            (transform, local_frame_id, other_robot_id, other_robot_frame_id))

