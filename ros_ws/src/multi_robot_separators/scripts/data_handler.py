from cv_bridge import CvBridge, CvBridgeError
import rospy
import sys
import cv2
import constants
import tensorflow as tf
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets
import numpy as np
from multi_robot_separators.srv import *
from sensor_msgs.msg import Image
from rtabmap_ros.msg import OdomInfo

from scipy.spatial.distance import cdist


class DataHandler:
    def __init__(self):
        self.images_l_queue = []
        self.images_r_queue = []
        self.images_l_kf = []
        self.images_r_kf = []
        self.timestamps_kf = []
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

    def save_image_l(self, image_l):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_l, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_l_queue.append((image_l.header.stamp, cv_image))

    def save_image_r(self, image_r):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_r, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_r_queue.append((image_r.header.stamp, cv_image))

    def compute_descriptors(self):
        # Check if enough data to fill a batch
        # if len(self.images_l_kf) - len(self.descriptors) >= constants.BATCH_SIZE:
        rospy.loginfo("Computing descriptors. Currently already computed " +
                      str(len(self.descriptors))+"/"+str(len(self.images_l_kf))+" frames")
        # If so, compute and store descriptors (as much as we can up to the batch size)
        batch = self.images_l_kf[len(
            self.descriptors):min(len(self.images_l_kf)-1, len(
                self.descriptors)+constants.BATCH_SIZE)]

        if len(batch) > 0:
            descriptors = self.sess.run(self.net_out, feed_dict={
                                        self.image_batch: batch})

            rospy.loginfo("Saving descriptors")
            self.descriptors.extend(
                descriptors[:, :constants.NETVLAD_DIMS].tolist())
        else:
            rospy.loginfo("Empty batch, no images to compute descriptors for")

    def find_matches(self, descriptors_to_comp):
        # find closest matches between self.descriptors and descriptors_to_comp, use scipy.spatial.distance.cdist
        local_descs = np.array(self.descriptors)
        distances = cdist(local_descs, descriptors_to_comp)
        indexes_smallest_values = np.argsort(distances, axis=None)
        indexes_smallest_values = np.unravel_index(
            indexes_smallest_values, (len(local_descs), len(descriptors_to_comp)))

        matches = []
        for i in range(min(len(indexes_smallest_values[0]), constants.MAX_MATCHES)):
            idx_local = indexes_smallest_values[0][i]
            idx_other = indexes_smallest_values[1][i]
            if distances[idx_local, idx_other] < constants.MATCH_DISTANCE:
                matches.append((idx_local, idx_other))
        return matches

    def get_images(self, image_id):
        return self.image_l[image_id], self.image_r[image_id]

    def save_separator(self, transform, local_frame_id, other_robot_id, other_robot_frame_id):
        self.separators_found.append(
            (transform, local_frame_id, other_robot_id, other_robot_frame_id))

    def get_keyframes(self, odom_info):
        if odom_info.keyFrameAdded:

            # Look for the index of the saved image corresponding to the timestamp
            try:
                idx_images_l_q = [y[0] for y in self.images_l_queue].index(
                    odom_info.header.stamp)
                idx_images_r_q = [y[0] for y in self.images_r_queue].index(
                    odom_info.header.stamp)
            except:
                rospy.logwarn(
                    "Keyframe timestamp not found in the saved images queue")
                return

            rospy.loginfo("Adding a keyframe and cleaning queue")
            self.timestamps_kf.append(odom_info.header.stamp)
            # Save keyframe images
            self.images_l_kf.append(self.images_l_queue[idx_images_l_q][1])
            self.images_r_kf.append(self.images_r_queue[idx_images_r_q][1])

            # Remove previous timestamps in the queues
            del self.images_l_queue[:idx_images_l_q]
            del self.images_r_queue[:idx_images_r_q]

    def find_matches_service(self, find_matches_req):

        rospy.loginfo("Reached service")
        descriptors_to_comp = np.array(
            find_matches_req.netvlad_descriptors).reshape(-1, constants.NETVLAD_DIMS)

        descriptors_vec = []
        kpts3d_vec = []
        kpts_vec = []

        # Find closest descriptors
        matches = self.find_matches(descriptors_to_comp)

        matches_local_resp = []
        matches_other_resp = []
        # Find corresponding visual keypoints and descriptors
        for match in matches:
            resp_feats_and_descs = self.get_features(match[0])

            if not resp_feats_and_descs:
                continue
            matches_local_resp.append(match[0])
            matches_other_resp.append(match[1])
            descriptors_vec.append(resp_feats_and_descs.descriptors)
            kpts3d_vec.append(resp_feats_and_descs.kpts3D)
            kpts_vec.append(resp_feats_and_descs.kpts)
        return FindMatchesResponse(matches_local_resp, matches_other_resp, descriptors_vec, kpts3d_vec, kpts_vec)

    def receive_separators_service(self, receive_separators_req):
        rospy.loginfo("Reached receiving separators service")
        for i in range(len(receive_separators_req.matched_ids_local)):
            self.separators_found.append(
                (receive_separators_req.matched_ids_local[i], receive_separators_req.matched_ids_other[i], receive_separators_req.separators[i]))
        rospy.loginfo("Currently found " +
                      str(len(self.separators_found))+" separators")
        return ReceiveSeparatorsResponse(True)

    def get_features(self, id):
        img_l = cv2.cvtColor(self.images_l_kf[id], cv2.COLOR_RGB2GRAY)
        img_r = cv2.cvtColor(self.images_r_kf[id], cv2.COLOR_RGB2GRAY)
        img_l_msg = self.bridge.cv2_to_imgmsg(img_l, encoding="mono8")
        img_r_msg = self.bridge.cv2_to_imgmsg(img_r, encoding="mono8")

        try:
            s_get_feats = rospy.ServiceProxy(
                'get_features_and_descriptor', GetFeatsAndDesc)
            resp_feats_and_descs = s_get_feats(img_l_msg, img_r_msg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            resp_feats_and_descs = []
        return resp_feats_and_descs
