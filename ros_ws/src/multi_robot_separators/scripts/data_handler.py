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
        self.images_rgb_queue = []
        self.geometric_feats = []
        self.images_rgb_kf = []
        self.timestamps_kf = []
        self.descriptors = []
        self.separators_found = []
        self.local_kf_already_used = []
        self.other_kf_already_used = []
        self.kf_pairs_ignored = []
        self.nb_kf_skipped = 0
        self.original_ids_of_kf = []
        self.orig_id_last_img_in_q = 0

        tf.reset_default_graph()
        self.image_batch = tf.placeholder(
            dtype=tf.float32, shape=[None, None, None, 3])

        self.net_out = nets.vgg16NetvladPca(self.image_batch)

        saver = tf.train.Saver()

        self.sess = tf.Session()
        saver.restore(self.sess, nets.defaultCheckpoint())

        self.bridge = CvBridge()

        self.local_robot_id = rospy.get_param("local_robot_id")
        self.other_robot_id = rospy.get_param("other_robot_id")
        self.s_add_seps_pose_graph = rospy.ServiceProxy(
            'add_separators_pose_graph', ReceiveSeparators)
        self.s_get_feats = rospy.ServiceProxy(
            'get_features_and_descriptor', GetFeatsAndDesc)

    def __del__(self):
        with open('/root/multi_robot_SLAM_separators/logs/kf_orig_ids_'+str(self.local_robot_id)+'.txt', 'w') as file:
            for id in self.original_ids_of_kf:
                file.write("%i\n" % id)

    def save_image_l(self, image_l):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_l, "rgb8")
        except CvBridgeError as e:
            print(e)

        self.images_l_queue.append((image_l.header.stamp, cv_image))
        self.orig_id_last_img_in_q += 1
        if len(self.images_l_queue) > constants.MAX_QUEUE_SIZE:
            self.images_l_queue.pop(0)

    def save_image_r(self, image_r):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_r, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_r_queue.append((image_r.header.stamp, cv_image))
        if len(self.images_r_queue) > constants.MAX_QUEUE_SIZE:
            self.images_r_queue.pop(0)

    def save_image_rgb(self, image_rgb):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_rgb, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_rgb_queue.append((image_rgb.header.stamp, cv_image))
        if len(self.images_rgb_queue) > constants.MAX_QUEUE_SIZE:
            self.images_rgb_queue.pop(0)

    def compute_descriptors(self):
        # Check if enough data to fill a batch
        # if len(self.images_l_kf) - len(self.descriptors) >= constants.BATCH_SIZE:
        rospy.loginfo("Computing descriptors. Currently already computed " +
                      str(len(self.descriptors))+" netvlad descriptors. Number of frames left in the queue:"+str(len(self.images_rgb_kf)))
        # If so, compute and store descriptors (as much as we can up to the batch size)
        nb_images_batch = min(len(self.images_rgb_kf), constants.BATCH_SIZE)
        batch = self.images_rgb_kf[:nb_images_batch]

        if len(batch) > 0:
            descriptors = self.sess.run(self.net_out, feed_dict={
                                        self.image_batch: batch})

            rospy.loginfo("Saving descriptors")
            self.descriptors.extend(
                descriptors[:, :constants.NETVLAD_DIMS].tolist())

            # Remove rgb images from memory
            self.images_rgb_kf = self.images_rgb_kf[nb_images_batch:]
        else:
            rospy.loginfo("Empty batch, no images to compute descriptors for")

    def find_matches(self, descriptors_to_comp):
        # find closest matches between self.descriptors and descriptors_to_comp, use scipy.spatial.distance.cdist
        local_descs = np.array(self.descriptors)

        distances = cdist(local_descs, descriptors_to_comp)

        # TODO Maybe don't ignore every possible matches for a frame (full line to inf), but only specific matches
        # Increase distances for local frames which were already matched so we can discover new frames
        rospy.loginfo("distances matrix size: " +
                      str(len(distances)) + " "+str(len(distances[0])))
        rospy.loginfo("Keyframes already used: ")
        rospy.loginfo(self.local_kf_already_used)
        if len(self.local_kf_already_used) > 0:
            distances[np.array(self.local_kf_already_used)] = np.inf
        if len(self.other_kf_already_used) > 0:
            distances[:,np.array(self.other_kf_already_used)] = np.inf

        for pair in self.kf_pairs_ignored:
            distances[pair[0], pair[1]] = np.inf


        indexes_smallest_values_each_frame = np.argsort(distances, axis=1)[:,0]
        smallest_values_each_frame = distances[np.arange(
            len(distances)), indexes_smallest_values_each_frame]

        indexes_smallest_values_all_frames = np.argsort(smallest_values_each_frame)

        matches = []
        for i in range(min(len(indexes_smallest_values_all_frames), constants.MAX_MATCHES)):
            idx_local = indexes_smallest_values_all_frames[i]
            idx_other = indexes_smallest_values_each_frame[indexes_smallest_values_all_frames[i]]

            # Check if other frame index already used
            if idx_other in [match[1] for match in matches]:
                continue
        
            if distances[idx_local, idx_other] < constants.MATCH_DISTANCE:
                matches.append((idx_local, idx_other))
            else:
                break

        rospy.loginfo("Matches found:")
        rospy.loginfo(matches)
        return matches

    # def get_images(self, image_id):
    #     return self.image_l[image_id], self.image_r[image_id]

    # def save_separator(self, transform, local_frame_id, other_robot_id, other_robot_frame_id):
    #     rospy.loginfo("Savng separator")
    #     self.separators_found.append(
    #         (transform, local_frame_id, other_robot_id, other_robot_frame_id))

    #     self.kf_already_used.append(local_frame_id)

    def get_keyframes(self, odom_info):
        rospy.loginfo("Size of images queue: " +
                      str(len(self.images_l_queue))+"\n")
        if odom_info.keyFrameAdded:
            if self.nb_kf_skipped < constants.NB_KF_SKIPPED:
                self.nb_kf_skipped += 1
            else:
                # Look for the index of the saved image corresponding to the timestamp
                try:
                    idx_images_l_q = [y[0] for y in self.images_l_queue].index(
                        odom_info.header.stamp)
                    idx_images_r_q = [y[0] for y in self.images_r_queue].index(
                        odom_info.header.stamp)
                    idx_images_rgb_q = [y[0] for y in self.images_rgb_queue].index(
                        odom_info.header.stamp)
                except:
                    rospy.logwarn(
                        "Keyframe timestamp not found in the saved images queue")
                    return

                rospy.loginfo("Adding a keyframe and cleaning queue")
                self.timestamps_kf.append(odom_info.header.stamp)
                # Save geometric features and descs of keyframe images
                self.geometric_feats.append(self.compute_geom_features(
                    self.images_l_queue[idx_images_l_q][1], self.images_r_queue[idx_images_r_q][1]))
                self.images_rgb_kf.append(
                    self.images_rgb_queue[idx_images_rgb_q][1])

                # Remove previous timestamps in the queues
                del self.images_l_queue[:idx_images_l_q]
                del self.images_r_queue[:idx_images_r_q]
                del self.images_rgb_queue[:idx_images_rgb_q]

                # Reset the counter of KF skipped
                self.nb_kf_skipped = 0

                # Store the original frame id of the kf added
                self.original_ids_of_kf.append(
                    self.orig_id_last_img_in_q - len(self.images_l_queue))

    def find_matches_service(self, find_matches_req):

        rospy.loginfo("Reached service")
        descriptors_to_comp = np.array(
            find_matches_req.netvlad_descriptors).reshape(-1, constants.NETVLAD_DIMS)

        descriptors_vec = []
        kpts3d_vec = []
        kpts_vec = []

        # Find closest descriptors
        if (len(descriptors_to_comp) > 0) and (len(self.descriptors) > 0):
            matches = self.find_matches(descriptors_to_comp)
        else:
            return FindMatchesResponse([], [], [], [], [])

        matches_computing_robot_resp = []
        matches_querying_robot_resp = []
        # Find corresponding visual keypoints and descriptors
        for match in matches:
            resp_feats_and_descs = self.get_geom_features(match[0])

            if not resp_feats_and_descs:
                continue
            matches_computing_robot_resp.append(match[0])
            matches_querying_robot_resp.append(match[1])
            descriptors_vec.append(resp_feats_and_descs.descriptors)
            kpts3d_vec.append(resp_feats_and_descs.kpts3D)
            kpts_vec.append(resp_feats_and_descs.kpts)
        
        # rospy.loginfo("Returning computed matches")
        # rospy.loginfo(matches_local_resp)
        # rospy.loginfo(matches_other_resp)
        # rospy.loginfo(matches_local_resp[0])
        # rospy.loginfo(matches_other_resp[0])
        # rospy.loginfo("Done returning")

        return FindMatchesResponse(matches_computing_robot_resp, matches_querying_robot_resp, descriptors_vec, kpts3d_vec, kpts_vec)

    def found_separators_local(self, matched_ids_from, matched_ids_to, transform_est_success, separators):
        rospy.loginfo("Separators found using the following KF ids: ")
        rospy.loginfo(matched_ids_from)
        rospy.loginfo(matched_ids_to)

        kept_from_id = []
        kept_to_id = []
        kept_sep = []
        kept_transform_est_success = []
        for i in range(len(matched_ids_from)):
            if transform_est_success[i]:
                kept_from_id.append(matched_ids_from[i])
                kept_to_id.append(matched_ids_to[i])
                kept_sep.append(separators[i])
                kept_transform_est_success.append(transform_est_success[i])
                self.separators_found.append((matched_ids_from[i], matched_ids_to[i], separators[i]))

        try:
            
            self.s_add_seps_pose_graph(self.local_robot_id, kept_from_id,
                                       kept_to_id, kept_transform_est_success, kept_sep)
        except rospy.ServiceException, e:
            print "Service call add sep to pose graph failed: %s" % e

        

            # Don't need to add here since this is not the robot computing the matches
            # self.local_kf_already_used.append(matched_ids_from[i])
            # self.other_kf_already_used.append(matched_ids_to[i])
        

    def receive_separators_service(self, receive_separators_req):
        rospy.loginfo("Reached receiving separators service, with the following KF ids")
        rospy.loginfo(receive_separators_req.matched_ids_from)
        rospy.loginfo(receive_separators_req.matched_ids_to)

        kept_from_id = []
        kept_to_id = []
        kept_sep = []
        kept_transform_est_success = []

        for i in range(len(receive_separators_req.matched_ids_from)):
            if receive_separators_req.transform_est_success[i]:

                kept_from_id.append(receive_separators_req.matched_ids_from[i])
                kept_to_id.append(receive_separators_req.matched_ids_to[i])
                kept_sep.append(receive_separators_req.separators[i])
                kept_transform_est_success.append(receive_separators_req.transform_est_success[i])

                self.separators_found.append(
                    (receive_separators_req.matched_ids_to[i], receive_separators_req.matched_ids_from[i], receive_separators_req.separators[i]))
                self.local_kf_already_used.append(
                    receive_separators_req.matched_ids_to[i])
                self.other_kf_already_used.append(
                    receive_separators_req.matched_ids_from[i])
            else:
                self.add_kf_pairs_to_ignore(receive_separators_req.matched_ids_to[i],receive_separators_req.matched_ids_from[i])

        # Add the separator to the factor graph
        try:
            # self.s_add_seps_pose_graph(receive_separators_req)
            self.s_add_seps_pose_graph(receive_separators_req.robot_computed_transform_id, kept_from_id,
                                       kept_to_id, kept_transform_est_success, kept_sep)
        except rospy.ServiceException, e:
            print "Service call add sep to pose graph failed: %s" % e

       



        rospy.loginfo("Currently found " +
                      str(len(self.separators_found))+" separators")
        return ReceiveSeparatorsResponse(True)

    def get_geom_features(self, id):
        return self.geometric_feats[id]

    def compute_geom_features(self, image_l, image_r):
        img_l = cv2.cvtColor(image_l, cv2.COLOR_RGB2GRAY)
        img_r = cv2.cvtColor(image_r, cv2.COLOR_RGB2GRAY)
        img_l_msg = self.bridge.cv2_to_imgmsg(img_l, encoding="mono8")
        img_r_msg = self.bridge.cv2_to_imgmsg(img_r, encoding="mono8")

        try:
            resp_feats_and_descs = self.s_get_feats(img_l_msg, img_r_msg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            resp_feats_and_descs = []
        return resp_feats_and_descs

    # def check_already_matched(match_id):
    #     if match_id in self.kf_already_used:
    #         return True
    #     else:
    #         return False

    def add_kf_pairs_to_ignore(self, id_local, id_other):
        self.kf_pairs_ignored.append([id_local, id_other])
