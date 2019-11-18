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
import collections
import copy
from scipy.spatial.distance import cdist

from bisect import bisect_left

# From https://stackoverflow.com/questions/12141150/from-list-of-integers-get-number-closest-to-a-given-value/12141511#12141511
def takeClosest(myList, myNumber):
    """
    Assumes myList is sorted. Returns closest value to myNumber.

    If two numbers are equally close, return the smallest number.
    """
    pos = bisect_left(myList, myNumber)
    if pos == 0:
        return myList[0], 0
    if pos == len(myList):
        return myList[-1], len(myList)-1
    before = myList[pos - 1]
    after = myList[pos]
    if after - myNumber < myNumber - before:
       return after, pos
    else:
       return before, pos-1

class DataHandler:
    def __init__(self):
        self.images_l_queue = collections.deque()
        self.images_r_queue = collections.deque()
        self.images_rgb_queue = collections.deque()
        self.geometric_feats = collections.deque()
        self.images_rgb_kf = collections.deque()
        self.timestamps_kf = collections.deque()
        self.local_descriptors = []
        self.nb_descriptors_already_sent = 0
        self.received_descriptors = []
        self.separators_found = collections.deque()
        self.local_kf_already_used = collections.deque()
        self.other_kf_already_used = collections.deque()
        self.frames_kept_pairs_ignored = collections.deque()
        self.nb_kf_skipped = 0
        self.original_ids_of_kf = collections.deque()
        self.orig_id_last_img_in_q = 0
        self.nb_kf_odom = 0
        self.kf_ids_of_frames_kept = collections.deque()

        tf.reset_default_graph()
        self.image_batch = tf.placeholder(
            dtype=tf.float32, shape=[None, None, None, 3])

        self.net_out = nets.vgg16NetvladPca(self.image_batch)

        saver = tf.train.Saver()

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=config)
        saver.restore(self.sess, nets.defaultCheckpoint())

        self.bridge = CvBridge()

        self.local_robot_id = rospy.get_param("local_robot_id")
        self.other_robot_id = rospy.get_param("other_robot_id")
        self.log_gps = rospy.get_param("log_gps")

        if self.log_gps:
            from dji_sdk.msg import GlobalPosition
            rospy.Subscriber("gps_topic", GlobalPosition,
                                 self.save_gps_queue)
            self.gps_data_queue = collections.deque()

        self.s_add_seps_pose_graph = rospy.ServiceProxy(
            'add_separators_pose_graph', ReceiveSeparators)
        self.s_get_feats = rospy.ServiceProxy(
            'get_features_and_descriptor', GetFeatsAndDesc)
        self.logs_location = rospy.get_param("logs_location")

        self.send_estimates_of_poses = rospy.get_param("use_estimates_of_poses")
        if self.send_estimates_of_poses:
            self.s_get_pose_estimates = rospy.ServiceProxy('get_pose_estimates',PoseEstimates)


        # Read params
        self.netvlad_distance = rospy.get_param("netvlad_distance")
        self.netvlad_dimensions = rospy.get_param("netvlad_dimensions")
        self.netvlad_batch_size = rospy.get_param("netvlad_batch_size")
        self.netvlad_max_matches_nb = rospy.get_param("netvlad_max_matches_nb")
        self.number_of_kf_skipped = rospy.get_param("number_of_kf_skipped")
        # Log params to file
        with open(self.logs_location+'params_'+str(self.local_robot_id)+'.txt', 'a') as file:
            file.write('netvlad_distance: '+str(self.netvlad_distance)+'\nnetvlad_dimensions: ' +
                       str(self.netvlad_dimensions)+'\nnetvlad_batch_size: '+str(self.netvlad_batch_size)+'\nnetvlad_max_matches_nb: '+str(self.netvlad_max_matches_nb)+'\nnumber_of_kf_skipped: '+str(self.number_of_kf_skipped)+'\nseparators_min_inliers: '+str(rospy.get_param("separators_min_inliers"))+'\n')

    def __del__(self):
        with open(self.logs_location+'kf_orig_ids_'+str(self.local_robot_id)+'.txt', 'w') as file:
            for id in self.original_ids_of_kf:
                file.write("%i\n" % id)
        self.sess.close()
        rospy.loginfo('Close TF session') 
        

    def save_image_l(self, image_l):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_l, "rgb8")
        except CvBridgeError as e:
            print(e)

        self.images_l_queue.append((image_l.header.stamp.to_sec(), cv_image))
        self.orig_id_last_img_in_q += 1
        if len(self.images_l_queue) > constants.MAX_QUEUE_SIZE:
            self.images_l_queue.popleft()

    def save_image_r(self, image_r):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_r, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_r_queue.append((image_r.header.stamp.to_sec(), cv_image))
        if len(self.images_r_queue) > constants.MAX_QUEUE_SIZE:
            self.images_r_queue.popleft()

    def save_image_rgb(self, image_rgb):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_rgb, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.images_rgb_queue.append((image_rgb.header.stamp.to_sec(), cv_image))
        if len(self.images_rgb_queue) > constants.MAX_QUEUE_SIZE:
            self.images_rgb_queue.popleft()

    def compute_descriptors(self):
        # Check if enough data to fill a batch
        # if len(self.images_l_kf) - len(self.descriptors) >= constants.BATCH_SIZE:
        rospy.loginfo("Computing descriptors. Currently already computed " +
                      str(len(self.local_descriptors))+" netvlad descriptors. Number of frames left in the queue:"+str(len(self.images_rgb_kf)))
        # If so, compute and store descriptors (as much as we can up to the batch size)
        nb_images_batch = min(len(self.images_rgb_kf), self.netvlad_batch_size)
        batch = collections.deque(self.images_rgb_kf[i] for i in range(0, nb_images_batch)) # self.images_rgb_kf[:nb_images_batch]

        if len(batch) > 0:
            descriptors = self.sess.run(self.net_out, feed_dict={
                                        self.image_batch: batch})

            rospy.loginfo("Saving descriptors")
            self.local_descriptors.extend(
                descriptors[:, :self.netvlad_dimensions].tolist())

            # Remove rgb images from memory
            for i in range(0, nb_images_batch):
                self.images_rgb_kf.popleft() # self.images_rgb_kf[nb_images_batch:]
        else:
            rospy.loginfo("Empty batch, no images to compute descriptors for")

    def find_matches(self):
        # find closest matches between self.descriptors and self.received_descriptors, use scipy.spatial.distance.cdist
        local_descs = np.array(self.local_descriptors)
        received_descs = np.array(self.received_descriptors)
        distances = cdist(local_descs, received_descs)

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

        for pair in self.frames_kept_pairs_ignored:
            distances[pair[0], pair[1]] = np.inf


        indexes_smallest_values_each_frame = np.argsort(distances, axis=1)[:,0]
        smallest_values_each_frame = distances[np.arange(
            len(distances)), indexes_smallest_values_each_frame]

        indexes_smallest_values_all_frames = np.argsort(smallest_values_each_frame)

        matches = collections.deque()
        for i in range(min(len(indexes_smallest_values_all_frames), self.netvlad_max_matches_nb)):
            idx_local = indexes_smallest_values_all_frames[i]
            idx_other = indexes_smallest_values_each_frame[indexes_smallest_values_all_frames[i]]

            # Check if other frame index already used
            if idx_other in [match[1] for match in matches]:
                continue
        
            if distances[idx_local, idx_other] < self.netvlad_distance:
                matches.append((idx_local, idx_other))
            else:
                break
        rospy.logwarn(distances)
        rospy.loginfo("Matches found:")
        rospy.loginfo(matches)
        return matches


    def get_keyframes(self, odom_info):
        rospy.loginfo("Size of images queue: " +
                      str(len(self.images_l_queue))+"\n")
        if odom_info.keyFrameAdded:
            self.nb_kf_odom += 1

            if self.nb_kf_skipped < self.number_of_kf_skipped:
                self.nb_kf_skipped += 1
            else:
                # Look for the index of the saved image corresponding to the timestamp
                try:
                    time_ref = odom_info.header.stamp.to_sec()
                    
                    #idx_images_l_q = [y[0] for y in self.images_l_queue].index(time_ref)
                    #idx_images_r_q = [y[0] for y in self.images_r_queue].index(time_ref)
                    # Find closest time stamps for the left and right images
                    stamp, pos = takeClosest([y[0] for y in self.images_l_queue], time_ref)
                    if np.abs(stamp-time_ref) > (constants.MAX_TIME_DIFF_RGB_STEREO):
                        rospy.logwarn(
                            "Keyframe stereo timestamps too far from the left timestamps")
                        return
                    else:
                        idx_images_l_q = pos

                    stamp, pos = takeClosest([y[0] for y in self.images_r_queue], time_ref)
                    if np.abs(stamp-time_ref) > (constants.MAX_TIME_DIFF_RGB_STEREO):
                        rospy.logwarn(
                            "Keyframe stereo timestamps too far from the left timestamps")
                        return
                    else:
                        idx_images_r_q = pos
                    
                    # Find closest time stamps of the rgb data                    
                    rgb_stamps = [y[0] for y in self.images_rgb_queue]
                    stamp, pos = takeClosest(rgb_stamps, time_ref)
                    if np.abs(stamp-time_ref) > (constants.MAX_TIME_DIFF_RGB_STEREO):
                        rospy.logwarn(
                            "Keyframe stereo timestamps too far from the RGB timestamps")
                        return
                    else:
                        idx_images_rgb_q = pos
                except:
                    rospy.logwarn(
                        "Keyframe timestamp not found in the saved images queue")
                    return

                self.timestamps_kf.append(odom_info.header.stamp)
                # Save geometric features and descs of keyframe images
                geometric_feats = self.compute_geom_features(self.images_l_queue[idx_images_l_q][1], self.images_r_queue[idx_images_r_q][1])

                # Make sure we were able to compute features
                if not geometric_feats:
                    return

                rospy.loginfo("Adding a keyframe and cleaning queue")

                self.geometric_feats.append(geometric_feats)
                self.images_rgb_kf.append(
                    self.images_rgb_queue[idx_images_rgb_q][1])

                # Remove previous timestamps in the queues
                #del self.images_l_queue[:idx_images_l_q]
                #del self.images_r_queue[:idx_images_r_q]
                #del self.images_rgb_queue[:idx_images_rgb_q]
                for i in range(0, idx_images_l_q):
                    self.images_l_queue.popleft()
                for i in range(0, idx_images_r_q):
                    self.images_r_queue.popleft()
                for i in range(0, idx_images_rgb_q):
                    self.images_rgb_queue.popleft()

                # Reset the counter of KF skipped
                self.nb_kf_skipped = 0

                # Keep the id based on all the kf seen by the odometry
                self.kf_ids_of_frames_kept.append(self.nb_kf_odom-1)

                # Log gps
                if self.log_gps:
                    self.log_gps_data(self.nb_kf_odom-1, odom_info.header.stamp)

                # Store the original frame id of the kf added
                self.original_ids_of_kf.append(
                    self.orig_id_last_img_in_q - len(self.images_l_queue))

    def find_matches_service(self, find_matches_req):
        rospy.loginfo("Reached service")

        self.received_descriptors.extend(np.array(
            find_matches_req.new_netvlad_descriptors).reshape(-1, self.netvlad_dimensions).tolist())

        descriptors_vec = collections.deque()
        kpts3d_vec = collections.deque()
        kpts_vec = collections.deque()

        # Find closest descriptors
        if (len(self.received_descriptors) > 0) and (len(self.local_descriptors) > 0):
            matches = self.find_matches()
        else:
            return FindMatchesResponse(collections.deque(), collections.deque(), collections.deque(), collections.deque(), collections.deque(), collections.deque(), collections.deque())

        matches_computing_robot_resp = collections.deque()
        matches_querying_robot_resp = collections.deque()
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
        
        kf_matched_ids = self.get_kf_ids_from_frames_kept_ids(
            matches_computing_robot_resp)

        pose_estimates = collections.deque()
        if self.send_estimates_of_poses:
            try:
                pose_estimates = self.s_get_pose_estimates(kf_matched_ids).pose_estimates
            except rospy.ServiceException, e:
                print "Service call pose_estimates failed: %s" % e
            
        return FindMatchesResponse(kf_matched_ids, matches_computing_robot_resp, matches_querying_robot_resp, descriptors_vec, kpts3d_vec, kpts_vec, pose_estimates)

    def found_separators_local(self, kf_ids_from, kf_ids_to, frames_kept_ids_from, frames_kept_ids_to, pose_estimates_from, pose_estimates_to, transform_est_success, separators):
        rospy.loginfo("Separators found using the following KF ids: ")
        rospy.loginfo(kf_ids_from)
        rospy.loginfo(kf_ids_to)

        kept_frames_kept_from_id = collections.deque()
        kept_frames_kept_to_id = collections.deque()
        kept_kf_from_id = collections.deque()
        kept_kf_to_id = collections.deque()
        kept_sep = collections.deque()
        kept_pose_est_from = collections.deque()
        kept_pose_est_to = collections.deque()
        kept_transform_est_success = collections.deque()
        for i in range(len(kf_ids_from)):
            if transform_est_success[i]:
                kept_kf_from_id.append(kf_ids_from[i])
                kept_kf_to_id.append(kf_ids_to[i])
                kept_sep.append(separators[i])
                kept_transform_est_success.append(transform_est_success[i])
                kept_frames_kept_from_id.append(frames_kept_ids_from[i])
                kept_frames_kept_to_id.append(frames_kept_ids_to[i])
                if self.send_estimates_of_poses:
                    kept_pose_est_from.append(pose_estimates_from[i])
                    kept_pose_est_to.append(pose_estimates_to[i])
                self.separators_found.append((kf_ids_from[i], kf_ids_to[i], separators[i]))

        try:
            rospy.logwarn("Adding to myself")
            self.s_add_seps_pose_graph(self.local_robot_id, self.other_robot_id, kept_kf_from_id,
                                       kept_kf_to_id, kept_frames_kept_from_id, kept_frames_kept_to_id, kept_pose_est_from, kept_pose_est_to, kept_transform_est_success, kept_sep)
        except rospy.ServiceException, e:
            print "Service call add sep to pose graph failed: %s" % e
        

    def receive_separators_service(self, receive_separators_req):
        rospy.loginfo("Reached receiving separators service, with the following KF ids")
        rospy.loginfo(receive_separators_req.kf_ids_from)
        rospy.loginfo(receive_separators_req.kf_ids_to)

        kept_frames_kept_from_id = collections.deque()
        kept_frames_kept_to_id = collections.deque()
        kept_kf_from_id = collections.deque()
        kept_kf_to_id = collections.deque()
        kept_sep = collections.deque()
        kept_pose_est_from = collections.deque()
        kept_pose_est_to = collections.deque()
        kept_transform_est_success = collections.deque()

        for i in range(len(receive_separators_req.kf_ids_from)):
            if receive_separators_req.transform_est_success[i]:
                
                kept_frames_kept_from_id.append(receive_separators_req.frames_kepts_ids_from[i])
                kept_frames_kept_to_id.append(receive_separators_req.frames_kepts_ids_to[i])
                kept_kf_from_id.append(receive_separators_req.kf_ids_from[i])
                kept_kf_to_id.append(receive_separators_req.kf_ids_to[i])
                kept_sep.append(receive_separators_req.separators[i])
                kept_transform_est_success.append(receive_separators_req.transform_est_success[i])
                if self.send_estimates_of_poses:
                    kept_pose_est_from.append(receive_separators_req.pose_estimates_from[i])
                    kept_pose_est_to.append(receive_separators_req.pose_estimates_to[i])

                self.separators_found.append(
                    (receive_separators_req.kf_ids_to[i], receive_separators_req.kf_ids_from[i], receive_separators_req.separators[i]))
                self.local_kf_already_used.append(
                    receive_separators_req.frames_kepts_ids_to[i])
                self.other_kf_already_used.append(
                    receive_separators_req.frames_kepts_ids_from[i])
            else:
                self.add_frames_kept_pairs_to_ignore(
                    receive_separators_req.frames_kepts_ids_to[i], receive_separators_req.frames_kepts_ids_from[i])

        # Add the separator to the factor graph
        try:
            rospy.logwarn("Adding to other")
            self.s_add_seps_pose_graph(receive_separators_req.robot_from_id, receive_separators_req.robot_to_id, kept_kf_from_id, kept_kf_to_id, kept_frames_kept_from_id, kept_frames_kept_to_id, kept_pose_est_from, kept_pose_est_to, kept_transform_est_success, kept_sep)
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
            print "Service call get_features_and_descriptor failed: %s" % e
            resp_feats_and_descs = []
        return resp_feats_and_descs

    def add_frames_kept_pairs_to_ignore(self, id_local, id_other):
        self.frames_kept_pairs_ignored.append([id_local, id_other])

    def get_kf_ids_from_frames_kept_ids(self,frames_kept_ids):
        return list(np.array(self.kf_ids_of_frames_kept)[frames_kept_ids])

    def save_gps_queue(self,gps_data):
        self.gps_data_queue.append((gps_data.header.stamp.to_sec(), gps_data))
        if len(self.gps_data_queue) > constants.MAX_GPS_QUEUE_SIZE:
            self.gps_data_queue.popleft()

    def log_gps_data(self,kf_id, kf_stamp):

        time_ref = kf_stamp.to_sec()
        # Find closest time stamps of the rgb data
        gps_queue = copy.deepcopy(self.gps_data_queue)
        gps_stamps = [y[0] for y in gps_queue]
        if not gps_stamps:
            with open(self.logs_location+'gps_of_kfs_full_robot_'+str(self.local_robot_id)+'.txt', 'a') as file:
                file.write('kf_id: '+str(kf_id)+'\n-1\n')
            with open(self.logs_location+'gps_of_kfs_short_robot_'+str(self.local_robot_id)+'.txt', 'a') as file:
                file.write('-1 -1 -1 -1\n')
            return
        stamp, pos = takeClosest(gps_stamps, time_ref)
        time_diff = np.abs(stamp-time_ref)
        gps_data_kept = gps_queue[pos][1]
        with open(self.logs_location+'gps_of_kfs_full_robot_'+str(self.local_robot_id)+'.txt', 'a') as file:
            file.write('kf_id: '+str(kf_id)+'\ntime_diff: ' +
                       str(time_diff)+'\n'+str(gps_data_kept)+'\n')
        
        with open(self.logs_location+'gps_of_kfs_short_robot_'+str(self.local_robot_id)+'.txt', 'a') as file:
            file.write(str(stamp)+' '+str(gps_data_kept.latitude)+' '+str(gps_data_kept.longitude)+' '+str(gps_data_kept.altitude)+'\n')

