from cv_bridge import CvBridge, CvBridgeError
import cv2
import constants
import tensorflow as tf
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets
import numpy

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

    def save_image_l(image_l):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_l, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.images_l.append(cv_image)

        # Compute descriptors (only done if enough images ready)
        self.compute_descriptors()

    def save_image_r(image_r):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_r, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.images_r.append(cv_image)

    def compute_descriptors():
        # Check if enough data to fill a batch
        if len(images_l) - len(descriptors) >= constants.BATCH_SIZE:
            # If so, compute and store descriptors
            batch = [np.expand_dims(inim, axis=0) for inim in images_l[len(descriptors):]]
            descriptors = self.sess.run(self.net_out, feed_dict={self.image_batch: batch})

            # Add descriptors to list

    def find_matches(descriptors_to_comp):
        # find closest matches between self.descriptors and descriptors_to_comp, use scipy.spatial.distance.cdist
        matches = []
        return matches


    def get_images(image_id):
        return self.image_l[image_id], self.image_r[image_id]


    def save_separator(transform, local_frame_id, other_robot_id, other_robot_frame_id):
        self.separators_found.append(
            (transform, local_frame_id, other_robot_id, other_robot_frame_id))

