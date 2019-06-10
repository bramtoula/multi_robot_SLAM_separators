#!/usr/bin/env python3
import rospy

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
import numpy as np
import tensorflow as tf
import glob
import scipy
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets
from std_msgs.msg import String


def test():
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
                tf.reset_default_graph()

                image_batch = tf.placeholder(
                        dtype=tf.float32, shape=[None, None, None, 3])

                net_out = nets.vgg16NetvladPca(image_batch)
                saver = tf.train.Saver()

                sess = tf.Session()
                saver.restore(sess, nets.defaultCheckpoint())

                # images = [cv2.imread(file) for file in glob.glob("/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/datasets/kitti/00_color/image_2/00000*.png")]
                # images = [cv2.cvtColor(image, cv2.COLOR_BGR2RGB) for image in images]
                # print(images)
                inim = cv2.imread(nfm.exampleImgPath())
                batch = np.expand_dims(inim,axis=0)
                # inim = cv2.imread(nfm.exampleImgPath())
                # inim = cv2.cvtColor(inim, cv2.COLOR_BGR2RGB)
                # batch = [np.expand_dims(inim, axis=0) for inim in images]
                result = sess.run(net_out, feed_dict={image_batch: batch})

                hello_str = "hello world %s" % rospy.get_time()
                rospy.loginfo(result)
                pub.publish(result)
                rate.sleep()



if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
