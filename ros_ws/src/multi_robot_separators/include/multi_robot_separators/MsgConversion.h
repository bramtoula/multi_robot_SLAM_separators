#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <multi_robot_separators/Descriptors.h>
#include <multi_robot_separators/KeyPoint3DVec.h>
#include <multi_robot_separators/KeyPointVec.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Transform.h>

std::vector<cv::Point3f> keypoints3DFromROS(const multi_robot_separators::KeyPoint3DVec &msg);
void keypoints3DToROS(const std::vector<cv::Point3f> &kpts, multi_robot_separators::KeyPoint3DVec &msg);

std::vector<cv::KeyPoint> keypointsFromROS(const multi_robot_separators::KeyPointVec &msg);
void keypointsToROS(const std::vector<cv::KeyPoint> &kpts, multi_robot_separators::KeyPointVec &msg);

void transformToPoseMsg(const rtabmap::Transform &transform, geometry_msgs::Pose &msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose &msg);

cv::Point3f point3fFromROS(const rtabmap_ros::Point3f &msg);
void point3fToROS(const cv::Point3f &kpt, rtabmap_ros::Point3f &msg);

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint &msg);
void keypointToROS(const cv::KeyPoint &kpt, rtabmap_ros::KeyPoint &msg);

cv::Mat descriptorsFromROS(const multi_robot_separators::Descriptors &msg);
void descriptorsToROS(const cv::Mat &descriptors, multi_robot_separators::Descriptors &msg);