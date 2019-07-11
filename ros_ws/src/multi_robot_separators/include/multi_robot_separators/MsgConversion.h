#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <multi_robot_separators/Descriptors.h>
#include <multi_robot_separators/KeyPoint3DVec.h>
#include <multi_robot_separators/KeyPointVec.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/StereoCameraModel.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

std::vector<cv::Point3f> keypoints3DFromROS(const multi_robot_separators::KeyPoint3DVec &msg);
void keypoints3DToROS(const std::vector<cv::Point3f> &kpts, multi_robot_separators::KeyPoint3DVec &msg);

std::vector<cv::KeyPoint> keypointsFromROS(const multi_robot_separators::KeyPointVec &msg);
void keypointsToROS(const std::vector<cv::KeyPoint> &kpts, multi_robot_separators::KeyPointVec &msg);

void covToFloat64Msg(const cv::Mat &covariance, boost::array<double, 36ul> &msg);
cv::Mat covFromFloat64Msg(const boost::array<double, 36ul> &msg);

void transformToPoseMsg(const rtabmap::Transform &transform, geometry_msgs::Pose &msg);
rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose &msg);

cv::Point3f point3fFromROS(const rtabmap_ros::Point3f &msg);
void point3fToROS(const cv::Point3f &kpt, rtabmap_ros::Point3f &msg);

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint &msg);
void keypointToROS(const cv::KeyPoint &kpt, rtabmap_ros::KeyPoint &msg);

cv::Mat descriptorsFromROS(const multi_robot_separators::Descriptors &msg);
void descriptorsToROS(const cv::Mat &descriptors, multi_robot_separators::Descriptors &msg);

void covarianceToMatrix(const boost::array<double, 36ul> &msg, gtsam::Matrix &cov_mat_out);

void transformToPose3(const geometry_msgs::Transform &msg, gtsam::Pose3 &pose3_out);
void poseROSToPose3(const geometry_msgs::Pose &msg, gtsam::Pose3 &pose3_out);

rtabmap::CameraModel cameraModelFromROS(
    const sensor_msgs::CameraInfo &camInfo,
    const rtabmap::Transform &localTransform = rtabmap::Transform::getIdentity());
rtabmap::StereoCameraModel stereoCameraModelFromROS(
    const sensor_msgs::CameraInfo &leftCamInfo,
    const sensor_msgs::CameraInfo &rightCamInfo,
    const rtabmap::Transform &localTransform = rtabmap::Transform::getIdentity(),
    const rtabmap::Transform &stereoTransform = rtabmap::Transform());
rtabmap::Transform transformFromTF(const tf::Transform &transform);