#include "multi_robot_separators/MsgConversion.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
// adapted from https://github.com/introlab/rtabmap_ros/blob/master/src/MsgConversion.cpp
std::vector<cv::Point3f> keypoints3DFromROS(const multi_robot_separators::KeyPoint3DVec &msg)
{
    std::vector<cv::Point3f> v(msg.size);

    for (unsigned int i = 0; i < msg.size; ++i)
    {

        v[i] = point3fFromROS(msg.kpts3DVec[i]);
    }
    return v;
}

void keypoints3DToROS(const std::vector<cv::Point3f> &kpts, multi_robot_separators::KeyPoint3DVec &msg)
{
    msg.size = kpts.size();
    rtabmap_ros::Point3f pt3f;

    for (unsigned int i = 0; i < msg.size; ++i)
    {
        pt3f.x = kpts[i].x;
        pt3f.y = kpts[i].y;
        pt3f.z = kpts[i].z;
        msg.kpts3DVec.push_back(pt3f);
    }
}

std::vector<cv::KeyPoint> keypointsFromROS(const multi_robot_separators::KeyPointVec &msg)
{
    std::vector<cv::KeyPoint> v(msg.size);
    for (unsigned int i = 0; i < msg.size; ++i)
    {
        v[i] = keypointFromROS(msg.kptsVec[i]);
    }
    return v;
}

void keypointsToROS(const std::vector<cv::KeyPoint> &kpts, multi_robot_separators::KeyPointVec &msg)
{
    msg.size = kpts.size();
    rtabmap_ros::KeyPoint kpt_tmp;
    for (unsigned int i = 0; i < msg.size; ++i)
    {
        kpt_tmp.angle = kpts[i].angle;
        kpt_tmp.class_id = kpts[i].class_id;
        kpt_tmp.octave = kpts[i].octave;
        kpt_tmp.pt.x = kpts[i].pt.x;
        kpt_tmp.pt.y = kpts[i].pt.y;
        kpt_tmp.response = kpts[i].response;
        kpt_tmp.size = kpts[i].size;
        msg.kptsVec.push_back(kpt_tmp);
    }
}

void covToFloat64Msg(const cv::Mat &covariance, boost::array<double, 36ul> &msg)
{
    memcpy(&msg, covariance.data, 36 * 8);
}

cv::Mat covFromFloat64Msg(const boost::array<double, 36ul> &msg)
{
    return cv::Mat(6,6, CV_64F, const_cast<double *>(&msg[0]));
}

void transformToPoseMsg(const rtabmap::Transform &transform, geometry_msgs::Pose &msg)
{
    if (!transform.isNull())
    {
        tf::poseEigenToMsg(transform.toEigen3d(), msg);
    }
    else
    {
        msg = geometry_msgs::Pose();
    }
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose &msg)
{
    if (msg.orientation.w == 0 &&
        msg.orientation.x == 0 &&
        msg.orientation.y == 0 &&
        msg.orientation.z == 0)
    {
        return rtabmap::Transform();
    }
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    return rtabmap::Transform::fromEigen3d(tfPose);
}

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint &msg)
{
    return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint &kpt, rtabmap_ros::KeyPoint &msg)
{
    msg.angle = kpt.angle;
    msg.class_id = kpt.class_id;
    msg.octave = kpt.octave;
    msg.pt.x = kpt.pt.x;
    msg.pt.y = kpt.pt.y;
    msg.response = kpt.response;
    msg.size = kpt.size;
}

cv::Mat descriptorsFromROS(const multi_robot_separators::Descriptors &msg)
{
    return cv::Mat(msg.rows, msg.cols, CV_8U, const_cast<uint8_t *> (& msg.data[0]));
}

void descriptorsToROS(const cv::Mat &descriptors, multi_robot_separators::Descriptors &msg)
{
    msg.rows = descriptors.rows;
    msg.cols = descriptors.cols;
    for (int r = 0; r < descriptors.rows; r++)
    {
        for (int c = 0; c < descriptors.cols; c++)
        {
            msg.data.push_back(descriptors.at<uchar>(r, c));
        }
    }
}

cv::Point3f point3fFromROS(const rtabmap_ros::Point3f &msg)
{
    return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f &kpt, rtabmap_ros::Point3f &msg)
{
    msg.x = kpt.x;
    msg.y = kpt.y;
    msg.z = kpt.z;
}

void covarianceToMatrix(const boost::array<double, 36ul> &msg, gtsam::Matrix &cov_mat_out)
{
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            cov_mat_out(row,col) = msg[col+row*6];
        }
    }
}

void transformToPose3(const geometry_msgs::Transform &msg, gtsam::Pose3 &pose3_out)
{
    gtsam::Rot3 rot(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
    gtsam::Point3 pt(msg.translation.x, msg.translation.y, msg.translation.z);
    pose3_out = gtsam::Pose3(rot,pt);
}

void poseROSToPose3(const geometry_msgs::Pose &msg, gtsam::Pose3 &pose3_out)
{
    gtsam::Rot3 rot(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    gtsam::Point3 pt(msg.position.x, msg.position.y, msg.position.z);
    pose3_out = gtsam::Pose3(rot, pt);
}

// Taken from RTAB-Map code
rtabmap::CameraModel cameraModelFromROS(
    const sensor_msgs::CameraInfo &camInfo,
    const rtabmap::Transform &localTransform)
{
    cv::Mat K;
    UASSERT(camInfo.K.empty() || camInfo.K.size() == 9);
    if (!camInfo.K.empty())
    {
        K = cv::Mat(3, 3, CV_64FC1);
        memcpy(K.data, camInfo.K.elems, 9 * sizeof(double));
    }

    cv::Mat D;
    if (camInfo.D.size())
    {
        if (camInfo.D.size() >= 4 &&
            (uStrContains(camInfo.distortion_model, "fisheye") ||
             uStrContains(camInfo.distortion_model, "equidistant")))
        {
            D = cv::Mat::zeros(1, 6, CV_64FC1);
            D.at<double>(0, 0) = camInfo.D[0];
            D.at<double>(0, 1) = camInfo.D[1];
            D.at<double>(0, 4) = camInfo.D[2];
            D.at<double>(0, 5) = camInfo.D[3];
        }
        else
        {
            D = cv::Mat(1, camInfo.D.size(), CV_64FC1);
            memcpy(D.data, camInfo.D.data(), D.cols * sizeof(double));
        }
    }

    cv::Mat R;
    UASSERT(camInfo.R.empty() || camInfo.R.size() == 9);
    if (!camInfo.R.empty())
    {
        R = cv::Mat(3, 3, CV_64FC1);
        memcpy(R.data, camInfo.R.elems, 9 * sizeof(double));
    }

    cv::Mat P;
    UASSERT(camInfo.P.empty() || camInfo.P.size() == 12);
    if (!camInfo.P.empty())
    {
        P = cv::Mat(3, 4, CV_64FC1);
        memcpy(P.data, camInfo.P.elems, 12 * sizeof(double));
    }

    return rtabmap::CameraModel(
        "ros",
        cv::Size(camInfo.width, camInfo.height),
        K, D, R, P,
        localTransform);
}

rtabmap::StereoCameraModel stereoCameraModelFromROS(
    const sensor_msgs::CameraInfo &leftCamInfo,
    const sensor_msgs::CameraInfo &rightCamInfo,
    const rtabmap::Transform &localTransform,
    const rtabmap::Transform &stereoTransform)
{
    return rtabmap::StereoCameraModel(
        "ros",
        cameraModelFromROS(leftCamInfo, localTransform),
        cameraModelFromROS(rightCamInfo, localTransform),
        stereoTransform);
}

rtabmap::Transform transformFromTF(const tf::Transform &transform)
{
    Eigen::Affine3d eigenTf;
    tf::transformTFToEigen(transform, eigenTf);
    return rtabmap::Transform::fromEigen3d(eigenTf);
}
