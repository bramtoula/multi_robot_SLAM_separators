#include "ros/ros.h"
#include "multi_robot_separators/GetFeatsAndDesc.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "multi_robot_separators/myRegistration.h"
#include "multi_robot_separators/MsgConversion.h"

using namespace rtabmap;

bool getFeaturesAndDescriptor(multi_robot_separators::GetFeatsAndDesc::Request &req,
                              multi_robot_separators::GetFeatsAndDesc::Response &res)
{

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    FILE *pFile = 0;
    std::string pathCalib = "/kitti_data_orig/calib.txt";

    pFile = fopen(pathCalib.c_str(), "r");
    if (!pFile)
    {
        UERROR("Cannot open calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    cv::Mat_<double> P0(3, 4);
    cv::Mat_<double> P1(3, 4);
    cv::Mat_<double> P2(3, 4);
    cv::Mat_<double> P3(3, 4);
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P0(0, 0), &P0(0, 1), &P0(0, 2), &P0(0, 3),
               &P0(1, 0), &P0(1, 1), &P0(1, 2), &P0(1, 3),
               &P0(2, 0), &P0(2, 1), &P0(2, 2), &P0(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P1(0, 0), &P1(0, 1), &P1(0, 2), &P1(0, 3),
               &P1(1, 0), &P1(1, 1), &P1(1, 2), &P1(1, 3),
               &P1(2, 0), &P1(2, 1), &P1(2, 2), &P1(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P2(0, 0), &P2(0, 1), &P2(0, 2), &P2(0, 3),
               &P2(1, 0), &P2(1, 1), &P2(1, 2), &P2(1, 3),
               &P2(2, 0), &P2(2, 1), &P2(2, 2), &P2(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P3(0, 0), &P3(0, 1), &P3(0, 2), &P3(0, 3),
               &P3(1, 0), &P3(1, 1), &P3(1, 2), &P3(1, 3),
               &P3(2, 0), &P3(2, 1), &P3(2, 2), &P3(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
        return -1;
    }
    fclose(pFile);

    cv::Mat image = cv::imread("kitti_data_orig/image_2/000000.png");

    StereoCameraModel cam("stereo_calib",
                          image.size(), P2.colRange(0, 3), cv::Mat(), cv::Mat(), P2,
                          image.size(), P3.colRange(0, 3), cv::Mat(), cv::Mat(), P3,
                          cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat());
    // StereoCameraModel cam(718.856, 718.856, 607.1928, 185.2157, 0.54, Transform::getIdentity(), cv::Size(1241,376));
    // cv::Mat img_l_1 = cv::imread("/kitti_data_orig/image_2/000000.png", CV_8UC1);
    // cv::Mat img_l_2 = cv::imread("/kitti_data_orig/image_2/000003.png", CV_8UC1);
    // cv::Mat img_r_1 = cv::imread("/kitti_data_orig/image_3/000000.png", CV_8UC1);
    // cv::Mat img_r_2 = cv::imread("/kitti_data_orig/image_3/000003.png", CV_8UC1);

    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr = cv_bridge::toCvShare(req.image_left, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat img_l = cv_bridge::toCvCopy(req.image_left,sensor_msgs::image_encodings::MONO8)->image;
    // cv_ptr = cv_bridge::toCvShare(req.image_right, sensor_msgs::image_encodings::MONO8); //->image;
    cv::Mat img_r = cv_bridge::toCvCopy(req.image_right,sensor_msgs::image_encodings::MONO8)->image;

    ParametersMap params;

    Registration *_registrationPipeline;
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpCorrespondenceRatio(), "0.1"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpIterations(), "10"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.3"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxTranslation(), "0"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlane(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpVoxelSize(), "0"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpEpsilon(), "0.01"));

    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), "1"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "2"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorFlowWinSize(), "16"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorType(), "0"));

    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), "0"));

    _registrationPipeline = Registration::create(params);

    RegistrationInfo info;

    Transform guess(0.0, 0.0, 0.0, 0, 0, 0);

    SensorData frame(img_l, img_r, cam);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::Point3f> kptsFrom3D;
    cv::Mat descriptorsFrom;

    Signature frameSig(frame);

    _registrationPipeline->getFeatures(kptsFrom3D, kptsFrom, descriptorsFrom, frameSig, &info);

    descriptorsToROS(descriptorsFrom, res.descriptors);
    keypointsToROS(kptsFrom, res.kpts);
    keypoints3DToROS(kptsFrom3D, res.kpts3D);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_feats_and_desc_server");
    ros::NodeHandle n;
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    FILE *pFile = 0;
    ros::ServiceServer service = n.advertiseService("get_features_and_descriptor", getFeaturesAndDescriptor);
    ROS_INFO("Ready to estimate transform.");
    ros::spin();

    return 0;
}