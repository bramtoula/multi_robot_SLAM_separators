#include "multi_robot_separators/stereoCamGeometricTools.h"

using namespace rtabmap;

StereoCamGeometricTools::StereoCamGeometricTools(/* args */)
{
    ROS_INFO("In constructor");
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    FILE *pFile = 0;
    std::string pathCalib = "/kitti_data_orig/calib.txt";
    pFile = fopen(pathCalib.c_str(), "r");
    if (!pFile)
    {
        UERROR("Cannot open calibration file \"%s\"", pathCalib.c_str());
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
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P1(0, 0), &P1(0, 1), &P1(0, 2), &P1(0, 3),
               &P1(1, 0), &P1(1, 1), &P1(1, 2), &P1(1, 3),
               &P1(2, 0), &P1(2, 1), &P1(2, 2), &P1(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P2(0, 0), &P2(0, 1), &P2(0, 2), &P2(0, 3),
               &P2(1, 0), &P2(1, 1), &P2(1, 2), &P2(1, 3),
               &P2(2, 0), &P2(2, 1), &P2(2, 2), &P2(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
    }
    if (fscanf(pFile, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &P3(0, 0), &P3(0, 1), &P3(0, 2), &P3(0, 3),
               &P3(1, 0), &P3(1, 1), &P3(1, 2), &P3(1, 3),
               &P3(2, 0), &P3(2, 1), &P3(2, 2), &P3(2, 3)) != 12)
    {
        UERROR("Failed to parse calibration file \"%s\"", pathCalib.c_str());
    }

    fclose(pFile);

    cv::Mat image = cv::imread("kitti_data_orig/image_2/000000.png");
    imageSize = image.size();
    cam = StereoCameraModel("stereo_calib",
                            image.size(), P2.colRange(0, 3), cv::Mat(), cv::Mat(), P2,
                            image.size(), P3.colRange(0, 3), cv::Mat(), cv::Mat(), P3,
                            cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat());
    ParametersMap params;
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
}

bool StereoCamGeometricTools::getFeaturesAndDescriptor(multi_robot_separators::GetFeatsAndDesc::Request &req,
                                                       multi_robot_separators::GetFeatsAndDesc::Response &res)
{

    cv::Mat img_l = cv_bridge::toCvCopy(req.image_left, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat img_r = cv_bridge::toCvCopy(req.image_right, sensor_msgs::image_encodings::MONO8)->image;

    SensorData frame(img_l, img_r, cam);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::Point3f> kptsFrom3D;
    cv::Mat descriptorsFrom;
    RegistrationInfo info1;
    Signature frameSig(frame);
    _registrationPipeline->getFeatures(kptsFrom3D, kptsFrom, descriptorsFrom, frameSig, &info1);

    descriptorsToROS(descriptorsFrom, res.descriptors);
    keypointsToROS(kptsFrom, res.kpts);
    keypoints3DToROS(kptsFrom3D, res.kpts3D);
    return true;
}

bool StereoCamGeometricTools::estimateTransformation(multi_robot_separators::EstTransform::Request &req,
                                                     multi_robot_separators::EstTransform::Response &res)
{

    Transform guess(0.0, 0.0, 0.0, 0, 0, 0);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::KeyPoint> kptsTo;
    std::vector<cv::Point3f> kptsFrom3D;
    std::vector<cv::Point3f> kptsTo3D;
    cv::Mat descriptorsFrom;
    cv::Mat descriptorsTo;

    kptsFrom3D = keypoints3DFromROS(req.kptsFrom3D);
    kptsTo3D = keypoints3DFromROS(req.kptsTo3D);
    kptsFrom = keypointsFromROS(req.kptsFrom);
    kptsTo = keypointsFromROS(req.kptsTo);
    descriptorsFrom = descriptorsFromROS(req.descriptorsFrom);
    descriptorsTo = descriptorsFromROS(req.descriptorsTo);
    Transform result = _registrationPipeline->computeTransformationFromFeats(
        cam,
        cam,
        descriptorsFrom,
        descriptorsTo,
        imageSize,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        guess,
        &info);
    Transform result2 = _registrationPipeline->computeTransformationFromFeats(
        cam,
        cam,
        descriptorsFrom,
        descriptorsTo,
        imageSize,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        result,
        &info);

    printf("Mine %s\n", result2.prettyPrint().c_str());

    covToFloat64Msg(info.covariance, res.poseWithCov.covariance);
    transformToPoseMsg(result2, res.poseWithCov.pose);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_cam_geometric_tools_node");
    ros::NodeHandle n;
    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools();
    ros::ServiceServer service_feats = n.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::ServiceServer service_transf = n.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ROS_INFO("Ready to do stuff.");
    ros::spin();

    return 0;
}