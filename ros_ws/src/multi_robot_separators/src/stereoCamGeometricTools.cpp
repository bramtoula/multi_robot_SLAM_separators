#include "multi_robot_separators/stereoCamGeometricTools.h"

using namespace rtabmap;

StereoCamGeometricTools::StereoCamGeometricTools(const cv::Mat &P_l, const cv::Mat &P_r, const cv::Size imageSize)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    
    cam = StereoCameraModel("stereo_calib",
                            imageSize, P_l.colRange(0, 3), cv::Mat(), cv::Mat(), P_l,
                            imageSize, P_r.colRange(0, 3), cv::Mat(), cv::Mat(), P_r,
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

    covToFloat64Msg(info.covariance, res.poseWithCov.covariance);
    transformToPoseMsg(result2, res.poseWithCov.pose);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_cam_geometric_tools_node");
    ros::NodeHandle n;

    // Get params
    std::vector<double> P_l_param;
    std::vector<double> P_r_param;
    int img_height, img_width;
    if (!n.getParam("image_size/height", img_height))
    {
        ROS_ERROR_STREAM("Height could not be read.");
        return 0;
    }
    if (!n.getParam("image_size/width", img_width))
    {
        ROS_ERROR_STREAM("Width could not be read.");
        return 0;
    }
    if (!n.getParam("P_l", P_l_param))
    {
        ROS_ERROR_STREAM("Left projection matrix could not be read.");
        return 0;
    }

    if (!n.getParam("P_r", P_r_param))
    {
        ROS_ERROR_STREAM("Right projection matrix could not be read.");
        return 0;
    }
    cv::Mat P_l ;
    if (P_l_param.size() == 12) // check that the rows and cols match the size of your vector
    {
        P_l = cv::Mat(P_l_param);
        P_l = P_l.reshape(1,3);
    }
    else
    {
        ROS_ERROR_STREAM("Wrong dimensions of the left calibration matrix");
        return 0;
    }

    cv::Mat P_r ;
    if (P_r_param.size() == 12)
    {
        P_r = cv::Mat(P_r_param);
        P_r = P_r.reshape(1, 3);
    }
    else
    {
        ROS_ERROR_STREAM("Wrong dimensions of the left calibration matrix");
        return 0;
    }

    cv::Size imageSize(img_width,img_height);

    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools(P_l,P_r,imageSize);
    ros::ServiceServer service_feats = n.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::ServiceServer service_transf = n.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ROS_INFO("Ready to do stuff.");
    ros::spin();

    return 0;
}