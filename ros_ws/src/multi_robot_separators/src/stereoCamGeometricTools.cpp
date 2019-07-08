#include "multi_robot_separators/stereoCamGeometricTools.h"

using namespace rtabmap;

void _PrintMatrix(char *pMessage, cv::Mat &mat)
{
    printf("%s\n", pMessage);

    for (int r = 0; r < mat.rows; r++)
    {
        for (int c = 0; c < mat.cols; c++)
        {

            switch (mat.depth())
            {
            case CV_8U:
            {
                printf("%*u ", 3, mat.at<uchar>(r, c));
                break;
            }
            case CV_8S:
            {
                printf("%*hhd ", 4, mat.at<schar>(r, c));
                break;
            }
            case CV_16U:
            {
                printf("%*hu ", 5, mat.at<ushort>(r, c));
                break;
            }
            case CV_16S:
            {
                printf("%*hd ", 6, mat.at<short>(r, c));
                break;
            }
            case CV_32S:
            {
                printf("%*d ", 6, mat.at<int>(r, c));
                break;
            }
            case CV_32F:
            {
                printf("%*.4f ", 10, mat.at<float>(r, c));
                break;
            }
            case CV_64F:
            {
                printf("%*.4f ", 10, mat.at<double>(r, c));
                break;
            }
            }
        }
        printf("\n");
    }
    printf("\n");
}

StereoCamGeometricTools::StereoCamGeometricTools(sensor_msgs::CameraInfo &camera_info_l, sensor_msgs::CameraInfo &camera_info_r)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);

    cv::Size image_size_l(camera_info_l.width, camera_info_l.height);

    cv::Mat D_l = cv::Mat::zeros(1, 5, CV_64F);
    if (camera_info_l.D.size())
    {
        cv::Mat D_l(1, camera_info_l.D.size(), CV_64F, &camera_info_l.D[0]);
    }
    cv::Mat K_l(3, 3, CV_64F, &camera_info_l.K[0]);
    cv::Mat R_l(3, 3, CV_64F, &camera_info_l.R[0]);
    cv::Mat P_l(3, 4, CV_64F, &camera_info_l.P[0]);
    cv::Size image_size_r(camera_info_r.width, camera_info_r.height);


    cv::Mat D_r = cv::Mat::zeros(1, 5, CV_64F);
    if (camera_info_r.D.size())
    {
        cv::Mat D_r(1, camera_info_r.D.size(), CV_64F, &camera_info_r.D[0]);
    }
    cv::Mat K_r(3, 3, CV_64F, &camera_info_r.K[0]);
    cv::Mat R_r(3, 3, CV_64F, &camera_info_r.R[0]);
    cv::Mat P_r(3, 4, CV_64F, &camera_info_r.P[0]);

    cam = StereoCameraModel("stereo_calib",
                            image_size_l, K_l, D_l, R_l, P_l,
                            image_size_r, K_r, D_r, R_r, P_r,
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
    if (result2.isNull())
    {
        res.success = false;
    }
    else
    {
        res.success = true;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_cam_geometric_tools_node");
    ros::NodeHandle n;


    sensor_msgs::CameraInfoConstPtr camera_info_l_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", n);
    sensor_msgs::CameraInfoConstPtr camera_info_r_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", n);
    sensor_msgs::CameraInfo camera_info_l = *camera_info_l_cst_ptr;
    sensor_msgs::CameraInfo camera_info_r = *camera_info_r_cst_ptr;

    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools(camera_info_l, camera_info_r);
    ros::ServiceServer service_feats = n.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::ServiceServer service_transf = n.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ROS_INFO("Stereo camera geometric tools ready");
    ros::spin();

    return 0;
}