#include "multi_robot_separators/stereoCamGeometricTools.h"

using namespace rtabmap;

StereoCamGeometricTools::StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_separators)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kInfo);

    // cv::Size image_size_l(camera_info_l.width, camera_info_l.height);

    // cv::Mat D_l = cv::Mat::zeros(1, 5, CV_64F);
    // if (camera_info_l.D.size())
    // {
    //     cv::Mat D_l(1, camera_info_l.D.size(), CV_64F, &camera_info_l.D[0]);
    // }
    // cv::Mat K_l(3, 3, CV_64F, &camera_info_l.K[0]);
    // cv::Mat R_l(3, 3, CV_64F, &camera_info_l.R[0]);
    // cv::Mat P_l(3, 4, CV_64F, &camera_info_l.P[0]);
    // cv::Size image_size_r(camera_info_r.width, camera_info_r.height);

    // cv::Mat D_r = cv::Mat::zeros(1, 5, CV_64F);
    // if (camera_info_r.D.size())
    // {
    //     cv::Mat D_r(1, camera_info_r.D.size(), CV_64F, &camera_info_r.D[0]);
    // }
    // cv::Mat K_r(3, 3, CV_64F, &camera_info_r.K[0]);
    // cv::Mat R_r(3, 3, CV_64F, &camera_info_r.R[0]);
    // cv::Mat P_r(3, 4, CV_64F, &camera_info_r.P[0]);

    // cam_ = StereoCameraModel("stereo_calib",
    //                         image_size_l, K_l, D_l, R_l, P_l,
    //                         image_size_r, K_r, D_r, R_r, P_r,
    //                         cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat());

    // Get the TF transforms necessary to create the stereo camera model
    tf::TransformListener listener;
    tf::StampedTransform local_transform_tf;

    ros::Time stamp = camera_info_l.header.stamp > camera_info_r.header.stamp ? camera_info_l.header.stamp : camera_info_r.header.stamp;

    try
    {
        listener.waitForTransform("frame_id", camera_info_l.header.frame_id,
                                  stamp, ros::Duration(3.0));

        listener.lookupTransform(frame_id, camera_info_l.header.frame_id,
                                 stamp, local_transform_tf);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    Transform local_transform = transformFromTF(local_transform_tf);

    Transform stereo_transform;
    if (estimate_stereo_transform_from_tf)
    {
        tf::StampedTransform stereo_transform_tf;
        try
        {
            listener.lookupTransform(camera_info_r.header.frame_id, camera_info_l.header.frame_id,
                                     camera_info_l.header.stamp, stereo_transform_tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        stereo_transform = transformFromTF(stereo_transform_tf);
    }


    cam_ = stereoCameraModelFromROS(camera_info_l, camera_info_r, local_transform, stereo_transform);

    ParametersMap params;
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpCorrespondenceRatio(), "0.1"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpIterations(), "10"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.3"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxTranslation(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlane(), "true"));
   // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpVoxelSize(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpEpsilon(), "0.01"));
    
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::to_string(nb_min_inliers_separators)));

    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), "1"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "2"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorFlowWinSize(), "16"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorType(), "0"));

    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), "0"));

    registrationPipeline_ = Registration::create(params);
}

bool StereoCamGeometricTools::getFeaturesAndDescriptor(multi_robot_separators::GetFeatsAndDesc::Request &req,
                                                       multi_robot_separators::GetFeatsAndDesc::Response &res)
{

    cv::Mat img_l = cv_bridge::toCvCopy(req.image_left, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat img_r = cv_bridge::toCvCopy(req.image_right, sensor_msgs::image_encodings::MONO8)->image;

    SensorData frame(img_l, img_r, cam_);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::Point3f> kptsFrom3D;
    cv::Mat descriptorsFrom;
    RegistrationInfo info1;
    Signature frameSig(frame);
    registrationPipeline_->getFeatures(kptsFrom3D, kptsFrom, descriptorsFrom, frameSig, &info1);

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
    Transform result = registrationPipeline_->computeTransformationFromFeats(
        cam_,
        cam_,
        descriptorsFrom,
        descriptorsTo,
        image_size_,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        guess,
        &info_);
    Transform result2 = registrationPipeline_->computeTransformationFromFeats(
        cam_,
        cam_,
        descriptorsFrom,
        descriptorsTo,
        image_size_,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        result,
        &info_);

    covToFloat64Msg(info_.covariance, res.poseWithCov.covariance);
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

    bool estimate_stereo_transform_from_tf;
    std::string frame_id;
    int nb_min_inliers_separators;
    if (!n.getParam("estimate_stereo_transform_from_tf", estimate_stereo_transform_from_tf))
    {
        ROS_ERROR("Couldn't find estimate_stereo_transform_from_tf param");
    }

    if (!n.getParam("frame_id", frame_id))
    {
        ROS_ERROR("Couldn't find frame ID");
    }

    if (!n.getParam("separators_min_inliers", nb_min_inliers_separators))
    {
        ROS_ERROR("Couldn't find separators_min_inliers param");
    }

    sensor_msgs::CameraInfoConstPtr camera_info_l_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", n);
    sensor_msgs::CameraInfoConstPtr camera_info_r_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", n);
    sensor_msgs::CameraInfo camera_info_l = *camera_info_l_cst_ptr;
    sensor_msgs::CameraInfo camera_info_r = *camera_info_r_cst_ptr;

    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools(camera_info_l, camera_info_r, frame_id, estimate_stereo_transform_from_tf, nb_min_inliers_separators);
    ros::ServiceServer service_feats = n.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::ServiceServer service_transf = n.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ROS_INFO("Stereo camera geometric tools ready");
    ros::spin();

    return 0;
}
