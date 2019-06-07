#include "ros/ros.h"
#include "multi_robot_separators/EstTransform.h"
#include "multi_robot_separators/MsgConversion.h"
#include "multi_robot_separators/GetFeatsAndDesc.h"

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "multi_robot_separators/myRegistration.h"

using namespace rtabmap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimate_transform_client");
    // if (argc != 3)
    // {
    //     ROS_INFO("usage: add_two_ints_client X Y");
    //     return 1;
    // }

    ros::NodeHandle n;
    ros::ServiceClient client_transform = n.serviceClient<multi_robot_separators::EstTransform>("estimate_transformation");
    ros::ServiceClient client_feats = n.serviceClient<multi_robot_separators::GetFeatsAndDesc>("get_features_and_descriptor");
    multi_robot_separators::EstTransform srv_transform;
    multi_robot_separators::GetFeatsAndDesc srv_feats_1;
    multi_robot_separators::GetFeatsAndDesc srv_feats_2;

    cv::Mat img_l_1 = cv::imread("/kitti_data_orig/image_2/000000.png", CV_8UC1);
    cv::Mat img_l_2 = cv::imread("/kitti_data_orig/image_2/000003.png", CV_8UC1);
    cv::Mat img_r_1 = cv::imread("/kitti_data_orig/image_3/000000.png", CV_8UC1);
    cv::Mat img_r_2 = cv::imread("/kitti_data_orig/image_3/000003.png", CV_8UC1);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header;         // empty header
    // // header.seq = counter;            // user defined counter
    // header.stamp = ros::Time::now(); // time
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    // img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    // cv_bridge::CvImage out_msg;
    // // out_msg.header = in_msg->header;                             // Same timestamp and tf frame as input image
    // out_msg.encoding = sensor_msgs::image_encodings::mono8; // Or whatever
    // out_msg.image = sal_float_image;                             // Your cv::Mat

    cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_l_1).toImageMsg(srv_feats_1.request.image_left);
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_l_2).toImageMsg(srv_feats_2.request.image_left);
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_r_1).toImageMsg(srv_feats_1.request.image_right);
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_r_2).toImageMsg(srv_feats_2.request.image_right);
    // srv_feats_2.request.image_left = cv_bridge::toImageMsg(img_l_2);
    // srv_feats_1.request.image_right = cv_bridge::toImageMsg(img_r_1);
    // srv_feats_2.request.image_right = cv_bridge::toImageMsg(img_r_2);

    if (client_feats.call(srv_feats_1))
    {
        ROS_INFO("Feature estimation 1\n");
        // _PrintMatrix("Covariance", covariance);
    }
    else
    {
        ROS_ERROR("Failed to call service call feats");
        return 1;
    }

    if (client_feats.call(srv_feats_2))
    {
        ROS_INFO("Feature estimation 2\n");
        // _PrintMatrix("Covariance", covariance);
    }
    else
    {
        ROS_ERROR("Failed to call service call feats");
        return 1;
    }

    // _registrationPipeline->getFeatures(kptsFrom3D, kptsFrom, descriptorsFrom, frame1Sig, &info);
    // _registrationPipeline->getFeatures(kptsTo3D, kptsTo, descriptorsTo, frame2Sig, &info);

    // keypoints3DToROS(kptsFrom3D, srv_transform.request.kptsFrom3D);
    // keypoints3DToROS(kptsTo3D, srv_transform.request.kptsTo3D);

    // keypointsToROS(kptsFrom, srv_transform.request.kptsFrom);
    // keypointsToROS(kptsTo, srv_transform.request.kptsTo);

    // descriptorsToROS(descriptorsFrom, srv_transform.request.descriptorsFrom);
    // descriptorsToROS(descriptorsTo, srv_transform.request.descriptorsTo);

    srv_transform.request.kptsFrom3D = srv_feats_1.response.kpts3D;
    srv_transform.request.kptsTo3D = srv_feats_2.response.kpts3D;
    srv_transform.request.kptsFrom = srv_feats_1.response.kpts;
    srv_transform.request.kptsTo = srv_feats_2.response.kpts;
    srv_transform.request.descriptorsFrom = srv_feats_1.response.descriptors;
    srv_transform.request.descriptorsTo = srv_feats_2.response.descriptors;

    if (client_transform.call(srv_transform))
    {
        ROS_INFO("Answered\n");
        rtabmap::Transform result = transformFromPoseMsg(srv_transform.response.poseWithCov.pose);
        cv::Mat covariance = covFromFloat64Msg(srv_transform.response.poseWithCov.covariance);
        printf("Mine %s\n", result.prettyPrint().c_str());
        // _PrintMatrix("Covariance", covariance);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }


    return 0;
}
