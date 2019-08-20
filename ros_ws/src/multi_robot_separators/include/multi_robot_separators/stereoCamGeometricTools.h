#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "multi_robot_separators/EstTransform.h"
#include "multi_robot_separators/MsgConversion.h"
#include "multi_robot_separators/GetFeatsAndDesc.h"

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "multi_robot_separators/myRegistration.h"

#include "sensor_msgs/CameraInfo.h"
using namespace rtabmap;
class StereoCamGeometricTools
{
private:
    StereoCameraModel cam_;
    Registration *registrationPipeline_;
    RegistrationInfo info_;
    cv::Size image_size_;

public:
    bool
    getFeaturesAndDescriptor(multi_robot_separators::GetFeatsAndDesc::Request &req,
                             multi_robot_separators::GetFeatsAndDesc::Response &res);
    bool estimateTransformation(multi_robot_separators::EstTransform::Request &req,
                                multi_robot_separators::EstTransform::Response &res);
    StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_separators);
    // void ~StereoCamGeometricTools();
};
