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
class StereoCamGeometricTools
{
private:
    /* data */
    StereoCameraModel cam;
    Registration *_registrationPipeline;
    RegistrationInfo info;
    cv::Size imageSize;

        public : bool
                 getFeaturesAndDescriptor(multi_robot_separators::GetFeatsAndDesc::Request &req,
                                          multi_robot_separators::GetFeatsAndDesc::Response &res);
    bool estimateTransformation(multi_robot_separators::EstTransform::Request &req,
                                multi_robot_separators::EstTransform::Response &res);
    StereoCamGeometricTools(const cv::Mat &P_lt, const cv::Mat &P_rt, const cv::Size imageSizet);
    // void ~StereoCamGeometricTools();
};
