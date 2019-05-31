#include "rtabmap/core/Rtabmap.h"
//#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "rtabmap/core/Registration.h"

using namespace rtabmap;


int main(int argc, char *argv[])
{
    StereoCameraModel cam(718.856, 718.856, 607.1928, 185.2157, 0.54, Transform::getIdentity(), cv::Size(1241,376));
    cv::Mat img_l_1 = cv::imread("left_1.png", CV_8UC1);
    cv::Mat img_l_2 = cv::imread("left_2.png", CV_8UC1);
    cv::Mat img_r_1 = cv::imread("right_1.png", CV_8UC1);
    cv::Mat img_r_2 = cv::imread("right_2.png", CV_8UC1);

    SensorData frame1(img_l_1,img_r_1,cam);
    SensorData frame2(img_l_2, img_r_2, cam);

    ParametersMap params;

    Registration *_registrationPipeline;

    _registrationPipeline = Registration::create(params);

    RegistrationInfo info;
    Transform res = _registrationPipeline->computeTransformation(frame1, frame2, Transform(), &info);

    printf("%s\n",res.prettyPrint().c_str());
    return 0;
}