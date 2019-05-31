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
    FILE *pFile = 0;
    std::string pathCalib =  "calib.txt";

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

    cv::Mat image = cv::imread("left_1.png");

    StereoCameraModel cam("stereo_calib",
                            image.size(), P2.colRange(0, 3), cv::Mat(), cv::Mat(), P2,
                            image.size(), P3.colRange(0, 3), cv::Mat(), cv::Mat(), P3,
                            cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat());
    // StereoCameraModel cam(718.856, 718.856, 607.1928, 185.2157, 0.54, Transform::getIdentity(), cv::Size(1241,376));
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