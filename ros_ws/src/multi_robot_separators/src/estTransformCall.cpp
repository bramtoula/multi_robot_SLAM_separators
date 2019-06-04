#include "ros/ros.h"
#include "multi_robot_separators/EstTransform.h"
// #include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimate_transform_client");
    // if (argc != 3)
    // {
    //     ROS_INFO("usage: add_two_ints_client X Y");
    //     return 1;
    // }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<multi_robot_separators::EstTransform>("estimate_transformation");
    multi_robot_separators::EstTransform srv;
    srv.request.descriptorsFrom.rows = 1;
    srv.request.descriptorsTo.rows = 1;
    srv.request.descriptorsFrom.cols = 1;
    srv.request.descriptorsTo.cols = 1;
    // srv.request.a = atoll(argv[1]);
    // srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
        ROS_INFO("Answered\n");
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}