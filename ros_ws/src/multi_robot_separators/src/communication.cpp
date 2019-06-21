#include "multi_robot_separators/communication.h"

Communicater::Communicater(){   }

bool Communicater::found_separators_send(multi_robot_separators::ReceiveSeparators::Request &req,
                                         multi_robot_separators::ReceiveSeparators::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::ReceiveSeparators>("found_separators_receive");
    multi_robot_separators::ReceiveSeparators srv;
    srv.request = req;
    if (client.call(srv))
    {

        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call find_matches_answer service");
        return 1;
    }
    return true;
}

bool Communicater::found_separators_receive(multi_robot_separators::ReceiveSeparators::Request &req,
                                            multi_robot_separators::ReceiveSeparators::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::ReceiveSeparators>("receive_separators_py");
    multi_robot_separators::ReceiveSeparators srv;
    srv.request = req;
    if (client.call(srv))
    {
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call find_matches_compute service");
        return 1;
    }
    return true;
}

bool Communicater::find_matches_query(multi_robot_separators::FindMatches::Request &req,
                        multi_robot_separators::FindMatches::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::FindMatches>("find_matches_answer");
    multi_robot_separators::FindMatches srv;
    srv.request = req;
    if (client.call(srv))
    {

        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call find_matches_answer service");
        return 1;
    }
    return true;
}

bool Communicater::find_matches_answer(multi_robot_separators::FindMatches::Request &req,
                                       multi_robot_separators::FindMatches::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::FindMatches>("find_matches_compute");
    multi_robot_separators::FindMatches srv;
    srv.request = req;
    if (client.call(srv))
    {
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call find_matches_compute service");
        return 1;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication_node");
    Communicater communicater = Communicater();

    ros::ServiceServer service_matches_query = communicater.n_.advertiseService("find_matches_query", &Communicater::find_matches_query, &communicater);
    ros::ServiceServer service_matches_answer = communicater.n_.advertiseService("find_matches_answer", &Communicater::find_matches_answer, &communicater);
    ros::ServiceServer service_sep_send = communicater.n_.advertiseService("found_separators_send", &Communicater::found_separators_send, &communicater);
    ros::ServiceServer service_sep_rec = communicater.n_.advertiseService("found_separators_receive", &Communicater::found_separators_receive, &communicater);

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}