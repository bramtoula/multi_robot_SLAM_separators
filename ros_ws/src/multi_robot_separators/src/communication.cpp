#include "multi_robot_separators/communication.h"

Communicater::Communicater() 
{
    if (n_.getParam("other_robot_id", other_robot_id_))
    {
        ROS_INFO("Other robot ID is %d ", other_robot_id_);
    }
    else
    {
        ROS_ERROR("Couldn't find other robot ID");
    }
}

bool Communicater::found_separators_query(multi_robot_separators::ReceiveSeparators::Request &req,
                                         multi_robot_separators::ReceiveSeparators::Response &res)
{
    std::string service_name = "/robot_"+std::to_string(other_robot_id_)+"/found_separators_receive";
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::ReceiveSeparators>(service_name);
    multi_robot_separators::ReceiveSeparators srv;
    srv.request = req;
    log_receive_separators_query(req);
    if (client.call(srv))
    {
        log_receive_separators_answer(res);
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call found_separators_receive service");
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
        ROS_ERROR("Failed to call receive_separators_py service");
        return 1;
    }
    return true;
}

bool Communicater::find_matches_query(multi_robot_separators::FindMatches::Request &req,
                                      multi_robot_separators::FindMatches::Response &res)
{
    std::string service_name = "/robot_" + std::to_string(other_robot_id_) + "/find_matches_answer";
    ros::ServiceClient client = n_.serviceClient<multi_robot_separators::FindMatches>(service_name);
    multi_robot_separators::FindMatches srv;
    srv.request = req;
    log_find_matches_query(req);

    if (client.call(srv))
    {

        res = srv.response;
        log_find_matches_answer(res);
    }
    else
    {
        ROS_WARN("Failed to call find_matches_answer service, maybe the robot isn't available");
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
    ros::ServiceServer service_sep_query = communicater.n_.advertiseService("found_separators_query", &Communicater::found_separators_query, &communicater);
    ros::ServiceServer service_sep_rec = communicater.n_.advertiseService("found_separators_receive", &Communicater::found_separators_receive, &communicater);

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}