#include "ros/ros.h"
#include "multi_robot_separators/ReceiveSeparators.h"
#include "multi_robot_separators/FindMatches.h"

class Communicater
{
private:
    /* data */


public:
    bool find_matches_query(multi_robot_separators::FindMatches::Request &req,
                            multi_robot_separators::FindMatches::Response &res);

    bool find_matches_answer(multi_robot_separators::FindMatches::Request &req,
                             multi_robot_separators::FindMatches::Response &res);

    bool found_separators_send(multi_robot_separators::ReceiveSeparators::Request &req,
                                             multi_robot_separators::ReceiveSeparators::Response &res);

    bool found_separators_receive(multi_robot_separators::ReceiveSeparators::Request &req,
                                             multi_robot_separators::ReceiveSeparators::Response &res);
        ros::NodeHandle n_;
    Communicater();
};