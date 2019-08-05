#include <string>

#include "ros/ros.h"
#include "multi_robot_separators/ReceiveSeparators.h"
#include "multi_robot_separators/FindMatches.h"
#include "multi_robot_separators/logger.h"

class Communicater
{
private:
    /* data */
    int other_robot_id_;

public:
    bool find_matches_query(multi_robot_separators::FindMatches::Request &req,
                            multi_robot_separators::FindMatches::Response &res);

    bool find_matches_answer(multi_robot_separators::FindMatches::Request &req,
                             multi_robot_separators::FindMatches::Response &res);

    bool found_separators_query(multi_robot_separators::ReceiveSeparators::Request &req,
                               multi_robot_separators::ReceiveSeparators::Response &res);

    bool found_separators_receive(multi_robot_separators::ReceiveSeparators::Request &req,
                                  multi_robot_separators::ReceiveSeparators::Response &res);
    ros::NodeHandle n_;
    Communicater();
};