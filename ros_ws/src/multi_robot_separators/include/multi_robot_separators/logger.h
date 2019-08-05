#include "ros/ros.h"
#include "multi_robot_separators/FindMatches.h"
#include "multi_robot_separators/ReceiveSeparators.h"
#include "stdio.h"
#include <boost/range.hpp>

void log_find_matches_query(multi_robot_separators::FindMatches::Request &req);
void log_find_matches_answer(multi_robot_separators::FindMatches::Response &res);

void log_receive_separators_query(multi_robot_separators::ReceiveSeparators::Request &req);
void log_receive_separators_answer(multi_robot_separators::ReceiveSeparators::Response &res);