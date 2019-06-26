#include "multi_robot_separators/factorGraph.h"
#include <stdlib.h>

FactorGraphData::FactorGraphData(ros::NodeHandle n)
{

    // Get local  robot id
    if (n.getParam("local_robot_id", local_robot_id_))
    {
        ROS_INFO("Local robot ID is %d ", local_robot_id_);
    }
    else
    {
        ROS_ERROR("Couldn't find local robot ID");
    }

    local_robot_id_char_ = local_robot_id_ + 'a';

    other_robot_id_char_ = other_robot_id_ + 'a';
    // Get other robot id
    if (n.getParam("other_robot_id", other_robot_id_))
    {
        ROS_INFO("Other robot ID is %d ", other_robot_id_);
    }
    else
    {
        ROS_ERROR("Couldn't find other robot ID");
    }

    other_robot_id_char_ = other_robot_id_ + 'a';

    // // Extract robot id from ROS namespace
    // std::string ns = ros::this_node::getNamespace();

    // // Only works up to robot 9 (2 chars after that)
    // robot_id_char_ = ns.back();
    // robot_id_ = int(robot_id_char_) - 'a';
    // ROS_INFO("Robot id found from namespace: %d", robot_id_);

    nb_keyframes_ = 0;
    resetPoseWithCovariance(accumulated_transform_);
    cur_pose_ = gtsam::Pose3();

    gtsam::Symbol init_symbol = gtsam::Symbol(local_robot_id_char_, nb_keyframes_);
    poses_initial_guess_.insert(init_symbol.key(), cur_pose_);
}

FactorGraphData::~FactorGraphData()
{
    ROS_INFO("Writing log");
    std::string dataset_file_name = "graph_with_separators_robot_" + boost::lexical_cast<std::string>(local_robot_id_) + ".g2o";
    gtsam::writeG2o(pose_graph_, poses_initial_guess_, dataset_file_name);
}

void resetPoseWithCovariance(PoseWithCovariance &toReset)
{
    toReset.pose = gtsam::Pose3();
    toReset.covariance_matrix = gtsam::zeros(6, 6);
}

bool FactorGraphData::addSeparators(multi_robot_separators::ReceiveSeparators::Request &req,
                                    multi_robot_separators::ReceiveSeparators::Response &res)
{
    gtsam::Matrix covariance_mat = gtsam::zeros(6, 6);
    gtsam::Pose3 separator;

    //////////////// Not using from lowest id to highest id anymore. Now using from the one who computed the transform
    // gtsam::Symbol low_id_robot_symbol;
    // gtsam::Symbol high_id_robot_symbol;
    gtsam::Symbol sending_robot_symbol;
    gtsam::Symbol receiving_robot_symbol;

    unsigned char sending_robot_id_char_ = req.sending_robot_id + 'a';
    for (int idx = 0; idx < req.matched_ids_local.size(); idx++)
    {

        // //////////////// Not using from lowest id to highest id anymore. Now using from the one who computed the transform
        // if (local_robot_id_ < req.sending_robot_id)
        // {
        //     // local robot goes with matched other since message comes from the sending robot
        //     low_id_robot_symbol = gtsam::Symbol(local_robot_id_char_, req.matched_ids_other[idx]);
        //     high_id_robot_symbol = gtsam::Symbol(other_robot_id_char_, req.matched_ids_local[idx]);
        // }
        // else
        // {
        //     // local robot goes with matched other since message comes from the sending robot
        //     high_id_robot_symbol = gtsam::Symbol(local_robot_id_char_, req.matched_ids_other[idx]);
        //     low_id_robot_symbol = gtsam::Symbol(other_robot_id_char_, req.matched_ids_local[idx]);
        // }

        // Case where the robot computing the separator is saving is to the pose graph
        if (local_robot_id_ == req.sending_robot_id)
        {
            // local robot goes with matched other since message comes from the sending robot
            sending_robot_symbol = gtsam::Symbol(local_robot_id_char_, req.matched_ids_other[idx]);
            receiving_robot_symbol = gtsam::Symbol(other_robot_id_char_, req.matched_ids_local[idx]);
        }

        // Case where the robot is receiving the separator computed by another robot
        else
        {
            // local robot goes with matched other since message comes from the sending robot
            receiving_robot_symbol = gtsam::Symbol(local_robot_id_char_, req.matched_ids_other[idx]);
            sending_robot_symbol = gtsam::Symbol(sending_robot_id_char_, req.matched_ids_local[idx]);
        }
        covarianceToMatrix(req.separators[idx].covariance, covariance_mat);

        poseROSToPose3(req.separators[idx].pose, separator);

        gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance_mat);

        // gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(low_id_robot_symbol, high_id_robot_symbol, separator, noise_model);
        gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(sending_robot_symbol, receiving_robot_symbol, separator, noise_model);
        pose_graph_.push_back(new_factor);
    }
    res.success = true;
    return true;
}

void FactorGraphData::poseCompose(const PoseWithCovariance &a,
                                  const PoseWithCovariance &b,
                                  PoseWithCovariance &out)
{
    gtsam::Matrix Ha, Hb;
    out.pose = a.pose.compose(b.pose, Ha, Hb);
    out.covariance_matrix = Ha * a.covariance_matrix * Ha.transpose() +
                            Hb * b.covariance_matrix * Hb.transpose();
}

void FactorGraphData::addOdometry(const rtabmap_ros::OdomInfo::ConstPtr &msg)
{
    PoseWithCovariance received_transform;
    resetPoseWithCovariance(received_transform);

    // Fill the received transform with the converted pose and covariance
    transformToPose3(msg->transform, received_transform.pose);
    covarianceToMatrix(msg->covariance, received_transform.covariance_matrix);

    // Accumulate the transforms
    poseCompose(accumulated_transform_,
                received_transform,
                accumulated_transform_);

    // Add a node if the current frame is a keyframe
    if (msg->keyFrameAdded)
    {
        gtsam::Symbol robot_cur_symbol = gtsam::Symbol(local_robot_id_char_, nb_keyframes_ + 1);
        gtsam::Symbol robot_prev_symbol = gtsam::Symbol(local_robot_id_char_, nb_keyframes_);

        cur_pose_ = cur_pose_.compose(accumulated_transform_.pose);
        poses_initial_guess_.insert(robot_cur_symbol.key(), cur_pose_);
        // ROS_INFO("Keyframe added");
        // ROS_INFO("%f %f %f", accumulated_transform_.pose.x(), accumulated_transform_.pose.y(), accumulated_transform_.pose.z());
        gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(accumulated_transform_.covariance_matrix);
        gtsam::BetweenFactor<gtsam::Pose3>
            new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_prev_symbol, robot_cur_symbol, accumulated_transform_.pose, noise_model);
        nb_keyframes_ += 1;

        pose_graph_.push_back(new_factor);

        // if(nb_keyframes_ == 30)
        // {
        //     ROS_INFO("Writing log");
        //     std::string dataset_file_name = "log.g2o";
        //     gtsam::writeG2o(pose_graph_, poses_initial_guess_,dataset_file_name);
        // }

        // Reset the accumulated transform to compute the one from the latest keyframe to the next one
        resetPoseWithCovariance(accumulated_transform_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "factor_graph");
    ros::NodeHandle n;

    FactorGraphData factorGraphData = FactorGraphData(n);

    ros::Subscriber sub_odom = n.subscribe("odom_info", 1000, &FactorGraphData::addOdometry, &factorGraphData);
    ros::ServiceServer s_add_separators = n.advertiseService("add_separators_pose_graph", &FactorGraphData::addSeparators, &factorGraphData);
    ros::spin();

    return 0;
}