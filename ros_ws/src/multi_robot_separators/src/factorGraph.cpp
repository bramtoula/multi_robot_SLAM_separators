#include "multi_robot_separators/factorGraph.h"
#include <stdlib.h>

FactorGraphData::FactorGraphData(ros::NodeHandle n)
{

    // Check if covariance is manually fixed
    if (n.getParam("set_fixed_covariance", set_fixed_covariance_))
    {
        if (set_fixed_covariance_)
        {

            // Get covariance params
            if (!n.getParam("translation_std", translation_std_))
            {
                ROS_INFO("Couldn't find translation std to fix covariance");
                return;
            }
            if (!n.getParam("rotation_std", rotation_std_))
            {
                ROS_INFO("Couldn't find rotation std to fix covariance");
                return;
            }

            ROS_INFO("Covariance will be manually fixed with %fm STD for translations and %frad STD for rotations", translation_std_, rotation_std_);
        }
        else
        {
            ROS_INFO("Covariance is taken from the RTAB-Map estimations");
            set_fixed_covariance_ = false;
        }
        
        
    }
    else
    {
        ROS_ERROR("Covariance is taken from the RTAB-Map estimations");
    }

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

    nb_keyframes_ = 0;
    resetPoseWithCovariance(accumulated_transform_);
    cur_pose_ = gtsam::Pose3();

    gtsam::Symbol init_symbol = gtsam::Symbol(local_robot_id_char_, nb_keyframes_);
    poses_initial_guess_.insert(init_symbol.key(), cur_pose_);

    // Delete existent log files
    std::string log_file_name = "/root/multi_robot_SLAM_separators/logs/graph_with_separators_robot_" + boost::lexical_cast<std::string>(local_robot_id_) + ".g2o";
    std::remove(log_file_name.c_str());
}

FactorGraphData::~FactorGraphData()
{
    ROS_INFO("Writing log");
    std::string dataset_file_name = "/root/multi_robot_SLAM_separators/logs/graph_with_separators_robot_" + boost::lexical_cast<std::string>(local_robot_id_) + ".g2o";
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
    gtsam::Symbol robot_symbol_from;
    gtsam::Symbol robot_symbol_to;

    unsigned char robot_from_id_char_ = req.robot_from_id + 'a';
    unsigned char robot_to_id_char_ = req.robot_to_id + 'a';
    for (int idx = 0; idx < req.kf_ids_from.size(); idx++)
    {
        robot_symbol_from = gtsam::Symbol(robot_from_id_char_, req.kf_ids_from[idx]);
        robot_symbol_to = gtsam::Symbol(robot_to_id_char_, req.kf_ids_to[idx]);

        covarianceToMatrix(req.separators[idx].covariance, covariance_mat);

        poseROSToPose3(req.separators[idx].pose, separator);

        // If covariance is manually set, replace the one in accumulated_transform_
        if (set_fixed_covariance_)
        {
            manuallySetCovMat(covariance_mat);
        }

        gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance_mat);

        gtsam::BetweenFactor<gtsam::Pose3> new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_symbol_from, robot_symbol_to, separator, noise_model);
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

        // If covariance is manually set, replace the one in accumulated_transform_
        if (set_fixed_covariance_)
        {
            manuallySetCovMat(accumulated_transform_.covariance_matrix);
        }

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

void FactorGraphData::manuallySetCovMat(gtsam::Matrix &cov_mat_to_replace)
{
    cov_mat_to_replace = gtsam::zeros(6,6);
    cov_mat_to_replace(0, 0) = rotation_std_ * rotation_std_;
    cov_mat_to_replace(1, 1) = rotation_std_ * rotation_std_;
    cov_mat_to_replace(2, 2) = rotation_std_ * rotation_std_;
    cov_mat_to_replace(3, 3) = translation_std_ * translation_std_;
    cov_mat_to_replace(4, 4) = translation_std_ * translation_std_;
    cov_mat_to_replace(5, 5) = translation_std_ * translation_std_;
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