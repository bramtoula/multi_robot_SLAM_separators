#include "multi_robot_separators/factorGraph.h"
#include <stdlib.h>

void resetPoseWithCovariance(PoseWithCovariance &toReset)
{
    toReset.pose = gtsam::Pose3();
    toReset.covariance_matrix = gtsam::zeros(6, 6);
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

FactorGraphData::FactorGraphData()
{
    // Extract robot id from ROS namespace
    std::string ns = ros::this_node::getNamespace();

    // Only works up to robot 9 (2 chars after that)
    robot_id_char_ = ns.back();
    robot_id_ = int(robot_id_char_) - '0';
    ROS_INFO("Robot id found from namespace: %d", robot_id_);

    nb_keyframes_ = 0;
    resetPoseWithCovariance(accumulated_transform_);

    gtsam::Symbol  init_symbol = gtsam::Symbol(robot_id_char_, nb_keyframes_);
    poses_initial_guess_.insert(init_symbol.key(), accumulated_transform_.pose);
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
        gtsam::Symbol robot_cur_symbol = gtsam::Symbol(robot_id_char_, nb_keyframes_+1);
        gtsam::Symbol robot_prev_symbol = gtsam::Symbol(robot_id_char_, nb_keyframes_);

        // ROS_INFO("Keyframe added");
        // ROS_INFO("%f %f %f", accumulated_transform_.pose.x(), accumulated_transform_.pose.y(), accumulated_transform_.pose.z());
        gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Gaussian::Covariance(accumulated_transform_.covariance_matrix);
        gtsam::BetweenFactor<gtsam::Pose3>
                new_factor = gtsam::BetweenFactor<gtsam::Pose3>(robot_prev_symbol, robot_cur_symbol, accumulated_transform_.pose, noise_model);
        nb_keyframes_ += 1;

        pose_graph_.push_back(new_factor);

        // if(nb_keyframes_ == 15)
        // {
        //     ROS_INFO("WRiting lOg");
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
    FactorGraphData factorGraphData = FactorGraphData();
    ros::NodeHandle n;

    ros::Subscriber sub_odom = n.subscribe("odom_info", 1000, &FactorGraphData::addOdometry, &factorGraphData);

    ros::spin();

    return 0;
}