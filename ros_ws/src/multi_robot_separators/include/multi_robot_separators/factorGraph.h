#include "ros/ros.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <rtabmap_ros/OdomInfo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/dataset.h>

#include "multi_robot_separators/MsgConversion.h"
#include "multi_robot_separators/ReceiveSeparators.h"
typedef struct PoseWithCovariance PoseWithCovariance;

struct PoseWithCovariance
{
    gtsam::Pose3 pose;
    gtsam::Matrix covariance_matrix;
};

class FactorGraphData
{
private:
    /* data */
    gtsam::NonlinearFactorGraph pose_graph_;
    gtsam::Values poses_initial_guess_;
    gtsam::Pose3 cur_pose_;
    int robot_id_;
    unsigned char robot_id_char_;
    int nb_keyframes_;
    PoseWithCovariance accumulated_transform_;
    void poseCompose(const PoseWithCovariance &a,
                     const PoseWithCovariance &b,
                     PoseWithCovariance &out);

public:
    void addOdometry(const rtabmap_ros::OdomInfo::ConstPtr &msg);
    bool addSeparator(multi_robot_separators::ReceiveSeparators::Request &req,
                      multi_robot_separators::ReceiveSeparators::Response &res);    

        FactorGraphData();
};

void resetPoseWithCovariance(PoseWithCovariance &toReset);