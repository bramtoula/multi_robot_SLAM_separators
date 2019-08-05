#include "multi_robot_separators/logger.h"
#include <sys/stat.h>

void log_find_matches_query(multi_robot_separators::FindMatches::Request &req)
{
    ros::Time t = ros::Time::now();
    FILE *pFile = fopen("/root/rdpgo_ws/src/robust_distributed_slam_module/scripts/log/find_matches_query.txt", "a");
    fprintf(pFile, "%u.%u\n", t.sec, t.nsec);

    fprintf(pFile, "number_of_values_in_descriptors ");
    fprintf(pFile, "%lu ", boost::size(req.new_netvlad_descriptors));
    fprintf(pFile, "\n");
    fprintf(pFile, "\n");
    fclose(pFile);
}

void log_find_matches_answer(multi_robot_separators::FindMatches::Response &res)
{
    ros::Time t = ros::Time::now();
    FILE *pFile = fopen("/root/rdpgo_ws/src/robust_distributed_slam_module/scripts/log/find_matches_answer.txt", "a");
    fprintf(pFile, "%u.%u\n", t.sec, t.nsec);

    fprintf(pFile, "number_of_kf_ids_computing_robot ");
    fprintf(pFile, "%lu ", boost::size(res.kf_ids_computing_robot));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_frames_kept_ids_computing_robot ");
    fprintf(pFile, "%lu ", boost::size(res.frames_kept_ids_computing_robot));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_frames_kept_ids_querying_robot ");
    fprintf(pFile, "%lu ", boost::size(res.frames_kept_ids_querying_robot));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_descriptors_vec ");
    fprintf(pFile, "%lu ", boost::size(res.descriptors_vec));
    fprintf(pFile, "\n");

    fprintf(pFile, "sizes_of_descriptors ");
    for (int i = 0; i < boost::size(res.descriptors_vec); i++)
    {
        fprintf(pFile, "%lu ", boost::size(res.descriptors_vec[i].data));
    }
    fprintf(pFile, "\n");

    fprintf(pFile, "kpts3D_vec ");
    fprintf(pFile, "%lu ", boost::size(res.kpts3D_vec));
    fprintf(pFile, "\n");
    fprintf(pFile, "sizes_of_kpts3D ");
    for (int i = 0; i < boost::size(res.kpts3D_vec); i++)
    {
        fprintf(pFile, "%d ", res.kpts3D_vec[i].size);
    }
    fprintf(pFile, "\n");

    fprintf(pFile, "sizes_of_kpts ");
    for (int i = 0; i < boost::size(res.kpts_vec); i++)
    {
        fprintf(pFile, "%d ", res.kpts_vec[i].size);
    }
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_pose_estimates ");
    fprintf(pFile, "%lu ", boost::size(res.pose_estimates));
    fprintf(pFile, "\n");

    fprintf(pFile, "\n");

    fclose(pFile);
}

void log_receive_separators_query(multi_robot_separators::ReceiveSeparators::Request &req)
{
    ros::Time t = ros::Time::now();
    FILE *pFile = fopen("/root/rdpgo_ws/src/robust_distributed_slam_module/scripts/log/receive_separators_query.txt", "a");
    fprintf(pFile, "%u.%u\n", t.sec, t.nsec);

    fprintf(pFile, "robot_from_id 1");
    fprintf(pFile, "\n");

    fprintf(pFile, "robot_to_id 1");
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_kf_ids_from ");
    fprintf(pFile, "%lu ", boost::size(req.kf_ids_from));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_kf_ids_to ");
    fprintf(pFile, "%lu ", boost::size(req.kf_ids_to));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_frames_kepts_ids_from ");
    fprintf(pFile, "%lu ", boost::size(req.frames_kepts_ids_from));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_frames_kepts_ids_to ");
    fprintf(pFile, "%lu ", boost::size(req.frames_kepts_ids_to));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_pose_estimates_from ");
    fprintf(pFile, "%lu ", boost::size(req.pose_estimates_from));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_transform_est_success ");
    fprintf(pFile, "%lu ", boost::size(req.transform_est_success));
    fprintf(pFile, "\n");

    fprintf(pFile, "number_of_separators ");
    fprintf(pFile, "%lu ", boost::size(req.separators));
    fprintf(pFile, "\n");

    fprintf(pFile, "\n");

    fclose(pFile);
}

void log_receive_separators_answer(multi_robot_separators::ReceiveSeparators::Response &res)
{
    ros::Time t = ros::Time::now();
    FILE *pFile = fopen("/root/rdpgo_ws/src/robust_distributed_slam_module/scripts/log/receive_separators_answer.txt", "a");
    fprintf(pFile, "%u.%u\n", t.sec, t.nsec);
    fprintf(pFile, "success 1 ");
    fprintf(pFile, "\n");

    fprintf(pFile, "\n");

    fclose(pFile);
}
