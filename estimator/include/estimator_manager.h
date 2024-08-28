#pragma once

#include <utils/sophus_utils.hpp>
#include <utils/yaml_utils.h>

#include "initialization.h"
#include "trajectory_manager.h"
#include "trajectory_viewer.h"

#include <mutex>

namespace estimator{

class EstimatorManager{
public:
    EstimatorManager(const YAML::Node& yaml_node, ros::NodeHandle& nh);
    ~EstimatorManager();
    
    void cornerArrayCallback(const corner_msgs::cornerArray& msg);
    void circleArrayCallback(const circle_msgs::circleArray& msg);

    void initialization();
    void optimization();

private:
    void readBags(std::string corner_bag_path, std::string circle_bag_path);
    void performEstimator();
    void setInitialState();
    void updateTrajectoryInitial();
    void calculateRepError(int CAM_TYPE, int tag = 0);
    void clearPreStatus();

    ros::NodeHandle nh_;
    ros::Subscriber corner_sub_;
    ros::Subscriber circle_sub_;
    rosbag::Bag read_bag_corner, read_bag_circle;

    Initializer::Ptr est_initializer;
    TrajectoryManager::Ptr trajectory_manager_;
    Trajectory::Ptr trajectory_;
    Camera::Ptr event_camera_, convent_camera_;
    CalibBoard::Ptr calib_board_;

    TrajectoryViewer traj_viewer;

    std::mutex init_lc;

    double knot_distance;
    double opt_time;
    std::vector<double> rep_err_conv, rep_err_ev, rep_err_ev_st, rep_err_ev_ed;
    ceres::Solver::Summary summary;
    int num_disp_pose = 0, continue_num = 10;
    int64_t current_start_time, current_time, current_end_time, prev_time;
    cv::Mat T;
    Eigen::Matrix4d eigenT;
    SE3d se3;
    
    std::vector<bool> corner_used, circle_used;
};

} // namespace estimator
