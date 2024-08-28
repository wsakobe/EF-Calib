#include "estimator_manager.h"

namespace estimator{

float computeMean(const std::vector<Eigen::Vector3d>& vectors) {
    float res = 0.0;
    for (const auto& vec : vectors) {
        res += vec.norm();
    }
    return res / vectors.size();
}

EstimatorManager::EstimatorManager(const YAML::Node &yaml_node, ros::NodeHandle& nh)
    : nh_(nh)
{
    circle_sub_ = nh_.subscribe("/circle_ev/circleArray", 1, &EstimatorManager::circleArrayCallback, this);
    corner_sub_ = nh_.subscribe("/corner_gs/cornerArray", 1, &EstimatorManager::cornerArrayCallback, this);
    
    std::string corner_bag_path = yaml_node["cornerBagPath"].as<std::string>();
    std::string circle_bag_path = yaml_node["circleBagPath"].as<std::string>();

    cv::Size conv_cam_size = cv::Size(yaml_node["conv_cam_width"].as<int>(), yaml_node["conv_cam_height"].as<int>());
    cv::Size ev_cam_size = cv::Size(yaml_node["ev_cam_width"].as<int>(), yaml_node["ev_cam_height"].as<int>());
    double square_size = yaml_node["square_size"].as<float>();
    cv::Size board_size = cv::Size(yaml_node["board_width"].as<int>(), yaml_node["board_height"].as<int>());
    knot_distance = yaml_node["knot_distance"].as<float>();
    continue_num = yaml_node["continue_number"].as<int>();

    opt_time = yaml_node["opt_time"].as<float>();

    event_camera_ = std::make_shared<Camera>(ev_cam_size);
    convent_camera_ = std::make_shared<Camera>(conv_cam_size);
    calib_board_ = std::make_shared<CalibBoard>(board_size, square_size);
    
    est_initializer = std::make_shared<Initializer>(event_camera_, convent_camera_, calib_board_);
    est_initializer->initial_window_size = yaml_node["init_window_size"].as<int>();

    trajectory_ = std::make_shared<Trajectory>(knot_distance);
    trajectory_manager_ = std::make_shared<TrajectoryManager>(trajectory_);

    traj_viewer.SetPublisher(nh_);

    readBags(corner_bag_path, circle_bag_path);
}

EstimatorManager::~EstimatorManager(){
    //shutdown every nodes
    circle_sub_.shutdown();
    corner_sub_.shutdown();
}

void EstimatorManager::readBags(std::string corner_bag_path, std::string circle_bag_path){
    read_bag_corner.open(corner_bag_path, rosbag::bagmode::Read);
    rosbag::View view_corner(read_bag_corner, rosbag::TopicQuery("/corner_gs/cornerArray"));

    for (const rosbag::MessageInstance& m : view_corner) {
        corner_msgs::cornerArray::ConstPtr msg = m.instantiate<corner_msgs::cornerArray>();
        est_initializer->corner_buffer_.emplace_back(*msg);
    }
    read_bag_corner.close();

    read_bag_circle.open(circle_bag_path, rosbag::bagmode::Read);
    rosbag::View view_circle(read_bag_circle, rosbag::TopicQuery("/circle_ev/circleArray"));

    for (const rosbag::MessageInstance& m : view_circle) {
        circle_msgs::circleArray::ConstPtr msg = m.instantiate<circle_msgs::circleArray>();
        est_initializer->circle_buffer_.emplace_back(*msg);
    }
    read_bag_circle.close();

    performEstimator();
}

void EstimatorManager::performEstimator(){
    for (est_initializer->epoch_time = 5.0; est_initializer->epoch_time <= opt_time; est_initializer->epoch_time += 0.5){
        est_initializer->b_conv_initialized = false;
        est_initializer->b_ev_initialized = false;
        est_initializer->b_both_initialized = false;
        if (est_initializer->judgeBufferStatus(CONV_CAM)){
            est_initializer->processConv();
        }
        else{
            ROS_ERROR("There are no sufficient convent frames for initialization.");
        }
        if (est_initializer->judgeBufferStatus(EV_CAM)){
            est_initializer->processEv();
        }
        else{
            ROS_ERROR("There are no sufficient event frames for initialization.");
        }

        if (est_initializer->evInitialSucc() && est_initializer->convInitialSucc()){
            est_initializer->estimateInitialExtrinsic();
        }
        else{
            ROS_ERROR("Initialize error, please record again.");
            continue;
        }
        if (est_initializer->b_both_initialized){
            clearPreStatus();
            setInitialState();
            updateTrajectoryInitial();
        }
    }
    std::cout << "======================== Finished experiment! =========================\n";
    ros::shutdown();
}

void EstimatorManager::cornerArrayCallback(const corner_msgs::cornerArray& msg){
    if (!est_initializer->b_both_initialized){
        est_initializer->corner_buffer_.push_back(msg);
        init_lc.lock();
        if (!est_initializer->convInitialSucc()){
            if (est_initializer->judgeBufferStatus(CONV_CAM)){
                est_initializer->processConv();
            }
        }
        else{
            if (est_initializer->evInitialSucc()){
                ROS_INFO("Both camera initialized");
                est_initializer->estimateInitialExtrinsic();
            }        
        }
        init_lc.unlock();
    }
    else{
        ROS_INFO("Conv add trajectory");
        est_initializer->corner_buffer_.emplace_back(msg);
        performEstimator();
    }    
}

void EstimatorManager::circleArrayCallback(const circle_msgs::circleArray& msg){
    if (!est_initializer->b_both_initialized){
        est_initializer->circle_buffer_.push_back(msg);
        init_lc.lock();
        if (!est_initializer->evInitialSucc()){
            if (est_initializer->judgeBufferStatus(EV_CAM)){
                est_initializer->processEv();
            }
        }
        else{
            if (est_initializer->convInitialSucc()){
                ROS_INFO("Both camera initialized");
                est_initializer->estimateInitialExtrinsic();
            }        
        }
        init_lc.unlock();
    }
    else{
        ROS_INFO("Ev add trajectory");
        est_initializer->circle_buffer_.emplace_back(msg);
        performEstimator();
    }    
}

void EstimatorManager::clearPreStatus(){
    trajectory_ = std::make_shared<Trajectory>(knot_distance);
    trajectory_manager_ = std::make_shared<TrajectoryManager>(trajectory_);
}

void EstimatorManager::setInitialState(){
    // set initial pose
    cv::Mat T;
    bool isSucc = false;
    isSucc = est_initializer->solveRelativePose(est_initializer->circle_buffer_.front(), event_camera_, T);
    while (!isSucc){
        est_initializer->circle_buffer_.pop_front();
        isSucc = est_initializer->solveRelativePose(est_initializer->circle_buffer_.front(), event_camera_, T);
    }

    int64_t trajectory_start_time = (est_initializer->circle_buffer_.begin())->header.stamp.toSec() * S_TO_NS;
    trajectory_->setDataStartTime(trajectory_start_time);
    Eigen::Matrix4d eigenT;
    cv::cv2eigen(T, eigenT);
    SE3d T_sop(eigenT);
    SO3d R0(T_sop.rotationMatrix());
    for (size_t i = 0; i <= trajectory_->numKnots(); i++) // only 4 control points at the very beginning
    {
        trajectory_->setKnotSO3(R0, i);
        trajectory_->setKnotPos(T_sop.translation(), i);
    }

    traj_viewer.PublishPose(T_sop, 1, 0, true);
}

void EstimatorManager::updateTrajectoryInitial(){
    TrajectoryEstimator::Ptr estimator_ = std::make_shared<TrajectoryEstimator>(trajectory_, event_camera_, convent_camera_, calib_board_);
    int64_t data_start_time = trajectory_->getDataStartTime();
    estimator_->trajectory_info_.push_back({trajectory_->DEG, -1});
    corner_used.clear();
    circle_used.clear();
    num_disp_pose = 0;

    std::cout << est_initializer->epoch_time << "\n";

    // add circle into trajectory
    for (auto &circles:est_initializer->circle_buffer_){
        bool isSucc = false;
        isSucc = est_initializer->solveRelativePose(circles, event_camera_, T);
        if (isSucc){
            cv::cv2eigen(T, eigenT);
            se3.setQuaternion(Eigen::Quaterniond(eigenT.block<3,3>(0,0)));
            se3.translation() = eigenT.block<3,1>(0,3);
            
            current_start_time = 1e12;
            current_time = 1e12;
            current_end_time = 0;
            for (auto &circle:circles.circles){
                if (circle.start_ts.toSec() * S_TO_NS - data_start_time < current_start_time){
                    current_start_time = circle.start_ts.toSec() * S_TO_NS - data_start_time;
                }
                if (circle.timestamp.toSec() * S_TO_NS - data_start_time < current_time){
                    current_time = circle.timestamp.toSec() * S_TO_NS - data_start_time;
                }
                if (circle.end_ts.toSec() * S_TO_NS - data_start_time > current_end_time){
                    current_end_time = circle.end_ts.toSec() * S_TO_NS - data_start_time;
                }
            }
            if (current_time > trajectory_->maxTimeNs() + (trajectory_->DEG - 2) * trajectory_->getDtNs()){
                estimator_->trajectory_info_.back().second = trajectory_->numKnots() - 1;
                estimator_->trajectory_info_.push_back({(long long)(current_time) / trajectory_->getDtNs() + trajectory_->DEG, -1});
            }  
           
            trajectory_manager_->extendTrajectory(current_end_time, se3);

            prev_time = current_end_time;

            if (trajectory_->maxTimeNs() > est_initializer->epoch_time * S_TO_NS) break;
        }        
    }
    estimator_->trajectory_info_.back().second = trajectory_->numKnots() - 1;
    
    for (auto &circles:est_initializer->circle_buffer_){
        circle_used.emplace_back(false);
        current_time = 1e12;
        for (auto &circle:circles.circles){
            if (circle.timestamp.toSec() * S_TO_NS - data_start_time < current_time){
                current_time = circle.timestamp.toSec() * S_TO_NS - data_start_time;
            }
        }
        int start_cp_idx = (long long)(current_time) / trajectory_->getDtNs() + trajectory_->DEG;
        for (auto traj:estimator_->trajectory_info_){
            if ((traj.second - traj.first) >= continue_num && start_cp_idx >= traj.first && start_cp_idx <= traj.second){
                bool isSucc = false;
                isSucc = est_initializer->solveRelativePose(circles, event_camera_, T);
                if (isSucc){
                    for (auto &circle:circles.circles){
                        if (circle.end_ts.toSec() * S_TO_NS - data_start_time > trajectory_->maxTimeNs()){
                            break;
                        }
                        estimator_->addEventFeature(circle);                        
                    }  
                    circle_used.back() = true;
                } 
                break;      
            }
        }           
    }

    traj_viewer.PublishSplineTrajectory(trajectory_, estimator_->trajectory_info_, trajectory_->minTimeNs(), continue_num, trajectory_->maxTimeNs(), 0.01 * S_TO_NS, 0);
    summary = estimator_->solve(20, true, 8);
    traj_viewer.PublishSplineTrajectory(trajectory_, estimator_->trajectory_info_, trajectory_->minTimeNs(), continue_num, trajectory_->maxTimeNs(), 0.01 * S_TO_NS, 1);
    calculateRepError(BOTH);
    est_initializer->estimateInitialExtrinsic();
    /*
    // display circle pose
    for (auto &circles:est_initializer->circle_buffer_){
        current_time = 1e12;
        for (auto &circle:circles.circles){
            if (circle.timestamp.toSec() * S_TO_NS - data_start_time < current_time){
                current_time = circle.timestamp.toSec() * S_TO_NS - data_start_time;
            }
        }
        int start_cp_idx = (long long)(current_time) / trajectory_->getDtNs() + trajectory_->DEG;
        for (auto traj:estimator_->trajectory_info_){
            if ((traj.second - traj.first) >= continue_num && start_cp_idx >= traj.first && start_cp_idx <= traj.second){
                bool isSucc = false;
                isSucc = est_initializer->solveRelativePose(circles, event_camera_, T);
                if (isSucc){
                    cv::cv2eigen(T, eigenT);
                    se3.setQuaternion(Eigen::Quaterniond(eigenT.block<3,3>(0,0)));
                    se3.translation() = eigenT.block<3,1>(0,3);
                    traj_viewer.PublishPose(se3, 2, num_disp_pose++);
                } 
                break;      
            }
        }           
    }*/

    // add corner into trajectory
    for (auto &corners:est_initializer->corner_buffer_){
        corner_used.emplace_back(false);
        if (corners.timestamp.toSec() * S_TO_NS < data_start_time + estimator_->max_time_delay * 2 * S_TO_NS){
            continue;
        }
        if (corners.timestamp.toSec() * S_TO_NS - data_start_time + estimator_->max_time_delay * 2 * S_TO_NS > trajectory_->maxTimeNs()){
            break;
        }
        
        int start_cp_idx = (long long)(corners.timestamp.toSec() * S_TO_NS - data_start_time - estimator_->max_time_delay) / trajectory_->getDtNs() + trajectory_->DEG;
        int end_cp_idx = (long long)(corners.timestamp.toSec() * S_TO_NS - data_start_time + estimator_->max_time_delay) / trajectory_->getDtNs() + trajectory_->DEG;
        for (auto traj:estimator_->trajectory_info_){
            if ((traj.second - traj.first) >= continue_num && (start_cp_idx > traj.first + 1) && (end_cp_idx < traj.second - 1)){
                bool isSucc = false;
                isSucc = est_initializer->solveRelativePose(corners, convent_camera_, T);
                if (isSucc){
                    corner_used.back() = true;
                    for (auto &corner:corners.corners){
                        estimator_->addConvFeature(corner, corners.timestamp);
                    }
                    break;
                }
            }
        }
    }
    calculateRepError(BOTH);
    summary = estimator_->solve1(30, true, 8);
    calculateRepError(BOTH, 1);

    for (auto &corners:est_initializer->corner_buffer_){
        if (corners.timestamp.toSec() * S_TO_NS < data_start_time + estimator_->max_time_delay * 2 * S_TO_NS){
            continue;
        }
        if (corners.timestamp.toSec() * S_TO_NS - data_start_time + estimator_->max_time_delay * 2 * S_TO_NS > trajectory_->maxTimeNs()){
            break;
        }
        
        int start_cp_idx = (long long)(corners.timestamp.toSec() * S_TO_NS - data_start_time + convent_camera_->time_delay) / trajectory_->getDtNs() + trajectory_->DEG;
        for (auto traj:estimator_->trajectory_info_){
            if ((traj.second - traj.first) >= continue_num && (start_cp_idx > traj.first + 1) && (start_cp_idx < traj.second - 1)){
                bool succ;
                cv::Mat T_c;
                succ = est_initializer->solveRelativePose(corners, convent_camera_, T_c);
                if (succ){
                    cv::cv2eigen(T_c, eigenT);
                    eigenT = convent_camera_->getExtrinsicMatrix().inverse() * eigenT.eval();

                    se3.setQuaternion(Eigen::Quaterniond(eigenT.block<3,3>(0,0)));
                    se3.translation() = eigenT.block<3,1>(0,3);
                    // display corner pose
                    traj_viewer.PublishPose(se3, 1, num_disp_pose++);
                    break;
                }
            }
        }
    }
    
    Eigen::Matrix3d rotation_matrix = convent_camera_->getExtrinsicMatrix().block<3, 3>(0, 0);
    Eigen::Vector3d euler_angle_ext = rotation_matrix.eulerAngles(2, 1, 0) * 180.0 / 3.1415926; // ZYX order
    Eigen::Vector3d translation_vector = convent_camera_->getExtrinsicMatrix().block<3, 1>(0, 3);

    // Output
    std::cout << "=============== Time offset: " << convent_camera_->time_delay << " =======================\n";
    std::cout << "Euler angles (XYZ):\n" << euler_angle_ext.transpose() << "\n";
    std::cout << "Translation vector:\n" << translation_vector.transpose() << "\n";
    std::cout << "Ev intrinsic:\n" << event_camera_->intrinsicParams[0] << " " << event_camera_->intrinsicParams[1] << " " << event_camera_->intrinsicParams[2] << " " << event_camera_->intrinsicParams[3] << "\n";
    std::cout << "Conv intrinsic:\n" << convent_camera_->intrinsicParams[0] << " " << convent_camera_->intrinsicParams[1] << " " << convent_camera_->intrinsicParams[2] << " " << convent_camera_->intrinsicParams[3] << "\n";
    std::cout << "Ev distortion:\n" << event_camera_->distortParams[0] << " " << event_camera_->distortParams[1] << " " << event_camera_->distortParams[2] << " " << event_camera_->distortParams[3] << " " << event_camera_->distortParams[4] << "\n";
    std::cout << "Conv distortion:\n" << convent_camera_->distortParams[0] << " " << convent_camera_->distortParams[1] << " " << convent_camera_->distortParams[2] << " " << convent_camera_->distortParams[3] << " " << convent_camera_->distortParams[4] << "\n";
    
    /*std::ofstream file("/data/EV-Calib/src/estimator/record_data/event_intrinsic.txt", std::ios_base::app);
    if (file.is_open()) {
        file << event_camera_->intrinsicParams[0] << " " << event_camera_->intrinsicParams[1] << " " << event_camera_->intrinsicParams[2] << " " << event_camera_->intrinsicParams[3] << " ";
        file << event_camera_->distortParams[0] << " " << event_camera_->distortParams[1] << " " << event_camera_->distortParams[2] << " " << event_camera_->distortParams[3] << " " << event_camera_->distortParams[4] << std::endl;
        file.close();
    }
    file.open("/data/EV-Calib/src/estimator/record_data/convent_intrinsic.txt", std::ios_base::app);
    if (file.is_open()) {
        file << convent_camera_->intrinsicParams[0] << " " << convent_camera_->intrinsicParams[1] << " " << convent_camera_->intrinsicParams[2] << " " << convent_camera_->intrinsicParams[3] << " ";
        file << convent_camera_->distortParams[0] << " " << convent_camera_->distortParams[1] << " " << convent_camera_->distortParams[2] << " " << convent_camera_->distortParams[3] << " " << convent_camera_->distortParams[4] << std::endl;
        file.close();
    }
    file.open("/data/EV-Calib/src/estimator/record_data/extrinsic.txt", std::ios_base::app);
    if (file.is_open()) {
        file << euler_angle_ext.transpose() << " ";
        file << translation_vector.transpose() << std::endl;
        file.close();
    }*/
    std::ofstream file("/data/EV-Calib/src/estimator/record_data/time_delay.txt", std::ios_base::app);
    if (file.is_open()) {
        file << convent_camera_->time_delay << std::endl;
        file.close();
    }
    file.open("/data/EV-Calib/src/estimator/record_data/time.txt", std::ios_base::app);
    if (file.is_open()) {
        file << est_initializer->epoch_time << std::endl;
        file.close();
    }
        
    // Calculate trans and rot error
    std::vector<Eigen::Vector3d> euler_angles, t_rels;
    SE3d pose;
    cv::Mat T_cv;
    Eigen::Matrix4d T_cveigen, T_ev, T_ev2cv;
    Eigen::Matrix3d R_rel;
    Eigen::Vector3d t_rel;
    for (auto &c:est_initializer->corner_buffer_){
        if (c.timestamp.toSec() * S_TO_NS < data_start_time + estimator_->max_time_delay * 2 * S_TO_NS){
            continue;
        }
        if (c.timestamp.toSec() * S_TO_NS - data_start_time + estimator_->max_time_delay * 2 * S_TO_NS > trajectory_->maxTimeNs()){
            break;
        }
        
        int start_cp_idx = (long long)(c.timestamp.toSec() * S_TO_NS - data_start_time + convent_camera_->time_delay) / trajectory_->getDtNs() + trajectory_->DEG;
        for (auto traj:estimator_->trajectory_info_){
            if ((traj.second - traj.first) >= continue_num && (start_cp_idx > traj.first + 1) && (start_cp_idx < traj.second - 1)){
                pose = trajectory_->poseNs((c.timestamp.toSec() + convent_camera_->time_delay) * S_TO_NS - data_start_time);
                T_ev = pose.matrix();
                bool isSucc = est_initializer->solveRelativePose(c, convent_camera_, T_cv);
                if (isSucc){
                    est_initializer->refinePose(c, convent_camera_, T_cv);
                    cv::cv2eigen(T_cv, T_cveigen);
                    T_ev2cv = convent_camera_->getExtrinsicMatrix() * T_ev;
                    R_rel = T_ev2cv.block<3,3>(0,0).inverse() * T_cveigen.block<3,3>(0,0);
                    t_rel = T_ev2cv.block<3,1>(0,3) - T_cveigen.block<3,1>(0,3);
                    Eigen::Vector3d euler_angle = R_rel.eulerAngles(2, 1, 0) * 180.0 / 3.14159265358;
                    for (int i = 0; i < 3; ++i) {
                        if (euler_angle(i) > 90.0) {
                            euler_angle(i) -= 180.0;
                        } else if (euler_angle(i) < -90.0) {
                            euler_angle(i) += 180.0;
                        }
                    }
                    euler_angles.push_back(euler_angle);
                    t_rels.push_back(t_rel); 
                }    
            }
        }
    }
    
    file.open("/data/EV-Calib/src/estimator/record_data/rot.txt", std::ios_base::app);
    if (file.is_open()) {
        float mean = computeMean(euler_angles);
        // file << mean << " " << rep_err_conv.size() << std::endl;
        std::cout << "Rot_err: " << mean << " " << rep_err_conv.size() << std::endl;
        file.close();
    }
    file.open("/data/EV-Calib/src/estimator/record_data/trans.txt", std::ios_base::app);
    if (file.is_open()) {
        float mean = computeMean(t_rels);
        // file << mean << " " << rep_err_conv.size() << std::endl;
        std::cout << "Trans_err: " << mean << " " << rep_err_conv.size() << std::endl;
        file.close();
    }

    std::cout << "=================== End for this epoch ====================\n";
}

void EstimatorManager::calculateRepError(int CAM_TYPE, int tag){
    int64_t data_start_time = trajectory_->getDataStartTime();
    rep_err_conv.clear();
    rep_err_ev.clear();
    rep_err_ev_st.clear();
    rep_err_ev_ed.clear();
    SE3d pose;
    Eigen::Matrix<double, 3, 1> point_in_W, point_in_E, point_in_C;
    Eigen::Matrix<double, 2, 1> point_in_cam, err;
    
    if (CAM_TYPE == EV_CAM || CAM_TYPE == BOTH){
        int cnt = 0;
        for (auto &c:est_initializer->circle_buffer_){
            if (!circle_used.empty()){
                if (!circle_used[cnt++]) continue;
            }
            if (c.header.stamp.toSec() * S_TO_NS - data_start_time > trajectory_->maxTimeNs()){
                break;
            }
            double rep_err = 0, rep_err_st = 0, rep_err_ed = 0;
            for (auto &circle:c.circles){
                if (circle.end_ts.toSec() * S_TO_NS - data_start_time > trajectory_->maxTimeNs()) break;
                pose = trajectory_->poseNs(circle.timestamp.toSec() * S_TO_NS - data_start_time);
                point_in_W = calib_board_->getBoardPointInWorld(circle.x_grid, circle.y_grid);
                point_in_E = pose.rotationMatrix() * point_in_W + pose.translation();
                point_in_cam = event_camera_->projectIntoImage(point_in_E);
                err = point_in_cam - Eigen::Matrix<double, 2, 1>(circle.x, circle.y);
                rep_err += err.norm();  

                pose = trajectory_->poseNs((circle.start_ts.toSec() + (circle.timestamp - circle.start_ts).toSec() * 0.5) * S_TO_NS - data_start_time);
                point_in_W = calib_board_->getBoardPointInWorld(circle.x_grid, circle.y_grid);
                point_in_E = pose.rotationMatrix() * point_in_W + pose.translation();
                point_in_cam = event_camera_->projectIntoImage(point_in_E);
                err = point_in_cam - Eigen::Matrix<double, 2, 1>(circle.x + circle.velo_x * ((circle.start_ts + (circle.timestamp - circle.start_ts) * 0.5 - circle.timestamp)).toSec() * S_TO_NS / 1000.0, circle.y + circle.velo_y * ((circle.start_ts + (circle.timestamp - circle.start_ts) * 0.5 - circle.timestamp)).toSec() * S_TO_NS / 1000.0);
                rep_err_st += err.norm();  

                pose = trajectory_->poseNs((circle.end_ts.toSec() + (circle.timestamp - circle.end_ts).toSec() * 0.5) * S_TO_NS - data_start_time);
                point_in_W = calib_board_->getBoardPointInWorld(circle.x_grid, circle.y_grid);
                point_in_E = pose.rotationMatrix() * point_in_W + pose.translation();
                point_in_cam = event_camera_->projectIntoImage(point_in_E);
                err = point_in_cam - Eigen::Matrix<double, 2, 1>(circle.x + circle.velo_x * ((circle.end_ts + (circle.timestamp - circle.end_ts) * 0.5 - circle.timestamp)).toSec() * S_TO_NS / 1000.0, circle.y + circle.velo_y * ((circle.end_ts + (circle.timestamp - circle.end_ts) * 0.5 - circle.timestamp)).toSec() * S_TO_NS / 1000.0);
                rep_err_ed += err.norm();  
            }
            rep_err_ev.push_back(rep_err / c.circles.size());
            rep_err_ev_st.push_back(rep_err_st / c.circles.size());
            rep_err_ev_ed.push_back(rep_err_ed / c.circles.size());
        }
        std::cout << "Rep err ev: " << std::accumulate(rep_err_ev.begin(), rep_err_ev.end(), 0.0) / rep_err_ev.size() << std::endl;
        std::cout << "Rep err ev st: " << std::accumulate(rep_err_ev_st.begin(), rep_err_ev_st.end(), 0.0) / rep_err_ev_st.size() << std::endl;
        std::cout << "Rep err ev ed: " << std::accumulate(rep_err_ev_ed.begin(), rep_err_ev_ed.end(), 0.0) / rep_err_ev_ed.size() << std::endl;
    }

    if (CAM_TYPE == CONV_CAM || CAM_TYPE == BOTH){
        int cnt = 0;
        for (auto &c:est_initializer->corner_buffer_){
            if (!corner_used.empty()){
                if (!corner_used[cnt++]) continue;
            }
            
            if (c.timestamp.toSec() * S_TO_NS < data_start_time + 0.08 * 2 * S_TO_NS) continue;
            if (c.timestamp.toSec() * S_TO_NS - data_start_time + 0.08 * 2 * S_TO_NS > trajectory_->maxTimeNs()) break;
            pose = trajectory_->poseNs((c.timestamp.toSec() + convent_camera_->time_delay) * S_TO_NS - data_start_time);
            double rep_err = 0;
            for (auto &corner:c.corners){
                point_in_W = calib_board_->getBoardPointInWorld(corner.x_grid, corner.y_grid);
                point_in_E = pose.rotationMatrix() * point_in_W + pose.translation();
                point_in_C = convent_camera_->getExtrinsicMatrix().block<3,3>(0,0) * point_in_E + convent_camera_->getExtrinsicMatrix().block<3,1>(0,3);
                point_in_cam = convent_camera_->projectIntoImage(point_in_C);
                err = point_in_cam - Eigen::Matrix<double, 2, 1>(corner.x, corner.y);
                rep_err += err.norm();
            }
            rep_err_conv.push_back(rep_err / c.corners.size());          
        }
        std::cout << "Rep err conv: " << std::accumulate(rep_err_conv.begin(), rep_err_conv.end(), 0.0) / rep_err_conv.size() << std::endl;
    }
}

};