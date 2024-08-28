#pragma once

#include "ros/ros.h"
#include "ceres/ceres.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eventmap_generator/eventmap.h>

#include <vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>

#include "cylindroid_factor.h"

namespace circle_detector{

struct Chessboard{
    int boardHeight, boardWidth;
}cb;

class CircleDetector{

struct pcaInfo{
    cv::Point2d comp1, comp2;
    double vertical;
    float magnitude_comp1, magnitude_comp2;
    cv::Point2d center;
};

struct cylinderoidInfo{
    cv::RotatedRect ellipse;
    cv::Point2d velo;
    int timestamp, start_ts, end_ts;
    std::vector<cv::Point> area;
};

struct circleInformation {
	cv::RotatedRect ellip_info;
    std::vector<cv::Point> area;
    cv::Point grid_pose;
    ros::Time timestamp, start_ts, end_ts;
    pcaInfo pca_res;
    cv::Point2d velo;
};

public:
    CircleDetector(ros::NodeHandle& nh) : nh_(nh){
        image_sub_ = nh_.subscribe("/eventmap", 1, &CircleDetector::eventmapCallback, this);
        nh_.param("BoardHeight", cb.boardHeight, 3);
        nh_.param("BoardWidth", cb.boardWidth, 6);
        nh_.param("StorePath", store_path, std::string("intermediateBags"));

        options_.max_num_iterations = 10;
        options_.linear_solver_type = ceres::DENSE_QR;
        options_.minimizer_progress_to_stdout = false;
        options_.num_threads = 1;
        
        readBag(store_path + "/eventmap.bag");
    };

    ~CircleDetector(){
        circle_pub_.shutdown();
        image_sub_.shutdown();
    };
    void eventmapCallback(const eventmap_generator::eventmap& msg);
    void eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative);
    
private:
    void readBag(std::string bagName);
    void connectedComponentLabeling(const cv::Mat& src, std::vector<std::vector<cv::Point>>& quadArea);
    pcaInfo pcaAnalysis(std::vector<cv::Point> points);
    double euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    double distanceFromTwoClusters(const circleInformation& cluster_1, const circleInformation& cluster_2);
    float distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction);
    bool coEllipse(const circleInformation& a, const circleInformation& b);
    void organizeCircles(const cv::Mat& event_map);
    cylinderoidInfo fitCylindroid(std::vector<cv::Point3i> cylinderoidPoints);

    bool sortByX(const circleInformation& a, const circleInformation& b) {
        return a.ellip_info.center.x < b.ellip_info.center.x;
    }

    bool sortByY(const circleInformation& a, const circleInformation& b) {
        return a.ellip_info.center.y < b.ellip_info.center.y;
    }

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher circle_pub_ = nh_.advertise<circle_msgs::circleArray>("/circle_ev/circleArray", 10);;
    circle_msgs::circle circle_msg;
    circle_msgs::circleArray circle_array;
    std::vector<ros::Time> timestamps;
    rosbag::Bag read_bag, write_bag;
    std::string bagName, store_path;;

    cv::Size sensor_size_;
    ros::Time timestamp;

    int count = 0;
    std::vector<std::vector<cv::Point>> quadArea_pos, quadArea_neg;
    std::deque<circleInformation> candidate_pos, candidate_neg;
    std::vector<circleInformation> candidate_circles, onboard_circles;
    std::vector<std::pair<circleInformation, circleInformation>> candidate_full;
    cv::RotatedRect m_ellipsetemp;
    cylinderoidInfo cylinderoid;
    circleInformation current;
    double cylindroid_coeffs[7]; //x0, y0, theta, a, b, vx, vy

    ceres::Solver::Options options_;
    ceres::Solver::Summary summary;

    float PCA_high = 20, PCA_low = 1;

    // organization
    std::vector<circleInformation> candidate_line1, candidate_line2;
    bool b_width_line_found, b_height_line_found;
    cv::Vec2f width_direction_, height_direction_;
};

double CircleDetector::euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool CircleDetector::coEllipse(const circleInformation& a, const circleInformation& b){
    bool isPass = true;
    std::vector<cv::Point> ellipse_points;
    ellipse_points.insert(ellipse_points.end(), a.area.begin(), a.area.end());
    ellipse_points.insert(ellipse_points.end(), b.area.begin(), b.area.end());
    m_ellipsetemp = cv::fitEllipse(ellipse_points);
    
    // point num restriction
    if ((a.area.size() + b.area.size()) < (int)(CV_PI * (m_ellipsetemp.size.height + m_ellipsetemp.size.width) / 3)){
        isPass = false;
    }

    // pca magnitude restriction
    if (std::abs(a.pca_res.magnitude_comp1 - b.pca_res.magnitude_comp1) / (a.pca_res.magnitude_comp1 + b.pca_res.magnitude_comp1) > 0.3){
        isPass = false;
    }

    cv::Point2f a_direction = (cv::Point2f)a.pca_res.center - m_ellipsetemp.center;
    cv::Point2f b_direction = (cv::Point2f)b.pca_res.center - m_ellipsetemp.center;
    a_direction = a_direction / sqrt(a_direction.x * a_direction.x + a_direction.y * a_direction.y);
    b_direction = b_direction / sqrt(b_direction.x * b_direction.x + b_direction.y * b_direction.y);
    // pca direction restriction
    if (std::abs(a_direction.x * a.pca_res.comp1.x + a_direction.y * a.pca_res.comp1.y) > 0.4 ||  // sin(15^{circ}) = 0.2588
        std::abs(b_direction.x * b.pca_res.comp1.x + b_direction.y * b.pca_res.comp1.y) > 0.4){
    }

    // fitting error restriction
    float fit_error = 0.0;
    for (auto p : ellipse_points){
        float x_rot = cos(m_ellipsetemp.angle / 180.0 * 3.14) * (p.x - m_ellipsetemp.center.x) + sin(m_ellipsetemp.angle / 180.0 * 3.14) * (p.y - m_ellipsetemp.center.y);
        float y_rot = sin(m_ellipsetemp.angle / 180.0 * 3.14) * (p.x - m_ellipsetemp.center.x) - cos(m_ellipsetemp.angle / 180.0 * 3.14) * (p.y - m_ellipsetemp.center.y);
        fit_error += abs(sqrt(std::abs((x_rot * x_rot) * 4.0 / (m_ellipsetemp.size.width * m_ellipsetemp.size.width) + (y_rot * y_rot) * 4.0 / (m_ellipsetemp.size.height * m_ellipsetemp.size.height))) - 1);
    }
    fit_error /= ellipse_points.size();
    if (fit_error > 0.2){
        isPass = false;
    }
    
    // ellipse size restriction
    if (m_ellipsetemp.size.width > sensor_size_.height / std::min(cb.boardHeight, cb.boardWidth) || m_ellipsetemp.size.height > sensor_size_.height / std::min(cb.boardHeight, cb.boardWidth)){
        isPass = false;
    }
    
    // angle restriction
    float angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1, angle_now, angle_min_a, angle_max_a, angle_min_b, angle_max_b;
    for (auto point : a.area){
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
        if (angle_now > angle_max_x) angle_max_x = angle_now;
        if (angle_now < angle_min_x) angle_min_x = angle_now;
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, -point.x + m_ellipsetemp.center.x);
        if (angle_now > angle_max_y) angle_max_y = angle_now;
        if (angle_now < angle_min_y) angle_min_y = angle_now;
    }
    if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
        angle_max_a = angle_max_x;
        angle_min_a = angle_min_x;
    }else{
        angle_max_a = angle_max_y;
        angle_min_a = angle_min_y;
    }

    angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1;
    for (auto point : b.area){
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
        if (angle_now > angle_max_x) angle_max_x = angle_now;
        if (angle_now < angle_min_x) angle_min_x = angle_now;
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, -point.x + m_ellipsetemp.center.x);
        if (angle_now > angle_max_y) angle_max_y = angle_now;
        if (angle_now < angle_min_y) angle_min_y = angle_now;
    }
    if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
        angle_max_b = angle_max_x;
        angle_min_b = angle_min_x;
    }else{
        angle_max_b = angle_max_y;
        angle_min_b = angle_min_y;
    }    
    
    if (std::abs(angle_max_a - angle_min_a - angle_max_b + angle_min_b) > 90 || angle_max_a - angle_min_a < 90 || angle_max_b - angle_min_b < 90) {
        isPass = false;
    };

    // distance restriction
    double dist = euclideanDistance(a.pca_res.center, b.pca_res.center);
    if (dist > std::max(sensor_size_.height / (1.0 * cb.boardHeight), sensor_size_.width / (1.0 * cb.boardWidth))) {
        isPass = false;
    }
    
    if (isPass)
    {
        return true;
    }
    else
        return false;
}

void CircleDetector::connectedComponentLabeling(const cv::Mat& src, std::vector<std::vector<cv::Point>>& quadArea){
    cv::Mat img_labeled, stats, centroids;
    std::vector<bool> illegal;
    int nccomp_area = 0;
    quadArea.clear();
    nccomp_area = cv::connectedComponentsWithStats(src, img_labeled, stats, centroids, 8, 4, cv::CCL_GRANA);
    quadArea.resize(nccomp_area);
    illegal.resize(nccomp_area);
    fill(illegal.begin(), illegal.end(), 0);

    for (int i = 0; i < nccomp_area; i++){
        if (stats.at<int>(i, cv::CC_STAT_AREA) < 20 || stats.at<int>(i, cv::CC_STAT_AREA) > round(0.01 * src.cols * src.rows)){
            illegal[i] = true;
        }
    }
    for (int i = 0; i < img_labeled.rows; i++){
        for (int j = 0; j < img_labeled.cols; j++){
            if (!illegal[img_labeled.at<int>(i, j)])
                quadArea[img_labeled.at<int>(i, j)].push_back(cv::Point(j, i));
        }
    } 
    int count = 0;
    for (auto iter = quadArea.begin(); iter != quadArea.end(); ){
        if (illegal[count++]){
            iter = quadArea.erase(iter);
        }
        else{
            iter++;
        }
    }
}

CircleDetector::pcaInfo CircleDetector::pcaAnalysis(std::vector<cv::Point> points){
    cv::Mat data(points.size(), 2, CV_32F);
    for (int i = 0; i < data.rows; ++i) {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
    }

    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

    pcaInfo pca_now;

    pca_now.comp1 = cv::Point2d(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    pca_now.comp2 = cv::Point2d(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));
    pca_now.vertical = pca_now.comp1.x * pca_now.comp2.x + pca_now.comp1.y * pca_now.comp2.y;
    pca_now.magnitude_comp1 = pca.eigenvalues.at<float>(0, 0);
    pca_now.magnitude_comp2 = pca.eigenvalues.at<float>(1, 0);
    
    pca_now.center = cv::Point2d(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

    return pca_now;
}

double CircleDetector::distanceFromTwoClusters(const circleInformation& cluster_1, const circleInformation& cluster_2){
    cv::Point2f center_1, center_2;
    for (auto a:cluster_1.area){
        center_1 += (cv::Point2f)a;
    }
    for (auto b:cluster_2.area){
        center_2 += (cv::Point2f)b;
    }
    center_1 = cv::Point2f(center_1.x / cluster_1.area.size(), center_1.y / cluster_1.area.size());
    center_2 = cv::Point2f(center_2.x / cluster_2.area.size(), center_2.y / cluster_2.area.size());

    return euclideanDistance(center_1, center_2);
}

CircleDetector::cylinderoidInfo CircleDetector::fitCylindroid(std::vector<cv::Point3i> cylindroidPoints){
    int ave_timestamp = 0;
    int min_timestamp = 1e12, max_timestamp = 0;
    for (auto p:cylindroidPoints){
        ave_timestamp += p.z;
        if (p.z > max_timestamp) max_timestamp = p.z;
        if (p.z < min_timestamp) min_timestamp = p.z;
    }
    ave_timestamp /= cylindroidPoints.size();
    
    std::vector<cv::Point> ellipse_points;
    for (auto p:cylindroidPoints){
        ellipse_points.emplace_back(cv::Point(p.x, p.y));
    }
    m_ellipsetemp = cv::fitEllipse(ellipse_points);

    float fit_error = 0.0;
    for (auto p : ellipse_points){
        float x_rot = cos(m_ellipsetemp.angle / 180.0 * 3.14) * (p.x - m_ellipsetemp.center.x) + sin(m_ellipsetemp.angle / 180.0 * 3.14) * (p.y - m_ellipsetemp.center.y);
        float y_rot = sin(m_ellipsetemp.angle / 180.0 * 3.14) * (p.x - m_ellipsetemp.center.x) - cos(m_ellipsetemp.angle / 180.0 * 3.14) * (p.y - m_ellipsetemp.center.y);
        fit_error += abs(sqrt(std::abs((x_rot * x_rot) * 4.0 / (m_ellipsetemp.size.width * m_ellipsetemp.size.width) + (y_rot * y_rot) * 4.0 / (m_ellipsetemp.size.height * m_ellipsetemp.size.height))) - 1);
    }
    fit_error /= ellipse_points.size();
    
    double vx = 0, vy = 0;
    cylindroid_coeffs[0] = (double)m_ellipsetemp.center.x;
    cylindroid_coeffs[1] = (double)m_ellipsetemp.center.y;
    cylindroid_coeffs[2] = m_ellipsetemp.angle;
    cylindroid_coeffs[3] = m_ellipsetemp.size.width;
    cylindroid_coeffs[4] = m_ellipsetemp.size.height;
    cylindroid_coeffs[5] = vx;
    cylindroid_coeffs[6] = vy;
    
    ceres::Problem problem_;
    for (auto p:cylindroidPoints){
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CylindroidResidual, 1, 7>(
                new CylindroidResidual(p, ave_timestamp));
        problem_.AddResidualBlock(cost_function, nullptr, cylindroid_coeffs);
    }

    ceres::Solve(options_, &problem_, &summary);
    
    cylinderoidInfo temp_cylinderoid;
    temp_cylinderoid.area = ellipse_points;
    temp_cylinderoid.ellipse.center = cv::Point2d(cylindroid_coeffs[0], cylindroid_coeffs[1]);
    temp_cylinderoid.ellipse.size = cv::Size(cylindroid_coeffs[3], cylindroid_coeffs[4]);
    temp_cylinderoid.ellipse.angle = cylindroid_coeffs[2];
    temp_cylinderoid.timestamp = ave_timestamp;
    temp_cylinderoid.velo = cv::Point2d(cylindroid_coeffs[5], cylindroid_coeffs[6]);
    temp_cylinderoid.start_ts = min_timestamp;
    temp_cylinderoid.end_ts = max_timestamp;
     
    return temp_cylinderoid;
}

void CircleDetector::eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative){
    cv::Mat event_show = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, event_show, cv::COLOR_GRAY2BGR);
    cv::Mat imgMark = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::Mat plot = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::Mat plot1 = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, imgMark, cv::COLOR_GRAY2RGB);
    
    cv::Mat imgPos = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_positive, imgPos, cv::COLOR_GRAY2RGB);
    cv::Mat imgNeg = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_negative, imgNeg, cv::COLOR_GRAY2RGB);
       
    cv::Mat event_map_positive_8U, event_map_negative_8U;
    event_map_positive.convertTo(event_map_positive_8U, CV_8UC1);
    event_map_negative.convertTo(event_map_negative_8U, CV_8UC1);

    candidate_pos.clear();
    connectedComponentLabeling(event_map_positive_8U, quadArea_pos);
    for (auto quad : quadArea_pos){
        pcaInfo temp_pca = pcaAnalysis(quad);
    
        if (temp_pca.magnitude_comp1 > event_map_no_polarity.rows * 0.3 || temp_pca.magnitude_comp2 < 1 || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 > PCA_high || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 < PCA_low){continue;}

        circleInformation temp_circ;
        temp_circ.pca_res = temp_pca;
        temp_circ.area = quad;
        candidate_pos.push_back(temp_circ);
    }    

    candidate_neg.clear();
    connectedComponentLabeling(event_map_negative_8U, quadArea_neg);
    for (auto quad : quadArea_neg){
        pcaInfo temp_pca = pcaAnalysis(quad);
     
        if (temp_pca.magnitude_comp1 > event_map_no_polarity.rows * 0.3 || temp_pca.magnitude_comp2 < 1 || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 > PCA_high || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 < PCA_low){continue;}

        circleInformation temp_circ;
        temp_circ.pca_res = temp_pca;
        temp_circ.area = quad;
        candidate_neg.push_back(temp_circ);
    }

    candidate_full.clear();
    while (!candidate_pos.empty()) {
        current = candidate_pos.front();
        candidate_pos.pop_front();

        std::sort(candidate_neg.begin(), candidate_neg.end(), [this](const circleInformation& p1, const circleInformation& p2) {
            return distanceFromTwoClusters(current, p1) < distanceFromTwoClusters(current, p2);
        });

        for (auto it = candidate_neg.begin(); it != candidate_neg.end();) {
            if (distanceFromTwoClusters(current, *it) > sensor_size_.height / std::max(cb.boardHeight, cb.boardWidth)) break;

            if (coEllipse(current, *it)) {
                candidate_full.emplace_back(std::make_pair(current, *it));
                it = candidate_neg.erase(it);
                break;
            } else {
                ++it;
            }            
        }
    }

    candidate_circles.clear();
    for (auto cand_pair : candidate_full){
        std::vector<cv::Point3i> cylinderoid_points;
        for (auto p:cand_pair.first.area){
            int time_in_p = (timestamps[p.y * event_map_no_polarity.cols + p.x] - timestamp).toSec() * 1e6;
            cylinderoid_points.emplace_back(cv::Point3i(p.x, p.y, time_in_p));
        }
        for (auto p:cand_pair.second.area){
            int time_in_p = (timestamps[p.y * event_map_no_polarity.cols + p.x] - timestamp).toSec() * 1e6;
            cylinderoid_points.emplace_back(cv::Point3i(p.x, p.y, time_in_p));
        }
        cylinderoid = fitCylindroid(cylinderoid_points);

        circleInformation temp_circ;
        temp_circ.area = cylinderoid.area;
        temp_circ.ellip_info = cylinderoid.ellipse;
        ros::Duration ns_comp, st_comp, ed_comp;
        ns_comp.fromNSec(cylinderoid.timestamp * 1e3);
        st_comp.fromNSec(cylinderoid.start_ts * 1e3);
        ed_comp.fromNSec(cylinderoid.end_ts * 1e3);
        temp_circ.timestamp = timestamp + ns_comp;
        temp_circ.start_ts = timestamp + st_comp;
        temp_circ.end_ts = timestamp + ed_comp;
        temp_circ.velo = cylinderoid.velo;
        candidate_circles.push_back(temp_circ);

        ellipse(imgMark, m_ellipsetemp, cv::Scalar(140/255, 1, 1), 2);
    }
    
    organizeCircles(event_map_no_polarity);

    if (onboard_circles.size() != cb.boardHeight * cb.boardWidth) return;
    circle_array.circles.clear();

    // Publish circle messages
    for (auto circle : onboard_circles){
        circle_msg.x = circle.ellip_info.center.x;
        circle_msg.y = circle.ellip_info.center.y;
        circle_msg.x_grid = circle.grid_pose.x;
        circle_msg.y_grid = circle.grid_pose.y;
        circle_msg.velo_x = circle.velo.x;
        circle_msg.velo_y = circle.velo.y;
        circle_msg.timestamp = circle.timestamp;
        circle_msg.start_ts = circle.start_ts;
        circle_msg.end_ts = circle.end_ts;
        circle_array.circles.push_back(circle_msg);
    }
    circle_array.header.stamp = timestamp;
    circle_pub_.publish(circle_array);
    ROS_INFO("Publishing circle message");

    write_bag.write("/circle_ev/circleArray", ros::Time::now(), circle_array);

    return;
}

float CircleDetector::distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction){
    cv::Point2f vector_to_input = input_point - line_point;
    double vector_magnitude = cv::norm(vector_to_input);
    cv::Point2f unit_vector = vector_to_input / vector_magnitude;

    if (cv::norm(line_direction) != 1){
        line_direction /= cv::norm(line_direction);
    }

    double dot_product = unit_vector.dot(line_direction);
    double distance = std::abs(vector_magnitude * std::sin(std::acos(dot_product)));

    return distance;
}

void CircleDetector::organizeCircles(const cv::Mat& event_map){
    cv::Mat image_plot = cv::Mat::zeros(event_map.size(), CV_32FC3);;
    cv::cvtColor(event_map, image_plot, cv::COLOR_GRAY2BGR);
    
    onboard_circles.clear();
    if (candidate_circles.size() < cb.boardHeight * cb.boardWidth) {
        return;
    }
    
    bool b_reverse = false;
    for (auto &now_circle:candidate_circles){
        std::vector<std::pair<float, circleInformation>> dist_from_nowcircle;
        std::vector<circleInformation> width_circle, height_circle;
        
        for (int i = 0; i < candidate_circles.size(); i++) {
            dist_from_nowcircle.push_back(std::make_pair(euclideanDistance(now_circle.ellip_info.center, candidate_circles[i].ellip_info.center), candidate_circles[i]));
        }
        // sort by distance
        std::sort(dist_from_nowcircle.begin(), dist_from_nowcircle.end(), 
                [](const std::pair<float, circleInformation> &a, const std::pair<float, circleInformation> &b) {
                    return a.first < b.first;
                });
        b_width_line_found = false;
        b_height_line_found = false;
        float min_dist_ = dist_from_nowcircle[1].first;
        for (int i = 1; i < dist_from_nowcircle.size(); i++){
            std::vector<circleInformation> width_circle_temp, height_circle_temp;
            int fitted_num = 0;
            cv::Vec2f direction_(now_circle.ellip_info.center.x - dist_from_nowcircle[i].second.ellip_info.center.x, now_circle.ellip_info.center.y - dist_from_nowcircle[i].second.ellip_info.center.y);

            for (int j = i + 1; j < dist_from_nowcircle.size(); j++){
                if (distanceFromLine(dist_from_nowcircle[j].second.ellip_info.center, now_circle.ellip_info.center, direction_) / euclideanDistance(dist_from_nowcircle[j].second.ellip_info.center, now_circle.ellip_info.center) < 0.1){
                    fitted_num++;
                    width_circle_temp.push_back(dist_from_nowcircle[j].second);
                    height_circle_temp.push_back(dist_from_nowcircle[j].second);
                }
            }
            if (!b_width_line_found && fitted_num == cb.boardWidth - 2){
                b_width_line_found = true;
                width_direction_ = direction_;
                width_circle_temp.push_back(dist_from_nowcircle[i].second);
                width_circle_temp.push_back(now_circle);
                width_circle = width_circle_temp;
            }
            if (!b_height_line_found && fitted_num == cb.boardHeight - 2){
                b_height_line_found = true;
                height_direction_ = direction_;
                height_circle_temp.push_back(dist_from_nowcircle[i].second);
                height_circle_temp.push_back(now_circle);
                height_circle = height_circle_temp;
            }
            
            if (b_width_line_found && b_height_line_found){
                std::sort(width_circle.begin(), width_circle.end(), std::abs(width_direction_[0]) > std::abs(width_direction_[1]) ? 
                    [](const circleInformation& a, const circleInformation& b) {
                        return a.ellip_info.center.x < b.ellip_info.center.x;
                    } :
                    [](const circleInformation& a, const circleInformation& b) {
                        return a.ellip_info.center.y < b.ellip_info.center.y;
                    });
                std::sort(height_circle.begin(), height_circle.end(), std::abs(height_direction_[0]) > std::abs(height_direction_[1]) ? 
                    [](const circleInformation& a, const circleInformation& b) {
                        return a.ellip_info.center.x < b.ellip_info.center.x;
                    } :
                    [](const circleInformation& a, const circleInformation& b) {
                        return a.ellip_info.center.y < b.ellip_info.center.y;
                    });
                circleInformation temp_corner;
                temp_corner = now_circle;
                for (int k = 0; k < width_circle.size(); k++){
                    if (width_circle[k].ellip_info.center == now_circle.ellip_info.center){
                        temp_corner.grid_pose.x = b_reverse ? cb.boardWidth - k - 1 : k;
                    }
                }
                for (int k = 0; k < height_circle.size(); k++){
                    if (height_circle[k].ellip_info.center == now_circle.ellip_info.center){
                        temp_corner.grid_pose.y = b_reverse ? cb.boardHeight - k - 1 : k;
                    }
                }   
                onboard_circles.push_back(temp_corner);
                break;
            }
        }
    }
    if (onboard_circles.size() != cb.boardWidth * cb.boardHeight){
        onboard_circles.clear();
        return;
    }

    std::vector<cv::Scalar> scas;
    scas.push_back(cv::Scalar(34/255.0, 147/255.0, 238/255.0));
    scas.push_back(cv::Scalar(217/255.0, 47/255.0, 54/255.0));
    scas.push_back(cv::Scalar(49/255.0, 63/255.0, 216/255.0));
    for (auto a : onboard_circles){
        int y_g = a.grid_pose.y + (a.grid_pose.x + 1) / 6;
        int x_g = (a.grid_pose.x + 1) % 6;
        ellipse(image_plot, a.ellip_info, scas[a.grid_pose.y], 2);
        for (auto b : onboard_circles){
            if (b.grid_pose.x == x_g && b.grid_pose.y == y_g){
                cv::line(image_plot, a.ellip_info.center, b.ellip_info.center, scas[a.grid_pose.y], 2, 8, 0);
            }
        }
    }
    cv::imshow("org corners", image_plot);
    cv::waitKey(1);
}

void CircleDetector::readBag(std::string bagName){
    read_bag.open(bagName, rosbag::bagmode::Read);
    write_bag.open(store_path + "/circle_info.bag", rosbag::bagmode::Write);

    rosbag::View view(read_bag, rosbag::TopicQuery("/eventmap"));

    for (const rosbag::MessageInstance& m : view) {
        eventmap_generator::eventmap::ConstPtr msg = m.instantiate<eventmap_generator::eventmap>();
        try {
            static bool first_init = true;
            if (first_init){
                sensor_size_.height = msg->eventmap.height;
                sensor_size_.width = msg->eventmap.width;
            }
            sensor_msgs::ImageConstPtr eventmap_msg = boost::make_shared<sensor_msgs::Image>(msg->eventmap);
            cv_bridge::CvImageConstPtr cv_ptr;
            cv_ptr = cv_bridge::toCvShare(eventmap_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat image = cv_ptr->image;

            if (image.channels() == 3)
                cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

            cv::Mat event_no_pola = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
            cv::Mat event_positive = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
            cv::Mat event_negative = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
            for(size_t x = 0; x < sensor_size_.width; ++x){
                for(size_t y = 0; y < sensor_size_.height; ++y)
                {
                    if ((image.at<float>(y, x) - 0.5) > 0.4){
                        event_no_pola.at<float>(y, x) = 255;
                        event_positive.at<float>(y, x) = 255;
                    }
                    else if ((image.at<float>(y, x) - 0.5) < -0.4){
                        event_no_pola.at<float>(y, x) = 255;
                        event_negative.at<float>(y, x) = 255;
                    }
                }
            }
            ROS_INFO("Receive eventmap");
            timestamp = msg->eventmap.header.stamp;
            timestamps = msg->timestamps;
            ROS_INFO("The timestamp is %d.%d:", timestamp.sec, timestamp.nsec);
            eventMaptDetect(event_no_pola, event_positive, event_negative);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    read_bag.close();
    write_bag.close();

    ros::shutdown();
}

void CircleDetector::eventmapCallback(const eventmap_generator::eventmap& msg) {
    try {
        static bool first_init = true;
        if (first_init){
            sensor_size_.height = msg.eventmap.height;
            sensor_size_.width = msg.eventmap.width;
        }
        sensor_msgs::ImageConstPtr eventmap_msg = boost::make_shared<sensor_msgs::Image>(msg.eventmap);
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(eventmap_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat image = cv_ptr->image;

        if (image.channels() == 3)
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        cv::Mat event_no_pola = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
        cv::Mat event_positive = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
        cv::Mat event_negative = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
        for(size_t x = 0; x < sensor_size_.width; ++x){
            for(size_t y = 0; y < sensor_size_.height; ++y)
            {
                if ((image.at<float>(y, x) - 0.5) > 0.4){
                    event_no_pola.at<float>(y, x) = 255;
                    event_positive.at<float>(y, x) = 255;
                }
                else if ((image.at<float>(y, x) - 0.5) < -0.4){
                    event_no_pola.at<float>(y, x) = 255;
                    event_negative.at<float>(y, x) = 255;
                }
            }
        }
        ROS_INFO("Receive eventmap");
        timestamp = msg.eventmap.header.stamp;
        timestamps = msg.timestamps;
        ROS_INFO("The timestamp is %d.%d:", timestamp.sec, timestamp.nsec);
        eventMaptDetect(event_no_pola, event_positive, event_negative);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

} // namespace circle_detector
