#pragma once

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "corner_msgs/corner.h"
#include "corner_msgs/cornerArray.h"

#include "string.h"
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#define PI 3.1415926

namespace corner_detector{
struct cornerInformation {
    cv::Point corner_position;
	cv::Point2f corner_position_subpixel;
	float response_score = -1;
    float angle_black_edge, angle_white_edge;
    cv::Point grid_pose;
};

struct Chessboard{
    int boardHeight, boardWidth;
};

class CornerTemplate {
public:
    CornerTemplate();
	CornerTemplate(int maskL);
	~CornerTemplate();

	cv::Mat tmp;
    cv::Mat hypersurface_temp;
    int maskSurface = 5;
	int maskSurfaceL = 2 * maskSurface + 1;

private:
    int maskL = 13;    
    cv::Mat hypersurface_temp_x2, hypersurface_temp_y2, hypersurface_temp_xy, hypersurface_temp_x, hypersurface_temp_y;
};

CornerTemplate::CornerTemplate() {
    //Search for the template in the dic. If not exist, generate once.
    
    std::string tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        cv::Mat tmpMSAA, tmpCrop;
        tmpMSAA = cv::Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = cv::Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(cv::Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, cv::Size(maskL, maskL), 0, 0, cv::INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        std::cout << "Write crosspoint template success!\n";
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = cv::imread(tmpFile);
        cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }

    //Generate the order of the hypersurface neighborhood
    hypersurface_temp_x2 = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y2 = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_xy = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_x  = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y  = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp    = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    for (int i = 0; i < maskSurfaceL; i++)
        for (int j = 0; j < maskSurfaceL; j++) {
            hypersurface_temp_x2.at<float>(i * maskSurfaceL + j, 0) = i * i;
            hypersurface_temp_y2.at<float>(i * maskSurfaceL + j, 0) = j * j;
            hypersurface_temp_xy.at<float>(i * maskSurfaceL + j, 0) = i * j;
            hypersurface_temp_x.at<float>(i * maskSurfaceL + j, 0) = i;
            hypersurface_temp_y.at<float>(i * maskSurfaceL + j, 0) = j;
        }
    hconcat(hypersurface_temp_x2, hypersurface_temp_xy, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y2, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_x, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y, hypersurface_temp);
    hconcat(hypersurface_temp, cv::Mat::ones(maskSurfaceL * maskSurfaceL, 1, CV_32FC1), hypersurface_temp); 
}

CornerTemplate::CornerTemplate(int maskL) {
    //Search for the template in the dic. If not exist, generate once.
    
    std::string tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        cv::Mat tmpMSAA, tmpCrop;
        tmpMSAA = cv::Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = cv::Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(cv::Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, cv::Size(maskL, maskL), 0, 0, cv::INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        std::cout << "Write file success!\n";
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = cv::imread(tmpFile);
        cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }
}

CornerTemplate::~CornerTemplate() {
    tmp.release();
}

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh);
    ~ImageProcessor(){
        image_sub_.shutdown();
        corner_pub_.shutdown();
    };

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    void readBag(std::string bagName);
    void extractCorner(const cv::Mat& input_image);
    void cornerPreFilter(const cv::Mat& image);
    void templateMatching(const cv::Mat& image);
    void organizeCorner(const cv::Mat& image);
    float distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction);
    float distanceFromTwoPoints(cv::Point2f point_1, cv::Point2f point_2);

    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    ros::Publisher corner_pub_ = nh_.advertise<corner_msgs::cornerArray>("/corner_gs/cornerArray",10);
    rosbag::Bag read_bag, write_bag;
    std::string bagName, store_path;

    CornerTemplate ct;
    Chessboard cb;
    ros::Time input_timestamp;
    
    std::vector<cornerInformation> corners;

    // corner pre filter
    cv::Mat image_blur;
    int maskR = 6;
	int kernal_size = 7;
	int sigma = 3;
	cv::Mat Gx, Gy, Gxx, Gyy, Gxy, G_score, G_score_after_NMS, score_sequence;
    float G_filtermin, G_filtermax;
	std::priority_queue <float, std::vector<float>, std::less<float> > Q;

    // corner template matching
    int maskL = 13, maskTemR = (maskL - 1) / 2;
	int edgeIdx, directIdx;
	float angle1, angle2, edge_angle, direction_angle;
	float response_score_max = -1, T_temp_max = 0.85, T_response = 0.2;
	cv::Mat hypersurface_coeffs, img_hypersurface;
	cv::Mat coeffs, roots;

    // corner organization
    float T_dis = 10.0, T_angle = 20.0;
    std::vector<cornerInformation> candidate_line1, candidate_line2, candidate_corners, onboard_corners;
    bool b_width_line_found, b_height_line_found;
    cv::Vec2f width_direction_, height_direction_;
};

ImageProcessor::ImageProcessor(ros::NodeHandle& nh) : nh_(nh){
    nh_.param("BoardHeight", cb.boardHeight, 3);
    nh_.param("BoardWidth", cb.boardWidth, 6);
    nh_.param("BagName", bagName, std::string("record.bag"));
    nh_.param("StorePath", store_path, std::string("intermediateBags"));
    
    // Corner template generation
    CornerTemplate ct(maskL);

    readBag(bagName);
}

void ImageProcessor::readBag(std::string bagName){
    read_bag.open(bagName, rosbag::bagmode::Read);
    write_bag.open(store_path + "/corner_info.bag", rosbag::bagmode::Write);

    rosbag::View view(read_bag, rosbag::TopicQuery("/hik_camera/image"));

    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
            if (image.channels() == 3)
                cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            if (image.channels() == 1) {
                image.convertTo(image, CV_32FC1, 1/255.0);
            }
            input_timestamp = msg->header.stamp;
            extractCorner(image);
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

void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        if (image.channels() == 3)
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        if (image.channels() == 1) {
            image.convertTo(image, CV_32FC1, 1/255.0);
        }
        input_timestamp = msg->header.stamp;
        extractCorner(image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void ImageProcessor::extractCorner(const cv::Mat& input_image) {
    corners.clear();

    cornerPreFilter(input_image);
    templateMatching(input_image);
    organizeCorner(input_image);
    
    if (onboard_corners.size() != cb.boardHeight * cb.boardWidth) return;
    std::vector<cv::Point2f> corners_refined;
    for (int i = 0; i < onboard_corners.size(); i++){
        corners_refined.emplace_back((cv::Point2f)onboard_corners[i].corner_position);
    }
    cv::cornerSubPix(input_image, corners_refined, cv::Size(7, 7), cv::Size(-1, -1), cv::TermCriteria());
    for (int i = 0; i < corners_refined.size(); i++){
        onboard_corners[i].corner_position_subpixel = corners_refined[i];
    }

    //Publish corner message
    corner_msgs::cornerArray corner_array;
    corner_msgs::corner corner_msg;
    
    for (int i = 0; i < onboard_corners.size(); i++){
        corner_msg.x = onboard_corners[i].corner_position_subpixel.x;
        corner_msg.y = onboard_corners[i].corner_position_subpixel.y;
        corner_msg.x_grid = onboard_corners[i].grid_pose.x;
        corner_msg.y_grid = onboard_corners[i].grid_pose.y;
        corner_array.corners.emplace_back(corner_msg);
    }
    ros::Duration time_gap_min(0, 0*1e6); // ms
    corner_array.timestamp = input_timestamp + time_gap_min;
    corner_pub_.publish(corner_array);
    ROS_INFO("Publishing corner message");

    write_bag.write("/corner_gs/cornerArray", ros::Time::now(), corner_array);

    return;
}

void ImageProcessor::cornerPreFilter(const cv::Mat& image){
    Q.empty();

    cv::GaussianBlur(image, image_blur, cv::Size(kernal_size, kernal_size), sigma);
    cv::Scharr(image_blur, Gx, CV_32FC1, 1, 0);
    cv::Scharr(image_blur, Gy, CV_32FC1, 0, 1);

    cv::Scharr(Gx, Gxx, CV_32FC1, 1, 0);
    cv::Scharr(Gy, Gyy, CV_32FC1, 0, 1);
    cv::Scharr(Gx, Gxy, CV_32FC1, 0, 1);
    
    G_score = Gxy.mul(Gxy) - Gxx.mul(Gyy);

    cv::dilate(G_score, G_score_after_NMS, cv::Mat());
    for (int i = maskR; i < image_blur.rows - maskR; i++)
        for (int j = maskR; j < image_blur.cols - maskR; j++)
            if (G_score.ptr<float>(i)[j] == G_score_after_NMS.ptr<float>(i)[j]) {
                Q.push(G_score_after_NMS.ptr<float>(i)[j]);
            }
            else {
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            }
             
    G_filtermax = Q.top();
    for (int i = 0; i < cb.boardHeight * cb.boardWidth * 3; i++) Q.pop();
    G_filtermin = Q.top();
    
    for (int i = maskR; i < image_blur.rows - maskR; i++)
        for (int j = maskR; j < image_blur.cols - maskR; j++) {
    		if (G_score_after_NMS.ptr<float>(i)[j] < G_filtermin)
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            else {
                cornerInformation temporal_corner;
                temporal_corner.corner_position = cv::Point(j ,i);
                corners.emplace_back(temporal_corner);
            }
        }	
        
    return;
}

void ImageProcessor::templateMatching(const cv::Mat& image){
    cv::Mat image_plot = cv::Mat::zeros(image.size(), CV_32FC3);;
    cv::cvtColor(image, image_plot, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < corners.size(); i++) {
        cv::Rect rect(corners[i].corner_position.x - ct.maskSurface, corners[i].corner_position.y - ct.maskSurface, ct.maskSurfaceL, ct.maskSurfaceL);
        img_hypersurface = image(rect).clone();
        img_hypersurface = img_hypersurface.reshape(1, ct.maskSurfaceL * ct.maskSurfaceL);
        
        solve(ct.hypersurface_temp, img_hypersurface, hypersurface_coeffs, cv::DECOMP_SVD);
        
        coeffs = (cv::Mat_<float>(3, 1) << hypersurface_coeffs.at<float>(0, 0), -1 * hypersurface_coeffs.at<float>(1, 0), hypersurface_coeffs.at<float>(2, 0));
        solvePoly(coeffs, roots);
        
        angle1 = atan(roots.at<float>(0, 0)) * 180.0 / PI;
        angle2 = atan(roots.at<float>(1, 0)) * 180.0 / PI;
        
        if ((angle1 * angle2 * hypersurface_coeffs.at<float>(0, 0)) < 0) {
            corners[i].angle_white_edge = std::max(angle1, angle2);
            corners[i].angle_black_edge = std::min(angle1, angle2);
        }
        else {
            corners[i].angle_white_edge = std::min(angle1, angle2);
            corners[i].angle_black_edge = std::max(angle1, angle2);
        }
    }

    for (int i = 0; i < corners.size(); i++) {        
        edgeIdx   = round((corners[i].angle_black_edge + 135) / 5);
        directIdx = round((corners[i].angle_white_edge + 135) / 5);

        edgeIdx %= 36;
        directIdx %= 36;
        
        if (edgeIdx < 0 || edgeIdx > 35) edgeIdx = 0;
        if (directIdx < 0 || directIdx > 35) directIdx = 0;
        
        cv::Mat tmpCrop(ct.tmp, cv::Rect(edgeIdx * maskL, directIdx * maskL, maskL, maskL));
        cv::Mat crop(image, cv::Rect(corners[i].corner_position.x - maskTemR, corners[i].corner_position.y - maskTemR, maskL, maskL));

        cv::Scalar meanTmp, meanCrop, stdTmp, stdCrop;
        meanStdDev(tmpCrop, meanTmp, stdTmp);
        meanStdDev(crop, meanCrop, stdCrop);

        float covar = (tmpCrop - meanTmp).dot(crop - meanCrop) / (maskL * maskL);
        corners[i].response_score = covar / (stdTmp[0] * stdCrop[0]);
        if (corners[i].response_score > response_score_max) {
            response_score_max = corners[i].response_score;
        }
    }

    if (response_score_max < T_temp_max) corners.clear();

    for (std::vector<cornerInformation>::iterator it = corners.begin(); it != corners.end();)
    {
        if (((*it).response_score) < (response_score_max - T_response))
            it = corners.erase(it);
        else
            it++;
    }
    
    return;
}

float ImageProcessor::distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction){
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

inline float ImageProcessor::distanceFromTwoPoints(cv::Point2f point_1, cv::Point2f point_2){
    return std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y));
}

bool sortByX(const cornerInformation& a, const cornerInformation& b) {
    return a.corner_position.x < b.corner_position.x;
}

bool sortByY(const cornerInformation& a, const cornerInformation& b) {
    return a.corner_position.y < b.corner_position.y;
}

void ImageProcessor::organizeCorner(const cv::Mat& image){
    cv::Mat image_plot = cv::Mat::zeros(image.size(), CV_32FC3);;
    cv::cvtColor(image, image_plot, cv::COLOR_GRAY2BGR);

    onboard_corners.clear();
    candidate_corners.clear();
    if (corners.size() < cb.boardHeight * cb.boardWidth) {
        cv::imshow("org corners", image_plot);
        cv::waitKey(1);
        return;
    }

    // sort by scores
    std::sort(corners.begin(), corners.end(), 
              [](const cornerInformation &a, const cornerInformation &b) {
                  return a.response_score > b.response_score;
              });
    
    cornerInformation best_corner = corners[0];

    candidate_corners.emplace_back(best_corner);
    for (auto it = corners.begin() + 1; it != corners.end(); it++) {
        if ((abs(it->angle_black_edge - best_corner.angle_black_edge) < T_angle && abs(it->angle_white_edge - best_corner.angle_white_edge) < T_angle) || (abs(it->angle_black_edge - best_corner.angle_white_edge) < T_angle && abs(it->angle_white_edge - best_corner.angle_black_edge) < T_angle)) {
            candidate_corners.emplace_back(*it);
        }
        else if ((abs(abs(it->angle_black_edge - best_corner.angle_black_edge) - 180) < T_angle && abs(it->angle_white_edge - best_corner.angle_white_edge) < T_angle) || (abs(it->angle_black_edge - best_corner.angle_black_edge) < T_angle && abs(abs(it->angle_white_edge - best_corner.angle_white_edge) - 180) < T_angle) || (abs(abs(it->angle_black_edge - best_corner.angle_white_edge) - 180) < T_angle && abs(it->angle_white_edge - best_corner.angle_black_edge) < T_angle) || (abs(it->angle_black_edge - best_corner.angle_white_edge) < T_angle && abs(abs(it->angle_white_edge - best_corner.angle_black_edge) - 180) < T_angle)){
            candidate_corners.emplace_back(*it);
        }
    }
    
    if (candidate_corners.size() < cb.boardHeight * cb.boardWidth) {
        cv::imshow("org corners", image_plot);
        cv::waitKey(1);
        return;
    }

    std::vector<std::pair<float, cornerInformation>> dist_from_bestcorner;
    for (int i = 1; i < candidate_corners.size(); i++) {
        dist_from_bestcorner.push_back(std::make_pair(distanceFromTwoPoints(best_corner.corner_position, candidate_corners[i].corner_position), candidate_corners[i]));
    }
    // sort by distance
    std::sort(dist_from_bestcorner.begin(), dist_from_bestcorner.end(), 
              [](const std::pair<float, cornerInformation> &a, const std::pair<float, cornerInformation> &b) {
                  return a.first < b.first;
              });
    b_width_line_found = false;
    b_height_line_found = false;
    float min_dist_ = dist_from_bestcorner[0].first;
    for (int i = 0; i < dist_from_bestcorner.size(); i++){
        if (dist_from_bestcorner[i].first > 1.2 * min_dist_) break;
        
        int fitted_num = 0;
        cv::Vec2f direction_(best_corner.corner_position.x - dist_from_bestcorner[i].second.corner_position.x, best_corner.corner_position.y - dist_from_bestcorner[i].second.corner_position.y);

        for (int j = i + 1; j < dist_from_bestcorner.size(); j++){
            if (distanceFromLine(dist_from_bestcorner[j].second.corner_position, best_corner.corner_position, direction_) / distanceFromTwoPoints(dist_from_bestcorner[j].second.corner_position, best_corner.corner_position) < 0.05){
                fitted_num++;
            }
        }
        if (!b_width_line_found && fitted_num == cb.boardWidth - 2){
            cv::Vec2f width_line = (dist_from_bestcorner[i].second.corner_position.y - best_corner.corner_position.y, dist_from_bestcorner[i].second.corner_position.x - best_corner.corner_position.x);
            width_direction_ = direction_;
            b_width_line_found = true;
        }
        if (!b_height_line_found && fitted_num == cb.boardHeight - 2){
            cv::Vec2f height_line = (dist_from_bestcorner[i].second.corner_position.y - best_corner.corner_position.y, dist_from_bestcorner[i].second.corner_position.x - best_corner.corner_position.x);
            height_direction_ = direction_;
            b_height_line_found = true;
        }
        
        if (b_width_line_found && b_height_line_found){
            break;
        } 
    }
   
    std::vector<std::vector<cornerInformation>> corner_cluster;
    std::vector<cornerInformation> height_closest_corners, width_corners;
    height_closest_corners.push_back(best_corner);
    for (int i = 0; i < dist_from_bestcorner.size(); i++){
        if (distanceFromLine(dist_from_bestcorner[i].second.corner_position, best_corner.corner_position, height_direction_) / distanceFromTwoPoints(dist_from_bestcorner[i].second.corner_position, best_corner.corner_position) < 0.05){
            height_closest_corners.push_back(dist_from_bestcorner[i].second);
        }
    }
    for (auto height_corner : height_closest_corners){
        width_corners.clear();
        width_corners.push_back(height_corner);
        for (auto corner : dist_from_bestcorner){
            if (distanceFromLine(corner.second.corner_position, height_corner.corner_position, width_direction_) / distanceFromTwoPoints(corner.second.corner_position, height_corner.corner_position) < 0.1){
                width_corners.push_back(corner.second);
            }
        }
        if (width_corners.size() < cb.boardWidth) 
        {
            cv::imshow("org corners", image_plot);
            cv::waitKey(1);
            return;
        }
        corner_cluster.push_back(width_corners);
    }
    if (corner_cluster.size() < cb.boardHeight) {
        cv::imshow("org corners", image_plot);
        cv::waitKey(1);
        return;
    }
    
    //sort twice
    for (auto& cluster : corner_cluster) {
        std::sort(cluster.begin(), cluster.end(), std::abs(width_direction_[0]) > std::abs(width_direction_[1]) ? sortByX : sortByY);
    }

    std::sort(corner_cluster.begin(), corner_cluster.end(), std::abs(width_direction_[0]) > std::abs(width_direction_[1]) ?
              [](const std::vector<cornerInformation>& a, const std::vector<cornerInformation>& b) {
                  return a[0].corner_position.y < b[0].corner_position.y;
              } :
              [](const std::vector<cornerInformation>& a, const std::vector<cornerInformation>& b) {
                  return a[0].corner_position.x > b[0].corner_position.x;
              });
    
    cv::Point2f sample_direction_width  = corner_cluster[0][0].corner_position - corner_cluster[0][1].corner_position;
    cv::Point2f sample_direction_height = corner_cluster[0][0].corner_position - corner_cluster[1][0].corner_position;
    cv::Point sample_position_1 = (sample_direction_width + sample_direction_height) * 0.08;
    cv::Point sample_position_2 = -(sample_direction_width + sample_direction_height) * 0.08;
    cv::Point sample_position_3 = (sample_direction_width - sample_direction_height) * 0.08;
    cv::Point sample_position_4 = -(sample_direction_width - sample_direction_height) * 0.08;
    bool b_reverse = false;
    if (image.at<float>(corner_cluster[0][0].corner_position + sample_position_1) + image.at<float>(corner_cluster[0][0].corner_position + sample_position_2) - (image.at<float>(corner_cluster[0][0].corner_position + sample_position_3) + image.at<float>(corner_cluster[0][0].corner_position + sample_position_4)) < 0.3){
        b_reverse = true;
    }
    for (int i = 0; i < corner_cluster.size(); i++){
        for (int j = 0; j < corner_cluster[i].size(); j++){
            cornerInformation temp_corner;
            temp_corner = corner_cluster[i][j];
            temp_corner.grid_pose.x = b_reverse ? j : cb.boardWidth - j - 1;
            temp_corner.grid_pose.y = b_reverse ? i : cb.boardHeight - i - 1;
            onboard_corners.push_back(temp_corner);
        }
    }
    std::vector<cv::Scalar> scas;
    scas.push_back(cv::Scalar(34/255.0, 147/255.0, 238/255.0));
    scas.push_back(cv::Scalar(217/255.0, 47/255.0, 54/255.0));
    scas.push_back(cv::Scalar(49/255.0, 63/255.0, 216/255.0));
    for (auto a : onboard_corners){
        int y_g = a.grid_pose.y + (a.grid_pose.x + 1) / 6;
        int x_g = (a.grid_pose.x + 1) % 6;
        cv::circle(image_plot, a.corner_position, 6, cv::Scalar(0, 200, 150), 2);
        for (auto b : onboard_corners){
            if (b.grid_pose.x == x_g && b.grid_pose.y == y_g){
                cv::line(image_plot, a.corner_position, b.corner_position, scas[a.grid_pose.y], 3, 8, 0);
            }
        }
    }
    cv::imshow("org corners", image_plot);
    cv::waitKey(1);
}

} // namespace corner_detector
