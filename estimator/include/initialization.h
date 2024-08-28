#pragma once
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>
#include <utils/parameter_struct.h>
#include <spline/trajectory.h>

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/rotation.h>

#define CONV_CAM 0
#define EV_CAM 1
#define BOTH 2

namespace estimator{

class Initializer{
public:
    typedef std::shared_ptr<Initializer> Ptr;

    Initializer(Camera::Ptr event_camera, Camera::Ptr convent_camera, CalibBoard::Ptr calib_board)
        : event_camera_(event_camera), convent_camera_(convent_camera), calib_board_(calib_board)
    {
        b_conv_initialized = false;
        b_ev_initialized = false;
        b_both_initialized = false;
    };

    bool convInitialSucc(){ return b_conv_initialized;}
    bool evInitialSucc(){ return b_ev_initialized;}
    bool judgeBufferStatus(int cam_type){
        if (cam_type == 0){
            if (getCornerBufferNum() < initial_window_size){
                ROS_INFO("Conv cam no sufficient frames, now: %d, %d", getCornerBufferNum(), initial_window_size);
                return false;
            }
            return true;
        }
        else{
            if (getCircleBufferNum() < initial_window_size){
                ROS_INFO("EV cam no sufficient frames, now: %d", getCircleBufferNum());
                return false;
            }
            return true;
        }
    }
    
    void processConv();
    void processEv();
    void estimateInitialExtrinsic();

    std::vector<corner_msgs::cornerArray> corner_buffer_, corner_buffer_selected;
    std::deque<circle_msgs::circleArray> circle_buffer_, circle_buffer_selected;
    int initial_window_size = 10;
    float square_size;
    cv::Size conv_cam_size, ev_cam_size;
    double epoch_time;

private:
    double computeReprojectionError(const cv::Mat& H, const std::vector<cv::Point2f>& point_1, const std::vector<cv::Point2f>& point_2) {
        double total_error = 0.0;
        int point_count = static_cast<int>(point_1.size());

        for (int i = 0; i < point_count; ++i) {
            cv::Point2f p1 = point_1[i];
            cv::Point3d p1_homogeneous((double)p1.x, (double)p1.y, 1.0);
            cv::Mat p1_transformed = H * cv::Mat(p1_homogeneous);
            p1_transformed /= p1_transformed.at<double>(2);

            cv::Point2f p2 = point_2[i];
            double error = cv::norm(cv::Point2f(p1_transformed.at<double>(0), p1_transformed.at<double>(1)) - p2);
            total_error += error;
        }

        return total_error / point_count; 
    }
    int getCornerBufferNum(){ 
        corner_buffer_selected.clear();
        legal_corner_size = 0;
        if (corner_buffer_.empty()) return 0;
        cv::Mat w, u, vt, rotationMatrix, rvec;
        for (auto c:corner_buffer_){
            if ((c.timestamp - corner_buffer_.front().timestamp).toSec() > epoch_time) break;
            static bool fisrt_judge = true;
            if (fisrt_judge){
                fisrt_judge = false;
                last_corner = c;
                corner_buffer_selected.emplace_back(c);
            }
            float dist = 0;
            std::vector<cv::Point2f> point_1, point_2;
            for (auto corner_last:last_corner.corners){
                for (auto corner_now:c.corners){
                    if (corner_last.x_grid == corner_now.x_grid && corner_last.y_grid == corner_now.y_grid){
                        dist += std::sqrt((corner_last.x - corner_now.x) * (corner_last.x - corner_now.x) + (corner_last.y - corner_now.y) * (corner_last.y - corner_now.y));
                        point_1.emplace_back(cv::Point2f(corner_last.x, corner_last.y));
                        point_2.emplace_back(cv::Point2f(corner_now.x, corner_now.y));
                    }
                }
            }
            cv::Mat H = cv::findHomography(point_1, point_2);
            cv::Mat h1 = H.col(0);
            cv::Mat h2 = H.col(1);
            cv::Mat h3 = H.col(2);

            double lambda = 1.0 / cv::norm(h1);

            cv::Mat r1 = lambda * h1;
            cv::Mat r2 = lambda * h2;
            cv::Mat r3 = r1.cross(r2);

            rotationMatrix = (cv::Mat_<double>(3, 3) << r1.at<double>(0), r2.at<double>(0), r3.at<double>(0),
                                                        r1.at<double>(1), r2.at<double>(1), r3.at<double>(1),
                                                        r1.at<double>(2), r2.at<double>(2), r3.at<double>(2));

            cv::SVDecomp(rotationMatrix, w, u, vt);
            rotationMatrix = u * vt;
            cv::Rodrigues(rotationMatrix, rvec);
            
            double reprojection_error = computeReprojectionError(H, point_1, point_2);
            dist /= last_circle.circles.size();
            if (dist > convent_camera_->image_height_ * 0.1 && reprojection_error < 1 && cv::norm(rvec) > 0.03){
                legal_corner_size++;
                last_corner = c;
                corner_buffer_selected.emplace_back(c);
            }
        }
        std::cout << "corner num:" << legal_corner_size << "\n";
        return legal_corner_size;
    }
    int getCircleBufferNum(){
        circle_buffer_selected.clear();
        legal_circle_size = 0;
        if (circle_buffer_.empty()) return 0;
        cv::Mat w, u, vt, rotationMatrix, rvec;
        for (auto cir:circle_buffer_){
            if ((cir.circles[0].timestamp - circle_buffer_.front().circles[0].timestamp).toSec() > epoch_time) break;
            static bool first_inside = true;
            if (first_inside){
                first_inside = false;
                last_circle = cir;
                circle_buffer_selected.emplace_back(cir);
            }
            float dist = 0;
            std::vector<cv::Point2f> point_1, point_2;
            for (auto circle_last:last_circle.circles){
                for (auto circle_now:cir.circles){
                    if (circle_last.x_grid == circle_now.x_grid && circle_last.y_grid == circle_now.y_grid){
                        dist += std::sqrt((circle_last.x - circle_now.x) * (circle_last.x - circle_now.x) + (circle_last.y - circle_now.y) * (circle_last.y - circle_now.y));
                        point_1.emplace_back(cv::Point2f(circle_last.x, circle_last.y));
                        point_2.emplace_back(cv::Point2f(circle_now.x, circle_now.y));
                    }
                }
            }
            cv::Mat H = cv::findHomography(point_1, point_2);
            cv::Mat h1 = H.col(0);
            cv::Mat h2 = H.col(1);
            cv::Mat h3 = H.col(2);

            double lambda = 1.0 / cv::norm(h1);

            cv::Mat r1 = lambda * h1;
            cv::Mat r2 = lambda * h2;
            cv::Mat r3 = r1.cross(r2);

            rotationMatrix = (cv::Mat_<double>(3, 3) << r1.at<double>(0), r2.at<double>(0), r3.at<double>(0),
                                                        r1.at<double>(1), r2.at<double>(1), r3.at<double>(1),
                                                        r1.at<double>(2), r2.at<double>(2), r3.at<double>(2));

            cv::SVDecomp(rotationMatrix, w, u, vt);
            rotationMatrix = u * vt;
            cv::Rodrigues(rotationMatrix, rvec);
            
            double reprojection_error = computeReprojectionError(H, point_1, point_2);
            dist /= last_circle.circles.size();
            if (dist > event_camera_->image_height_ * 0.2 && reprojection_error < 0.5 && cv::norm(rvec) > 0.03){
                legal_circle_size++;
                last_circle = cir;
                circle_buffer_selected.emplace_back(cir);
            }            
        }
        std::cout << "circle num:" << legal_circle_size << "\n";
        return legal_circle_size;
    }

    bool solveRelativePose(const corner_msgs::cornerArray& corners, Camera::Ptr cam, cv::Mat& Transformation);
    bool solveRelativePose(const circle_msgs::circleArray& circles, Camera::Ptr cam, cv::Mat& Transformation);
    void refinePose(const corner_msgs::cornerArray& corners, Camera::Ptr cam, cv::Mat& Transformation);
    void buildProblem(ceres::Problem* problem, std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> worldPoints, Camera::Ptr cam, cv::Mat& Transformation);

    std::vector<std::vector<cv::Point2f>> corner_image_cluster, circle_image_cluster, circle_image_cluster_random; 
    std::vector<std::vector<cv::Point3f>> corner_world_cluster, circle_world_cluster, circle_world_cluster_random;

    bool b_conv_initialized, b_ev_initialized, b_both_initialized;
    double rms_conv, rms_ev;
    corner_msgs::cornerArray last_corner;
    circle_msgs::circleArray last_circle;
    int legal_corner_size, legal_circle_size;

    Trajectory::Ptr trajectory_;
    Camera::Ptr event_camera_, convent_camera_;
    CalibBoard::Ptr calib_board_;

    cv::Mat convCameraMatrix, convDistCoeffs;
    std::vector<cv::Mat> convRvecs, convTvecs;
    cv::Mat evCameraMatrix, evDistCoeffs;
    std::vector<cv::Mat> evRvecs, evTvecs;

    struct SnavelyReprojectionError;
    ceres::Problem problem;
    double rot[3], trans[3];

    friend class EstimatorManager;
};

};