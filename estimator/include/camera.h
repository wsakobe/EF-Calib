#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

namespace estimator{

class Camera{
public:
    typedef std::shared_ptr<Camera> Ptr;
    Camera(cv::Size image_size)
        : image_size_(image_size)
    {
        image_width_  = image_size_.width;
        image_height_ = image_size_.height;
    }

    double intrinsicParams[4]; //fx_, fy_, cx_, cy_
    double distortParams[5];   //k1_, k2_, d1_, d2_, k3_

    int image_width_, image_height_;
    cv::Size image_size_;

    Sophus::SO3<double> R_C2W;
    Eigen::Matrix<double, 3, 1> t_C2W;

    double time_delay = 0;

    cv::Mat getIntrinsicMatrixOC(){
        cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsicParams[0], 0, intrinsicParams[2], 0, intrinsicParams[1], intrinsicParams[3], 0, 0, 1);
        return K;
    }

    cv::Mat getDistortionMatrixOC(){
        cv::Mat K = (cv::Mat_<double>(5, 1) << distortParams[0], distortParams[1], distortParams[2], distortParams[3], distortParams[4]);
        return K;
    }

    Eigen::Matrix<double, 3, 3> getIntrinsicMatrix(){
        Eigen::Matrix<double, 3, 3> K;
        K(0, 0) = intrinsicParams[0];
        K(1, 1) = intrinsicParams[1];
        K(0, 2) = intrinsicParams[2];
        K(1, 2) = intrinsicParams[3];
        K(2, 2) = 1;

        return K;
    }

    Eigen::Matrix<double, 5, 1> getDistortionMatrix(){
        Eigen::Matrix<double, 5, 1> K;
        K(0, 0) = distortParams[0];
        K(1, 0) = distortParams[1];
        K(2, 0) = distortParams[2];
        K(3, 0) = distortParams[3];
        K(4, 0) = distortParams[4];
        return K;
    }

    Eigen::Matrix<double, 4, 4> getExtrinsicMatrix(){
        Eigen::Matrix<double, 4, 4> T;
        T.block<3, 3>(0, 0) = R_C2W.matrix();
        T.block<3, 1>(0, 3) = t_C2W;
        T.block<1, 4>(3, 0) << 0, 0, 0, 1;
        return T;
    }

    void updateIntrinsic(cv::Mat& intrinsic, cv::Mat& distortion){
        intrinsicParams[0] = intrinsic.at<double>(0, 0);
        intrinsicParams[1] = intrinsic.at<double>(1, 1);
        intrinsicParams[2] = intrinsic.at<double>(0, 2);
        intrinsicParams[3] = intrinsic.at<double>(1, 2);

        distortParams[0] = distortion.at<double>(0, 0);
        distortParams[1] = distortion.at<double>(0, 1);
        distortParams[2] = distortion.at<double>(0, 2);
        distortParams[3] = distortion.at<double>(0, 3);
        distortParams[4] = distortion.at<double>(0, 4);
    }

    void updateExtrinsic(cv::Mat& Transformation){
        Eigen::Matrix4d eigenT;
        cv::cv2eigen(Transformation, eigenT);
        SE3d se3(eigenT);
        R_C2W = se3.rotationMatrix();
        t_C2W = se3.translation();
    }

    Eigen::Vector2d projectIntoImage(Eigen::Vector3d input_point){
        Eigen::Vector2d normalized_input_point(input_point.x() / input_point.z(), input_point.y() / input_point.z());

        // Apply distortion correction
        double r2 = normalized_input_point.squaredNorm();
        double radial_distortion = 1.0 + distortParams[0] * r2 + distortParams[1] * r2 * r2 + distortParams[4] * r2 * r2 * r2;
        
        double x_corr = normalized_input_point.x() * radial_distortion + 
                        2.0 * distortParams[2] * normalized_input_point.x() * normalized_input_point.y() +
                        distortParams[3] * (r2 + 2.0 * normalized_input_point.x() * normalized_input_point.x());
        double y_corr = normalized_input_point.y() * radial_distortion + 
                        2.0 * distortParams[3] * normalized_input_point.x() * normalized_input_point.y() + 
                        distortParams[2] * (r2 + 2.0 * normalized_input_point.y() * normalized_input_point.y());

        // Construct undistorted coordinates
        Eigen::Vector3d undistorted_point(x_corr, y_corr, 1);
        //Eigen::Vector3d undistorted_point1(normalized_input_point.x(), normalized_input_point.y(), 1);

        Eigen::Vector3d projected_point;
        projected_point = getIntrinsicMatrix() * undistorted_point;

        // Normalize the projected point
        Eigen::Vector2d normalized_point(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

        //std::cout << "input:\n" << input_point << "\nproject:\n" << undistorted_point << "\nnormalized:\n" << normalized_point << "x:" << x_corr << " y:" << y_corr << std::endl;

        return normalized_point;
    }

private:
};

} // namespace estimator
