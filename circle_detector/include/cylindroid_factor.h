#pragma once

#include "ceres/ceres.h"
#include "glog/logging.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace circle_detector{

struct CylindroidResidual{

CylindroidResidual(cv::Point3i point, int time)
    : point_(point), time_(time) {}

template <typename T>
bool operator()(const T* const coeffs, T* residual) const {
    T x_rot = ceres::cos(coeffs[2] / 180.0 * 3.14) * ((T)point_.x - (coeffs[0] + coeffs[5] * (T)(point_.z - time_))) + ceres::sin(coeffs[2] / 180.0 * 3.14) * ((T)point_.y - (coeffs[1] + coeffs[6] * (T)(point_.z - time_)));
    T y_rot = ceres::sin(coeffs[2] / 180.0 * 3.14) * ((T)point_.x - (coeffs[0] + coeffs[5] * (T)(point_.z - time_))) - ceres::cos(coeffs[2] / 180.0 * 3.14) * ((T)point_.y - (coeffs[1] + coeffs[6] * (T)(point_.z - time_)));
    
    residual[0] = ceres::sqrt((x_rot * x_rot * 4.0) / (coeffs[3] * coeffs[3]) + (y_rot * y_rot * 4.0) / (coeffs[4] * coeffs[4])) - T(1);

    return true;
}

private:
    const cv::Point3i point_;
    const int time_;
};    

} // namespace circle_detector