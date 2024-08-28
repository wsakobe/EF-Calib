#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

namespace estimator{
class CalibBoard{
public:
    typedef std::shared_ptr<CalibBoard> Ptr;

    CalibBoard(cv::Size board_size, double squarelen)
        : board_size_(board_size), square_len(squarelen)
    {
    }

    Eigen::Vector3d getBoardPointInWorld(int x, int y){
        Eigen::Vector3d point_world;
        point_world = Eigen::Vector3d(x * square_len, y * square_len, 0);
        
        return point_world;
    }
    
    double square_len;

private:
    cv::Size board_size_;

};
} //namespace estimator
