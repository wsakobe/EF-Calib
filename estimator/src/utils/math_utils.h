#pragma once

// #include "sophus_utils.hpp"
#include "parameter_struct.h"

namespace estimator
{

  inline PoseData XYThetaToPoseData(double x, double y, double theta,
                                    double timestamp = 0)
  {
    PoseData pose;
    Eigen::Vector3d p(x, y, 0);
    Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(rotation_vector);
    pose.timestamp = timestamp;
    pose.position = p;
    pose.orientation.setQuaternion(q);

    return pose;
  }

  inline PoseData SE3ToPoseData(SE3d se3_pose, double time = 0)
  {
    PoseData pose;
    pose.timestamp = time;
    pose.position = se3_pose.translation();
    pose.orientation = se3_pose.so3();
    return pose;
  }

  inline SE3d Matrix4fToSE3d(Eigen::Matrix4f matrix)
  {
    Eigen::Vector3d trans(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    Eigen::Quaterniond q(matrix.block<3, 3>(0, 0).cast<double>());
    q.normalize();
    return SE3d(q, trans);
  }

  inline void SE3dToPositionEuler(SE3d se3_pose, Eigen::Vector3d &position,
                                  Eigen::Vector3d &euler)
  {
    position = se3_pose.translation();
    Eigen::Quaterniond q = se3_pose.unit_quaternion();
    euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    euler *= 180 / M_PI;
  }

} // namespace estimator
