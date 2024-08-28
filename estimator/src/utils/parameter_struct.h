#pragma once

// #include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <utils/yaml_utils.h>
#include <utils/sophus_utils.hpp>
#include "camera.h"
#include "calibration_board.h"

namespace estimator
{

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

  enum MODE
  {
    Calibration = 0,
    Odometry_Offline, //
    Odometry_Online,  //
    Location_Offline,
    Location_Online,
    Visualization
  };

  struct VecData
  {
    double timestamp;
    Eigen::Vector3d p;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct IMUData
  {
    int64_t timestamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    SO3d orientation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct IMUBias
  {
    IMUBias()
        : gyro_bias(Eigen::Vector3d::Zero()),
          accel_bias(Eigen::Vector3d::Zero()) {}
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d accel_bias;
  };

  struct IMUState
  {
    IMUState()
        : timestamp(0),
          p(Eigen::Vector3d::Zero()),
          v(Eigen::Vector3d::Zero()),
          g(Eigen::Vector3d(0, 0, -9.8)) {}
    double timestamp;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    IMUBias bias;
    Eigen::Vector3d g;
  };

  struct PoseData
  {
    PoseData() : timestamp(0), position(Eigen::Vector3d::Zero()) {}

    double timestamp;
    Eigen::Vector3d position;
    SO3d orientation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct SystemState : public IMUState
  {
    SystemState() : name("") {}

    SystemState(const IMUState &imu)
    {
      timestamp = imu.timestamp;
      p = imu.p;
      v = imu.v;
      q = imu.q;
      bias = imu.bias;
      g = imu.g;

      name = "";
    }

    std::string name;
  };

  struct ExtrinsicParam
  {
    ExtrinsicParam()
        : p(Eigen::Vector3d::Zero()),
          q(Eigen::Quaterniond::Identity()),
          t_offset(0) {}

    // node["Extrinsics"]
    void Init(const YAML::Node &node)
    {
      if (!(node["time_offset"] && node["Trans"] && node["Rot"]))
      {
        LOG(WARNING)
            << "[ExtrinsicParam::Init] input yaml node has not parameters "
               "of Extrinsics struct. Return without Initialziation.";
        return;
      }

      t_offset = yaml::GetValue<double>(node, "time_offset");
      std::vector<double> params_vec;
      yaml::GetValues<double>(node, "Trans", 3, params_vec);
      p << params_vec[0], params_vec[1], params_vec[2];

      yaml::GetValues<double>(node, "Rot", 9, params_vec);
      Eigen::Matrix3d rot;
      rot << params_vec[0], params_vec[1], params_vec[2], params_vec[3],
          params_vec[4], params_vec[5], params_vec[6], params_vec[7],
          params_vec[8];

      q = Eigen::Quaterniond(rot);
      q.normalized();
      so3 = SO3d(q);
      se3 = SE3d(so3, p);
    }

    Eigen::Vector3d p;
    SO3d so3;
    Eigen::Quaterniond q;
    SE3d se3;
    double t_offset;
  };

} // namespace estimator
