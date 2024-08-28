#pragma once

#include <glog/logging.h>
#include "../utils/parameter_struct.h"
#include <utils/mypcl_cloud_type.h>
#include "se3_spline.h"

namespace estimator{

enum CameraType{
    Event = 1,
    Conv = 0
};

class TrajectoryManager;

class Trajectory : public Se3Spline<SplineOrder, double>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Trajectory> Ptr;

    Trajectory(double time_interval, double start_time = 0)
        : Se3Spline<SplineOrder, double>(time_interval * S_TO_NS, start_time * S_TO_NS),
            data_start_time_(-1)
    {
        this->extendKnotsTo(start_time, SO3d(Eigen::Quaterniond::Identity()), Eigen::Vector3d(0, 0, 0));
        std::cout << GREEN << "[init maxTime] " << this->maxTimeNs() * NS_TO_S << RESET << std::endl;
    }

    void setDataStartTime(int64_t time) { data_start_time_ = time; }
    void setExtrinsicMat(SE3d& T) {    }
    int64_t getDataStartTime() { return data_start_time_; }

    bool fix_ld;
    double ld_lower;
    double ld_upper;


private:
    int64_t data_start_time_;
    std::map<CameraType, ExtrinsicParam> EP_Ev2Conv_;

    friend TrajectoryManager;

};

} // namespace estimator
