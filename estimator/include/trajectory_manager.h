#pragma once
#include "trajectory_estimator.h"
#include <spline/trajectory.h>

namespace estimator
{

class TrajectoryManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<TrajectoryManager> Ptr;

    TrajectoryManager(Trajectory::Ptr trajectory);

    void extendTrajectory(int64_t max_time, SE3d now_knot);
    void setOriginalPose();

private:
    Trajectory::Ptr trajectory_;

    int64_t max_bef_ns;
    int max_bef_idx;
    int max_aft_idx;
};

} //namespace estimator
