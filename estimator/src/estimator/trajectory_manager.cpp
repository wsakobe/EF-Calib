#include "trajectory_manager.h"

namespace estimator{

TrajectoryManager::TrajectoryManager(Trajectory::Ptr trajectory)
    : trajectory_(trajectory)
{

}

void TrajectoryManager::extendTrajectory(int64_t max_time, SE3d now_knot){
	double max_bef = trajectory_->maxTimeNs() * NS_TO_S;
	max_bef_ns = trajectory_->maxTimeNs();
	max_bef_idx = trajectory_->cpnum() - 1;

	trajectory_->extendKnotsTo(max_time, now_knot); // maxTime>=max_time
	//SE3d last_knot = trajectory_->getLastKnot();
	//trajectory_->extendKnotsTo(max_time, last_knot);
	double max_aft = trajectory_->maxTimeNs() * NS_TO_S;
	max_aft_idx = trajectory_->cpnum() - 1;
}

} //namespace estimator