#pragma once

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include "factor/ceres_local_param.h"
#include <utils/parameter_struct.h>
#include <spline/trajectory.h>
#include <factor/analytic_diff/trajectory_value_factor.h>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>

namespace estimator{
class TrajectoryEstimator{
    static ceres::Problem::Options DefaultProblemOptions()
    {
        ceres::Problem::Options options;
        options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
        options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        return options;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<TrajectoryEstimator> Ptr;

    TrajectoryEstimator(Trajectory::Ptr trajectory, Camera::Ptr event_camera, Camera::Ptr convent_camera, CalibBoard::Ptr calib_board);

    ~TrajectoryEstimator(){
        delete analytic_local_parameterization_;
    }

    void addConvFeature(const corner_msgs::corner &corner, ros::Time timestamp);

    void addEventFeature(const circle_msgs::circle &circle);

    void addControlPoints(const SplineMeta<SplineOrder> &spline_meta, std::vector<double *> &vec, std::shared_ptr<ceres::Problem> problem, bool addPosKnot = false, bool isEvent = false);
   
    ceres::Solver::Summary solve(int max_iterations, bool progress, int num_threads);
    ceres::Solver::Summary solve1(int max_iterations, bool progress, int num_threads);

    class UpdateTrajectoryCallback : public ceres::IterationCallback {
    public:
        UpdateTrajectoryCallback(TrajectoryEstimator* estimator) : estimator_(estimator) {}

        virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
            // for (auto traj : estimator_->trajectory_info_) {
            //     estimator_->trajectory_->getKnotPos(traj.first) = estimator_->trajectory_->getKnotPos(traj.first + 1);
            //     estimator_->trajectory_->getKnotSO3(traj.first) = estimator_->trajectory_->getKnotSO3(traj.first + 1);
            // }
            // for (auto traj : estimator_->trajectory_info_) {
            //     estimator_->trajectory_->getKnotPos(traj.second) = estimator_->trajectory_->getKnotPos(traj.second - 1);
            //     estimator_->trajectory_->getKnotSO3(traj.second) = estimator_->trajectory_->getKnotSO3(traj.second - 1);
            // }
            return ceres::SOLVER_CONTINUE;
        }

    private:
        TrajectoryEstimator* estimator_;
    };

    std::shared_ptr<ceres::Problem> problem_, problem_1;
    std::vector<std::pair<int, int>> trajectory_info_;

    double max_time_delay = 0.05; // unit: second

private:
    Trajectory::Ptr trajectory_;

    Camera::Ptr event_camera_, convent_camera_;
    CalibBoard::Ptr calib_board_;

    // std::shared_ptr<ceres::Problem> problem_;
    ceres::LocalParameterization *analytic_local_parameterization_;
    
    friend class UpdateTrajectoryCallback;
};

} //namespace estimator