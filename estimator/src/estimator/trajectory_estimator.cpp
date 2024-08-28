#include "trajectory_estimator.h"

namespace estimator{

TrajectoryEstimator::TrajectoryEstimator(Trajectory::Ptr trajectory, Camera::Ptr event_camera, Camera::Ptr convent_camera, CalibBoard::Ptr calib_board)
    :trajectory_(trajectory), event_camera_(event_camera), convent_camera_(convent_camera), calib_board_(calib_board)
    {
    problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());
    problem_1 = std::make_shared<ceres::Problem>(DefaultProblemOptions());
    analytic_local_parameterization_ = new LieAnalyticLocalParameterization<SO3d>();
}

void TrajectoryEstimator::addControlPoints(
      const SplineMeta<SplineOrder> &spline_meta, std::vector<double *> &vec, std::shared_ptr<ceres::Problem> problem,
      bool addPosKnot, bool fixed){
    for (auto const &seg : spline_meta.segments)
    {
        size_t start_idx = trajectory_->computeTIndexNs(seg.t0_ns).second;
        for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i){
            if (addPosKnot){
                vec.emplace_back(trajectory_->getKnotPos(i).data());
                problem->AddParameterBlock(vec.back(), 3);
                if (fixed){
                    problem->SetParameterBlockConstant(vec.back());
                }             
            }
            else{
                vec.emplace_back(trajectory_->getKnotSO3(i).data());
                problem->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
                if (fixed){
                    problem->SetParameterBlockConstant(vec.back());
                }
            }
        }
    }
}

void TrajectoryEstimator::addConvFeature(const corner_msgs::corner &corner, ros::Time timestamp){
    SplineMeta<SplineOrder> spline_meta;
    int64_t data_start_time = trajectory_->getDataStartTime();
    trajectory_->CaculateSplineMeta({{timestamp.toSec() * S_TO_NS - data_start_time - max_time_delay * S_TO_NS, timestamp.toSec() * S_TO_NS - data_start_time + max_time_delay * S_TO_NS}}, spline_meta);
    ceres::CostFunction *cost_function_conv = new ConventFactor(corner, timestamp, spline_meta.segments.at(0), convent_camera_, calib_board_, data_start_time);
    
    std::vector<double *> vec;
    addControlPoints(spline_meta, vec, problem_1, false, true); // 4 * 4 
    addControlPoints(spline_meta, vec, problem_1, true, true); // 4 * 4 + 4 * 3

    // add intrinsic & distortion
    vec.emplace_back(convent_camera_->intrinsicParams);
    problem_1->AddParameterBlock(convent_camera_->intrinsicParams, 4);
    //problem_1->SetParameterBlockConstant(vec.back());
    vec.emplace_back(convent_camera_->distortParams);
    problem_1->AddParameterBlock(convent_camera_->distortParams, 5);
    //problem_1->SetParameterBlockConstant(vec.back());

    // add extrinsic
    vec.emplace_back(convent_camera_->R_C2W.data()); // 4 * 7 + 9 + 4
    problem_1->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
    //problem_1->SetParameterBlockConstant(vec.back());
    vec.emplace_back(convent_camera_->t_C2W.data()); // 4 * 7 + 9 + 7
    problem_1->AddParameterBlock(vec.back(), 3);
    //problem_1->SetParameterBlockConstant(vec.back());

    // add time delay
    vec.emplace_back(&convent_camera_->time_delay); // 4 * 7 + 9 + 7 + 1 = 45 dims
    problem_1->AddParameterBlock(vec.back(), 1);
    problem_1->SetParameterLowerBound(&convent_camera_->time_delay, 0, -max_time_delay);
    problem_1->SetParameterUpperBound(&convent_camera_->time_delay, 0,  max_time_delay);
    //problem_1->SetParameterBlockConstant(vec.back());

    problem_1->AddResidualBlock(cost_function_conv, new ceres::HuberLoss(2.0), vec);
}

void TrajectoryEstimator::addEventFeature(const circle_msgs::circle &circle){
    SplineMeta<SplineOrder> spline_meta;
    int64_t data_start_time = trajectory_->getDataStartTime();
    
    trajectory_->CaculateSplineMeta({{circle.timestamp.toSec() * S_TO_NS - data_start_time, circle.timestamp.toSec() * S_TO_NS - data_start_time}}, spline_meta);
    ceres::CostFunction *cost_function_ev = new EventFactor(circle, spline_meta.segments.at(0), event_camera_, calib_board_, data_start_time);

    std::vector<double *> vec;
    addControlPoints(spline_meta, vec, problem_, false); // 4 * 4
    addControlPoints(spline_meta, vec, problem_, true); // 4 * 4 + 4 * 3

    // add intrinsic & distortion
    vec.emplace_back(event_camera_->intrinsicParams); // 4 * 7 + 4
    problem_->AddParameterBlock(vec.back(), 4); 
    vec.emplace_back(event_camera_->distortParams);  // 4 * 7 + 4 + 5
    problem_->AddParameterBlock(vec.back(), 5);

    problem_->AddResidualBlock(cost_function_ev, NULL, vec);
    
    for (float ratio_st = 0.3; ratio_st < 1.0; ratio_st += 0.5){
        SplineMeta<SplineOrder> spline_meta1;
        trajectory_->CaculateSplineMeta({{(circle.start_ts.toSec() + (circle.timestamp - circle.start_ts).toSec() * ratio_st) * S_TO_NS - data_start_time, (circle.start_ts.toSec() + (circle.timestamp - circle.start_ts).toSec() * ratio_st) * S_TO_NS - data_start_time}}, spline_meta1);
        ceres::CostFunction *cost_function_ev1 = new EventFactor(circle, spline_meta1.segments.at(0), event_camera_, calib_board_, data_start_time, 1, ratio_st);
        
        std::vector<double *> vec1;
        addControlPoints(spline_meta1, vec1, problem_, false); // 4 * 4
        addControlPoints(spline_meta1, vec1, problem_, true); // 4 * 4 + 4 * 3

        // add intrinsic & distortion
        vec1.emplace_back(event_camera_->intrinsicParams); // 4 * 7 + 4
        problem_->AddParameterBlock(vec1.back(), 4); 
        vec1.emplace_back(event_camera_->distortParams);  // 4 * 7 + 4 + 5
        problem_->AddParameterBlock(vec1.back(), 5);

        problem_->AddResidualBlock(cost_function_ev1, NULL, vec1);
    }

    for (float ratio_end = 0.3; ratio_end < 1.0; ratio_end += 0.5){
        SplineMeta<SplineOrder> spline_meta2;
        
        trajectory_->CaculateSplineMeta({{(circle.end_ts.toSec() + (circle.timestamp - circle.end_ts).toSec() * ratio_end) * S_TO_NS - data_start_time, (circle.end_ts.toSec() + (circle.timestamp - circle.end_ts).toSec() * ratio_end) * S_TO_NS - data_start_time}}, spline_meta2);
        ceres::CostFunction *cost_function_ev2 = new EventFactor(circle, spline_meta2.segments.at(0), event_camera_, calib_board_, data_start_time, 2, ratio_end);
        
        std::vector<double *> vec2;
        addControlPoints(spline_meta2, vec2, problem_, false); // 4 * 4
        addControlPoints(spline_meta2, vec2, problem_, true); // 4 * 4 + 4 * 3

        // add intrinsic & distortion
        vec2.emplace_back(event_camera_->intrinsicParams); // 4 * 7 + 4
        problem_->AddParameterBlock(vec2.back(), 4); 
        vec2.emplace_back(event_camera_->distortParams);  // 4 * 7 + 4 + 5
        problem_->AddParameterBlock(vec2.back(), 5);

        problem_->AddResidualBlock(cost_function_ev2, NULL, vec2);
    }
}

ceres::Solver::Summary TrajectoryEstimator::solve(int max_iterations = 50, bool progress = false, int num_threads = 1){
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = progress;
    options.max_num_iterations = num_threads;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.update_state_every_iteration = true;
    UpdateTrajectoryCallback callback(this);
    options.callbacks.push_back(&callback);
    //options.function_tolerance = 1e-12;
    //options.gradient_tolerance = 1e-12;
    //options.parameter_tolerance = 1e-10;

    if (num_threads < 1){
        num_threads = 1; // std::thread::hardware_concurrency(); // mine is 8
    }
    options.num_threads = num_threads;
    options.max_num_iterations = max_iterations;

    //trajectory_->print_knots();
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_.get(), &summary);
    
    //trajectory_->print_knots();

    std::cout << summary.BriefReport() << std::endl;

    return summary;
}

ceres::Solver::Summary TrajectoryEstimator::solve1(int max_iterations = 50, bool progress = false, int num_threads = 1){
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = progress;
    options.max_num_iterations = num_threads;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    //options.function_tolerance = 1e-12;
    //options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;

    if (num_threads < 1){
        num_threads = 1; // std::thread::hardware_concurrency(); // mine is 8
    }
    options.num_threads = num_threads;
    options.max_num_iterations = max_iterations;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_1.get(), &summary); //old
    
    std::cout << convent_camera_->getExtrinsicMatrix() << std::endl;

    std::cout << summary.BriefReport() << std::endl;

    return summary;
}



} // namespace estimator