#pragma once

#include <ceres/ceres.h>
#include <spline/spline_segment.h>
#include <utils/parameter_struct.h>

#include <factor/analytic_diff/split_spline_view.h>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>

namespace estimator
{

class EventFactor : public ceres::CostFunction, SplitSpineView
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using SO3View = So3SplineView;
    using R3View = RdSplineView;
    using SplitView = SplitSpineView;

    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec4d = Eigen::Matrix<double, 4, 1>;
    using Vec5d = Eigen::Matrix<double, 5, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    using SO3d = Sophus::SO3<double>;

    EventFactor(const circle_msgs::circle circle, const SplineSegmentMeta<SplineOrder> &spline_segment_meta, const Camera::Ptr event_camera, const CalibBoard::Ptr calib_board, const int64_t data_start_time, const int MODE = 0, float ratio = 0.5)
        : circle_(circle), spline_segment_meta_(spline_segment_meta), event_camera_(event_camera), calib_board_(calib_board), data_start_time_(data_start_time), MODE_(MODE), ratio_(ratio)
    {
        set_num_residuals(2);

        size_t knot_num = this->spline_segment_meta_.NumParameters();
        for (size_t i = 0; i < knot_num; ++i){
            mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < knot_num; ++i){
            mutable_parameter_block_sizes()->push_back(3);
        }
        mutable_parameter_block_sizes()->push_back(4); // intrinsic
        mutable_parameter_block_sizes()->push_back(5); // distortion
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
    {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        Eigen::IOFormat Fmt(Eigen::StreamPrecision, 0, ", ", "; ", "", "", "[", "]");
        //LOG(INFO) << "New event step:";
        size_t knot_num = this->spline_segment_meta_.NumParameters();

        Eigen::Map<Vec4d const> intrinsic(parameters[2 * knot_num]);
        Eigen::Map<Vec5d const> distortion(parameters[2 * knot_num + 1]);
        
        SO3d R_W2C;
        Vec3d t_W2C = Vec3d::Zero();
        Vec3d Omega_W2C = Vec3d::Zero();
        Vec3d v_W2C = Vec3d::Zero();
        
        int64_t now_time_;
        if (MODE_ == 0){
            now_time_ = circle_.timestamp.toSec() * S_TO_NS;
        }
        if (MODE_ == 1){
            now_time_ = circle_.start_ts.toSec() * S_TO_NS + (circle_.timestamp - circle_.start_ts).toSec() * S_TO_NS * ratio_;
        }
        if (MODE_ == 2){
            now_time_ = circle_.end_ts.toSec() * S_TO_NS + (circle_.timestamp - circle_.end_ts).toSec() * S_TO_NS * ratio_;
        }
        if (jacobians){
            R_W2C = SO3View::EvaluateRotation(now_time_ - data_start_time_, spline_segment_meta_, parameters, &J_R);
            t_W2C = R3View::evaluate(now_time_ - data_start_time_, spline_segment_meta_, parameters, &J_p);
        }
        else{
            R_W2C = SO3View::EvaluateRotation(now_time_ - data_start_time_, spline_segment_meta_, parameters);
            t_W2C = R3View::evaluate(now_time_ - data_start_time_, spline_segment_meta_, parameters);
        }

        Vec3d point_in_board = calib_board_->getBoardPointInWorld(circle_.x_grid, circle_.y_grid);
        Vec3d point_in_event = R_W2C * point_in_board + t_W2C;
        Eigen::Vector2d normalized_input_point(point_in_event.x() / point_in_event.z(), point_in_event.y() / point_in_event.z());

        // Apply distortion correction
        double r2 = normalized_input_point.squaredNorm();
        double radial_distortion = 1.0 + distortion(0) * r2 + distortion(1) * r2 * r2 + distortion(4) * r2 * r2 * r2;
        
        double x_corr = normalized_input_point.x() * radial_distortion + 
                        2.0 * distortion(2) * normalized_input_point.x() * normalized_input_point.y() +
                        distortion(3) * (r2 + 2.0 * normalized_input_point.x() * normalized_input_point.x());
        double y_corr = normalized_input_point.y() * radial_distortion + 
                        2.0 * distortion(3) * normalized_input_point.x() * normalized_input_point.y() + 
                        distortion(2) * (r2 + 2.0 * normalized_input_point.y() * normalized_input_point.y());

        // Normalize the projected point
        Eigen::Vector2d normalized_point(x_corr * intrinsic(0) + intrinsic(2), y_corr * intrinsic(1) + intrinsic(3));
                
        // reprojection
        Eigen::Map<Vec2d> residual(residuals);
        residual = normalized_point - Vec2d(circle_.x + circle_.velo_x * (double)(now_time_ - circle_.timestamp.toSec() * S_TO_NS) / 1000.0, circle_.y + circle_.velo_y * (double)(now_time_ - circle_.timestamp.toSec() * S_TO_NS) / 1000.0);
        
        LOG(INFO) << "residual:" << residual.format(Fmt);
        
        if (jacobians){
            // jacobians of projection model
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_pi, J_pi_3;
            Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J_pi_1, J_pi_2;
            double p_x, p_y, r_2;
            p_x = point_in_event(0)/point_in_event(2);
            p_y = point_in_event(1)/point_in_event(2);
            r_2 = p_x * p_x + p_y * p_y;
            
            // 2 * 2 
            J_pi_1 << intrinsic(0), 0, 0, intrinsic(1);
            // 2 * 2
            J_pi_2 << 1 + distortion(0) * (r_2 + 2 * p_x * p_x) + distortion(1) * (r_2 * r_2 + 4 * p_x * p_x * r_2) + 
                        distortion(4) * (r_2 * r_2 * r_2 + 6 * p_x * p_x * r_2 * r_2) + 2 * distortion(2) * p_y + 6 * distortion(3) * p_x,
                      p_x * (2 * distortion(0) * p_y + 4 * distortion(1) * r_2 * p_y + 6 * distortion(4) * r_2 * r_2 * p_y) + 
                        2 * distortion(2) * p_x + 2 * distortion(3) * p_y,
                      p_y * (2 * distortion(0) * p_x + 4 * distortion(1) * r_2 * p_x + 6 * distortion(4) * r_2 * r_2 * p_x) + 
                        2 * distortion(3) * p_y + 2 * distortion(2) * p_x,
                      1 + distortion(0) * (r_2 + 2 * p_y * p_y) + distortion(1) * (r_2 * r_2 + 4 * p_y * p_y * r_2) + 
                        distortion(4) * (r_2 * r_2 * r_2 + 6 * p_y * p_y * r_2 * r_2) + 2 * distortion(3) * p_x + 6 * distortion(2) * p_y;
            // 2 * 3
            J_pi_3 << 1.0 / point_in_event(2), 0, -point_in_event(0)/(point_in_event(2) * point_in_event(2)),
                      0, 1.0 / point_in_event(2), -point_in_event(1)/(point_in_event(2) * point_in_event(2));
            // (2 * 2) x (2 * 2) x (2 * 3) = 2 * 3
            J_pi = J_pi_1 * J_pi_2 * J_pi_3;
            
            // [1] jacobians of control points
            Eigen::Matrix<double, 3, 3> J_R_PinE;
            J_R_PinE = R_W2C.matrix() * SO3::hat(point_in_board);
                     
            /// rotation control point
            for (size_t i = 0; i < knot_num; i++){
                if (jacobians[i]){
                    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_knot_R(jacobians[i]);
                    jac_knot_R.block<2, 3>(0, 0) = - J_pi * J_R_PinE * J_R.d_val_d_knot[i];
                    jac_knot_R.rightCols<1>().setZero();
                }
            }

            /// translation control point
            for (size_t i = 0; i < knot_num; i++){
                size_t idx = knot_num + i;
                if (jacobians[idx]){
                    Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_knot_p(jacobians[idx]);
                    jac_knot_p = J_pi * J_p.d_val_d_knot[i];
                }
            }
            
            // [2] intrinsic params
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_intrinsic(jacobians[knot_num * 2]);
            jac_intrinsic << x_corr, 0, 1, 0,
                             0, y_corr, 0, 1;   
            
            // [3] distortion params
            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jac_distortion(jacobians[knot_num * 2 + 1]);
            jac_distortion << p_x * r_2, p_x * r_2 * r_2, 2 * p_x * p_y, r_2 + 2 * p_x * p_x, p_x * r_2 * r_2 * r_2,
                              p_y * r_2, p_y * r_2 * r_2, r_2 + 2 * p_y * p_y, 2 * p_x * p_y, p_y * r_2 * r_2 * r_2;
            jac_distortion = J_pi_1 * jac_distortion.eval();
            
            return true;
        }
    }

private:
    Camera::Ptr event_camera_;
    CalibBoard::Ptr calib_board_;

    circle_msgs::circle circle_;
    SplineSegmentMeta<SplineOrder> spline_segment_meta_;

    int64_t data_start_time_;

    int MODE_;
    float ratio_;
};

class ConventFactor : public ceres::CostFunction, SplitSpineView
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SO3View = So3SplineView;
    using R3View = RdSplineView;
    using SplitView = SplitSpineView;

    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec4d = Eigen::Matrix<double, 4, 1>;
    using Vec5d = Eigen::Matrix<double, 5, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    using SO3d = Sophus::SO3<double>;

    ConventFactor(const corner_msgs::corner corner, const ros::Time timestamp, const SplineSegmentMeta<SplineOrder> &spline_segment_meta, const Camera::Ptr convent_camera, const CalibBoard::Ptr calib_board, const int64_t data_start_time)
        : corner_(corner), timestamp_(timestamp), spline_segment_meta_(spline_segment_meta), calib_board_(calib_board), convent_camera_(convent_camera), data_start_time_(data_start_time)
    {
        set_num_residuals(2);

        size_t knot_num = this->spline_segment_meta_.NumParameters();
        for (size_t i = 0; i < knot_num; ++i){
            mutable_parameter_block_sizes()->push_back(4);
        }
        for (size_t i = 0; i < knot_num; ++i){
            mutable_parameter_block_sizes()->push_back(3);
        }
        mutable_parameter_block_sizes()->push_back(4); // intrinsic
        mutable_parameter_block_sizes()->push_back(5); // distortion
        mutable_parameter_block_sizes()->push_back(4); // extrinsic R
        mutable_parameter_block_sizes()->push_back(3); // extrinsic T
        mutable_parameter_block_sizes()->push_back(1); // time delay
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
    {
        typename SO3View::JacobianStruct J_R;
        typename R3View::JacobianStruct J_p;

        Eigen::IOFormat Fmt(Eigen::StreamPrecision, 0, ", ", "; ", "", "", "[", "]");
        LOG(INFO) << "New convent step:";
        size_t knot_num = this->spline_segment_meta_.NumParameters();
        Eigen::Map<Vec4d const> intrinsic(parameters[knot_num * 2]);
        Eigen::Map<Vec5d const> distortion(parameters[knot_num * 2 + 1]);
        Eigen::Map<SO3d const> extrinsic_q(parameters[knot_num * 2 + 2]);
        Eigen::Map<Vec3d const> extrinsic_t(parameters[knot_num * 2 + 3]);        
        double time_delay = parameters[knot_num * 2 + 4][0];
        
        SO3d R_W2C;
        Vec3d t_W2C = Vec3d::Zero();
        Vec3d Omega_W2C = Vec3d::Zero();
        Vec3d v_W2C = Vec3d::Zero();
        if (jacobians){
            Omega_W2C = SO3View::VelocityBody(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, nullptr);
            v_W2C = R3View::velocity(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, nullptr);
            R_W2C = SO3View::EvaluateRp(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, &J_R);
            t_W2C = R3View::evaluate(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, &J_p);
        }
        else{
            R_W2C = SO3View::EvaluateRp(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, nullptr);
            t_W2C = R3View::evaluate(timestamp_.toSec() * S_TO_NS - data_start_time_ + time_delay * S_TO_NS, spline_segment_meta_, parameters, nullptr);
        }
        
        Eigen::Map<Vec2d> residual(residuals);
        residual = Vec2d::Zero();
        Vec3d point_in_board = calib_board_->getBoardPointInWorld(corner_.x_grid, corner_.y_grid);
        Vec3d point_in_event = R_W2C * point_in_board + t_W2C;
        Vec3d point_in_camera = extrinsic_q.matrix() * point_in_event + extrinsic_t; 
        Eigen::Vector2d normalized_input_point(point_in_camera.x() / point_in_camera.z(), point_in_camera.y() / point_in_camera.z());

        // Apply distortion correction
        double r2 = normalized_input_point.squaredNorm();
        double radial_distortion = 1.0 + distortion(0) * r2 + distortion(1) * r2 * r2 + distortion(4) * r2 * r2 * r2;
        
        double x_corr = normalized_input_point.x() * radial_distortion + 
                        2.0 * distortion(2) * normalized_input_point.x() * normalized_input_point.y() +
                        distortion(3) * (r2 + 2.0 * normalized_input_point.x() * normalized_input_point.x());
        double y_corr = normalized_input_point.y() * radial_distortion + 
                        2.0 * distortion(3) * normalized_input_point.x() * normalized_input_point.y() + 
                        distortion(2) * (r2 + 2.0 * normalized_input_point.y() * normalized_input_point.y());

        // Normalize the projected point
        Eigen::Vector2d normalized_point(x_corr * intrinsic(0) + intrinsic(2), y_corr * intrinsic(1) + intrinsic(3));
        
        residual = normalized_point - Vec2d(corner_.x, corner_.y);
        LOG(INFO) << "Residual: " << residual.format(Fmt);

        if (jacobians){
            // jacobians of projection model
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_pi, J_pi_3;
            Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J_pi_1, J_pi_2;
            double p_x, p_y, r_2;
            p_x = point_in_camera(0) / point_in_camera(2);
            p_y = point_in_camera(1) / point_in_camera(2);
            r_2 = p_x * p_x + p_y * p_y;
            
            // 2 * 2 
            J_pi_1 << intrinsic(0), 0, 0, intrinsic(1);
            // 2 * 2
            J_pi_2 << 1 + distortion(0) * (r_2 + 2 * p_x * p_x) + distortion(1) * (r_2 * r_2 + 4 * p_x * p_x * r_2) + 
                        distortion(4) * (r_2 * r_2 * r_2 + 6 * p_x * p_x * r_2 * r_2) + 2 * distortion(2) * p_y + 6 * distortion(3) * p_x,
                      p_x * (2 * distortion(0) * p_y + 4 * distortion(1) * r_2 * p_y + 6 * distortion(4) * r_2 * r_2 * p_y) + 
                        2 * distortion(2) * p_x + 2 * distortion(3) * p_y,
                      p_y * (2 * distortion(0) * p_x + 4 * distortion(1) * r_2 * p_x + 6 * distortion(4) * r_2 * r_2 * p_x) + 
                        2 * distortion(3) * p_y + 2 * distortion(2) * p_x,
                      1 + distortion(0) * (r_2 + 2 * p_y * p_y) + distortion(1) * (r_2 * r_2 + 4 * p_y * p_y * r_2) + 
                        distortion(4) * (r_2 * r_2 * r_2 + 6 * p_y * p_y * r_2 * r_2) + 2 * distortion(3) * p_x + 6 * distortion(2) * p_y;
            // 2 * 3
            J_pi_3 << 1 / point_in_camera(2), 0, -point_in_camera(0)/(point_in_camera(2) * point_in_camera(2)),
                      0, 1 / point_in_camera(2), -point_in_camera(1)/(point_in_camera(2) * point_in_camera(2));
            // 2 * 2 x 2 * 3 = 2 * 3
            J_pi = J_pi_1 * J_pi_2 * J_pi_3;
            
            // [1] jacobians of intrinsic params
            Eigen::Matrix<double, 3, 3> J_R_PinE;
            J_R_PinE = R_W2C.matrix() * SO3::hat(point_in_board);
            
            // intrinsic params
            double p_x_dis, p_y_dis;
            p_x_dis = p_x * (1 + distortion(0) * r_2 + distortion(1) * r_2 * r_2 + distortion(4) * r_2 * r_2 * r_2) + 2 * distortion(2) * p_x * p_y + distortion(3) * (r_2 + 2 * p_x * p_x);
            p_y_dis = p_y * (1 + distortion(0) * r_2 + distortion(1) * r_2 * r_2 + distortion(4) * r_2 * r_2 * r_2) + 2 * distortion(3) * p_x * p_y + distortion(2) * (r_2 + 2 * p_y * p_y);
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_intrinsic(jacobians[knot_num * 2]);
            jac_intrinsic << p_x_dis, 0, 1, 0,
                             0, p_y_dis, 0, 1;   

            // distortion params
            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jac_distortion(jacobians[knot_num * 2 + 1]);
            jac_distortion << p_x * r_2, p_x * r_2 * r_2, 2 * p_x * p_y, r_2 + 2 * p_x * p_x, p_x * r_2 * r_2 * r_2,
                              p_y * r_2, p_y * r_2 * r_2, r_2 + 2 * p_y * p_y, 2 * p_x * p_y, p_y * r_2 * r_2 * r_2;
            jac_distortion = J_pi_1 * jac_distortion.eval();
            
            // [2] extrinsic params
            /// rotation
            Eigen::Matrix<double, 3, 3> J_PinE_PinC;
            J_PinE_PinC = extrinsic_q.matrix() * SO3::hat(point_in_event);
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jac_Ex_R(jacobians[knot_num * 2 + 2]);
            jac_Ex_R.block<2, 3>(0, 0) = - J_pi * J_PinE_PinC;
            jac_Ex_R.rightCols<1>().setZero();

            /// translation
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jac_Ex_t(jacobians[knot_num * 2 + 3]);
            jac_Ex_t = J_pi;
            
            // [3] time delay
            Eigen::Matrix3d Omega_matrix;
            Omega_matrix <<     0, -Omega_W2C.z(),  Omega_W2C.y(),
                  Omega_W2C.z(),         0, -Omega_W2C.x(),
                 -Omega_W2C.y(),  Omega_W2C.x(),        0;
            Eigen::Map<Eigen::Matrix<double, 2, 1>> jac_delay(jacobians[knot_num * 2 + 4]);
            jac_delay = J_pi * extrinsic_q.matrix() * (Omega_matrix * point_in_board + v_W2C);
            
            return true;
        }
    }

private:
    Camera::Ptr convent_camera_;
    CalibBoard::Ptr calib_board_;

    corner_msgs::corner corner_;
    ros::Time timestamp_;

    SplineSegmentMeta<SplineOrder> spline_segment_meta_;

    int64_t data_start_time_;
};


} // namespace estimator
