#include "initialization.h"

namespace estimator{

//conventional camera intrinsic calibration
void Initializer::processConv(){
    corner_image_cluster.clear();
    corner_world_cluster.clear();
    for (auto corners:std::vector<corner_msgs::cornerArray>(corner_buffer_selected.begin(), corner_buffer_selected.begin() + std::min(30, (int)corner_buffer_selected.size()))){
        std::vector<cv::Point2f> corner_image; 
        std::vector<cv::Point3f> corner_world;
        for (auto corner:corners.corners){
            corner_image.emplace_back(cv::Point2f(corner.x, corner.y));
            corner_world.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * calib_board_->square_len);
        }
        corner_image_cluster.emplace_back(corner_image);
        corner_world_cluster.emplace_back(corner_world);
    }
    rms_conv = cv::calibrateCamera(corner_world_cluster, corner_image_cluster, convent_camera_->image_size_, convCameraMatrix, convDistCoeffs, convRvecs, convTvecs);
    std::cout << "Conv P: " << convCameraMatrix << std::endl << convDistCoeffs << std::endl << "ReErr: " << rms_conv << std::endl;
    
    if (rms_conv < 0.3){
        ROS_INFO("Conv cam initial succ");
        convent_camera_->updateIntrinsic(convCameraMatrix, convDistCoeffs);
        b_conv_initialized = true;
    }
    else{
        ROS_INFO("Frame-based camera initialization failed, try record again.");
        return;
    }
}

//event camera intrinsic calibration
void Initializer::processEv(){
    circle_image_cluster.clear();
    circle_world_cluster.clear();
    for (auto circles:circle_buffer_selected){
        std::vector<cv::Point2f> circle_image; 
        std::vector<cv::Point3f> circle_world;
        for (auto circle:circles.circles){
            circle_image.emplace_back(cv::Point2f(circle.x, circle.y));
            circle_world.emplace_back(cv::Point3f(circle.x_grid, circle.y_grid, 0) * calib_board_->square_len);
        }
        circle_image_cluster.emplace_back(circle_image);
        circle_world_cluster.emplace_back(circle_world);
    }
    rms_ev = cv::calibrateCamera(circle_world_cluster, circle_image_cluster, event_camera_->image_size_, evCameraMatrix, evDistCoeffs, evRvecs, evTvecs);
    std::cout << " Event P: " << evCameraMatrix << std::endl << evDistCoeffs << std::endl << "ReErr: " << rms_ev << std::endl;
   
    if (rms_ev < 0.5){
        ROS_INFO("Ev cam initial succ");
        event_camera_->updateIntrinsic(evCameraMatrix, evDistCoeffs);
        b_ev_initialized = true;
    }
    else{
        ROS_INFO("Event camera initialization failed, try record again.");
        return;
    }
}

bool Initializer::solveRelativePose(const corner_msgs::cornerArray& features, Camera::Ptr cam, cv::Mat& Transformation){
    std::vector<cv::Point2f> feature_image; 
    std::vector<cv::Point3f> feature_world;
    for (auto corner:features.corners){
        feature_image.emplace_back(cv::Point2f(corner.x, corner.y));
        feature_world.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * calib_board_->square_len);
    }
    cv::Mat rvec, tvec;
    cv::solvePnP(feature_world, feature_image, cam->getIntrinsicMatrixOC(), cam->getDistortionMatrixOC(), rvec, tvec, false, cv::SOLVEPNP_EPNP);
    
    cv::Mat rotationMat;
    cv::Rodrigues(rvec, rotationMat);
    cv::Mat transformationMat;
    cv::hconcat(rotationMat, tvec, Transformation);
    cv::Mat_<double> last_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::vconcat(Transformation, last_row, Transformation);

    //std::cout << "Conv pose:" << Transformation << std::endl;
    double rep_err = 0.0;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RMat, tMat;
    cv::cv2eigen(rotationMat, RMat);
    cv::cv2eigen(tvec, tMat);
    for (auto &corner:features.corners){
        Eigen::Matrix<double, 3, 1> point_in_W = calib_board_->getBoardPointInWorld(corner.x_grid, corner.y_grid);
        Eigen::Matrix<double, 3, 1> point_in_C = RMat * point_in_W + tMat;
        Eigen::Matrix<double, 2, 1> point_in_cam = convent_camera_->projectIntoImage(point_in_C);
        Eigen::Matrix<double, 2, 1> err = point_in_cam - Eigen::Matrix<double, 2, 1>(corner.x, corner.y);
        rep_err += err.norm();
    }
    //std::cout << rep_err / features.corners.size() << std::endl;

    if (rep_err / features.corners.size() < rms_conv * 3) return true;
    else return false;
}

struct Initializer::SnavelyReprojectionError
{
	cv::Point2d observed;
	Camera::Ptr cam;
	cv::Point3d point_ID;

	SnavelyReprojectionError(cv::Point2d observation, Camera::Ptr cam, cv::Point3d point_ID) :observed(observation), cam(cam), point_ID(point_ID) {}

	template <typename T>
	bool operator()(const T* const rotation,
		const T* const translation,
		T* residuals)const {
		T predictions[2], pos_proj[3], pos_world[3];

		pos_world[0] = T(point_ID.x);
		pos_world[1] = T(point_ID.y);
		pos_world[2] = T(point_ID.z);
		ceres::AngleAxisRotatePoint(rotation, pos_world, pos_proj);

		pos_proj[0] += translation[0];
		pos_proj[1] += translation[1];
		pos_proj[2] += translation[2];

		const T fx = T(cam->getIntrinsicMatrixOC().at<double>(0, 0));
		const T fy = T(cam->getIntrinsicMatrixOC().at<double>(1, 1));
		const T cx = T(cam->getIntrinsicMatrixOC().at<double>(0, 2));
		const T cy = T(cam->getIntrinsicMatrixOC().at<double>(1, 2));

		predictions[0] = fx * (pos_proj[0] / pos_proj[2]) + cx;
		predictions[1] = fy * (pos_proj[1] / pos_proj[2]) + cy;

		residuals[0] = predictions[0] - T(observed.x);
		residuals[1] = predictions[1] - T(observed.y);

		return true;
	}

	static ceres::CostFunction* Create(cv::Point2d observed, Camera::Ptr cam, cv::Point3d point_ID) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 3>(
			new SnavelyReprojectionError(observed, cam, point_ID)));
	}

};

void Initializer::refinePose(const corner_msgs::cornerArray& features, Camera::Ptr cam, cv::Mat& Transformation){
    std::vector<cv::Point2f> imagePoints; 
    std::vector<cv::Point3f> worldPoints;
    for (auto corner:features.corners){
        imagePoints.emplace_back(cv::Point2f(corner.x, corner.y));
        worldPoints.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * calib_board_->square_len);
    }

	cv::undistortPoints(imagePoints, imagePoints, cam->getIntrinsicMatrixOC(), cam->getDistortionMatrixOC(), cv::noArray(), cam->getIntrinsicMatrixOC());
	ceres::Problem problem;
	buildProblem(&problem, imagePoints, worldPoints, cam, Transformation);

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.gradient_tolerance = 1e-15;
	options.function_tolerance = 1e-15;
	options.parameter_tolerance = 1e-10;
	ceres::Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

    cv::Mat R;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1);
	rvec.at<double>(0, 0) = rot[0];
	rvec.at<double>(1, 0) = rot[1];
	rvec.at<double>(2, 0) = rot[2];
    cv::Rodrigues(rvec, R);
    Transformation(cv::Rect(0, 0, 3, 3)) = R.clone();
	Transformation.at<double>(0, 3) = trans[0];
	Transformation.at<double>(1, 3) = trans[1];
	Transformation.at<double>(2, 3) = trans[2];
}

void Initializer::buildProblem(ceres::Problem* problem, std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> worldPoints, Camera::Ptr cam, cv::Mat& Transformation){
    cv::Mat R = Transformation(cv::Rect(0, 0, 3, 3));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
	rot[0] = rvec.at<double>(0, 0);
	rot[1] = rvec.at<double>(1, 0);
	rot[2] = rvec.at<double>(2, 0);
	trans[0] = Transformation.at<double>(0, 3);
	trans[1] = Transformation.at<double>(1, 3);
	trans[2] = Transformation.at<double>(2, 3);

	for (int i = 0; i < imagePoints.size(); ++i) {
			ceres::CostFunction* cost_function;
			cost_function = SnavelyReprojectionError::Create((cv::Point2d)imagePoints[i], cam, (cv::Point3d)worldPoints[i]);
			problem->AddResidualBlock(cost_function, NULL, rot, trans);
	}
}

bool Initializer::solveRelativePose(const circle_msgs::circleArray& features, Camera::Ptr cam, cv::Mat& Transformation){
    std::vector<cv::Point2f> feature_image; 
    std::vector<cv::Point3f> feature_world;
    for (auto circle:features.circles){
        feature_image.emplace_back(cv::Point2f(circle.x, circle.y));
        feature_world.emplace_back(cv::Point3f(circle.x_grid, circle.y_grid, 0) * calib_board_->square_len);
    }
    cv::Mat rvec, tvec;
    cv::solvePnP(feature_world, feature_image, cam->getIntrinsicMatrixOC(), cam->getDistortionMatrixOC(), rvec, tvec, false, cv::SOLVEPNP_EPNP);
    
    cv::Mat rotationMat;
    cv::Rodrigues(rvec, rotationMat);
    cv::Mat transformationMat;
    cv::hconcat(rotationMat, tvec, Transformation);
    cv::Mat_<double> last_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::vconcat(Transformation, last_row, Transformation);

    //std::cout << "Event pose:" << Transformation << std::endl;

    double rep_err = 0.0;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RMat, tMat;
    cv::cv2eigen(rotationMat, RMat);
    cv::cv2eigen(tvec, tMat);
    for (auto &circle:features.circles){
        Eigen::Matrix<double, 3, 1> point_in_W = calib_board_->getBoardPointInWorld(circle.x_grid, circle.y_grid);
        Eigen::Matrix<double, 3, 1> point_in_E = RMat * point_in_W + tMat;
        Eigen::Matrix<double, 2, 1> point_in_cam = event_camera_->projectIntoImage(point_in_E);
        Eigen::Matrix<double, 2, 1> err = point_in_cam - Eigen::Matrix<double, 2, 1>(circle.x, circle.y);
        rep_err += err.norm();
    }
    //std::cout << rep_err / features.circles.size() << std::endl;

    if (rep_err / features.circles.size() < rms_ev * 3) return true;
    else return false;
}

void Initializer::estimateInitialExtrinsic(){
    ros::Duration time_gap_min(1, 0);
    int pos_corner = -1, pos_circle = -1;
    double min_time_interval = 0.01;
    for (size_t i = 0; i < corner_buffer_.size(); ++i){
        ros::Time time_ = corner_buffer_[i].timestamp;
        for (size_t j = 0; j < circle_buffer_.size(); ++j){
            ros::Duration time_interval = time_ - circle_buffer_[j].header.stamp;
            if (abs(time_interval.toSec()) < abs(time_gap_min.toSec())){
                time_gap_min = time_interval;
                pos_corner = i;
                pos_circle = j;
            }
        }
        if (abs(time_gap_min.toSec()) < min_time_interval){
            cv::Mat T_ev, T_c;
            bool succ1, succ2;
            succ1 = solveRelativePose(circle_buffer_[pos_circle], event_camera_, T_ev);
            succ2 = solveRelativePose(corner_buffer_[pos_corner], convent_camera_, T_c);
            cv::Mat T = T_c * T_ev.inv();
            
            if (succ1 && succ2){
                convent_camera_->updateExtrinsic(T);
                b_both_initialized = true;
                min_time_interval = abs(time_gap_min.toSec());
            }
            
        }
    }
    
    if (!b_both_initialized){
        ROS_ERROR("Estimate time interval failed, the minimal is: %f", abs(time_gap_min.toSec()));
    }
    else{
        std::cout << "Estimate time interval: " << min_time_interval << std::endl;
        std::cout << "Extrinsic params guess:\n" << convent_camera_->getExtrinsicMatrix().matrix() << std::endl;
    }
    return;
}

};
