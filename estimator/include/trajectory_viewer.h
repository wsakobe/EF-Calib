#pragma once

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <spline/trajectory.h>
#include <utils/parameter_struct.h>

namespace estimator
{
class TrajectoryViewer
{
private:
    ros::Publisher pub_cam_pose_;

    // PublishSplineTrajectory
    ros::Publisher pub_spline_trajectory_;
    ros::Publisher pub_spline_trajectory_before_opt_;

public:
    void SetPublisher(ros::NodeHandle &nh)
    {
        pub_cam_pose_  = nh.advertise<visualization_msgs::MarkerArray>("/camera/pose", 10);

        /// spline trajectory
        pub_spline_trajectory_ = nh.advertise<nav_msgs::Path>("/spline/trajectory", 10);
        pub_spline_trajectory_before_opt_ = nh.advertise<nav_msgs::Path>("/spline/trajectory_bef_opt", 10);
        std::cout << "[SetPublisher] init done.\n";
    }

    void PublishPose(SE3d pose, int cam_type, int id, bool refresh = false)
    {
        ros::Time time_now = ros::Time::now();
        visualization_msgs::MarkerArray marker_array;

        if (refresh){
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);

            // Publish the MarkerArray message
            pub_cam_pose_.publish(marker_array);

            return;
        }

        geometry_msgs::PoseStamped poseIinG;
        poseIinG.header.stamp = time_now;
        poseIinG.header.frame_id = "/map";
        tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
        tf::quaternionEigenToMsg(pose.unit_quaternion(),
                                poseIinG.pose.orientation);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = time_now;
        marker.ns = "pose_markers";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = poseIinG.pose;

        if (cam_type == 1){
            marker.scale.x = 2;
            marker.scale.y = 2;
            marker.scale.z = 2;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        }else if (cam_type == 0){
            marker.scale.x = 5;
            marker.scale.y = 5;
            marker.scale.z = 5;
            marker.color.r = 0.3;
            marker.color.g = 0.3;
            marker.color.b = 0.9;
            marker.color.a = 0.9;
        }else if (cam_type == 2){
            marker.scale.x = 2;
            marker.scale.y = 2;
            marker.scale.z = 2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.9;
        }else if (cam_type == 3){
            marker.scale.x = 5;
            marker.scale.y = 5;
            marker.scale.z = 5;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 0.9;
        }

        marker_array.markers.push_back(marker);
        pub_cam_pose_.publish(marker_array);
    }

    void PublishSplineTrajectory(Trajectory::Ptr trajectory, std::vector<std::pair<int, int>> trajectory_info, int64_t min_time, int continue_num,
                                    int64_t max_time, int64_t dt, bool stage)
    {
        ros::Time time_now = ros::Time::now();
        ros::Time t_temp;

        if (stage){
            if (pub_spline_trajectory_.getNumSubscribers() != 0)
            {
                std::vector<geometry_msgs::PoseStamped> poses_geo;
                int num = -1;
                for (auto traj:trajectory_info){
                    if ((traj.second - traj.first) >= continue_num){
                        PublishPose(trajectory->poseNs(std::max(0, (traj.first - trajectory->DEG + 1)) * trajectory->getDtNs()), 3, num--);
                        for (int64_t t = std::max(0, (traj.first - trajectory->DEG + 1)) * trajectory->getDtNs(); t < std::min(max_time, (traj.second - 3) * trajectory->getDtNs()); t += dt){
                            SE3d pose = trajectory->poseNs(t);
                            geometry_msgs::PoseStamped poseIinG;
                            poseIinG.header.stamp = t_temp.fromSec(t * NS_TO_S);
                            poseIinG.header.frame_id = "/map";
                            tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
                            tf::quaternionEigenToMsg(pose.unit_quaternion(),
                                                    poseIinG.pose.orientation);
                            poses_geo.push_back(poseIinG);
                        }
                        PublishPose(trajectory->poseNs(std::min(max_time, (traj.second - 3) * trajectory->getDtNs()) - 0.0001 * S_TO_NS), 0, num--);
                    }
                }
                
                nav_msgs::Path traj_path;
                traj_path.header.stamp = time_now;
                traj_path.header.frame_id = "/map";
                traj_path.poses = poses_geo;

                pub_spline_trajectory_.publish(traj_path);
            }
        }
        else{
            if (pub_spline_trajectory_before_opt_.getNumSubscribers() != 0)
            {
                std::vector<geometry_msgs::PoseStamped> poses_geo;
                for (int64_t t = min_time; t < max_time; t += dt)
                {
                    int start_cp_idx = (long long)(t) / trajectory->getDtNs();
                    for (auto traj:trajectory_info){
                        if ((traj.second - traj.first) >= continue_num && start_cp_idx >= traj.first && start_cp_idx <= traj.second){
                            SE3d pose = trajectory->poseNs(t);
                            geometry_msgs::PoseStamped poseIinG;
                            poseIinG.header.stamp = t_temp.fromSec(t * NS_TO_S);
                            poseIinG.header.frame_id = "/map";
                            tf::pointEigenToMsg(pose.translation(), poseIinG.pose.position);
                            tf::quaternionEigenToMsg(pose.unit_quaternion(),
                                                    poseIinG.pose.orientation);
                            poses_geo.push_back(poseIinG);
                        }
                    }
                }

                nav_msgs::Path traj_path;
                traj_path.header.stamp = time_now;
                traj_path.header.frame_id = "/map";
                traj_path.poses = poses_geo;

                pub_spline_trajectory_before_opt_.publish(traj_path);
            }
        }       
    }
};

} // namespace estimator