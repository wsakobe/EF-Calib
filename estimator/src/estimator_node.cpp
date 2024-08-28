/*
 * EF-Calib
 * Copyright (C) 2023 Shaoan Wang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "estimator_manager.h"

#include <ros/package.h>
#include <ros/ros.h>

using namespace estimator;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.param<std::string>("config_path", config_path, "../config/setup.yaml");
    ROS_INFO("Estimator load %s.", config_path.c_str());
    YAML::Node config_node = YAML::LoadFile(config_path);
    std::string log_path = config_node["log_path"].as<std::string>();
    FLAGS_log_dir = log_path;
    FLAGS_colorlogtostderr = true;
    LOG(INFO) << "Start Estimator";

    EstimatorManager estimator_manager(config_node, nh);

    ros::spin();

    return 0;
}

