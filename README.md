# EF-Calib: Spatiotemporal Calibration of Event- and Frame-Based Cameras Using Continuous-Time Trajectories

<div align=center>
    <img src="./asset/overview.png" alt="overview image" width="400">
</div>

# 1. Installation

We have tested EF-Calib on machines with the following configurations
* Ubuntu 18.04.5 LTS + ROS melodic + OpenCV 4 + Ceres 1.13.0
* Ubuntu 20.04 LTS + ROS Noetic + OpenCV 4 + Ceres 1.13.0

## 1.1 Driver Installation

To work with event cameras, especially for the Dynamic Vision Sensors (DVS/DAVIS), you need to install some drivers. Please follow the instructions at [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) before moving on to the next step. Note that you need to replace the name of the ROS distribution with the one installed on your computer.

We use catkin tools to build the code. You should have it installed during the driver installation.

## 1.2 Dependencies Installation

You should have created a catkin workspace in Section 1.1. If not, please go back and create one.

**Clone this repository** into the `src` folder of your catkin workspace.

	$ cd ~/catkin_ws/src 
	$ git clone https://github.com/wsakobe/EF-Calib.git
