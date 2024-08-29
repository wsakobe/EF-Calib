# EF-Calib: Spatiotemporal Calibration of Event- and Frame-Based Cameras Using Continuous-Time Trajectories

<div align=center>
    <img src="./asset/overview.png" alt="overview image" width="400">
</div>

# Installation

We have tested EF-Calib on machines with the following configurations:
* Ubuntu 20.04 LTS
* ROS Noetic
* OpenCV 4+
* Ceres 1.13.0

## Driver Installation

To work with event cameras, especially for the Dynamic Vision Sensors (DVS/DAVIS), you need to install some drivers. Please follow the instructions at [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) before moving on to the next step. Note that you need to replace the name of the ROS distribution with the one installed on your computer.

We use catkin tools to build the code. You should have it installed during the driver installation.

## Dependencies Installation

### Ceres-solver

Please follow this [link](http://ceres-solver.org/installation.html) to install Ceres-solver. Note that versions above 2.0.0 might encounter issues due to some recent changes.

### OpenCV 4

Please follow this [link](https://opencv.org/get-started/) to install OpenCV 4.

## EF-Calib Installation

You should have created a catkin workspace in Driver Installation. If not, please go back and create one.

Clone this repository into the `src` folder of your catkin workspace.

    $ cd ~/catkin_ws/src 
    $ git clone https://github.com/wsakobe/EF-Calib.git

Build all the packages of EF-Calib.

    $ catkin build


# Usage

First, record the image data and event streams synchronized according to the method described in the paper. Alternatively, you can download an example rosbag from this [link](https://drive.google.com/file/d/1UXtpTBYqeFjvbvVHNQ-meKYFzR9RP7e2/view?usp=sharing) 

Configure parameters in the `config/setup.yaml` file.

  - `BagName`: File path of the recorded rosbag
  - `square_size`: Adjacent feature spacing on the calibration board
  - `init_window_size`: Minimum number of frames required for initialization
  - `board_width`: Calibration board size (number of features in the width direction)
  - `board_height`: Calibration board size (number of features in the height direction)
  - `knot_distance`: B-spline knot spacing
  - `continue_number`: Minimum number of consecutive B-spline knots
  - `opt_time`: Total duration of the rosbag

Next, run the following commands to extract features from frames and events into `corner_info.bag` and `circle_info.bag`, respectively.

    $ source ../devel/setup.bash
    $ cd EF-Calib
    $ ./run.sh

Once valid `corner_info.bag` and `circle_info.bag` files are obtained, run the estimator node to complete the calibration process. If successful, the calibration results will be displayed in the Terminal.

    $ roslaunch estimator estimator.launch

# Credits

This code was developed by [Shaoan Wang](https://wsakobe.github.io/) from Peking University.

For researchers that have leveraged or compared to this work, please cite the following:

```latex
@article{wang2024ef,
  title={EF-Calib: Spatiotemporal Calibration of Event-and Frame-Based Cameras Using Continuous-Time Trajectories},
  author={Wang, Shaoan and Xin, Zhanhua and Hu, Yaoqing and Li, Dongyue and Zhu, Mingzhu and Yu, Junzhi},
  journal={arXiv preprint arXiv:2405.17278},
  year={2024}
}
```

# Acknowledgement

The B-spline module is adapted from [basalt](https://gitlab.com/VladyslavUsenko/basalt-headers). Thanks for the excellent job!
