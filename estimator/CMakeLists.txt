cmake_minimum_required(VERSION 3.0.2)
project(estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    circle_msgs
    corner_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    visualization_msgs
    eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Sophus REQUIRED)
find_package(OpenCV 3 REQUIRED)

#find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

catkin_package(
    #INCLUDE_DIRS src
)

###########
## Build ##
###########
include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    # ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
)

######################
### Estimator
######################

add_library(spline_lib src/spline/trajectory.cpp)
target_link_libraries(spline_lib ${thirdparty_libraries})

add_executable(estimator 
    src/estimator_node.cpp
    src/estimator/estimator_manager.cpp
    src/estimator/initialization.cpp
    src/estimator/trajectory_manager.cpp
    src/estimator/trajectory_estimator.cpp
 )
 target_link_libraries(estimator
    spline_lib
    ${thirdparty_libraries}
 )