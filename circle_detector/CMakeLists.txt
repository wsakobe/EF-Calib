cmake_minimum_required(VERSION 3.0.2)
project(circle_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")

find_package(catkin REQUIRED COMPONENTS
    circle_msgs
    cv_bridge
    image_transport
    roscpp
    rosbag
    std_msgs
    dvs_msgs
    eventmap_generator
)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES circle_detector
#  CATKIN_DEPENDS circle_msgs cv_bridge image_transport opencv3 roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

add_executable(circle_detector
src/circle_detector.cpp)

target_link_libraries(circle_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    -lglog
)

add_dependencies(circle_detector circle_msgs_generate_messages_cpp)

