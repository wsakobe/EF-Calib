cmake_minimum_required(VERSION 3.0.2)
project(eventmap_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake) 

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    rosbag
    std_msgs
    dvs_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    ${messages_to_build}
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

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
    CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs sensor_msgs 
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(generator
src/generator_node.cpp
src/generator.cpp)

target_link_libraries(generator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

