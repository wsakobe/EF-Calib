cmake_minimum_required(VERSION 3.0.2)
project(corner_detector)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  corner_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES corner_detector
#  CATKIN_DEPENDS message_generation message_runtime cv_bridge image_transport opencv3 roscpp sensor_msgs std_msgs 
#  DEPENDS system_lib 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(corner_detector
src/corner_detector.cpp)

target_link_libraries(corner_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(corner_detector corner_msgs_generate_messages_cpp)
