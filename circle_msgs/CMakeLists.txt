cmake_minimum_required(VERSION 3.0.2)
project(circle_msgs)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  ${messages_to_build}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES corner_detector
#  CATKIN_DEPENDS message_generation message_runtime cv_bridge image_transport opencv3 roscpp sensor_msgs std_msgs 
#  DEPENDS system_lib 
CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
