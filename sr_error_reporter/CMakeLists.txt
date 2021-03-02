cmake_minimum_required(VERSION 2.8.3)
project(sr_error_reporter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  genmsg
  moveit_core
  moveit_ros_planning_interface)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(error_reporter
  src/error_reporter.cpp
  src/UnderactuationErrorReporter.cpp)
target_link_libraries(error_reporter ${catkin_LIBRARIES})
