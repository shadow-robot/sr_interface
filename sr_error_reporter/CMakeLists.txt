cmake_minimum_required(VERSION 3.0.2)
project(sr_error_reporter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  eigen_conversions
  genmsg
  moveit_core
  moveit_ros_planning_interface
  message_generation
  sr_utilities_common)

add_message_files(
  FILES
    UnderactuationError.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(error_reporter
  src/error_reporter.cpp
  src/UnderactuationErrorReporter.cpp)
target_link_libraries(error_reporter ${catkin_LIBRARIES})
add_dependencies(error_reporter sr_error_reporter_generate_messages_cpp)
