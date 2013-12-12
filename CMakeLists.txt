cmake_minimum_required(VERSION 2.8.3)
project(sr_example)
find_package(catkin REQUIRED COMPONENTS roscpp rospy sr_robot_msgs sensor_msgs std_msgs)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
set(ROS_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${Boost_INCLUDE_DIRS})
add_executable(link_joints src/link_joints.cpp)
add_dependencies(link_joints sr_robot_msgs_gencpp)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy sr_robot_msgs sensor_msgs std_msgs
    INCLUDE_DIRS
    LIBRARIES
)