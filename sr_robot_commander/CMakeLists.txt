cmake_minimum_required(VERSION 2.8.3)
project(sr_robot_commander)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rostest
        rospy
        moveit_commander
        pluginlib
        moveit_core
        moveit_ros_planning_interface
        moveit_ros_planning
        moveit_ros_move_group
        moveit_ros_perception
        interactive_markers
        roscpp
        cmake_modules
)

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  python
  system
  thread
)
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS 
  include
#  common_planning_interface_objects/include

  #  LIBRARIES sr_robot_commander

  CATKIN_DEPENDS
  rospy
  moveit_commander
  pluginlib
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_perception
  interactive_markers

#  planning_scene_interface/include
#  move_group_interface/include

#  DEPENDS system_lib
)

find_package(Eigen REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(include)

link_directories(${Boost_LIBRARY_DIRS})

## Declare a cpp library
# add_library(sr_robot_commander
#   src/${PROJECT_NAME}/sr_robot_commander.cpp
# )

# Declare a cpp executable
add_executable(get_named_poses src/sr_robot_commander/get_named_poses.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sr_robot_commander_node sr_robot_commander_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(get_named_poses
  ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sr_robot_commander sr_robot_commander_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sr_robot_commander.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_hand_commander.test)
endif ()
