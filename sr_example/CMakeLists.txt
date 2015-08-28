cmake_minimum_required(VERSION 2.8.3)
project(sr_example)
find_package(catkin REQUIRED COMPONENTS roscpp rospy sr_robot_msgs sensor_msgs std_msgs control_msgs)
include_directories(${Boost_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy sr_robot_msgs sensor_msgs std_msgs control_msgs
    INCLUDE_DIRS
    LIBRARIES
)

add_executable(link_joints src/link_joints.cpp)
add_dependencies(link_joints ${catkin_EXPORTED_TARGETS})
target_link_libraries(link_joints ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS link_joints
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# These are executable python files that will be found via rosrun. So they are installed in the BIN_DESTINATION
file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PROJECT_NAME}/*.py")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
file(GLOB PYTHON_SCRIPTS_COMMANDER RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${PROJECT_NAME}/commander/*.py")
install(PROGRAMS ${PYTHON_SCRIPTS_COMMANDER}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
