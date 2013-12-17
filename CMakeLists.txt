cmake_minimum_required(VERSION 2.8.3)
project(sr_example)
find_package(catkin REQUIRED COMPONENTS roscpp rospy sr_robot_msgs sensor_msgs std_msgs)
include_directories(${Boost_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(link_joints src/link_joints.cpp)
add_dependencies(link_joints ${catkin_EXPORTED_TARGETS})
target_link_libraries(link_joints ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy sr_robot_msgs sensor_msgs std_msgs
    INCLUDE_DIRS
    LIBRARIES
)

install(TARGETS link_joints
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)