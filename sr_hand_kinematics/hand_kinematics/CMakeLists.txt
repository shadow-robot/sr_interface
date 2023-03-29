cmake_minimum_required(VERSION 3.0.2)
project(hand_kinematics)
find_package(catkin REQUIRED COMPONENTS roscpp rostest tf urdf kdl_parser kdl_coupling sensor_msgs tf_conversions
        moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning sr_utilities angles)

find_package(orocos_kdl REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
        DEPENDS orocos_kdl
        CATKIN_DEPENDS roscpp tf urdf kdl_parser kdl_coupling sensor_msgs tf_conversions moveit_core geometry_msgs pluginlib moveit_msgs moveit_ros_planning sr_utilities
        INCLUDE_DIRS include
        LIBRARIES hand_kinematics hand_kinematics_coupling_lib
)
add_compile_options(-std=c++17)

add_library(hand_kinematics src/hand_kinematics_coupling_plugin.cpp
        src/hand_kinematics_utils.cpp)
add_dependencies(hand_kinematics ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_kinematics ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(hand_kinematics_coupling_lib src/hand_kinematics_coupling.cpp
        src/hand_kinematics_utils.cpp)
add_dependencies(hand_kinematics_coupling_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_kinematics_coupling_lib ${moveit_core_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(hand_kinematics_coupling src/hand_kinematics_coupling.cpp)
target_link_libraries(hand_kinematics_coupling ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(test_kinematics_as_plugin test/hand_kinematics_plugin_test.test test/test_kinematics_as_plugin.cpp)
  target_link_libraries(test_kinematics_as_plugin ${moveit_core_LIBRARIES} hand_kinematics ${catkin_LIBRARIES})
  add_dependencies(test_kinematics_as_plugin ${hand_kinematics_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_rostest_gtest(test_kinematics_as_service test/hand_kinematics_services_test.test test/test_kinematics_as_service.cpp)
  target_link_libraries(test_kinematics_as_service ${catkin_LIBRARIES})
  add_dependencies(test_kinematics_as_service ${hand_kinematics_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS hand_kinematics hand_kinematics_coupling_lib
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS hand_kinematics_coupling
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
        FILES
        hand_kinematics_plugins.xml
        DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION})
