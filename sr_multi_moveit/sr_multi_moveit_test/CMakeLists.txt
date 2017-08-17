cmake_minimum_required(VERSION 2.8.3)
project(sr_multi_moveit_test)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning_interface)

catkin_package()
add_compile_options(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_iksolvers test/test_iksolvers.cpp)
target_link_libraries(test_iksolvers ${catkin_LIBRARIES})
install(TARGETS test_iksolvers DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_biotac_collada_name.test)
    add_rostest_gtest(test_end_effector test/test_end_effector.test test/test_end_effector.cpp)
	target_link_libraries(test_end_effector ${catkin_LIBRARIES})
endif ()

