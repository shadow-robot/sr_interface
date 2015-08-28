cmake_minimum_required(VERSION 2.8.3)
project(sr_multi_moveit_test)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_planners_left_arm.test)
  add_rostest(test/test_planners_right_arm.test)
endif()
