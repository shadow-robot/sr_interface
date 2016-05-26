cmake_minimum_required(VERSION 2.8.3)
project(sr_moveit_planner_benchmarking)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_executable(descartes_example examples/src/descartes_example.cpp)
target_link_libraries(descartes_example ${catkin_LIBRARIES})
add_dependencies(descartes_example sr_moveit_planner_testsuite_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)