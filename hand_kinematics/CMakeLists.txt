cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#  Uncomment this if you want the plugin interface to be compiled (requires arm_navigation stack)
#rosbuild_add_library(hand_kinematics
#	 src/hand_kinematics_coupling_plugin.cpp
#	    src/hand_kinematics_utils.cpp
#  )

rosbuild_add_library(hand_kinematics_coupling_lib
	         src/hand_kinematics_coupling.cpp )

rosbuild_add_executable(hand_kinematics_coupling src/hand_kinematics_coupling.cpp)

rosbuild_add_executable(circle_ik test/circle_ik.cpp)
rosbuild_add_executable(square_ik_th test/square_ik_th.cpp)
rosbuild_add_executable(upmc_fkik_test test/upmc_fkik_test.cpp)

target_link_libraries(circle_ik hand_kinematics_coupling_lib)
target_link_libraries(square_ik_th hand_kinematics_coupling_lib)
