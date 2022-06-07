cmake_minimum_required(VERSION 3.0.2)
project(sr_robot_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sr_robot_launch
        CATKIN_DEPENDS controller_manager_msgs joint_state_publisher robot_state_publisher rospy sr_multi_description sr_robot_msgs ur_gazebo
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sr_robot_launch
#   src/${PROJECT_NAME}/sr_robot_launch.cpp
# )

## Declare a cpp executable
# add_executable(sr_robot_launch_node src/sr_robot_launch_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sr_robot_launch_node sr_robot_launch_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sr_robot_launch_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS sr_robot_launch sr_robot_launch_node
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
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(PROGRAMS
        scripts/teach_mode_node
        scripts/sr_ur_arm_unlock
        scripts/construct_robot_description
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sr_robot_launch.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

# TODO: Add hand extra light in the section below
#HAND TESTS
    #add_rostest(test/test_hand_joint_movement.test ARGS hand_type:=hand_e hand_id:=rh)
    #add_rostest(test/test_hand_joint_movement.test ARGS hand_type:=hand_e hand_id:=lh)
    #add_rostest(test/test_hand_joint_movement.test ARGS hand_type:=hand_g hand_id:=rh hand_version:=G1M5 fingers:=th,ff,mf,rf)
    #add_rostest(test/test_hand_joint_movement.test ARGS hand_type:=hand_g hand_id:=lh hand_version:=G1M5 fingers:=th,ff,mf,rf)

#HAND AND ARM TESTS - SPECIFIC LAUNCH FILES
    #add_rostest(test/test_ur10_right.test ARGS scene:=true)
    #add_rostest(test/test_ur10_left.test ARGS scene:=true)
    #add_rostest(test/test_ur5_right.test)
    #add_rostest(test/test_ur5_right.test ARGS scene:=true)
    #add_rostest(test/test_ur5_left.test)
    #add_rostest(test/test_ur5_left.test ARGS scene:=true)

#Bimanual Tests
    # This wont work as robotcommander group cannot have ground for collision so it fails
    # add_rostest(test/test_bimanual_hand_and_arm.test)
    #add_rostest(test/test_bimanual_hand_and_arm.test ARGS scene:=true)
    
#ARM UNLOCK/BRAKE/RELEASE TESTS
    #add_rostest(test/test_sr_ur_unlock.test)

# TODO: Add hand extra light in the section below
#HAND AND ARM TESTS - GENERIC LAUNCH FILE
    # ur10
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_e robot_model:=ur10)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_e robot_model:=ur10)

    # ur10e
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_e robot_model:=ur10e)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_e robot_model:=ur10e)
    # We dont have this xacro yet
    # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_g robot_model:=ur10e hand_version:=G1M5 fingers:=th,ff,mf,rf)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_g robot_model:=ur10e hand_version:=G1M5 fingers:=th,ff,mf,rf)

    # ur5
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_e robot_model:=ur5)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_g robot_model:=ur5 hand_version:=G1M5 fingers:=th,ff,mf,rf)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_e robot_model:=ur5)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_g robot_model:=ur5 hand_version:=G1M5 fingers:=th,ff,mf,rf)

    # ur5e
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_g robot_model:=ur5e hand_version:=G1M5 fingers:=th,ff,mf,rf)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=left hand_type:=hand_e robot_model:=ur5e)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_e robot_model:=ur5e)
    #add_rostest(test/test_hand_and_arm_generic_scene.test ARGS side:=right hand_type:=hand_g robot_model:=ur5e hand_version:=G1M5 fingers:=th,ff,mf,rf)

# TODO: Remove section below after new xacros are merged
#FAILS - broken/outdated xacro does not display biotacs correctly
   # add_rostest(test/test_hand_and_arm_generic_no_scene.test ARGS robot_description:=right_srhand_ur10_biotacs.urdf.xacro robot_config_file:=right_sh_ur10_biotac.yaml side:=right hand_type:=hand_e)
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=right_srhand_ur10_biotacs.urdf.xacro robot_config_file:=right_sh_ur10_biotac.yaml side:=right hand_type:=hand_e robot_model:=ur10)
   # There is no way to pass sensor type at the moment so this test is excluded
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=right_srhand_ur10_plus_biotacs.urdf.xacro robot_config_file:=right_sh_ur10_plus_biotac.yaml side:=right hand_type:=hand_e robot_model:=ur10)  
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=left_srhand_ur10e_bt_sp_biotacs.urdf.xacro robot_config_file:=left_sh_ur10e_biotac.yaml side:=left hand_type:=hand_e robot_model:=ur10e)
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=right_srhand_ur10_bt_sp_biotacs.urdf.xacro robot_config_file:=right_sh_ur10_btsp_biotac.yaml side:=right hand_type:=hand_e robot_model:=ur10)
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=right_srhand_ur10e_bt_sp_biotacs.urdf.xacro robot_config_file:=right_sh_ur10e_btsp_biotac.yaml side:=right hand_type:=hand_e robot_model:=ur10e)
   # add_rostest(test/test_hand_and_arm_generic_scene.test ARGS robot_description:=left_srhand_ur10_bt_sp_biotacs.urdf.xacro side:=left hand_type:=hand_e robot_model:=ur10)

endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
