cmake_minimum_required(VERSION 2.8.3)
project(sr_multi_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/generate_load_moveit_config.py
                 scripts/generate_moveit_config.py
                 scripts/generate_robot_srdf.py
                 scripts/virtual_joint_broadcaster.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
