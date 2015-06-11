cmake_minimum_required(VERSION 2.8.3)
project(sr_multi_moveit_test)

find_package(catkin REQUIRED)

catkin_package()

catkin_add_nosetests(test)
