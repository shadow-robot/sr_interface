#!/bin/bash
set -e

# setup ros environment
source "/workspace/shadow_robot/base/devel/setup.bash"
roslaunch sr_moveit_planner_benchmarking test_ompl_planning.launch complex:=true visualization:=false
