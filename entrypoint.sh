#!/bin/bash
set -e

# setup ros environment
source "/workspace/shadow_robot/base/devel/setup.bash"

roscore &

rosrun sr_moveit_planner_benchmarking benchmark_planners.py _data:=`rospack find sr_moveit_planner_benchmarking`/data _results:=/results
