#!/bin/bash
set -e

# setup ros environment
source "/workspace/shadow_robot/base/devel/setup.bash"
python /workspace/shadow_robot/base/src/sr_interface/sr_multi_moveit/sr_moveit_planner_benchmarking/src/benchmark_planners.py
