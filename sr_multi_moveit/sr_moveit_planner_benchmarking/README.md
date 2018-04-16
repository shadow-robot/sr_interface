# sr_moveit_planner_benchmarking

A package to test the different planners using the Moveit Benchmark interface

Launch only robot to check scenes:
```
roslaunch sr_robot_launch sr_ur10arm_box.launch
```

To run Moveit benchmarks with our robot:
```
roslaunch sr_moveit_planner_benchmarking benchmarking.launch
```

Saving the benchmarks to a database file to load in planner arena:
```
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py [name_of_log_file]
```

example:
```
rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py /tmp/moveit_benchmarks/KitchenPick1_Start1_Pick1_lenovo_2018-04-01T16:49:56.873263.log
```