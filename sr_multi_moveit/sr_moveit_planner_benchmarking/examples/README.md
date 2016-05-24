# Examples for testing planners

# Descartes

Descartes is a joint trajectory planner for semi-constrained cartesian paths, where waypoints may have less than a fully specified 6DOF pose. This planner needs to be provided with the waypoints which the end effector must follow and it calculates sensible joint angles which allow this to happen. No additional points are added to the trajectory, so planning around objects is not possible, unless the specified cartesian end efector waypoints describe a path that does so.
The example 'descartes_example' is adapted from the Descartes tutorial package. 
Launch the hand and UR10 arm in an environment with:
```
roslaunch sr_moveit_planner_benchmarking benchmarking.launch
```
4 waypoints are generated with position and orientation specified, these points are then sent to the Dense planner which attempts to find a solution. If a solution is found, the trajectory is displayed in Rviz.
Collision checking in the planning scene is enabled in this example but sometimes this is not honoured and the arm finishes in a state of collision with either the floor or a box. 
```
rosrun sr_moveit_planner_benchmarking descartes_example
```
The package used was cloned from [here](https://github.com/ros-industrial-consortium/descartes).