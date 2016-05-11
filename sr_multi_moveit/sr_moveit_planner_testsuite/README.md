# sr_moveit_planner_testsuite

A package to test the different MoveIt planners. Planners currently configured are OMPL, SBPL and STOMP.
There are two test scenes, 'collision_scene_1' and 'collision_scene_2'; with the first being a simple scene with box 
obstacles and the second being more complex with an octomap generated from a kinect2 point cloud. The scpl repository can be found [here](https://github.com/shadow-robot/sandbox) and stomp [here](https://github.com/ros-industrial/industrial_moveit).
# Launch

For east scene there are a series of tests, moving the robot around the scene to different joint goals; stomp does not accept pose goals. Metrics for the success of each test are generated and displayed at the end of the test in a table. The test can be launched with any of the three planning libraries; ompl, stomp or sbpl and all tests will be run for each of the planners in the library. To launch the UR10 arm and shadowhand in simulation with complex collision scene and run a test for ompl planners, run:
```
roslaunch sr_moveit_planner_testsuite test_ompl_planning.launch complex:=true
```
To select either sbpl or stomp, test_sbpl_planning.launch or test_stomp_planning.launch should be selected. To run the test with the simple scene, remove the 'complex' parameter, this defaults to false.

# Configuration

Currently the configurations for SBPL and STOMP are not generated automatically as OMPL is. The argument 
'generate_planning_config' in [planning_pipeline.launch.xml](https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_multi_moveit/sr_multi_moveit_config/launch/planning_pipeline.launch.xml) is set to True by default but is set to false in the launch files for the sbpl and stomp tests.
In each of the launch files, a script is launched as a node, test_planners.py or test_planners_complex.py. 2 arguments are set, the first being the number of planning attempts for each goal and the second being the planning library name. ompl is currently set to 10, sbpl is set to 1 because of the way that search based planning works, all possible paths are explored for each planning request. Stomp has also been set to 1, currently it takes far longer to search for a plan, the config file has been edited to meet sugestions [here](https://groups.google.com/forum/#!msg/swri-ros-pkg-dev/sNvFmkQsMtg/mGPrXDy8EwAJ), but probably needs more tweaks.

# Notes
The point cloud for collision_scene_2 is generated from two rosbags of pointcloud data, 
office_scene.bag and office_scene_2.bag, [here](pointclouds/). These bags are played during launch and have a duration 
of 1s. To synchronise the time of the rosbags with the simulated time in Gazebo, a 
[simple script](scripts/header_time_adjust.py) is run. The start of the rosbag 'play' is delayed for 25s to allow everything to load.
