# Warehouse trajectories

## Description

It provides methods which can be used to plan trajectories based on previously saved states.

## Saving waypoints into the database

  a. First the robot must be loaded into rviz, and the Mongo DB started. This can be most easily done by running:

  ```bash
  roslaunch sr_robot_launch right_srhand_ur10arm.launch
  ```
  b. Connect to the Mongo DB using the "Connect" button on the "Context" tab of the motion planning plugin in rviz.

  c. Position the robot in the pose to be saved.

  d. Waypoints can then be saved using the "Stored States" tab of the motion planner. If you plan to use prefix-based trajectories, now is the time to name waypoints accordingly.

 **N.B.** Although the waypoints are saved in the db as ```moveit_msgs/RobotState```, the cartesian pose of the end effector actually defines the waypoint, i.e. the joint angles of the robot as the trajectory plays back may not be the same as they appear in rviz as the point is saved.

## Planning trajectories using saved waypoints
 1. Using the service directly
 Run:
 ```bash
 roslaunch sr_robot_launch warehouse_trajectory_planner.launch
 ```
 Trajectories can be planned in two ways.

  * From named waypoints using:
  ```bash
   rosservice call /plan_trajectory_from_list "waypoint_names: [waypoint_1, waypoint_2,...]"
  ```

  * From a prefix based filter, which selects all waypoints from the db whose names begin with the prefix, sorted in ASCIIbetical order:
  ```bash
   rosservice call /plan_trajectory_from_prefix "prefix: waypoint_name_prefix"
  ```

  Once a trajectory is planned, execute it with:
  ```bash
  rosservice call /execute_planned_trajectory
  ```

 2. Using predefined trajectories
  ```bash
  roslaunch sr_robot_launch right_srhand_ur10arm.launch
  ```

  A list of available named trajectories can be found with:
  ```bash
  rosservice call /list_named_trajectories
  ```

  Named trajectories can then be planned with:
   ```bash
  rosservice call /plan_named_trajectory "name: trajectory_name"
  ```

  Named trajectories are defined by either list or prefix based waypoint selection as outlined above. These definitions are stored in ```sr_robot_launch/launch/warehouse_trajectory_mapping.yaml``` The mapping for controlling these services from a joypad are defined in the same file.

  **Defining Named Trajectories**

 ```yaml
 service_mapping:
    - name: trajectory_1
      list:
            - waypoint_1
            - waypoint_2
            - waypoint_3

    - name: trajectory_2
      prefix: waypoint_
 ```

  Named trajectories are described in the parameter ```service_mapping```. Each definition contains a string field,```name```, which contains the name of the trajectory and one of either a list of waypoint names called ```list``` or a single string called ```prefix```.

 Particular care must be taken when using prefix based trajectories. Adding/removing/renaming poses to the database could have unexpected effects as waypoints are selected automatically when the planning service is called.

