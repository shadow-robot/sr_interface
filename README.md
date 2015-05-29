Launch files for UR10 arm and SR hand.

## Starting the robots

### Simulation
To start the simulation of the hand and arm, you can run:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch
```

or, for the left hand

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch
```

### Real Robots
To start the real robots, do:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch sim:=false
```

or, for the left hand

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch sim:=false
```

## Starting the moveit configuration
For more advanced behaviour (inverse kinematics, planning, collision detectection, etc...), you can use the moveit config:

```bash
roslaunch right_sr_ur10_moveit_config moveit_planning_and_execution.launch load_robot_description:=false
```

or, for the left hand

```bash
roslaunch left_sr_ur10_moveit_config moveit_planning_and_execution.launch load_robot_description:=false
```
