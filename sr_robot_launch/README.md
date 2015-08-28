Launch files for UR10 arm and SR hand.

## Starting the robots

### Simulation
To start the simulation of the hand and arm, you can run:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch
```

### Real Robots
To start the real robots, do:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch sim:=false hand_serial:=1178
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch sim:=false hand_serial:=1178
```

To find the hand serial you can launch the command without the hand_serial argument and then check the program output. You should see something like:

```
Trying to read mapping for: /hand/mapping/1178
```

In this case 1178 is the serial number of the hand.

### Real Robot hand only

To start the hand without an arm:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch sim:=false arm_ctrl:=false arm_trajectory:=false hand_serial:=1178
```

or, for the left hand

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch sim:=false arm_ctrl:=false arm_trajectory:=false hand_serial:=1178
```

### Real Robots, using the normal (not limited) joint range

By default the URDF used for the UR10 arm uses a limited range for the joints, as that helps moveit find a planning solution. But as that restricts the robot movements, the user might want to start the robots with the full joint range. To do that:

```bash
roslaunch sr_robot_launch right_srhand_ur10arm.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10.urdf.xacro hand_serial:=1178
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch left_srhand_ur10arm.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/left_srhand_ur10.urdf.xacro hand_serial:=1178
```

### Hand with Biotacs

If your hand has biotacs, simply append `_biotacs` to the `robot_description:=` as seen below:

```bash
robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10_joint_limited_biotacs.urdf.xacro
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
