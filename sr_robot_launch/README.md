# Starting the robots (simulated / real)

## Shadow hand only

### Simulation

These are the hands available:

| Right                                          | Left                                    | 
| ---------------------------------------------- |-----------------------------------------| 
| shadowhand_motor.urdf.xacro                    | shadowhand_left_motor.urdf.xacro        | 
| shadowhand_motor_biotac.urdf.xacro             | shadowhand_left_motor_biotac.urdf.xacro |
| shadowhand_motor_ff_biotac.urdf.xacro          |                                         |
| shadowhand_motor_btsp.urdf.xacro               |                                         |
| shadowhand_motor_ellipsoid.urdf.xacro          |                                         |
| shadowhand_motor_th_ff_rf_ellipsoid.urdf.xacro |                                         |
| shadowhand_motor_btsp.urdf.xacro               |                                         |
| shadowhand_muscle.urdf.xacro                   | shadowhand_left_muscle.urdf.xacro       |
| shadowhand_muscle_biotac.urdf.xacro            |                                         |
| shadowhand_lite.urdf.xacro                     |                                         |
| shadowhand_extra_lite.urdf.xacro               |                                         |

To start the simulation of a shadow hand, you can run:

```bash
roslaunch sr_robot_launch srhand.launch use_moveit:=true robot_description:=`rospack find sr_description`/robots/shadowhand_motor.urdf.xacro
```

* The `robot description` param can be changed to start any of the available Shadow hands:
* If it is a left hand `hand_id:=lh` should be added e.g.: 
```bash
roslaunch sr_robot_launch srhand.launch use_moveit:=true robot_description:=`rospack find sr_description`/robots/shadowhand_left_motor.urdf.xacro hand_id:=lh
```
* Moveit will enable advanced behaviour (inverse kinematics, planning, collision detectection, etc...), but if it is not needed, you can set `use_moveit:=false`

### Real hand

To start a real hand, you can run:
```bash
roslaunch sr_ethercat_hand_config sr_rhand.launch
```

## Shadow hand with UR10 arm

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
