# Starting the robots (simulated / real)

## Shadow hand only

### Simulation

These are the hands available:

|                             | Right                                          | Left                                    | 
|-----------------------------| ---------------------------------------------- |-----------------------------------------| 
|![shadowhand_motor]          | shadowhand_motor.urdf.xacro                    | shadowhand_left_motor.urdf.xacro        | 
|![shadowhand_motor_biotac]   | shadowhand_motor_biotac.urdf.xacro             | shadowhand_left_motor_biotac.urdf.xacro |
|![shadowhand_motor_ff_biotac]| shadowhand_motor_ff_biotac.urdf.xacro          |                                         |
|![shadowhand_motor_btsp]     | shadowhand_motor_btsp.urdf.xacro               |                                         |
|![shadowhand_motor_ellipsoid]| shadowhand_motor_ellipsoid.urdf.xacro          |                                         |
|![shadowhand_motor_th_ff_rf_ellipsoid]| shadowhand_motor_th_ff_rf_ellipsoid.urdf.xacro |                                |
|![shadowhand_motor_btsp]     | shadowhand_motor_btsp.urdf.xacro               |                                         |
|![shadowhand_muscle]         | shadowhand_muscle.urdf.xacro                   | shadowhand_left_muscle.urdf.xacro       |
|![shadowhand_muscle_biotac]  | shadowhand_muscle_biotac.urdf.xacro            |                                         |
|![shadowhand_lite]           | shadowhand_lite.urdf.xacro                     |                                         |
|![shadowhand_extra_lite]     | shadowhand_extra_lite.urdf.xacro               |                                         |

[shadowhand_motor]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadow_motor.png 
[shadowhand_motor_biotac]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadow_motor_biotac.png
[shadowhand_motor_ff_biotac]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_motor_ff_biotac.png
[shadowhand_motor_btsp]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_motor_btsp.png
[shadowhand_motor_ellipsoid]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_motor_ellipsoid.png
[shadowhand_motor_th_ff_rf_ellipsoid]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_motor_th_ff_rf_ellipsoid.png
[shadowhand_motor_btsp]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_motor_btsp.png
[shadowhand_muscle]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_muscle.png
[shadowhand_muscle_biotac]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_muscle_biotac.png
[shadowhand_lite]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_lite.png
[shadowhand_extra_lite]: https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/shadowhand_extra_lite.png

To start the simulation of a shadow hand, you can run:

```bash
roslaunch sr_robot_launch srhand.launch use_moveit:=true robot_description:=`rospack find sr_description`/robots/shadowhand_motor.urdf.xacro
```

* The `robot description` param can be changed to start any of the available Shadow hands shown in the table.
* If it is a left hand, `hand_id:=lh` should be added. For example: 
```bash
roslaunch sr_robot_launch srhand.launch use_moveit:=true robot_description:=`rospack find sr_description`/robots/shadowhand_left_motor.urdf.xacro hand_id:=lh
```
* Moveit will enable advanced behaviour (inverse kinematics, planning, collision detectection, etc...), but if it is not needed, you can set `use_moveit:=false`

### Real hand

To start a real hand, you can run:
```bash
roslaunch sr_ethercat_hand_config sr_rhand.launch
```
It has the specific configuration to launch your hand, including the ethernet port, the hand serial and robot description.

## Shadow hand with UR10 arm
![alt text](https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/ur10hand.png)

### Simulation
To start the simulation of the hand and arm, you can run:

```bash
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch sr_left_ur10arm_hand.launch
```

### Real Robots
To start the real robots, do:

```bash
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=false hand_serial:=1178
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch sr_left_ur10arm_hand.launch sim:=false hand_serial:=1178
```

To find the hand serial you can launch the command without the hand_serial argument and then check the program output. You should see something like:

```
Trying to read mapping for: /hand/mapping/1178
```

In this case 1178 is the serial number of the hand.

#### Real Robots, using the normal (not limited) joint range

By default the URDF used for the UR10 arm uses a limited range for the joints, as that helps moveit find a planning solution. But as that restricts the robot movements, the user might want to start the robots with the full joint range. To do that:

```bash
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10.urdf.xacro hand_serial:=1178
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch sr_left_ur10arm_hand.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/left_srhand_ur10.urdf.xacro hand_serial:=1178
```

#### Hand with Biotacs

If your hand has biotacs, simply append `_biotacs` to the `robot_description:=` and to the `robot_config_file:=` as seen below:

```bash
robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10_joint_limited_biotacs.urdf.xacro robot_config_file:=`rospack find sr_multi_moveit_config`/config/robot_configs/right_sh_ur10_biotac.yaml
``` 
## Bimanual system
![alt text](https://github.com/shadow-robot/sr_interface/blob/F%23402_improve_documentation/images/bimanual.png)

### Simulation
To start the simulation of a bimanual system, you can run:

```bash
roslaunch sr_robot_launch sr_bimanual.launch use_moveit:=true
```

### Real Robots
To start the real robots, do:

```bash
roslaunch sr_robot_launch sr_bimanual.launch use_moveit:=true sim:=false rh_serial:=1290 lh_serial:=1338
```
