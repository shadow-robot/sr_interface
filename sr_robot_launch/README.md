# Starting the robots (simulated / real)

## Shadow hand only

### Simulation

To start our hand, simply do:
```bash
roslaunch sr_robot_launch srhand.launch
```

This will launch the five finger hand (shadowhand_motor) by default . If you want to launch another hand, these are the hands available:

|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor_biotac.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor_ff_biotac.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor_btsp.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor_ellipsoid.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_motor_th_ff_rf_ellipsoid.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_muscle.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_muscle_biotac.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_lite.png)|![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/shadowhand_extra_lite.png)|
--- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |

|   | Right                                          | Left                                    | 
|---| ---------------------------------------------- |-----------------------------------------| 
|1  | shadowhand_motor.urdf.xacro                    | shadowhand_left_motor.urdf.xacro        | 
|2  | shadowhand_motor_biotac.urdf.xacro             | shadowhand_left_motor_biotac.urdf.xacro |
|3  | shadowhand_motor_ff_biotac.urdf.xacro          |                                         |
|4  | shadowhand_motor_btsp.urdf.xacro               |                                         |
|5  | shadowhand_motor_ellipsoid.urdf.xacro          |                                         |
|6  | shadowhand_motor_th_ff_rf_ellipsoid.urdf.xacro |                                |
|7  | shadowhand_muscle.urdf.xacro                   | shadowhand_left_muscle.urdf.xacro       |
|8  | shadowhand_muscle_biotac.urdf.xacro            |                                         |
|9  | shadowhand_lite.urdf.xacro                     |                                         |
|10 | shadowhand_extra_lite.urdf.xacro               |                                         |



To start the simulation of a shadow hand, you can run:

```bash
roslaunch sr_robot_launch srhand.launch robot_description:=`rospack find sr_description`/robots/shadowhand_motor.urdf.xacro
```

* The `robot description` param can be changed to start any of the available Shadow hands shown in the table.
* If it is a left hand, `hand_id:=lh` should be added. For example: 
```bash
roslaunch sr_robot_launch srhand.launch robot_description:=`rospack find sr_description`/robots/shadowhand_left_motor.urdf.xacro hand_id:=lh
```
* Moveit will enable advanced behaviour (inverse kinematics, planning, collision detectection, etc...), but if it is not needed, you can set `use_moveit:=false`

### Real hand

To start a real hand, you can run:
```bash
roslaunch sr_ethercat_hand_config sr_rhand.launch
```
It has the specific configuration to launch your hand, including the ethernet port, the hand serial and robot description.

## Shadow hand with UR10 arm
![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/ur10hand.png)

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

To find the hand serial you can launch the command without the `hand_serial` argument and then check the program output. You should see something like:

```
Trying to read mapping for: /hand/mapping/1178
```

In this case 1178 is the serial number of the hand.

To change the hand mapping, you can set the `mapping_path` argument. For example adding:
```
mapping_path:=`rospack find sr_edc_launch`/mappings/default_mappings/rh_E_v3.yaml
```

To change the ethernet port used for your hand, you can add the `eth_port` argument, such as:
```
eth_port:=eth6
```

#### Real Robots, using the normal (not limited) joint range

By default the URDF used for the UR10 arm uses a limited range for the joints, as that helps moveit find a planning solution. But as that restricts the robot movements, the user might want to start the robots with the full joint range. To do that:

```bash
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10.urdf.xacro hand_serial:=1178
```

or, for the left hand and arm

```bash
roslaunch sr_robot_launch sr_left_ur10arm_hand.launch sim:=false robot_description:=`rospack find sr_multi_description`/urdf/left_srhand_ur10.urdf.xacro hand_serial:=1178
```

#### Hand with tactile sensors

If your hand has biotacs sensors, simply append `_biotacs` to the `robot_description:=` and to the `robot_config_file:=` as seen below:

```bash
robot_description:=`rospack find sr_multi_description`/urdf/right_srhand_ur10_joint_limited_biotacs.urdf.xacro robot_config_file:=`rospack find sr_multi_moveit_config`/config/robot_configs/right_sh_ur10_biotac.yaml
``` 

## Bimanual system
![](https://raw.githubusercontent.com/shadow-robot/sr_interface/indigo-devel/images/bimanual.png)

### Simulation
To start the simulation of a bimanual system, you can run:

```bash
roslaunch sr_robot_launch sr_bimanual.launch use_moveit:=true
```

### Real Robots
To start the real robots, do:

```bash
roslaunch sr_robot_launch sr_bimanual.launch sim:=false rh_serial:=1290 lh_serial:=1338
```
