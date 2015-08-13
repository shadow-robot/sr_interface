# Robot Commander

## Description

Main purpose of the commander is to provide simplified access to [hand](HandCommander.md) or [arm](ArmCommander.md).
It provides methods which can be used on both [hand](HandCommander.md) and [arm](ArmCommander.md).

Examples of usage can be found in the package **sr_example** in files **sr_hand_examples.py**, **sr_arm_examples.py** and **sr_handfinder_examples.py**.

## Constructor

The constructors for `SrArmCommander` and `SrHandCommander` take a name parameter that should match the group name of the robot to be used.

### Example

For a right arm:

```python
arm_commander = SrArmCommander(name="right_arm", set_ground=True)
```

For a left arm:

```python
arm_commander = SrArmCommander(name="left_arm", set_ground=True)
```
You can use HandFinder utility to find the hand launched on the system.

```python
hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()

hand_serial = hand_parameters.mapping.keys()[0]

hand_mapping = hand_parameters.mapping[hand_serial]

prefix = hand_parameters.joint_prefix[hand_serial]

if hand_mapping == 'rh':
    hand_commander = SrHandCommander(name="right_hand", prefix="rh")
else:
    hand_commander = SrHandCommander(name="left_hand", prefix="lh")
```
Alternatively you can hardcode the hand you are launching.

For a right hand:

```python
hand_commander = SrHandCommander(name="right_hand", prefix="rh")
```

For a left hand:

```python
hand_commander = SrHandCommander(name="left_hand", prefix="lh")
```

## move_to_joint_value_target

### Description

This method sets target of the robot's links and moves to it.

Parameters:

   * *joint_states* is a dictionary with joint name and value. It can contain joints values of which need to be changed.
   * *wait* indicates if method should wait for movement end or not (default value is True)

*IMPORTANT:* Bear in mind that the names of the joints are different for the right arm/hand and for the left one.

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

arm_commander = SrArmCommander(name="right_arm", set_ground=True)
joints_states = {'ra_shoulder_pan_joint': 0.5157461682721474,
                 'ra_elbow_joint': 0.6876824920327893,
                 'ra_wrist_1_joint': -0.7695210732233582,
                 'ra_wrist_2_joint': 0.2298871642157314,
                 'ra_shoulder_lift_joint': -0.9569080092786892,
                 'ra_wrist_3_joint': -0.25991215955733704}
arm_commander.move_to_joint_value_target(joints_states)
```

## move_to_named_target

### Description

Using this method will allow to move hand or arm to predefined pose. This pose can be define using MoveIt assistant.

Parameters:

   * *name* is the unique identifier of the target pose defined in SRDF
   * *wait* indicates if method should wait for movement end or not (default value is True)

In order to created a new named pose you can do following:

* Run shell command
```bash
roslaunch ur10srh_moveit_config setup_assistant.launch
```
* In UI wizard press "Load Files" button
* Wait until files load successfully
* Go to section "Robot Poses" of the wizard (select from list on the left)
* Press "Add Pose"
* On the screen which will appear you can add your pose for at least two "Planing Group" (it depends on the robot you are running right or left), e.g.:
  * right_hand
  * right_arm
* You should provide the unique name of the pose (which will be referred in move_to_named_target method) and select joints position for this pose using slider and simulated image of robot
* Press save button
* Go to "Configurations File" section of the wizard
* Tick checkbox with text "config/ur10srh.srdf" in the checkbox list
* Press "Generate Package" and wait until progress is 100%
* Exit wizard

![MoveIt Setup Assistant](/sr_robot_commander/doc/tutorial/images/moveit_setup_assistant.gif)


### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()

hand_serial = hand_parameters.mapping.keys()[0]

hand_mapping = hand_parameters.mapping[hand_serial]

if hand_mapping == 'rh':
    hand_commander = SrHandCommander(name="right_hand", prefix="rh")
else:
    hand_commander = SrHandCommander(name="left_hand", prefix="lh")

# pack is predefined pose from SRDF file
hand_commander.move_to_named_target("pack")
```
Note: you can hardcode the parameters instead of using HandFinder utility```python
```python
rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander("left_hand", "lh")

# pack is predefined pose from SRDF file
hand_commander.move_to_named_target("pack")
``` 

## get_joints_position and get_joints_velocity

### Description

These methods do not take any parameters and return dictionary with position and velocity of the robot joints

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

arm_commander = SrArmCommander(name="right_arm", set_ground=True)

joints_position = arm_commander.get_joints_position()
joints_velocity = arm_commander.get_joints_velocity()

print("Arm joints position\n" + str(joints_position) + "\n")
print("Arm joints velocity\n" + str(joints_velocity) + "\n")


```
