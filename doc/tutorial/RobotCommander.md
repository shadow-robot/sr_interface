# Robot Commander

## Description

Main purpose of the commander is to provide simplified access to [hand](HandCommander.md) or [arm](ArmCommander.md).
It provides methods which can be used on both [hand](HandCommander.md) and [arm](ArmCommander.md).

Examples of usage can be found in the package **sr_example** in files **sr_hand_examples.py**, **sr_arm_examples.py** and **sr_handfinder_examples.py**.

**Warning** RobotCommander should not direcly be used. Unless necessary use [Hand commander](HandCommander.md) or [Arm commander](ArmCommander.md).
## Constructor

The constructors for `SrArmCommander` and `SrHandCommander` take a name parameter that should match the group name of the robot to be used.

### Example
This example uses [HandFinder](../../../sr_utilities/README.md) for finding launched hand.
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

hand_id = hand_parameters.mapping[hand_serial]

prefix = hand_parameters.joint_prefix[hand_serial]

if hand_id == 'rh':
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
   * *angle_degrees* should be set to true if the input angles are in degrees (default value is False)

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
This example demonstrates how joint states for an arm can be sent to SrArmCommander, as neither the 'wait' nor 'angle_degrees' arguments are specified, they take the default values of 'True' and 'False, respectively. 


### Example 2

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander(name="right_hand")
joints_states = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 0.0,
                 'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 0.0,
                 'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 0.0,
                 'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                 'rh_THJ1': 40, 'rh_THJ2': 35, 'rh_THJ3': 0.0, 'rh_THJ4': 65, 'rh_THJ5': 15,
                 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
hand_commander.move_to_joint_value_target(joints_states, wait=False, angle_degrees=True))
```

In this example, joint states for a hand are sent to SrHandCommander, the method is prompted by the 'wait=False' argument to not wait for the movement to finish executing before moving on to the next command and the 'angle_degrees=True' argument tells the method that the input angles are in degrees, so require a conversion to radians. 

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

hand_id = hand_parameters.mapping[hand_serial]

if hand_id == 'rh':
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
**Warning** All of above codes will crash if hand is not launched yet. If you are using HandFinder, you can avoid this by checking the length of the mapping. Otherwise you can check the parameter server directly to see if the hand is launched.
