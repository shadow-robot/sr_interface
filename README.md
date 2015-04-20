# sr_robot_commander
This package contains functionality to send commands to robotic arm and hand


## Robot Commander

### General

Main purpose of the commander is to provide simplified access to hand or arm.
It provides methods which can be used on both arm and hand.

Examples of usage can be found in the package **sr_example** in files **sr_hand_examples.py** and **sr_arm_examples.py**

### move_to_joint_value_target 

#### Description

This method sets target of the robot's links and moves to it. 

Parameters:
 
   * *joint_states* is a dictionary with joint name and value
   * *wait_result* indicates if method should wait for movement end or not (default value is True)

#### Example 

```python

rospy.init_node("robot_commander_examples", anonymous=True)

arm_commander = SrArmCommander()
joints_states = {'ra_shoulder_pan_joint': 0.5157461682721474, 
                   'ra_elbow_joint': 0.6876824920327893,
                   'ra_wrist_1_joint': -0.7695210732233582,
                   'ra_wrist_2_joint': 0.2298871642157314,
                   'ra_shoulder_lift_joint': -0.9569080092786892,
                   'ra_wrist_3_joint': -0.25991215955733704}
arm_commander.move_to_joint_value_target(joints_states)
```

### move_to_named_target

#### Description

Using this method will allow to move hand or arm to predefined pose. This pose can be define using MoveIt assistant.

Parameters:
 
   * *name* is the unique identifier of the target pose defined in SRDF
   * *wait_result* indicates if method should wait for movement end or not (default value is True)

In order to created new named pose you can do following: 

* Run shell command 
```bash
roslaunch ur10srh_moveit_config setup_assistant.launch
```
* In UI wizard press "Load Files" button
* Wait until files load successfully 
* Go to section "Robot Poses" of the wizard (select from list on the left)
* Press "Add Pose"
* On the screen which will appear you can add you pose for at least two "Planing Group"
  * right_hand
  * right_arm
* You should provide unique name of the pose (which will be referred in move_to_named_target method) and select joints position for this pose using slider and simulated image of robot
* Press save button
* Go to "Configurations File" section of the wizard
* Tick checkbox with text "config/ur10srh.srdf" in the checkbox list
* Press "Generate Package" and wait until progress is 100%
* Exit wizard  

#### Example 

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander()

# pack is predefined pose from SRDF file
hand_commander.move_to_named_target("pack")
```

### get_joints_position and get_joints_velocity

#### Description 

These methods do not take any parameters and return dictionary with position and velocity of the robot joints

#### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

arm_commander = SrArmCommander()

joints_position = arm_commander.get_joints_position()
joints_velocity = arm_commander.get_joints_velocity()

print("Arm joints position\n" + str(joints_position) + "\n")
print("Arm joints velocities\n" + str(joints_velocity) + "\n")
```

## Arm Commander

### General

This commander provide commands specific to arm which allows to move execute all actions of the robot commander.
Also it allows to move to certain position in Cartesian space, to joints states values and to move using certain trajectory.
 

### move_to_position_target

#### Description

This method allows to move end effector of the arm to certain point in the space represented by (x, y, z) coordinates.
The orientation of the end effector can be any.

Parameters:
 
  * *xyz* new position of end-effector
  * *end_effector_link* name of the end effector link (default value is empty string)
  * *wait* indicates if method should wait for movement end or not (default value is True)

#### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

arm_commander = SrArmCommander()

new_position = [0.25527, 0.36682, 0.5426]
arm_commander.move_to_position_target(new_position)
```

## Hand Commander
 
### General

### Main Methods
 
 