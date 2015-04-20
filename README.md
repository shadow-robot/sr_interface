# sr_robot_commander
This package contains functionality to send commands to robotic arm and hand


## Robot Commander

### General

Main purpose of the commander is to provide simplified access to hand or arm.
It provides methods which can be used on both arm and hand.

Examples of usage can be found in the package **sr_example** in files **sr_hand_examples.py** and **sr_arm_examples.py**

### Main methods
 
**move_to_joint_value_target** set target of the robot's links and moves to it. 
The parameter *joint_states* is a dictionary with joint name and value and *wait_result* indicates if method should wait for movement end or not (default value is True)

Example of usage 

''' python
joints_states_1 = {'ra_shoulder_pan_joint': 0.5157461682721474, 'ra_elbow_joint': 0.6876824920327893,
                   'ra_wrist_1_joint': -0.7695210732233582, 'ra_wrist_2_joint': 0.2298871642157314,
                   'ra_shoulder_lift_joint': -0.9569080092786892, 'ra_wrist_3_joint': -0.25991215955733704}
arm_commander.move_to_joint_value_target(joints_states_1)
'''

## Arm Commander

### General

This commander provide commands specific to arm which allows to move execute all actions of the robot commander.
Also it allows to move to certain position in Cartesian space, to joints states values and to move using certain trajectory.
 

### Main methods


## Hand Commander
 
### General

### Main Methods
 
 