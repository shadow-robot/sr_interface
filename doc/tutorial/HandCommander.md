# Hand Commander
 
## Description

This commander provide commands specific to hand which allows to move execute all actions of the [robot commander](RobotCommander.md).
Also it allows to get state of tactile sensors, set max force and get joints effort.

## get_joints_effort

### Description 

This method do not take any parameters and return dictionary with efforts of the robot joints

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander()

hand_joints_effort = hand_commander.get_joints_effort()

print("Hand joints effort \n " + str(hand_joints_effort) + "\n")
```

## set_max_force

### Description

  This method sets maximum force for hand
 
Parameters:
 
  * *joint_name* name of the joint.
  * *value* maximum force value

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander()

hand_commander.set_max_force("rh_FFJ3", 1)
```

## get_tactile_type and get_tactile_state

### Description

*get_tactile_type* returns a string indicating the type of tactile sensors present (e.g. PST, biotac, UBI0). 
*get_tactile_state* returns an object containing tactile data. The structure of the data is different for every tactile_type .

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander()

tactile_type = hand_commander.get_tactile_type()
tactile_state = hand_commander.get_tactile_state()

print("Hand tactile type\n" + tactile_type + "\n")
print("Hand tactile state\n" + str(tactile_state) + "\n")
```
