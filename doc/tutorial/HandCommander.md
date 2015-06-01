# Hand Commander
 
## Description

This commander provides commands specific to hand which allows to execute all actions of the [robot commander](RobotCommander.md).
Also it allows to get state of tactile sensors, set max force and get joints effort.

## get_joints_effort

### Description 

This method does not take any parameters and returns dictionary with efforts of the robot joints

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander("right_hand", "rh")

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

hand_commander = SrHandCommander("right_hand", "rh")

# The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units) 
hand_commander.set_max_force("rh_FFJ3", 600)
```

## get_tactile_type and get_tactile_state

### Description

*get_tactile_type* returns a string indicating the type of tactile sensors present (e.g. PST, biotac, UBI0). 
*get_tactile_state* returns an object containing tactile data. The structure of the data is different for every tactile_type .

### Example

```python

rospy.init_node("robot_commander_examples", anonymous=True)

hand_commander = SrHandCommander("right_hand", "rh")

tactile_type = hand_commander.get_tactile_type()
tactile_state = hand_commander.get_tactile_state()

print("Hand tactile type\n" + tactile_type + "\n")
print("Hand tactile state\n" + str(tactile_state) + "\n")
```
