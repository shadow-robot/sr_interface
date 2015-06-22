#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("print_left_joints_position", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")

print("Joints positions")

all_joints_state = hand_commander.get_joints_position()

hand_joints_state = {k: v for k, v in all_joints_state.items() if k.startswith("lh_")}
arm_joints_state = {k: v for k, v in all_joints_state.items() if k.startswith("la_")}


print("Hand joints position \n " + str(hand_joints_state) + "\n")
print("Arm joints position \n " + str(arm_joints_state) + "\n")
