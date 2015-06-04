#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("store_right_hand", anonymous=True)

hand_commander = SrHandCommander()

named_target_1 = "open"
print("Moving to hand named target " + named_target_1)
hand_commander.move_to_named_target(named_target_1)
rospy.sleep(1)

named_target_2 = "pack"
print("Moving to hand named target " + named_target_2)
hand_commander.move_to_named_target(named_target_2)
rospy.sleep(1)
