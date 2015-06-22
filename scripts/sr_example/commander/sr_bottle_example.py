#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("grab_bottle_examples", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

#start a bit higher
#arm_commander.move_to_named_target("gamma", True)

#approach
hand_commander.move_to_named_target("open_bottle", False)
arm_commander.move_to_named_target("approach_bottle", True)

#grab
arm_commander.move_to_named_target("hold_bottle", True)
hand_commander.move_to_named_target("hold_bottle", True)

#pour
arm_commander.move_to_named_target("pour_bottle", True)

#release
arm_commander.move_to_named_target("hold_bottle", True)
hand_commander.move_to_named_target("open_bottle", True)

#and move back
arm_commander.move_to_named_target("approach_bottle", True)

arm_commander.move_to_named_target("victory")
hand_commander.move_to_named_target("victory")

