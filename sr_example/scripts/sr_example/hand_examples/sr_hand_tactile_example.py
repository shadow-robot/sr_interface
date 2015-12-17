#!/usr/bin/env python

# Reading the tactiles from the hand.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("tactile_reader", anonymous=True)
hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

rospy.sleep(1.0)

print "Tactile type: ", hand_commander.get_tactile_type()
print "Tactile state: ", hand_commander.get_tactile_state()
