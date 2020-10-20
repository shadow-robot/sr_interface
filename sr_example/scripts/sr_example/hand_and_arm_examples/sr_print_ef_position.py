#!/usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# An example of how to print the current end effector of the hand and arm

# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander


import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_utilities.hand_finder import HandFinder
from math import pi
import argparse


rospy.init_node("print_ef_position", anonymous=True)


hand_commander = SrHandCommander(name="left_hand")
arm_commander = SrArmCommander(name="left_arm")

print("The end effectors:\n")

eff_pose_arm = arm_commander.get_current_pose()

print("The arm end effector pose:\n" + str(eff_pose_arm))

eff_pose_hand = arm_commander.get_current_pose("rh_palm")

print("The hand end effector pose:\n" + str(eff_pose_hand))
