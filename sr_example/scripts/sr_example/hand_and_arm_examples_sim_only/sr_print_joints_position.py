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

# An example of how to print the current joint positions for all joints.
# Useful when used in conjunction with 'teach mode', as new motion sequences can
# be quickly produced from physically moving the robot into position and then
# recording those positions with this script. Angles can be printed in radians or degrees, an argument should be added
# when the script is called of either 'radians' or 'degrees', default is radians

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from math import pi
import argparse


rospy.init_node("print_joints_position", anonymous=True)

parser = argparse.ArgumentParser(description='A script to print hand and arm joint positions. ',
                                 add_help=True, usage='%(prog)s [-h] --angle_type ANGLE_TYPE',
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument(dest='--angle_type', help="ANGLE_TYPE should be either degrees or radians")
args = parser.parse_args()

angle_type = args.angle_type

# Use the hand finder to get the hand prefix, to allow this script to be used with either left or right hands
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = hand_parameters.mapping.values()[0]
hand_serial = hand_parameters.mapping.keys()[0]
scale = 1

if angle_type == "degrees":
    scale = 1 * (180/pi)

hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

print("Joints positions")

all_joints_state = hand_commander.get_joints_position()


hand_joints_state = {
    k: (v * scale) for k, v in all_joints_state.items()
    if k.startswith(prefix + "_")and not k.startswith(prefix + "_W")}
arm_joints_state = {
    k: (v * scale) for k, v in all_joints_state.items()
    if k.startswith(prefix[0] + "a_") or k.startswith(prefix + "_W")}


print("Hand joints position \n " + str(hand_joints_state) + "\n")

print("Arm joints position \n " + str(arm_joints_state) + "\n")
