#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.

# An example of how to print the current joint positions for all joints.
# Useful when used in conjunction with 'teach mode', as new motion sequences can
# be quickly produced from physically moving the robot into position and then
# recording those positions with this script. Angles can be printed in radians or degrees, an argument should be added
# when the script is called of either 'radians' or 'degrees', default is radians


from math import pi
import argparse
from numpy import arange
from sr_utilities.hand_finder import HandFinder
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("print_hand_joints_position", anonymous=True)

parser = argparse.ArgumentParser(description='A script to print hand joint positions. ',
                                 add_help=True, usage='%(prog)s [-h] --angle_type ANGLE_TYPE',
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument("--angle_type", default="radians", help="ANGLE_TYPE should be either degrees or radians")
args = parser.parse_args()

angle_type = args.angle_type

# Use the hand finder to get the hand prefix, to allow this script to be used with either left or right hands
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()

prefix = hand_parameters.mapping.values()
hand_serial = hand_parameters.mapping.keys()
scale = 1

if angle_type == "degrees":
    scale = 1 * (180 / pi)

for i in arange(len(prefix)):
    hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial[i])

    print("Joints positions")

    all_joints_state = hand_commander.get_joints_position()

    hand_joints_state = {
        k: (v * scale) for k, v in all_joints_state.items() if k.startswith(prefix[i] + "_")}

    print("Hand joints position \n " + str(hand_joints_state) + "\n")
