#!/usr/bin/env python

# An example of how to print the current joint positions for all joints.
# Useful when used in conjunction with 'teach mode', as new motion sequences can
# be quickly produced from physically moving the robot into position and then
# recording those positions with this script. Angles can be printed in radians or degrees, an argument should be added
# when the script is called of either 'radians' or 'degrees', default is radians

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from numpy import arange
from math import pi
import argparse


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
    scale = 1 * (180/pi)

for i in arange(0, len(prefix)):
    hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial[i])

    print("Joints positions")

    all_joints_state = hand_commander.get_joints_position()

    hand_joints_state = {
        k: (v * scale) for k, v in all_joints_state.items() if k.startswith(prefix[i] + "_")}

    print("Hand joints position \n " + str(hand_joints_state) + "\n")
