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

# An example of how to print the current joint positions of the right arm and hand for all joints.
# be quickly produced from physically moving the robot into position and then
# recording those positions with this script. Angles can be printed in radians or degrees, an argument should be added
# when the script is called of either 'radians' or 'degrees', default is radians

# For more information, please see https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_right_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_right_ur10arm_hand.launch sim:=true scene:=true

import rospy
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from math import pi
import argparse


rospy.init_node("print_joints_position", anonymous=True)


parser = argparse.ArgumentParser(description='A script to print hand and arm joint positions. ',
                                 add_help=True, usage='%(prog)s [-h] --angle_type ANGLE_TYPE',
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument('--angle_type', dest='angle_type', default='radians', help="ANGLE_TYPE should be either degrees or radians")
args = parser.parse_args()

angle_type = args.angle_type

if angle_type not in ['radians', 'degrees']:
    parser.print_help()
    exit(1)

scale = 1

if angle_type == "degrees":
    scale = 1 * (180/pi)

# The constructors for SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
robot_commander = SrRobotCommander(name="right_arm_and_hand")

print("Joints positions")

# get_joints_position returns a dictionary with joints poisitions
all_joints_state = robot_commander.get_joints_position()

hand_joints_state = {
    k: (v * scale) for k, v in all_joints_state.items()
    if k.startswith("rh_")and not k.startswith("rh_W")}
arm_joints_state = {
    k: (v * scale) for k, v in all_joints_state.items()
    if k.startswith("ra_") or k.startswith("rh_W")}


print("Hand joints position \n " + str(hand_joints_state) + "\n")

print("Arm joints position \n " + str(arm_joints_state) + "\n")