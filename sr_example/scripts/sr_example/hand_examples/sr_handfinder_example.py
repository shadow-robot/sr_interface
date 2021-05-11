#!/usr/bin/env python3
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

# Example use of handfinder for a 5 fingered hand. The serial number and hand parameters
# are read and it is detected whether the hand is left or right and if there are tactiles present.
# The correct prefix and parameters are then configured.

from __future__ import absolute_import
import sys
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("basic_hand_examples", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

hand_mapping = hand_parameters.mapping[hand_serial]

# Hand joints are detected
joints = hand_finder.get_hand_joints()[hand_mapping]

position_values = [0.35, 0.18, 0.38]

# Moving to a target determined by the values in position_values.
rospy.loginfo("Hand moving to script specified target")
position_1 = dict(zip(joints, position_values))
hand_commander.move_to_joint_value_target(position_1)

named_target_1 = "pack"
rospy.loginfo("Hand moving to named target: " + named_target_1)
hand_commander.move_to_named_target(named_target_1)

named_target_2 = "open"
rospy.loginfo("Hand moving to named target: " + named_target_2)
hand_commander.move_to_named_target(named_target_2)

# Hand joints state, velocity and effort are read and displayed to screen.
hand_joints_state = hand_commander.get_joints_position()
hand_joints_velocity = hand_commander.get_joints_velocity()
hand_joints_effort = hand_commander.get_joints_effort()

rospy.loginfo("Hand joints position \n " + str(hand_joints_state) + "\n")
rospy.loginfo("Hand joints velocity \n " + str(hand_joints_velocity) + "\n")
rospy.loginfo("Hand joints effort \n " + str(hand_joints_effort) + "\n")

# Tactile type and state are read and displayed to screen.
tactile_type = hand_commander.get_tactile_type()
tactile_state = hand_commander.get_tactile_state()
rospy.loginfo("Tactile type \n " + str(tactile_type) + "\n")
rospy.loginfo("Tactile state \n " + str(tactile_state) + "\n")
