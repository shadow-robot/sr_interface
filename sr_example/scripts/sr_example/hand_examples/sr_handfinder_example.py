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


import rospy
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander

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
