#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2022 belongs to Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import argparse
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


def execute_trajectory(hand_commander, joint_states_no_id, joint_prefix, msg, time=5.0):
    joints_target = {}
    for key, value in joint_states_no_id.items():
        if joint_prefix == 'both':
            joints_target['rh_' + key] = value
            joints_target['lh_' + key] = value
        else:
            joints_target[joint_prefix + key] = value

    rospy.loginfo(msg)
    hand_commander.move_to_joint_value_target_unsafe(joints_target, time, True)
    rospy.sleep(1)


if __name__ == "__main__":

    rospy.init_node("close_hand", anonymous=True)

    parser = argparse.ArgumentParser(description="Hand side")
    parser.add_argument("-s", "--side",
                        dest="side",
                        type=str,
                        required=True,
                        help="Please select hand side, can be 'right', 'left' or 'both'.",
                        default=None,
                        choices=["right", "left", "both"])

    args = parser.parse_args(rospy.myargv()[1:])

    if args.side == 'right':
        joint_prefix = 'rh_'
    elif args.side == 'left':
        joint_prefix = 'lh_'
    else:
        joint_prefix = 'both'

    if 'rh_' == joint_prefix:
        hand_name = 'right_hand'
    elif 'lh_' == joint_prefix:
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    hand_commander = SrHandCommander(name=hand_name)

    open_thumb = {'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0}
    open_fingers = {'FFJ1': 0.0, 'FFJ2': 0.0, 'FFJ3': 0.0, 'FFJ4': 0.0,
                    'MFJ1': 0.0, 'MFJ2': 0.0, 'MFJ3': 0.0, 'MFJ4': 0.0,
                    'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
                    'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0, 'LFJ5': 0.0,
                    'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                    'WRJ1': 0.0, 'WRJ2': 0.0}
    close_fingers = {'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                     'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                     'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                     'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                     'LFJ5': 0.0,
                     'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                     'WRJ1': 0.0, 'WRJ2': 0.0}
    close_thumb = {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17}

    execute_trajectory(hand_commander, open_thumb, joint_prefix, "Moving thumb to open position")
    execute_trajectory(hand_commander, open_fingers, joint_prefix, "Moving fingers to open position")
    execute_trajectory(hand_commander, close_fingers, joint_prefix, "Moving fingers to close position")
    execute_trajectory(hand_commander, close_thumb, joint_prefix, "Moving thumb to close position")
