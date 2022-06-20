#!/usr/bin/env python3

# Copyright 2021-2022 Shadow Robot Company Ltd.
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

# Script to move a shadow hand into open position.

import argparse
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


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

    rospy.init_node("open_hand", anonymous=True)

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
        joint_prefix_main = 'rh_'
    elif args.side == 'left':
        joint_prefix_main = 'lh_'
    else:
        joint_prefix_main = 'both'

    if joint_prefix_main == 'rh_':
        hand_name = 'right_hand'
    elif joint_prefix_main == 'lh_':
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    hand_commander_class = SrHandCommander(name=hand_name)

    open_hand_no_id = {'FFJ1': 0.0, 'FFJ2': 0.0, 'FFJ3': 0.0, 'FFJ4': 0.0,
                       'MFJ1': 0.0, 'MFJ2': 0.0, 'MFJ3': 0.0, 'MFJ4': 0.0,
                       'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
                       'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0, 'LFJ5': 0.0,
                       'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                       'WRJ1': 0.0, 'WRJ2': 0.0}

    execute_trajectory(hand_commander_class, open_hand_no_id, joint_prefix_main, "Moving to open position")
