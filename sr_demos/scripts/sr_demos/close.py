#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


def execute_trajectory(hand_commander, joint_states_no_id, joint_prefix, msg, time=5.0):
    joints_target = {}
    for key, value in joint_states_no_id.items():
        joints_target[joint_prefix + key] = value

    rospy.loginfo(msg)
    hand_commander.move_to_joint_value_target_unsafe(joints_target, time, True)
    rospy.sleep(1)


if __name__ == "__main__":

    rospy.init_node("close_hand", anonymous=True)

    # Search for gazebo to confirm if in simulation or not
    sim = rospy.search_param('gazebo')

    if sim is None:
        hand_finder = HandFinder()
        joint_prefix = hand_finder.get_hand_parameters().joint_prefix['1']
    else:
        # Default parameter for simulated hand
        joint_prefix = rospy.get_param('/hand/joint_prefix/1322')

    if 'rh_' == joint_prefix:
        hand_name = 'right_hand'
    elif 'lh_' == joint_prefix:
        hand_name = 'left_hand'

    hand_commander = SrHandCommander(name=hand_name)

    open_hand_no_id = {'FFJ1': 0.0, 'FFJ2': 0.0, 'FFJ3': 0.0, 'FFJ4': 0.0,
                       'MFJ1': 0.0, 'MFJ2': 0.0, 'MFJ3': 0.0, 'MFJ4': 0.0,
                       'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
                       'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0, 'LFJ5': 0.0,
                       'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                       'WRJ1': 0.0, 'WRJ2': 0.0}
    close_hand_no_id = {'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                        'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                        'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                        'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                        'LFJ5': 0.0,
                        'WRJ1': 0.0, 'WRJ2': 0.0}
    close_thumb_no_id = {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17}
    # sr-config one
    # close_hand_no_id = {'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
    #                       'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
    #                       'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
    #                       'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0, 'LFJ5': 0.0,
    #                       'THJ1': 0.75, 'THJ2': 0.45, 'THJ3': 0.0, 'THJ4': 1.2, 'THJ5': 0.35,
    #                       'WRJ1': 0.0, 'WRJ2': 0.0}

    execute_trajectory(hand_commander, open_hand_no_id, joint_prefix, "Move hand to open position")
    execute_trajectory(hand_commander, close_hand_no_id, joint_prefix, "Move fingers to close position")
    execute_trajectory(hand_commander, close_thumb_no_id, joint_prefix, "Move thumb to close position")
