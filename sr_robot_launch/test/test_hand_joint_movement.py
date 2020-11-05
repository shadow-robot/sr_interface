#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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

import rospy
import rostest
from sr_robot_commander.sr_hand_commander import SrHandCommander
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

PKG = "sr_robot_launch"


def move_to_target_and_check_error(hand_commander, target):
    hand_commander.move_to_joint_value_target(target, wait=True)
    rospy.sleep(5)
    final_joint_values = hand_commander.get_current_state()

    expected_and_final_joint_value_diff = 0
    for expected_value, final_value in zip(sorted(target), sorted(final_joint_values)):
        expected_and_final_joint_value_diff += abs(target[expected_value] -
                                                   final_joint_values[final_value])
    return expected_and_final_joint_value_diff


class TestHandJointMovement(TestCase):
    """
    Tests the Hand Commander
    """
    @classmethod
    def setUpClass(cls):
        cls.hand_type = rospy.get_param('~test_sim/hand_type', 'hand_e_plus')
        if cls.hand_type == 'hand_e':
            cls.hand_type = 'hand_e_plus'
        elif cls.hand_type not in ('hand_e_plus', 'hand_lite', 'hand_extra_lite'):
            raise Exception("The specified hand_type is incorrect.")
        cls.hand_id = rospy.get_param('~test_sim/hand_id', 'rh')

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if cls.hand_id == 'rh':
            cls.hand_commander = SrHandCommander(name='right_hand')
        elif cls.hand_id == 'lh':
            cls.hand_commander = SrHandCommander(name='left_hand')
        else:
            raise Exception("The specified hand_id is incorrect.")

    @classmethod
    def tearDownClass(cls):
        pass

    def test_hand_open(self):
        open_joints_target = self.hand_commander.get_joints_position()
        for key in open_joints_target:
            open_joints_target[key] = 0.0

        expected_and_final_joint_value_diff = move_to_target_and_check_error(self.hand_commander, open_joints_target)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.1)

    def test_hand_pack(self):
        hand_pack_joint_targets = {
            'hand_e_plus': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                            'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                            'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                            'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                            'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                            'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
            'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                          'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                          'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                          'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0},
            'hand_extra_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                                'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                                'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0}
        }

        pack_joints_target_no_id = hand_pack_joint_targets[self.hand_type]
        pack_joints_target = {}
        for key, value in pack_joints_target_no_id.items():
            pack_joints_target[self.hand_id + '_' + key] = value

        expected_and_final_joint_value_diff = move_to_target_and_check_error(self.hand_commander, pack_joints_target)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.1)


if __name__ == "__main__":
    rospy.init_node('test_sim', anonymous=True)
    rostest.rosrun(PKG, "test_sim", TestHandJointMovement)
