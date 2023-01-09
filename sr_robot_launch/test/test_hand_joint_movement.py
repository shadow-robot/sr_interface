#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2020-2023 belongs to Shadow Robot Company Ltd.
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

from unittest import TestCase
import rostest
import rospy
from actionlib_msgs.msg import GoalStatusArray
from sr_robot_commander.sr_hand_commander import SrHandCommander


class TestHandJointMovement(TestCase):
    """
    Tests the Hand Commander
    """

    @classmethod
    def setUpClass(cls):
        cls.hand_type = rospy.get_param('~test_sim/hand_type', 'hand_e')
        if cls.hand_type not in ('hand_e', 'hand_g'):
            raise TypeError("The specified hand_type is incorrect.")
        cls.hand_id = rospy.get_param('~test_sim/hand_id', 'rh')

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if cls.hand_id == 'rh':
            cls.hand_commander = SrHandCommander(name='right_hand')
        elif cls.hand_id == 'lh':
            cls.hand_commander = SrHandCommander(name='left_hand')
        else:
            raise TypeError("The specified hand_id is incorrect.")

    @classmethod
    def tearDownClass(cls):
        pass

    @staticmethod
    def joints_error_check(expected_joint_values, received_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, received_value in zip(sorted(expected_joint_values), sorted(received_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       received_joint_values[received_value])
        return expected_and_final_joint_value_diff

    def test_hand_open(self):
        open_joints_target = self.hand_commander.get_joints_position()
        for key in open_joints_target:
            open_joints_target[key] = 0.0

        self.hand_commander.move_to_joint_value_target(open_joints_target, wait=True)
        rospy.sleep(5)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = self.joints_error_check(open_joints_target, final_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.1)

    def test_hand_pack(self):
        hand_pack_joint_targets = {
            'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                       'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                       'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                       'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                       'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                       'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
            'hand_g': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                       'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                       'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                       'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0},
            'hand_g_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                            'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                            'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0}
        }

        pack_joints_target_no_id = hand_pack_joint_targets[self.hand_type]
        pack_joints_target = {}
        for key, value in pack_joints_target_no_id.items():
            pack_joints_target[self.hand_id + '_' + key] = value

        self.hand_commander.move_to_joint_value_target(pack_joints_target, wait=True)
        rospy.sleep(5)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = self.joints_error_check(pack_joints_target, final_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.1)


if __name__ == "__main__":
    pkg_name = "sr_robot_launch"
    rospy.init_node('test_sim', anonymous=True)
    rostest.rosrun(pkg_name, "test_sim", TestHandJointMovement)
