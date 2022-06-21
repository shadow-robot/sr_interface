#!/usr/bin/env python3

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

from unittest import TestCase
import rospy
import rostest
from actionlib_msgs.msg import GoalStatusArray
from sr_robot_commander.sr_hand_commander import SrHandCommander

PKG = "sr_robot_launch"


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

    def test_hand_open(self):
        open_joints_target = self.hand_commander.get_joints_position()
        for key in open_joints_target:
            open_joints_target[key] = 0.0

        self.hand_commander.move_to_joint_value_target(open_joints_target, wait=True)
        rospy.sleep(5)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = joints_error_check(open_joints_target, final_joint_values)

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

        expected_and_final_joint_value_diff = joints_error_check(pack_joints_target, final_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.1)

def joints_error_check(expected_joint_values, recieved_joint_values):
    expected_and_final_joint_value_diff = 0
    for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
        expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                    recieved_joint_values[recieved_value])
    return expected_and_final_joint_value_diff

if __name__ == "__main__":
    rospy.init_node('test_sim', anonymous=True)
    rostest.rosrun(PKG, "test_sim", TestHandJointMovement)
