#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
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
hand_type = rospy.get_param('/test_hand_joint_movement/test_sim/hand_type', "hand_e_plus")
if hand_type == 'hand_e_plus':
    hand_type = 'hand_e'
hand_id = rospy.get_param('/test_hand_joint_movement/test_sim/hand_id', "rh")

ConfigFingersPack = {
    'hand_e': {'THJ1': 0.0, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
               'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
               'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
               'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
               'LFJ1': 1.5707, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
               'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
    'hand_lite': {'THJ1': 0.0, 'THJ2': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                  'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                  'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                  'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0},
    'hand_extra_lite': {'THJ1': 0.0, 'THJ2': 0.0, 'THJ4': 0.0, 'THJ5': 0.0,
                        'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                        'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0}
}


class TestHandJointMovement(TestCase):
    """
    Tests the Hand Commander
    """
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_sim', anonymous=True)
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if hand_id == 'rh':
            cls.hand_commander = SrHandCommander(name='right_hand')
        if hand_id == 'lh':
            cls.hand_commander = SrHandCommander(name='left_hand')
        # else:
        #     #make it fail - check test to show this

    @classmethod
    def tearDownClass(cls):
        pass

    def test_hand_open(self):
        open_joints_target = self.hand_commander.get_joints_position()

        for k in open_joints_target:
            open_joints_target[k] = 0.0

        self.hand_commander.move_to_joint_value_target(open_joints_target, wait=True)
        rospy.sleep(5)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = 0
        for expected_value, final_value in zip(sorted(open_joints_target), sorted(final_joint_values)):
            if expected_value == final_value:
                expected_and_final_joint_value_diff += abs(open_joints_target[expected_value] -
                                                       final_joint_values[final_value])

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, places=2)

    def test_hand_fingers_pack(self):
        joints_target = ConfigFingersPack[hand_type]

        pack_joints_target = {}

        for k, v in joints_target.items():
            pack_joints_target[hand_id + '_' + k] = v

        self.hand_commander.move_to_joint_value_target(pack_joints_target, wait=True)
        rospy.sleep(5)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = 0
        for expected_value, final_value in zip(sorted(pack_joints_target), sorted(final_joint_values)):
            if expected_value == final_value:
                expected_and_final_joint_value_diff += abs(pack_joints_target[expected_value] -
                                                       final_joint_values[final_value])

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, places=2)


if __name__ == "__main__":
    rostest.rosrun(PKG, "test_sim", TestHandJointMovement)
