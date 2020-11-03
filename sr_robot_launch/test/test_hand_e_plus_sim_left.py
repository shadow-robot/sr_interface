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
from sr_utilities.hand_finder import HandFinder

PKG = "sr_robot_launch"


class TestHandEPlusSimLeft(TestCase):
    """
    Tests the Hand Commander
    """

    def setUp(self):
        rospy.init_node('test_hand_e_plus_sim_left', anonymous=True)
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        self.hand_commander = SrHandCommander(name='left_hand')

    def test_hand_e_plus_sim_left(self):
        expected_joints_target = {'lh_FFJ1': 0.0, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0, 'lh_FFJ4': 0.0,
                                  'lh_MFJ1': 0.0, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0, 'lh_MFJ4': 0.0,
                                  'lh_RFJ1': 0.0, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0, 'lh_RFJ4': 0.0,
                                  'lh_LFJ1': 0.0, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0, 'lh_LFJ4': 0.0,
                                  'lh_THJ1': 0.0, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0,
                                  'lh_THJ5': 0.0, 'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}

        self.hand_commander.move_to_joint_value_target(expected_joints_target)
        rospy.sleep(10)
        final_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = 0
        for expected_value, final_value in zip(expected_joints_target, final_joint_values):
            expected_and_final_joint_value_diff += abs(expected_joints_target[expected_value] -
                                                       final_joint_values[final_value])

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, 1)

if __name__ == "__main__":
    rostest.rosrun(PKG, "test_handEPlusSimLeft", TestHandEPlusSimLeft)
