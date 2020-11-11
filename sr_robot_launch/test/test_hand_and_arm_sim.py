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
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

PKG = "sr_robot_launch"


class TestHandAndArmSim(TestCase):
    """
    Tests the Robot Commander
    """

    def setUp(self):
        rospy.init_node('test_hand_and_arm_sim', anonymous=True)
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        self.robot_commander = SrRobotCommander(name="right_arm_and_hand")

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

#Test for Arm and Hand E only
    def test_hand_and_arm_sim(self):
        hand_arm_joints_goal = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                                'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                                'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                                'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                                'rh_LFJ5': 0.0, 'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0,
                                'rh_THJ5': 0.0, 'rh_WRJ1': 0.6, 'rh_WRJ2': 0.0,
                                'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 0.00,
                                'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                                'ra_shoulder_lift_joint': -0.58, 'ra_wrist_3_joint': 0.00,
                                'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}

        self.robot_commander.move_to_joint_value_target_unsafe(hand_arm_joints_goal, 6.0, True)
        rospy.sleep(10)
        final_joint_values = self.robot_commander.get_current_state()
        expected_and_final_joint_value_diff = self.joints_error_check(hand_arm_joints_goal, final_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.2)

if __name__ == "__main__":
    rostest.rosrun(PKG, "test_handAndArmSim", TestHandAndArmSim)
