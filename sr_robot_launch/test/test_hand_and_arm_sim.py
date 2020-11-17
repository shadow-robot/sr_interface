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
    @classmethod
    def setUpClass(cls):
        cls.launch_file = rospy.get_param('~test_sim/launch_file')
        if 'ur5' in cls.launch_file or 'ur5e' in cls.launch_file:
            cls.hand_type = 'hand_lite'
        elif 'ur10' in cls.launch_file or 'ur10e' in cls.launch_file:
            cls.hand_type = 'hand_e' 
        if 'right' in cls.launch_file:
            cls.robot_side = 'right'
            cls.hand_id = 'rh'
            cls.arm_id = 'ra'
        elif 'left' in cls.launch_file:
            cls.robot_side = 'left'
            cls.hand_id = 'lh'
            cls.arm_id = 'la'

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if cls.robot_side == 'right':
            self.robot_commander = SrRobotCommander(name="right_arm_and_hand")
        elif cls.robot_side == 'left':
            cls.robot_commander = SrRobotCommander(name="left_arm_and_hand")

    @classmethod
    def tearDownClass(cls):
        pass

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

    def test_hand_and_arm_sim(self):
        hand_joints_target = {
        'hand_e' = {'FFJ1': 0.35, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                    'MFJ1': 0.35, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                    'RFJ1': 0.35, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                    'LFJ1': 0.35, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                    'LFJ5': 0.0, 'THJ1': 0.35, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0,
                    'THJ5': 0.0, 'WRJ1': 0.6, 'WRJ2': 0.0}
        'hand_lite' = {'FFJ1': 0.35, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                       'MFJ1': 0.35, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                       'RFJ1': 0.35, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                       'THJ1': 0.35, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0,
                       'THJ5': 0.0, 'WRJ1': 0.6, 'WRJ2': 0.0}}

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in pack_joints_target_no_id.items():
            hand_joints_target[self.hand_id + '_' + key] = value                                  

        arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
                                  'shoulder_pan_joint': 0.00, 'elbow_joint': 2.00,
                                  'shoulder_lift_joint': -0.58, 'wrist_3_joint': 0.00,
                                  'shoulder_lift_joint': -1.25, 'wrist_1_joint': -0.733,
                                  'wrist_2_joint': 1.5708, 'wrist_3_joint': 0.00}

        arm_joints_target_no_id = arm_joints_target
        arm_joints_target = {}
        for key, value in pack_joints_target_no_id.items():
            arm_joints_target[self.arm_id + '_' + key] = value
        
        arm_and_hand_joints_target = {}
        arm_and_hand_joints_target = dict(hand_joints_target.items() + arm_joints_target.items())

        self.robot_commander.move_to_joint_value_target_unsafe(arm_and_hand_joints_target, 6.0, True)
        rospy.sleep(10)
        final_joint_values = self.robot_commander.get_current_state()
        expected_and_final_joint_value_diff = self.joints_error_check(arm_and_hand_joints_target, final_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, delta=0.2)

if __name__ == "__main__":
    rostest.rosrun(PKG, "test_handAndArmSim", TestHandAndArmSim)
