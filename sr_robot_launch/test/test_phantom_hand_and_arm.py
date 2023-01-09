#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
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
from multiprocessing import Process
import yaml
import rostest
import rospy
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander


class TestHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim
    """
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        cls.hand_type = 'hand_e'
        cls.robot_commander = SrRobotCommander(name="two_arms_and_hands")
        cls.rh_commander = SrHandCommander(name='right_hand')
        cls.lh_commander = SrHandCommander(name='left_hand')
        cls.hand_commander = SrHandCommander(name='two_hands')
        cls.arm_commander = SrArmCommander(name='two_arms', set_ground=False)

        cls.arm_commander.set_max_velocity_scaling_factor(0.2)
        cls.robot_commander.set_max_velocity_scaling_factor(0.2)

        rospy.sleep(3)

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

    @staticmethod
    def open_yaml(path):
        with open(path, encoding="utf-8") as hand_config_file:
            hands_config = yaml.load(hand_config_file, Loader=yaml.FullLoader)
        return hands_config

    @staticmethod
    def check_topic_prefix(prefix):
        joint_state = rospy.wait_for_message('/joint_states', JointState)
        joints_exist = False
        for joint in joint_state.name:  # pylint: disable=E1101
            if joint.startswith(prefix):
                joints_exist = True
        return joints_exist

    def test_1_autodetection_right_hand(self):
        # Needs to be changed for the right phantom hand
        hand_to_confirm = {113: 'enx000ec6bdeafe'}
        hands_detected = self.open_yaml('/tmp/sr_hand_detector.yaml')

        hand_in_config = False
        for hand_serial, hand_eth in hands_detected.items():
            if hand_serial in hand_to_confirm and hand_to_confirm[hand_serial] == hand_eth:
                hand_in_config = True
        self.assertTrue(hand_in_config)

    def test_2_autodetection_left_hand(self):
        # Needs to be changed for the left phantom hand
        hand_to_confirm = {114: 'enx000ec6c2911c'}
        hands_detected = self.open_yaml('/tmp/sr_hand_detector.yaml')

        hand_in_config = False
        for hand_serial, hand_eth in hands_detected.items():
            if hand_serial in hand_to_confirm and hand_to_confirm[hand_serial] == hand_eth:
                hand_in_config = True
        self.assertTrue(hand_in_config)

    def test_3_joint_state_topic(self):
        joint_state = rospy.wait_for_message('/joint_states', JointState)
        joints = len(joint_state.name)  # pylint: disable=E1101
        self.assertEqual(joints, 60)

    def test_4_joint_state_prefix_right_hand(self):
        joint_state_exists = self.check_topic_prefix('rh_')
        self.assertTrue(joint_state_exists)

    def test_5_joint_state_prefix_left_hand(self):
        joint_state_exists = self.check_topic_prefix('lh_')
        self.assertTrue(joint_state_exists)

    def test_6_joint_state_prefix_right_arm(self):
        joint_state_exists = self.check_topic_prefix('ra_')
        self.assertTrue(joint_state_exists)

    def test_7_joint_state_prefix_left_arm(self):
        joint_state_exists = self.check_topic_prefix('la_')
        self.assertTrue(joint_state_exists)

    def test_8_right_hand(self):
        self.rh_commander.move_to_named_target('pack', wait=True)
        rospy.sleep(2)
        goal_status = rospy.wait_for_message('/move_group/status', GoalStatusArray)
        goal_status = goal_status.status_list[0].status  # pylint: disable=E1101
        self.assertEqual(goal_status, 3)

    def test_9_left_hand(self):
        self.lh_commander.move_to_named_target('pack', wait=True)
        rospy.sleep(2)
        goal_status = rospy.wait_for_message('/move_group/status', GoalStatusArray)
        goal_status = goal_status.status_list[0].status  # pylint: disable=E1101
        self.assertEqual(goal_status, 3)

    def test_10_joint_states(self):
        joint_state = rospy.wait_for_message('/joint_states', JointState)
        joints = len(joint_state.name)  # pylint: disable=E1101
        self.assertEqual(joints, 60)

    def test_11_arms(self):

        la_arm_joints_target = {'la_shoulder_pan_joint': 0.00, 'la_elbow_joint': -1.43,
                                'la_shoulder_lift_joint': -1.82, 'la_wrist_1_joint': 3.24,
                                'la_wrist_2_joint': -1.57, 'la_wrist_3_joint': 3.13}
        ra_arm_joints_target = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 1.43,
                                'ra_shoulder_lift_joint': -1.27, 'ra_wrist_1_joint': -0.1,
                                'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 3.13}
        arm_joints_target = {}
        arm_joints_target.update(ra_arm_joints_target)
        arm_joints_target.update(la_arm_joints_target)
        service_call = rospy.ServiceProxy("/la_sr_ur_robot_hw/resend_robot_program", Trigger)
        service_call()
        service_call = rospy.ServiceProxy("/ra_sr_ur_robot_hw/resend_robot_program", Trigger)
        service_call()
        rospy.sleep(2)
        self.arm_commander.move_to_joint_value_target(arm_joints_target, wait=True)
        rospy.sleep(5)
        final_arm_joint_values = self.arm_commander.get_current_state()

        expected_and_final_joint_value_diff_arm = self.joints_error_check(arm_joints_target, final_arm_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff_arm, 0, delta=0.2)

    def test_12_hand_and_arm(self):
        hand_joints_target = self.hand_commander.get_current_state()

        ra_arm_joints_target = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.0,
                                'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                'ra_wrist_2_joint': 1.578, 'ra_wrist_3_joint': -3.1416}
        la_arm_joints_target = {'la_shoulder_pan_joint': 0.0, 'la_elbow_joint': -2.0,
                                'la_shoulder_lift_joint': -1.89, 'la_wrist_1_joint': 3.8,
                                'la_wrist_2_joint': -1.5708, 'la_wrist_3_joint': 3.1416}

        arm_joints_target = {}
        arm_joints_target.update(ra_arm_joints_target)
        arm_joints_target.update(la_arm_joints_target)
        hand_and_arm_joints_target = {}
        hand_and_arm_joints_target.update(hand_joints_target)
        hand_and_arm_joints_target.update(arm_joints_target)
        service_call = rospy.ServiceProxy("/la_sr_ur_robot_hw/resend_robot_program", Trigger)
        service_call()
        service_call = rospy.ServiceProxy("/ra_sr_ur_robot_hw/resend_robot_program", Trigger)
        service_call()
        rospy.sleep(2)
        self.robot_commander.move_to_joint_value_target_unsafe(hand_and_arm_joints_target, 10.0, True)

        rospy.sleep(5)

        final_hand_and_arm_joint_values = self.robot_commander.get_current_state()

        joint_value_diff_arm_and_hand = self.joints_error_check(hand_and_arm_joints_target,
                                                                final_hand_and_arm_joint_values)

        self.assertAlmostEqual(joint_value_diff_arm_and_hand, 0, delta=0.4)


def run_unit_tests():
    """This function is used to execute all the tests within this file.
        Its in a seperate func so a timeout can be added to the function call."""
    pkg_name = 'sr_robot_launch'
    node_name = 'test_phantom_hand_and_arm'
    rospy.init_node(node_name, anonymous=True)
    rostest.rosrun(pkg_name, node_name, TestHandAndArmSim)


if __name__ == "__main__":
    process = Process(target=run_unit_tests)
    process.start()
    # 15 min timeout.
    process.join(timeout=900)
    process.terminate()
