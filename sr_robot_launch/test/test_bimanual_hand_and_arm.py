#!/usr/bin/env python

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

from __future__ import absolute_import
import rospy
import rostest
from moveit_msgs.msg import PlanningScene
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from rospy import get_rostime
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase


class TestBiHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim
    """

    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        cls.scene = rospy.get_param('~test_hand_and_arm_sim/scene')
        cls.robot_commander = SrRobotCommander(name="two_arms_and_hands")
        cls.hand_commander = SrHandCommander(name='two_hands')
        cls.arm_commander = SrArmCommander(name='two_arms', set_ground=False)

        rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, cls.scene_data_cb)

        rospy.sleep(10)

    @classmethod
    def tearDownClass(cls):
        pass

    @classmethod
    def scene_data_cb(cls, result):
        scene_data = ()
        cls.scene_data = result.world.collision_objects

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

    def wait_for_topic_with_scene(self, timeout=50):
        counter = 0
        while counter < timeout:
            current_value = self.scene_data
            if len(current_value) != 0:
                return current_value
            rospy.sleep(0.1)
            counter += 1

    def test_1_home_position(self):
        start_arm_angles = self.arm_commander.get_current_state()
        self.expected_home_angles = {'la_shoulder_pan_joint': 0.0, 'la_elbow_joint': -2.0,
                                     'la_shoulder_lift_joint': -1.89, 'la_wrist_1_joint': -2.1,
                                     'la_wrist_2_joint': -1.5708, 'la_wrist_3_joint': 2,
                                     'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.0,
                                     'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -1,
                                     'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': -2}

        expected_and_actual_home_angles = self.joints_error_check(self.expected_home_angles, start_arm_angles)
        self.assertAlmostEqual(expected_and_actual_home_angles, 0, delta=0.2)

    def test_2_scene(self):
        scene = ()
        self.scene = rospy.get_param('~test_hand_and_arm_sim/scene')
        self.scene_value = self.wait_for_topic_with_scene()
        if self.scene is True:
            self.assertNotEqual(len(self.scene_value), 0)
        elif self.scene is False:
            self.assertTrue(self.scene_value is None)

    def test_3_arms(self):
        arm_joints_target = {'la_shoulder_pan_joint': 0.00, 'la_elbow_joint': -1.43,
                             'la_shoulder_lift_joint': -1.82, 'la_wrist_1_joint': 3.24,
                             'la_wrist_2_joint': -1.57, 'la_wrist_3_joint': 3.13,
                             'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 1.43,
                             'ra_shoulder_lift_joint': -1.27, 'ra_wrist_1_joint': -0.1,
                             'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 3.13}

        self.arm_commander.move_to_joint_value_target(arm_joints_target, wait=True)
        rospy.sleep(5)
        final_arm_joint_values = self.arm_commander.get_current_state()

        expected_and_final_joint_value_diff_arm = self.joints_error_check(arm_joints_target, final_arm_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff_arm, 0, delta=0.2)

    def test_4_hands(self):
        hand_joints_target = {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                              'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                              'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                              'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                              'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                              'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0}

        hand_joints_target_no_id = hand_joints_target

        hand_joints_target_left = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target_left['lh' + '_' + key] = value

        hand_joints_target_right = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target_right['rh' + '_' + key] = value

        hands_joints_target = dict()
        hands_joints_target.update(hand_joints_target_right)
        hands_joints_target.update(hand_joints_target_left)
        self.hand_commander.move_to_joint_value_target(hands_joints_target, wait=True)
        rospy.sleep(5)
        final_hand_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff_hand = self.joints_error_check(hands_joints_target, final_hand_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff_hand, 0, delta=1)

    def test_5_arms_and_hands(self):
        hand_joints_target = {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                              'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                              'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                              'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
                              'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                              'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0}

        arm_joints_target_right = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.0,
                                   'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                   'ra_wrist_2_joint': 1.578, 'ra_wrist_3_joint': -3.1416}

        arm_joints_target_left = {'la_shoulder_pan_joint': 0.0, 'la_elbow_joint': -2.0,
                                  'la_shoulder_lift_joint': -1.89, 'la_wrist_1_joint': 3.8,
                                  'la_wrist_2_joint': -1.5708, 'la_wrist_3_joint': 3.1416}

        hand_joints_target_no_id = hand_joints_target

        hand_joints_target_left = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target_left['lh' + '_' + key] = value

        hand_joints_target_right = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target_right['rh' + '_' + key] = value

        hands_and_arms_joints_target = dict()
        hands_and_arms_joints_target.update(hand_joints_target_right)
        hands_and_arms_joints_target.update(arm_joints_target_right)
        hands_and_arms_joints_target.update(hand_joints_target_left)
        hands_and_arms_joints_target.update(arm_joints_target_left)

        self.robot_commander.move_to_joint_value_target_unsafe(hands_and_arms_joints_target, 10.0, True)

        rospy.sleep(5)

        final_hand_and_arm_joint_values = self.robot_commander.get_current_state()
        joint_value_diff_arm_and_hand = self.joints_error_check(hands_and_arms_joints_target,
                                                                final_hand_and_arm_joint_values)

        self.assertAlmostEqual(joint_value_diff_arm_and_hand, 0, delta=0.4)


if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_bimanual_hand_and_arm'

    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestBiHandAndArmSim)
