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
from moveit_msgs.msg import PlanningScene
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from rospy import get_rostime
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

class TestHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim 
    """
    @classmethod
    def setUpClass(cls):
        cls.launch_file = rospy.get_param('~test_sim/launch_file', 'sr_right_ur10arm_hand.launch')
        if 'ur10' in cls.launch_file:
            cls.hand_type = 'hand_e'
        elif 'ur5' in cls.launch_file:
            cls.hand_type = 'hand_lite'
        if 'right' in cls.launch_file:
            cls.hand_id = 'rh'
            cls.arm_id = 'ra'
        elif 'left' in cls.launch_file:
            cls.hand_id = 'lh'
            cls.arm_id = 'la'

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if cls.hand_id == 'rh':
            cls.robot_commander = SrRobotCommander(name="right_arm_and_hand")
            cls.hand_commander = SrHandCommander(name='right_hand')
            cls.arm_commander = SrArmCommander(name='right_arm', set_ground=False)
        elif cls.hand_id == 'lh':
            cls.robot_commander = SrRobotCommander(name="left_arm_and_hand")
            cls.hand_commander = SrHandCommander(name='left_hand')
            cls.arm_commander = SrArmCommander(name='left_arm')
        cls.scene_data = None
        rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, cls.scene_data_cb)

        rospy.sleep(10)  

    @classmethod
    def tearDownClass(cls):
        pass

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

    # def scene_data_cb(self, result):
    #     cls.scene_data = result.world.collision_objects

    # def wait_for_topic_with_scene(self, timeout=80):
    #     counter = 0
    #     while counter < timeout:
    #         current_value = self.scene_data
    #         if len(current_value) != 0:
    #             return current_value
    #         else:
    #             current_value == 0
    #             return current_value
    #         rospy.sleep(0.1)
    #         counter += 1

    # def test_home_position(self):
    #     self.start_home = rospy.get_param('~test_sim/start_home', 'false')
    #     start_arm_angles = self.arm_commander.get_current_state()

    #     if self.arm_id == 'ra':
    #         self.expected_home_angles = {'shoulder_pan_joint': 1.147, 'elbow_joint': 1.695,
    #                                          'shoulder_lift_joint': -1.926, 'wrist_3_joint': 0.00,
    #                                          'wrist_1_joint': -1.395, 'wrist_2_joint': -1.584, 
    #                                          'wrist_3_joint': 1.830}
    #     elif self.arm_id == 'la':
    #         self.expected_home_angles = {'shoulder_pan_joint': 1.147, 'elbow_joint': -1.695,
    #                                          'shoulder_lift_joint': -1.22, 'wrist_3_joint': 0.00,
    #                                          'wrist_1_joint': -1.75, 'wrist_2_joint': 1.57, 
    #                                          'wrist_3_joint': -1.830}

    #     if self.start_home == False:
    #         self.expected_home_angles = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
    #                                          'shoulder_lift_joint': 0.00, 'wrist_3_joint': 0.00,
    #                                          'wrist_1_joint': 0.00, 'wrist_2_joint': 0.00, 
    #                                          'wrist_3_joint': 0.00}
    #     elif self.start_home == True:
    #         self.expected_home_angles = self.expected_home_angles

    #     home_angles_no_id = self.expected_home_angles
    #     expected_start_angles = {}
    #     for key, value in home_angles_no_id.items():
    #         expected_start_angles[self.arm_id + '_' + key] = value

    #     expected_and_actual_home_angles = self.joints_error_check(expected_start_angles, start_arm_angles)

    #     self.assertAlmostEqual(expected_and_actual_home_angles, 0, delta=0.2)

    # def test_scene(self):
    #     scene = ()
    #     self.scene = rospy.get_param('~test_sim/scene', 'false')
    #     self.scene_value = self.wait_for_topic_with_scene()
    #     if self.scene == True:
    #         self.assertNotEqual(len(self.scene_value), 0)
    #     #using assertAlmostEqual because assertEqual tries to call an addTpeEqualityFunc() here
    #     elif self.scene == False:
    #         self.assertAlmostEqual(len(self.scene_value), 0, 0)

    def test_hand(self):
        hand_joints_target = {
           'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                            'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                            'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                            'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                            'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                            'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
           'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                          'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                          'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                          'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0},
                }

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target[self.hand_id + '_' + key] = value

        self.hand_commander.move_to_joint_value_target(hand_joints_target, wait=True)
        rospy.sleep(5)
        final_hand_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff_hand = self.joints_error_check(hand_joints_target, final_hand_joint_values)

        self.assertAlmostEqual(expected_and_final_joint_value_diff_hand, 0, delta=0.2)

    # def test_arm(self):
    #     arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
    #                               'shoulder_pan_joint': 0.00, 'elbow_joint': 2.00,
    #                               'shoulder_lift_joint': -0.58, 'wrist_3_joint': 0.00,
    #                               'shoulder_lift_joint': -1.25, 'wrist_1_joint': -0.733,
    #                               'wrist_2_joint': 1.5708, 'wrist_3_joint': 0.00}

    #     arm_joints_target_no_id = arm_joints_target
    #     arm_joints_target = {}
    #     for key, value in arm_joints_target_no_id.items():
    #         arm_joints_target[self.arm_id + '_' + key] = value

    #     self.arm_commander.move_to_joint_value_target(arm_joints_target, wait=True)
    #     rospy.sleep(5)
    #     final_arm_joint_values = self.arm_commander.get_current_state()

    #     expected_and_final_joint_value_diff_arm = self.joints_error_check(arm_joints_target, final_arm_joint_values)

    #     self.assertAlmostEqual(expected_and_final_joint_value_diff_arm, 0, delta=0.2)

    # def test_hand_and_arm(self):
    #     hand_joints_target = {
    #         'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
    #                          'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
    #                          'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
    #                          'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
    #                          'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
    #                          'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
    #         'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
    #                        'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
    #                        'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
    #                        'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0},
    #              }

    #     hand_joints_target_no_id = hand_joints_target[self.hand_type]
    #     hand_joints_target = {}
    #     for key, value in hand_joints_target_no_id.items():
    #          hand_joints_target[self.hand_id + '_' + key] = value

    #     arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
    #                                'shoulder_pan_joint': 0.00, 'elbow_joint': 0.0,
    #                                'shoulder_lift_joint': -0.58, 'wrist_3_joint': 0.00,
    #                                'shoulder_lift_joint': -1.25, 'wrist_1_joint': -0.733,
    #                                'wrist_2_joint': 1.5708, 'wrist_3_joint': 0.00}

    #     arm_joints_target_no_id = arm_joints_target
    #     arm_joints_target = {}
    #     for key, value in arm_joints_target_no_id.items():
    #          arm_joints_target[self.arm_id + '_' + key] = value
        
    #     hand_and_arm_joints_target = dict(hand_joints_target.items() + arm_joints_target.items())

    #     self.robot_commander.move_to_joint_value_target_unsafe(hand_and_arm_joints_target, 6.0, True)

    #     final_hand_and_arm_joint_values = self.robot_commander.get_current_state()

    #     joint_value_diff_arm_and_hand = self.joints_error_check(hand_and_arm_joints_target, final_hand_and_arm_joint_values)

    #     self.assertAlmostEqual(joint_value_diff_arm_and_hand, 0, delta=0.4)


if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_hand_and_arm_sim'

    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestHandAndArmSim)