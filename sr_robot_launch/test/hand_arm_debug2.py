#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.
import rospy
import rostest
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

PKG = "sr_robot_launch"

rospy.init_node("hand_and_arm_test", anonymous=True)
hand_type = ()
launch_file = ()
arm_id = ()
hand_id = ()
robot_commander = ()
hand_commander = ()
arm_commander = ()

#DOES NOT WORK FOR LEFT. CAN'T FIND MOVEIT GROUP

class TestHandAndArmSim():
    """
    Tests the Hand and Arm in Sim
    """
    def __init__(self):
#        self.launch_file = 'sr_left_ur10arm_hand.launch'
#        self.launch_file = 'sr_left_ur5arm_hand.launch'
        self.launch_file = 'sr_right_ur10arm_hand.launch'
#        self.launch_file = 'sr_right_ur5arm_hand.launch'
        if 'ur10' in self.launch_file:
            self.hand_type = 'hand_e'
        elif 'ur5' in self.launch_file:
            self.hand_type = 'hand_lite'
        if 'right' in self.launch_file:
            self.hand_id = 'rh'
            self.arm_id = 'ra'
        elif 'left' in self.launch_file:
            self.hand_id = 'lh'
            self.arm_id = 'la'
        print('hand_type')
        print(self.hand_type)
        print('hand_id')
        print(self.hand_id)
        print('arm_id')
        print(self.arm_id)

        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        if self.hand_id == 'rh':
#            self.robot_commander = SrRobotCommander(name="right_arm_and_hand")
            self.hand_commander = SrHandCommander(name='right_hand')
            self.arm_commander = SrArmCommander(name='right_arm')
        elif self.hand_id == 'lh':
#            self.robot_commander = SrRobotCommander(name="left_arm_and_hand")
            self.hand_commander = SrHandCommander(name='left_hand')
            self.arm_commander = SrArmCommander(name='left_arm')

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])

    def find_joint_targets(self):
        hand_joints_target = {
           'hand_e': {'FFJ1': 0.35, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                      'MFJ1': 0.35, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                      'RFJ1': 0.35, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                      'LFJ1': 0.35, 'LFJ2': 1.5707, 'LFJ3': 1.5707, 'LFJ4': 0.0,
                      'LFJ5': 0.0, 'THJ1': 0.35, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0,
                      'THJ5': 0.0, 'WRJ1': 0.6, 'WRJ2': 0.0},
           'hand_lite': {'FFJ1': 0.35, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                         'MFJ1': 0.35, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                         'RFJ1': 0.35, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                         'THJ1': 0.35, 'THJ2': 0.0, 'THJ3': 0.0, 'THJ4': 0.0,
                         'THJ5': 0.0, 'WRJ1': 0.6, 'WRJ2': 0.0}
                }

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target[self.hand_id + '_' + key] = value

        print('hand joints target')
        print(hand_joints_target)

        arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
                                  'shoulder_pan_joint': 0.00, 'elbow_joint': 2.00,
                                  'shoulder_lift_joint': -0.58, 'wrist_3_joint': 0.00,
                                  'shoulder_lift_joint': -1.25, 'wrist_1_joint': -0.733,
                                  'wrist_2_joint': 1.5708, 'wrist_3_joint': 0.00}

        arm_joints_target_no_id = arm_joints_target
        arm_joints_target = {}
        for key, value in arm_joints_target_no_id.items():
            arm_joints_target[self.arm_id + '_' + key] = value

        print('arm joints target')
        print(arm_joints_target)

        self.hand_commander.move_to_joint_value_target(hand_joints_target, wait=True)
        rospy.sleep(5)
        final_hand_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff = self.joints_error_check(hand_joints_target, final_hand_joint_values)

        self.arm_commander.move_to_pose_target(arm_joints_target, wait=True)
        rospy.sleep(5)

        
        
#    def test_test_things



if __name__ == '__main__':
#initialising and setting everything up
    TestHandAndArmSim()
    rospy.sleep(10)
    TestHandAndArmSim().find_joint_targets()
#run test
#    TestHandAndArmSim().test_heartbeat_true()