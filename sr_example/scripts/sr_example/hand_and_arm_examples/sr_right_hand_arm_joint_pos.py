#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2020, 2022-2023 belongs to Shadow Robot Company Ltd.
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

# This example demonstrates how to move the right hand and arm through a sequence of joint goals.
# At the start and end of the sequence, both the hand and arm will spend 20s in teach mode,
# This allows the user to manually move the hand and arm. New goals can be easily generated
# using the script 'sr_right_print_joints_pos.py
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
#               external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true scene:=true

# It is recommended to run this script in simulation first.

from builtins import input
import sys
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander

rospy.init_node("right_hand_arm_joint_pos", anonymous=True)

# The constructors for the SrArmCommander, SrHandCommander and SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
# How to command the hand or arm separately
hand_commander = SrHandCommander(name="right_hand")
arm_commander = SrArmCommander(name="right_arm")
# How to command the arm and hand together
robot_commander = SrRobotCommander(name="right_arm_and_hand")
arm_commander.set_max_velocity_scaling_factor(0.1)

# Specify goals for hand and arm if not a saved state
arm_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                        'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                        'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': -3.1416}

arm_hand_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                             'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                             'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': -3.1416,
                             'rh_THJ1': 0.52, 'rh_THJ2': 0.61, 'rh_THJ3': 0.0, 'rh_THJ4': 1.20,
                             'rh_THJ5': 0.17, 'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707,
                             'rh_FFJ4': 0.0, 'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707,
                             'rh_MFJ4': 0.0, 'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707,
                             'rh_RFJ4': 0.0, 'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707,
                             'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

hand_arm_joints_goal_1 = {'rh_FFJ1': 1.57, 'rh_FFJ2': 1.57, 'rh_FFJ3': 1.574, 'rh_FFJ4': -0.00,
                          'rh_THJ4': -0.00, 'rh_THJ5': 6.49, 'rh_THJ1': -1.49, 'rh_THJ2': 4.72,
                          'rh_THJ3': 3.19, 'rh_LFJ2': 1.57, 'rh_LFJ3': 1.57, 'rh_LFJ1': 1.57,
                          'rh_LFJ4': 0.00, 'rh_LFJ5': -3.13, 'rh_RFJ4': -0.00, 'rh_RFJ1': 1.57,
                          'rh_RFJ2': 1.57, 'rh_RFJ3': 1.57, 'rh_MFJ1': 1.57, 'rh_MFJ3': 1.57,
                          'rh_MFJ2': 1.57, 'rh_MFJ4': -0.00, 'rh_WRJ2': -1.29, 'rh_WRJ1': 0.00,
                          'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                          'ra_shoulder_lift_joint': -1.57, 'ra_wrist_3_joint': 0.00}

hand_arm_joints_goal_2 = {'rh_FFJ1': -0.00, 'rh_FFJ2': 1.31, 'rh_FFJ3': 1.36, 'rh_FFJ4': -0.00,
                          'rh_MFJ1': 0.42, 'rh_MFJ2': 1.57, 'rh_MFJ3': 1.21, 'rh_MFJ4': 0.0,
                          'rh_RFJ1': -0.00, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.41, 'rh_RFJ4': -0.00,
                          'rh_LFJ1': 1.57, 'rh_LFJ2': 0.84, 'rh_LFJ3': 0.36, 'rh_LFJ4': 0.22,
                          'rh_LFJ5': 0.19, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
                          'rh_THJ4': 0.0, 'rh_THJ5': 0.0, 'rh_WRJ1': 0.6, 'rh_WRJ2': 0.0,
                          'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.10,
                          'ra_shoulder_lift_joint': -1.4, 'ra_wrist_1_joint': -0.733,
                          'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}

# Evaluate the plan quality from starting position of robot.
# https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py#L263-L310
# This is to confirm that the arm is in a safe place to move to the home joints goal,
# if the outcome is poor please refer to the readthedocs of where the start arm home position is.
arm_to_home_plan = arm_commander.plan_to_joint_value_target(arm_home_joints_goal)
arm_to_home_plan_quality = arm_commander.evaluate_given_plan(arm_to_home_plan)
eval_arm_home_plan_quality = arm_commander.evaluate_plan_quality(arm_to_home_plan_quality)

if eval_arm_home_plan_quality != 'good':
    rospy.logfatal("Plan quality to the home position is poor! " +
                   "For safety please refer to the hand and arm documentation " +
                   "for where to start the arm " +
                   "to ensure no unexpected movements during plan and execute.")
    sys.exit("Exiting script to allow for the arm to be manually moved to better start position ...")

# Execute arm to home plan
rospy.loginfo("Planning and moving arm to home joint states\n" + str(arm_home_joints_goal) + "\n")
arm_commander.execute_plan(arm_to_home_plan)
rospy.sleep(2.0)

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.
# https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py#L723

# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")
rospy.sleep(2.0)

# Move arm and hand together
input("Press Enter to continue...")
rospy.loginfo("Moving hand to joint states\n" + str(hand_arm_joints_goal_1) + "\n")
robot_commander.move_to_joint_value_target_unsafe(hand_arm_joints_goal_1, 6.0, True)
rospy.sleep(2.0)

# Move arm and hand together
input("Press Enter to continue...")
rospy.loginfo("Moving hand and arm to joint states\n" + str(hand_arm_joints_goal_2) + "\n")
robot_commander.move_to_joint_value_target_unsafe(hand_arm_joints_goal_2, 6.0, True)
rospy.sleep(2.0)

# Finish arm at home and hand at pack
input("Press Enter to continue...")
rospy.loginfo("Moving arm and hand to joint states\n" + str(arm_hand_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_hand_home_joints_goal, 6.0, True)
rospy.sleep(2.0)
