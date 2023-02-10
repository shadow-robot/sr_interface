#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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

# This example demonstrates how to move the left hand and arm through a sequence of joint goals that
# comprise a simple 'pick and place' demo.
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_left_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true start_home:=true
# simulated robot:
#     roslaunch sr_left_ur10arm_hand.launch sim:=true scene:=true start_home:=true

# PLEASE NOTE: No scene is used in this script and fast movements of the robot. Use on real robot at your own risk.
# It is recommended to run this script in simulation first.

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("basic_arm_examples", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")
arm_commander = SrArmCommander(name="left_arm", set_ground=False)

# Specify goals for hand and arm
hand_joint_goals_1 = {'lh_RFJ2': 1.59, 'lh_RFJ3': 1.49, 'lh_RFJ1': 1.47, 'lh_RFJ4': -0.01, 'lh_LFJ4': 0.02,
                      'lh_LFJ5': 0.061, 'lh_LFJ1': 1.41, 'lh_LFJ2': 1.60, 'lh_LFJ3': 1.49, 'lh_THJ2': 0.64,
                      'lh_THJ3': -0.088, 'lh_THJ1': 0.43, 'lh_THJ4': 0.49, 'lh_THJ5': 0.35, 'lh_FFJ4': -0.02,
                      'lh_FFJ2': 1.71, 'lh_FFJ3': 1.49, 'lh_FFJ1': 1.25, 'lh_MFJ3': 1.49, 'lh_MFJ2': 1.66,
                      'lh_MFJ1': 1.31, 'lh_MFJ4': -0.02}

hand_joint_goals_2 = {'lh_RFJ2': 0.55, 'lh_RFJ3': 0.08, 'lh_RFJ1': 0.03, 'lh_RFJ4': -0.15, 'lh_LFJ4': -0.35,
                      'lh_LFJ5': 0.23, 'lh_LFJ1': 0.02, 'lh_LFJ2': 0.49, 'lh_LFJ3': -0.02, 'lh_THJ2': -0.08,
                      'lh_THJ3': -0.08, 'lh_THJ1': 0.15, 'lh_THJ4': 0.56, 'lh_THJ5': -0.17, 'lh_FFJ4': -0.34,
                      'lh_FFJ2': 0.30, 'lh_FFJ3': 0.16, 'lh_FFJ1': 0.01, 'lh_MFJ3': 0.19, 'lh_MFJ2': 0.50,
                      'lh_MFJ1': 0.00, 'lh_MFJ4': -0.07}

hand_joint_goals_3 = {'lh_RFJ2': 0.63, 'lh_RFJ3': 0.77, 'lh_RFJ1': 0.033, 'lh_RFJ4': -0.02, 'lh_LFJ4': -0.32,
                      'lh_LFJ5': 0.67, 'lh_LFJ1': 0.02, 'lh_LFJ2': 0.73, 'lh_LFJ3': 0.21, 'lh_THJ2': -0.06,
                      'lh_THJ3': -0.04, 'lh_THJ1': 0.39, 'lh_THJ4': 0.85, 'lh_THJ5': 0.40, 'lh_FFJ4': -0.35,
                      'lh_FFJ2': 0.90, 'lh_FFJ3': 0.56, 'lh_FFJ1': 0.02, 'lh_MFJ3': 0.59, 'lh_MFJ2': 0.84,
                      'lh_MFJ1': 0.05, 'lh_MFJ4': -0.08}

hand_joint_goals_4 = {'lh_RFJ2': 0.57, 'lh_RFJ3': 0.27, 'lh_RFJ1': 0.04, 'lh_RFJ4': -0.01, 'lh_LFJ4': -0.28,
                      'lh_LFJ5': 0.39, 'lh_LFJ1': 0.01, 'lh_LFJ2': 0.72, 'lh_LFJ3': -0.12, 'lh_THJ2': -0.19,
                      'lh_THJ3': -0.05, 'lh_THJ1': 0.38, 'lh_THJ4': 0.85, 'lh_THJ5': -0.12, 'lh_FFJ4': -0.32,
                      'lh_FFJ2': 0.64, 'lh_FFJ3': -0.03, 'lh_FFJ1': 0.04, 'lh_MFJ3': 0.04, 'lh_MFJ2': 0.83,
                      'lh_MFJ1': 0.01, 'lh_MFJ4': -0.05}

hand_joint_goals_5 = {'lh_RFJ2': 1.58, 'lh_RFJ3': 1.52, 'lh_RFJ1': 1.34, 'lh_RFJ4': -0.06, 'lh_LFJ4': -0.20,
                      'lh_LFJ5': 0.09, 'lh_LFJ1': 1.47, 'lh_LFJ2': 1.57, 'lh_LFJ3': 1.40, 'lh_THJ2': -0.01,
                      'lh_THJ3': -0.041, 'lh_THJ1': 0.29, 'lh_THJ4': 0.59, 'lh_THJ5': -1.36, 'lh_FFJ4': 0.03,
                      'lh_FFJ2': 1.72, 'lh_FFJ3': 1.41, 'lh_FFJ1': 1.21, 'lh_MFJ3': 1.39, 'lh_MFJ2': 1.65,
                      'lh_MFJ1': 1.33, 'lh_MFJ4': 0.12}

arm_joint_goals_1 = {'la_shoulder_lift_joint': -1.87, 'la_elbow_joint': 1.76, 'la_wrist_2_joint': 0.03,
                     'la_wrist_1_joint': -0.86, 'la_shoulder_pan_joint': -2.64, 'la_wrist_3_joint': 0.69,
                     'lh_WRJ2': -0.02, 'lh_WRJ1': 0.03}

arm_joint_goals_2 = {'la_shoulder_lift_joint': -1.86, 'la_elbow_joint': 1.85, 'la_wrist_2_joint': -0.19,
                     'la_wrist_1_joint': -0.96, 'la_shoulder_pan_joint': -1.78, 'la_wrist_3_joint': 1.06,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': -0.02}

arm_joint_goals_3 = {'la_shoulder_lift_joint': -1.86, 'la_elbow_joint': 1.90, 'la_wrist_2_joint': -0.18,
                     'la_wrist_1_joint': -0.96, 'la_shoulder_pan_joint': -1.78, 'la_wrist_3_joint': 1.06,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': 0.15}

arm_joint_goals_4 = {'la_shoulder_lift_joint': -1.33, 'la_elbow_joint': 1.11, 'la_wrist_2_joint': 1.00,
                     'la_wrist_1_joint': 0.13, 'la_shoulder_pan_joint': -1.49, 'la_wrist_3_joint': 3.27,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': 0.15}

arm_joint_goals_5 = {'la_shoulder_lift_joint': -1.45, 'la_elbow_joint': 1.11, 'la_wrist_2_joint': 0.90,
                     'la_wrist_1_joint': 0.45, 'la_shoulder_pan_joint': -0.95, 'la_wrist_3_joint': 0.09,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': 0.16}

arm_joint_goals_6 = {'la_shoulder_lift_joint': -1.35, 'la_elbow_joint': 1.16, 'la_wrist_2_joint': 0.96,
                     'la_wrist_1_joint': 0.39, 'la_shoulder_pan_joint': -0.91, 'la_wrist_3_joint': 0.09,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': 0.15}

arm_joint_goals_7 = {'la_shoulder_lift_joint': -1.35, 'la_elbow_joint': 1.04, 'la_wrist_2_joint': 1.55,
                     'la_wrist_1_joint': 0.08, 'la_shoulder_pan_joint': -1.64, 'la_wrist_3_joint': -1.41,
                     'lh_WRJ2': -0.03, 'lh_WRJ1': 0.15}

arm_joint_goals_8 = {'la_shoulder_lift_joint': -1.55, 'la_elbow_joint': 1.41, 'la_wrist_2_joint': 0.02,
                     'la_wrist_1_joint': 0.61, 'la_shoulder_pan_joint': -1.55, 'la_wrist_3_joint': -0.57,
                     'lh_WRJ2': -0.04, 'lh_WRJ1': 0.16}

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.

# Move hand and arm
joint_goals = hand_joint_goals_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
joint_goals = arm_joint_goals_1
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

# Move hand and arm
joint_goals = hand_joint_goals_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
joint_goals = arm_joint_goals_2
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

# Move arm
joint_goals = arm_joint_goals_3
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, True)

# Move hand
joint_goals = hand_joint_goals_3
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

# Move arm
joint_goals = arm_joint_goals_6
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

# Move hand
joint_goals = hand_joint_goals_4
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

# Move arm
joint_goals = arm_joint_goals_5
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 2.0, True)

# Move arm and hand
joint_goals = arm_joint_goals_7
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)
joint_goals = hand_joint_goals_5
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)

# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
named_target = "la_home"
print("Moving arm to named target " + named_target)
arm_commander.move_to_named_target(named_target)

rospy.sleep(2.0)

# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")
