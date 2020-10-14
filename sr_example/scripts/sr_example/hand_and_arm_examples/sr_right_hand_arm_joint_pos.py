#!/usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
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

# This example demonstrates how to move the right hand and arm through a sequence of joint goals.
# At the start and end of the sequence, both the hand and arm will spend 20s in teach mode,
# This allows the user to manually move the hand and arm. New goals can be easily generated
# using the script 'sr_print_joints_position.py
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!
# For more information, please see https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot:
#     roslaunch sr_right_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_right_ur10arm_hand.launch sim:=true scene:=true

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
                     'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}

hand_joints_goal_1 = {'rh_FFJ1': 1.16, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 0.99, 'rh_FFJ4': 0.0,
                      'rh_MFJ1': 0.35, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                      'rh_RFJ1': 0.35, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                      'rh_LFJ1': 0.35, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
                      'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 
                      'rh_THJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

hand_joints_goal_2 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                      'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                      'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                      'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                      'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 
                      'rh_THJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

arm_joints_goal_1 = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                     'ra_shoulder_lift_joint': -1.57, 'ra_wrist_3_joint': 0.00}

arm_joints_goal_2 = {'ra_shoulder_pan_joint': -0.05, 'ra_elbow_joint': -91.9, 
                     'ra_wrist_1_joint': -53.4, 'ra_shoulder_lift_joint': 8.3, 
                     'ra_wrist_3_joint': 125.8, 'ra_wrist_2_joint': -0.000}

hand_arm_joints_goal_1 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                          'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                          'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                          'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                          'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 
                          'rh_THJ5': 0.0, 'rh_WRJ1': 0.6, 'rh_WRJ2': 0.0, 
                          'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 0.00, 
                          'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                          'ra_shoulder_lift_joint': -0.58, 'ra_wrist_3_joint': 0.00, 
                          'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733, 
                          'ra_wrist_2_joint': 1.5708,'ra_wrist_3_joint': 0.00}

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.
# move_to_joint_value_target_unsafe(joint_states, time, wait, angle_degrees)
# Set target of the robot's links and moves to it.
#         @param joint_states - dictionary with joint name and value. It can
#         contain only joints values of which need to be changed.
#         @param time - time in s (counting from now) for the robot to reach the
#         target (it needs to be greater than 0.0 for it not to be rejected by
#         the trajectory controller).
#         @param wait - should method wait for movement end or not.
#         @param angle_degrees - are joint_states in degrees or not.
#         """


# Start arm at home
joint_goals = arm_home_joints_goal
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 10.0, True)

# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")

rospy.sleep(2.0)

# Move hand to pack
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: pack")
hand_commander.move_to_named_target("pack")

# Move arm
joint_goals = arm_joints_goal_1
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True)

raw_input("Press Enter to continue...")

# Move hand to open
joint_goals = hand_joints_goal_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True, True)

raw_input("Press Enter to continue...")

# Move hand to first finger closed
joint_goals = hand_joints_goal_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True, False)

raw_input("Press Enter to continue...")

# Move arm
joint_goals = arm_joints_goal_2
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True, True)

raw_input("Press Enter to continue...")

# Move hand to fingers_pack_thumb_open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: fingers pack, thumb open")
hand_commander.move_to_named_target("fingers_pack_thumb_open")

rospy.sleep(2.0)

raw_input("Press Enter to continue...")

# Move arm and hand together back to home 
joint_goals = hand_arm_joints_goal_1
rospy.loginfo("Moving hand and arm to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True)

rospy.sleep(2.0)

raw_input("Press Enter to continue...")

# Finish arm at home
joint_goals = arm_home_joints_goal
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
robot_commander.move_to_joint_value_target_unsafe(joint_goals, 6.0, True)

# Move hand to open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")

rospy.sleep(rospy.Duration(3))