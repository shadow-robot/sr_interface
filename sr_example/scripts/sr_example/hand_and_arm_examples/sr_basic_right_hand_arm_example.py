#!/usr/bin/env python

# This example demonstrates how to move the right hand and arm through a sequence of joint goals.
# At the start and end of the sequence, both the hand and arm will spend 20s in teach mode,
# This allows the user to manually move the hand and arm. New goals can be easily generated
# using the script 'sr_print_joints_position.py
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_hand_arm_example", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

# sleep for some time during which the arm can be moved around by pushing it
# but be careful to get away before the time runs out. You are warned
rospy.loginfo("Set arm teach mode ON")
arm_commander.set_teach_mode(True)
rospy.sleep(20.0)

rospy.loginfo("Set arm teach mode OFF")
arm_commander.set_teach_mode(False)

# Specify goals for hand and arm
hand_joints_goal_1 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
                      'rh_MFJ1': 0.35, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                      'rh_RFJ1': 0.35, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                      'rh_LFJ1': 0.35, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
                      'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

hand_joints_goal_2 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                      'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                      'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                      'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                      'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

hand_joints_goal_3 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_MFJ3': 0.0}

arm_joints_goal_1 = {'ra_shoulder_pan_joint': 0.43, 'ra_elbow_joint': 2.12,
                     'ra_wrist_1_joint': -1.71, 'ra_wrist_2_joint': 1.48,
                     'ra_shoulder_lift_joint': -2.58, 'ra_wrist_3_joint': 1.62,
                     'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

arm_joints_goal_2 = {'ra_shoulder_pan_joint': 0.42, 'ra_elbow_joint': 1.97,
                     'ra_wrist_1_joint': -0.89, 'ra_wrist_2_joint': -0.92,
                     'ra_shoulder_lift_joint': -1.93, 'ra_wrist_3_joint': 0.71,
                     'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

# Move through each goal
# joint states are sent to the commanders, with a time for execution and a flag as to whether
# or not the commander should wait for the command to complete before moving to the next command.

# Move hand
joint_goals = hand_joints_goal_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 1.0, False)

# Move arm
joint_goals = arm_joints_goal_1
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)


# Move hand
joint_goals = hand_joints_goal_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)


# Move hand
joint_goals = hand_joints_goal_3
rospy.loginfo("Moving hand to joint states\n" + str(joint_goals) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, False)

# Move arm
joint_goals = arm_joints_goal_2
rospy.loginfo("Moving arm to joint states\n" + str(joint_goals) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_goals, 3.0, True)


hand_commander.set_teach_mode(True)
# sleep for some time during which the hand joints can be moved manually
rospy.sleep(20.0)
rospy.loginfo("Set hand teach mode OFF")
hand_commander.set_teach_mode(False)
