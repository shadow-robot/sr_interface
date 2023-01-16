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

# This example demonstrates some of the functions of the hand, arm and robot commander.
# The arm, hand and robot will be moved through a sequence of goals generated via different functions in the commander.
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
import tf
import geometry_msgs.msg
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("hand_arm_waypoints", anonymous=True)

# The constructors for the SrArmCommander, SrHandCommander and SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
# How to command the hand or arm separately
hand_commander = SrHandCommander(name="right_hand")
arm_commander = SrArmCommander(name="right_arm")
# How to command the arm and hand together
robot_commander = SrRobotCommander(name="right_arm_and_hand")
arm_commander.set_max_velocity_scaling_factor(0.1)

rospy.sleep(1.0)

arm_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                        'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                        'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': -3.1416}

hand_home_joints_goal = {'rh_THJ1': 0.52, 'rh_THJ2': 0.61, 'rh_THJ3': 0.0, 'rh_THJ4': 1.20,
                         'rh_THJ5': 0.17, 'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707,
                         'rh_FFJ4': 0.0, 'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707,
                         'rh_MFJ4': 0.0, 'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707,
                         'rh_RFJ4': 0.0, 'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707,
                         'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

arm_hand_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                             'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                             'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': -3.1416,
                             'rh_THJ1': 0.52, 'rh_THJ2': 0.61, 'rh_THJ3': 0.0, 'rh_THJ4': 1.20,
                             'rh_THJ5': 0.17, 'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707,
                             'rh_FFJ4': 0.0, 'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707,
                             'rh_MFJ4': 0.0, 'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707,
                             'rh_RFJ4': 0.0, 'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707,
                             'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

tf_listener = tf.TransformListener()
rospy.sleep(0.2)
pose_xyzw = []
try:
    (position, orientation) = tf_listener.lookupTransform('ra_base', 'ra_flange',
                                                          tf_listener.getLatestCommonTime('ra_base',
                                                                                          'ra_flange'))
except Exception as exception:
    raise ValueError(f"{exception}") #from exception

pose_msg = geometry_msgs.msg.PoseStamped()
pose_msg.header.frame_id = 'ra_base'
pose_msg.header.stamp = rospy.get_rostime()

pose_msg.pose.position.x = position[0]
pose_msg.pose.position.y = position[1] - 0.1
pose_msg.pose.position.z = position[2] + 0.1

pose_msg.pose.orientation.x = orientation[0]
pose_msg.pose.orientation.y = orientation[1]
pose_msg.pose.orientation.z = orientation[2]
pose_msg.pose.orientation.w = orientation[3]


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
rospy.loginfo("Moving hand to home joint states\n" + str(hand_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(hand_home_joints_goal, 4.0, True)
rospy.sleep(2.0)

# Plan & move arm to initial pose
# This also includes a plan quality evaluation.
input("Press Enter to continue...")
ik_solution = arm_commander.get_ik(pose_msg, avoid_collisions=True)
if ik_solution is not None:
    ik_plan = arm_commander.plan_to_joint_value_target(ik_solution)
    ik_plan_quality = arm_commander.evaluate_given_plan(ik_plan)
    eval_ik_plan_quality = arm_commander.evaluate_plan_quality(ik_plan_quality)
    if eval_ik_plan_quality != 'good':
        rospy.logfatal("Plan quality to the IK solution is poor!")
        sys.exit("Exiting script to allow for the arm to be manually moved to better start position ...")
    # Execute plan
    rospy.loginfo("Moving arm to pose:\n" + str(pose_msg) + "\n")
    arm_commander.execute_plan(ik_plan)
    rospy.sleep(2.0)
else:
    rospy.logerr("IK solution was None!")
    sys.exit("Exiting script as IK solution was None...")

input("Press Enter to continue...")
rospy.loginfo("Following trajectory defined by waypoints")
waypoints = []

# start with the initial position
initial_pose = arm_commander.get_current_pose()
# Using the method plan_to_waypoints_target, it is possible to specify a set of waypoints
# for the end-effector and create a plan to follow it.
# https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py#L830

wpose = geometry_msgs.msg.Pose()
wpose.position.x = initial_pose.position.x
wpose.position.y = initial_pose.position.y - 0.10
wpose.position.z = initial_pose.position.z
wpose.orientation = initial_pose.orientation
waypoints.append(wpose)

wpose = geometry_msgs.msg.Pose()
wpose.position.x = initial_pose.position.x - 0.10
wpose.position.y = initial_pose.position.y
wpose.position.z = initial_pose.position.z + 0.1
wpose.orientation = initial_pose.orientation
waypoints.append(wpose)

waypoints.append(initial_pose)

arm_commander.plan_to_waypoints_target(waypoints, eef_step=0.02)
arm_commander.execute()

rospy.sleep(2.0)

# Finish arm at home and hand at pack
input("Press Enter to continue...")
rospy.loginfo("Moving arm to joint states\n" + str(arm_hand_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_hand_home_joints_goal, 6.0, True)
rospy.sleep(2.0)
