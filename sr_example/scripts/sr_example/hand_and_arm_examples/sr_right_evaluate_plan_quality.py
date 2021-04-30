#!/usr/bin/env python3
# Copyright 2020 Shadow Robot Company Ltd.
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
#
# This example demonstrates some the function of the robot commander for evaluating plan quality.
# https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py#L263-L310
#
# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander
#
# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
#               external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true scene:=true
#
# It is recommended to run this script in simulation first.

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_robot_commander import SrRobotCommander

rospy.init_node("right_hand_manipulator_plan_quality", anonymous=True)

robot_commander = SrRobotCommander(name="right_arm_and_manipulator")
robot_commander.set_max_velocity_scaling_factor(0.4)

rospy.sleep(2.0)

arm_manipulator_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                                    'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                    'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00,
                                    'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

example_goal_1 = [1.18773740212, 0.175152574887,
                  1.00722873872, -0.509149713327,
                  0.504302807033, -0.490693581737,
                  0.49564610064]

example_goal_2 = [1.03497881882, 0.174190990124,
                  1.00916719803, -0.510952874905,
                  0.510169511476, -0.4888330603,
                  0.489588059847]

example_goal_3 = [1.18972252888, 0.174200052352,
                  1.12320616864, -0.516434824527,
                  0.516354718344, -0.483015844727,
                  0.483082364202]

# Evaluate the plan quality from starting position of robot.
# https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py#L263-L310
# This is to confirm that the arm is in a safe place to move to the home joints goal,
# if the outcome is poor please refer to the readthedocs of where the start arm home position is.
rospy.loginfo("Planning to move arm and manipulator to joint states\n" + str(arm_manipulator_home_joints_goal) + "\n")
plan = robot_commander.plan_to_joint_value_target(arm_manipulator_home_joints_goal)
plan_quality = robot_commander.evaluate_given_plan(plan)
eval_plan_quality = robot_commander.evaluate_plan_quality(plan_quality)

if eval_plan_quality != 'good':
    rospy.logfatal("Plan quality to the home position is poor! " +
                   "For safety please refer to the hand and arm documentation " +
                   "for where to start the arm " +
                   "to ensure no unexpected movements during plan and execute.")
    sys.exit("Exiting script to allow for the arm to be manually moved to better start position ...")

raw_input("Press Enter to execute plan...")
robot_commander.execute_plan(plan)

rospy.loginfo("Planning the move to the first pose:\n" + str(example_goal_1) + "\n")
plan2 = robot_commander.plan_to_pose_target(example_goal_1)
plan_quality = robot_commander.evaluate_given_plan(plan2)
eval_plan_quality = robot_commander.evaluate_plan_quality(plan_quality)

raw_input("Press Enter to execute plan...")
robot_commander.execute_plan(plan2)
rospy.sleep(2.0)

rospy.loginfo("Planning the move to the second pose:\n" + str(example_goal_1) + "\n")
plan3 = robot_commander.plan_to_pose_target(example_goal_2)
plan_quality = robot_commander.evaluate_given_plan(plan3)
eval_plan_quality = robot_commander.evaluate_plan_quality(plan_quality)

raw_input("Press Enter to execute plan...")
robot_commander.execute_plan(plan3)
rospy.sleep(2.0)

rospy.loginfo("Planning the move to the third pose:\n" + str(example_goal_1) + "\n")
plan4 = robot_commander.plan_to_pose_target(example_goal_3)
plan_quality = robot_commander.evaluate_given_plan(plan4)
eval_plan_quality = robot_commander.evaluate_plan_quality(plan_quality)

raw_input("Press Enter to execute plan...")
robot_commander.execute_plan(plan4)
rospy.sleep(2.0)
