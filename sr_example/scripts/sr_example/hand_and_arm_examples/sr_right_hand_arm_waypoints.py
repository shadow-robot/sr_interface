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

# This example demonstrates some of the functions of the hand, arm and robot commander.
# The arm, hand and robot will be moved through a sequence of goals generated via different functions in the commander.
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
#               external_control_loop:=true sim:=false scene:=true include_wrist_in_arm_controller:=false
# simulated robot:
#     roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true scene:=true

# It is recommended to run this script in simulation first.

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
import geometry_msgs.msg


rospy.init_node("hand_arm_waypoints", anonymous=True)

# The constructors for the SrArmCommander, SrHandCommander and SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
# How to command the hand or arm separately
hand_commander = SrHandCommander(name="right_hand")
arm_commander = SrArmCommander(name="right_arm")
# How to command the arm and hand together
robot_commander = SrRobotCommander(name="right_arm_and_hand")
arm_commander.set_max_velocity_scaling_factor(0.1)

rospy.sleep(rospy.Duration(1))

# Start arm at home and hand at pack
arm_hand_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                             'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                             'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00,
                             'rh_THJ1': 0.52, 'rh_THJ2': 0.61, 'rh_THJ3': 0.0, 'rh_THJ4': 1.20,
                             'rh_THJ5': 0.17, 'rh_FFJ1': 1.5707, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707,
                             'rh_FFJ4': 0.0, 'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707,
                             'rh_MFJ4': 0.0, 'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707,
                             'rh_RFJ4': 0.0, 'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707,
                             'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
rospy.loginfo("Moving arm and hand to joint states\n" + str(arm_hand_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_hand_home_joints_goal, 6.0, True)

# Moving arm to initial pose
raw_input("Press Enter to continue...")
pose_1 = [0.9, 0.16, 0.95, -0.99, 8.27, -0.0, 1.4]
print("Moving to initial pose")
arm_commander.plan_to_pose_target(pose_1)
arm_commander.execute()
rospy.sleep(rospy.Duration(2))

raw_input("Press Enter to continue...")
print("Following trajectory defined by waypoints")
waypoints = []

# start with the initial position
initial_pose = arm_commander.get_current_pose()

# Using the method plan_to_waypoints_target, it is possible to specify a set of waypoints
# for the end-effector and create a plan to follow it.
# Specify a set of waypoints for the end-effector and plans.
# This is a blocking method.
# plan_to_waypoints_target(swaypoints, reference_frame, eef_step, jump_threshold, custom_start_state):
#       @param reference_frame - the reference frame in which the waypoints are given.
#       @param waypoints - an array of poses of end-effector.
#       @param eef_step - configurations are computed for every eef_step meters.
#       @param jump_threshold - maximum distance in configuration space between consecutive points in the
#       resulting path.
#       @param custom_start_state - specify a start state different than the current state.
#       @return - motion plan (RobotTrajectory msg) that contains the trajectory to the set wayapoints targets.

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
raw_input("Press Enter to continue...")
rospy.loginfo("Moving arm to joint states\n" + str(arm_hand_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_hand_home_joints_goal, 6.0, True)
rospy.sleep(2.0)
