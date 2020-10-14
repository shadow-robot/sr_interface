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
# For more information, please see https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot:
#     roslaunch sr_right_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_right_ur10arm_hand.launch sim:=true scene:=true

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

# Start arm at home
arm_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                     'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733, 
                     'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}
rospy.loginfo("Moving arm to joint states\n" + str(arm_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_home_joints_goal, 6.0, True)

# Move hand to pack
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: pack")
hand_commander.move_to_named_target("pack")

# Moving arm to initial pose
# pose_1 = [1.2, 0.54, 0.92, -0.87, -0.07, -0.03, 0.39]
pose_1 = [1.1, 0.5, 1.2, -0.9, 0.0, -0.0]


print "Moving to initial pose"
arm_commander.plan_to_pose_target(pose_1)
arm_commander.execute()

rospy.sleep(rospy.Duration(2))

raw_input("Press Enter to continue...")

print "Following trajectory defined by waypoints"
waypoints = []

# start with the initial position
initial_pose = arm_commander.get_current_pose()


# Using the method plan_to_waypoints_target, it is possible to specify a set of waypoints 
# for the end-effector and create a plan to follow it.
# Parameters:
#       reference_frame is the reference frame in which the waypoints are given
#       waypoints is an array of poses of the end-effector.
#       eef_step indicates that the configurations are goint to be computed for every eef_step meters (0.005 by default)
#       jump_threshold specify the maximum distance in configuration space between consecutive points in the resulting path (0.0 by default)

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

raw_input("Press Enter to continue...")


# Finish arm at home
arm_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                     'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733, 
                     'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}
rospy.loginfo("Moving arm to joint states\n" + str(arm_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_home_joints_goal, 6.0, True)

# Move hand to open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")

rospy.sleep(rospy.Duration(3))