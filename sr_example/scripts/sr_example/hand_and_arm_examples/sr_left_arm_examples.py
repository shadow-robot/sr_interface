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

# This example demonstrates some of the functions of the arm commander using poses.
# The arm is moved through a sequence of goals generated via different pose functions in the commander.
# A pose can be a PoseStamped message,
# or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z],
# or a list of 7 floats [x, y, z, qx, qy, qz, qw]
# PLEASE NOTE: move_to_joint_value_target_unsafe is used in this script, so collision checks
# between the hand and arm are not made!

# For more information, please see:
# https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#robot-commander

# roslaunch commands used with this script to launch the robot:
# real robot with a NUC (or a separate computer with an RT kernel):
#     roslaunch sr_left_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true start_home:=true
# simulated robot:
#     roslaunch sr_left_ur10arm_hand.launch sim:=true scene:=true start_home:=true

# It is recommended to run this script in simulation first.

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("left_hand_arm_ef_pos", anonymous=True)

# The constructors for the SrArmCommander, SrHandCommander and SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
# How to command the hand or arm separately
hand_commander = SrHandCommander(name="left_hand")
arm_commander = SrArmCommander(name="left_arm")
# How to command the arm and hand together
robot_commander = SrRobotCommander(name="left_arm_and_hand")
arm_commander.set_max_velocity_scaling_factor(0.1)
rospy.sleep(2.0)

arm_home_joints_goal = {'la_shoulder_pan_joint': 1.47, 'la_elbow_joint': -1.695,
                        'la_shoulder_lift_joint': -1.22, 'la_wrist_1_joint': -01.75,
                        'la_wrist_2_joint': 1.57, 'la_wrist_3_joint': -1.830}

example_pose_1 = [0.6, 0.3, 1.5, -0.7, 0.04, -0.67]

example_pose_2 = [0.6, 0.3, 1.46, -0.7, 0.04, 0.69, 0.08]

example_pose_3 = [0.69, 0.64, 1.4, -0.71, 0.04, 0.69, 0.08]

# Start arm at home
rospy.loginfo("Moving arm to joint states\n" + str(arm_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_home_joints_goal, 6.0, True)

# Move hand to open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")
rospy.sleep(3.0)

# Move hand to pack
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: pack")
hand_commander.move_to_named_target("pack")

# The arm commander generates a plan to a new pose before the pose is executed.
raw_input("Press Enter to continue...")
rospy.loginfo("Planning the move to the first pose:\n" + str(example_pose_1) + "\n")
arm_commander.plan_to_pose_target(example_pose_1)
rospy.loginfo("Finished planning, moving the arm now.")
# Can only execute if a plan has been generated.
arm_commander.execute()
rospy.sleep(2.0)

raw_input("Press Enter to continue...")
rospy.loginfo("Planning the move to the second pose:\n" + str(example_pose_2) + "\n")
arm_commander.plan_to_pose_target(example_pose_2)
rospy.loginfo("Finished planning, moving the arm now.")
arm_commander.execute()

# Here a pose is provided and the arm commander moves the arm to it
raw_input("Press Enter to continue...")
rospy.loginfo("Moving arm to pose:\n" + str(example_pose_1) + "\n")
arm_commander.move_to_pose_target(example_pose_1, wait=True)
rospy.sleep(2.0)

# Here a pose is provided and the arm commander moves the arm to it
raw_input("Press Enter to continue...")
rospy.loginfo("Moving arm to pose:\n" + str(example_pose_3) + "\n")
arm_commander.move_to_pose_target(example_pose_3, wait=True)
rospy.sleep(2.0)

# Finish arm at home
raw_input("Press Enter to continue...")
rospy.loginfo("Moving arm to joint states\n" + str(arm_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_home_joints_goal, 6.0, True)

# Move hand to open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")
rospy.sleep(2.0)
