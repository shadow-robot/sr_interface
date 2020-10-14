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
#     roslaunch sr_right_ur10arm_hand.launch external_control_loop:=true sim:=false scene:=true
# simulated robot:
#     roslaunch sr_right_ur10arm_hand.launch sim:=true scene:=true

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("right_hand_arm_ef_pos", anonymous=True)

# The constructors for the SrArmCommander, SrHandCommander and SrRobotCommander
# take a name parameter that should match the group name of the robot to be used.
# How to command the hand or arm separately
hand_commander = SrHandCommander(name="right_hand")
arm_commander = SrArmCommander(name="right_arm")
# How to command the arm and hand together
robot_commander = SrRobotCommander(name="right_arm_and_hand")
arm_commander.set_max_velocity_scaling_factor(0.1)

rospy.sleep(rospy.Duration(2))

# Start arm at home
arm_home_joints_goal = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                        'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                        'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 0.00}
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

raw_input("Press Enter to continue...")

# The arm commander generates a plan to a new pose before the pose is executed.
# plan_to_pose_target(pose, end_effector_link, alternative_method, custom_start_state):
#         Specify a target pose for the end-effector and plans.
#         This is a blocking method.
#         @param pose - new pose of end-effector: a Pose message, a PoseStamped
#         message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
#         of 7 floats [x, y, z, qx, qy, qz, qw].
#         @param end_effector_link - name of the end effector link.
#         @param alternative_method - use set_joint_value_target instead of set_pose_target.
#         @param custom_start_state - specify a start state different than the current state.
pose_1 = [1.1, 0.5, 1.2, -0.9, 0.0, -0.0]
print("Planning the move to the first pose:\n" + str(pose_1) + "\n")
arm_commander.plan_to_pose_target(pose_1)
print("Finished planning, moving the arm now.")
# Can only execute if a plan has been generated.
arm_commander.execute()

raw_input("Press Enter to continue...")

rospy.sleep(2.0)

pose_2 = [1.1, -0.1, 1.2, -0.9, 0.0, -0.0]
print("Planning the move to the second pose:\n" + str(pose_2) + "\n")
arm_commander.plan_to_pose_target(pose_2)
print("Finished planning, moving the arm now.")
arm_commander.execute()

raw_input("Press Enter to continue...")

# Here a pose is provided and the arm commander moves the arm to it
# move_to_pose_target(pose, end_effector_link, wait):
#         @param pose - new pose of end-effector: a Pose message, a PoseStamped
#         message or a list of 6 floats: [x, y, z, rot_x, rot_y, rot_z] or a list
#         of 7 floats [x, y, z, qx, qy, qz, qw].
#         @param end_effector_link - name of the end effector link.
#         @param wait - should method wait for movement end or not.
pose_1 = [1.1, 0.5, 1.2, -0.9, 0.0, -0.0]
print("Moving arm to pose:\n" + str(pose_1) + "\n")
arm_commander.move_to_pose_target(pose_1, wait=True)

raw_input("Press Enter to continue...")

rospy.sleep(2.0)

# Here a pose is provided and the arm commander moves the arm to it
pose_3 = [1.1, 0.5, 1.0, -0.9, 0.1, 0, 0.4]
print("Moving arm to pose:\n" + str(pose_3) + "\n")
arm_commander.move_to_pose_target(pose_3, wait=True)

raw_input("Press Enter to continue...")

rospy.sleep(2.0)

# Finish arm at home
rospy.loginfo("Moving arm to joint states\n" + str(arm_home_joints_goal) + "\n")
robot_commander.move_to_joint_value_target_unsafe(arm_home_joints_goal, 6.0, True)

# Move hand to open
# Moving to a stored named target, stored targets can be viewed in MoveIt in the planning tab
rospy.loginfo("Moving hand to joint state: open")
hand_commander.move_to_named_target("open")

rospy.sleep(rospy.Duration(3))

# Current positions and velocities are read
print("Arm joints position:\n" +
      str(arm_commander.get_joints_position()) + "\n")
print("Arm joints velocities:\n" +
      str(arm_commander.get_joints_velocity()) + "\n")
