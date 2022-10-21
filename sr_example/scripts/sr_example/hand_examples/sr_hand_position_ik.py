#!/usr/bin/python3

# Copyright 2022 Shadow Robot Company Ltd.
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

# This snippet demonstrates position-only (no orientation) inverse kinematics of the Dexterous Hand. This is often
# necessary, as humanoid fingers can't reach the majority of Pose (position AND orientation) goals.

import math
import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped

# Initialise this ROS node
rospy.init_node('sr_hand_position_ik')
# Initialise the Move Group Commander
commander = moveit_commander.MoveGroupCommander('rh_first_finger')
# Set the goal orientation tolerance to 2 PI to ensure the IK solver doesn't care about orientation
commander.set_goal_orientation_tolerance(2 * math.pi)
# Construct a PoseStamped, allowing us to specify the frame in which we want to express the goal
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'rh_palm'
goal_pose.pose.position.x = 0.033
goal_pose.pose.position.y = -0.01
goal_pose.pose.position.z = 0.18
# Make the new rotation quaternion a valid one - unused but prevents complaints
goal_pose.pose.orientation.w = 1.0
# Make sure we are planning from the current state of the robot
commander.set_start_state_to_current_state()
# Apply the above PoseStamped as a goal
commander.set_pose_target(goal_pose)
# Plan, and if successful, execute the trajectory
success, trajectory, *_other = commander.plan()
if success:
    commander.execute(trajectory)
