#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2022-2023 belongs to Shadow Robot Company Ltd.
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
