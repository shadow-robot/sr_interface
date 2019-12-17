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

# This example demonstrates some of the functions of the arm commander.
# The arm is moved through a sequence of goals generated via different functions in the commander.

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_arm_commander import SrArmCommander
import geometry_msgs.msg


rospy.init_node("move_arm_following_waypoint_trajectory", anonymous=True)

arm_commander = SrArmCommander(set_ground=True)

rospy.sleep(rospy.Duration(1))

# Moving arm to initial pose
pose_1 = [0.32, 0.27, 1.026, 0.0, 0.0, 0.0, 1.0]

print "Moving to initial pose"
arm_commander.plan_to_pose_target(pose_1)
arm_commander.execute()

rospy.sleep(rospy.Duration(2))

print "Following trajectory defined by waypoints"
waypoints = []

# start with the initial position
initial_pose = arm_commander.get_current_pose()


wpose = geometry_msgs.msg.Pose()
wpose.position.x = initial_pose.position.x
wpose.position.y = initial_pose.position.y - 0.20
wpose.position.z = initial_pose.position.z
wpose.orientation = initial_pose.orientation
waypoints.append(wpose)

wpose = geometry_msgs.msg.Pose()
wpose.position.x = initial_pose.position.x
wpose.position.y = initial_pose.position.y - 0.20
wpose.position.z = initial_pose.position.z - 0.20
wpose.orientation = initial_pose.orientation
waypoints.append(wpose)

wpose = geometry_msgs.msg.Pose()
wpose.position.x = initial_pose.position.x
wpose.position.y = initial_pose.position.y
wpose.position.z = initial_pose.position.z - 0.20
wpose.orientation = initial_pose.orientation
waypoints.append(wpose)

waypoints.append(initial_pose)

arm_commander.plan_to_waypoints_target(waypoints, eef_step=0.02)
arm_commander.execute()
