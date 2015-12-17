#!/usr/bin/env python

# An example of how to follow a trajectory defined using a set of waypoints for the first finger.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
import geometry_msgs.msg
import copy

rospy.init_node("hand_example_following_finger_traj", anonymous=True)

hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = hand_parameters.mapping.values()[0]
hand_serial = hand_parameters.mapping.keys()[0]

# Close all fingers except thumb
hand_commander = SrHandCommander()
hand_commander.move_to_named_target("fingers_pack_thumb_open")

# Change to first finger group
group_id = prefix + "_first_finger"
hand_commander = SrHandCommander(name=group_id)

# Open first finger
hand_commander.move_to_named_target("first_finger_open")

rospy.sleep(4)

rospy.loginfo("\nMoving Index finger following a set of waypoints")

waypoints = []

start_pose = hand_commander.get_current_pose(reference_frame=prefix + "_palm")
waypoints.append(start_pose)

# Move horizontally
pose = geometry_msgs.msg.Pose()
pose.position.x = 0.067
pose.position.y = -0.010
pose.position.z = 0.184
pose.orientation.w = 1.00
waypoints.append(pose)

waypoints.append(start_pose)

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.011
pose.position.y = -0.014
pose.position.z = 0.187
pose.orientation.w = 1.00
waypoints.append(pose)

waypoints.append(start_pose)

# Move vertically
pose1 = geometry_msgs.msg.Pose()
pose1.position.x = 0.033
pose1.position.y = -0.016
pose1.position.z = 0.190
pose1.orientation.w = 1.00
waypoints.append(pose1)

pose2 = geometry_msgs.msg.Pose()
pose2.position.x = 0.033
pose2.position.y = -0.035
pose2.position.z = 0.178
pose2.orientation.w = 1.00
waypoints.append(pose2)

pose3 = geometry_msgs.msg.Pose()
pose3.position.x = 0.033
pose3.position.y = -0.068
pose3.position.z = 0.151
pose3.orientation.w = 1.00
waypoints.append(pose3)

waypoints.append(pose2)

waypoints.append(pose1)

waypoints.append(start_pose)

hand_commander.plan_to_waypoints_target(waypoints, reference_frame=prefix + "_palm")
hand_commander.execute()

rospy.sleep(3.0)
