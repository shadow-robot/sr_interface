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

# Open fingers
group_id = prefix + "_fingers"
hand_commander = SrHandCommander(name=group_id)
hand_commander.move_to_named_target("fingers_open")

group_id = prefix + "_first_finger"
hand_commander = SrHandCommander(name=group_id)

rospy.sleep(2)
rospy.loginfo("\nMoving Index finger following a set of waypoints")

waypoints = []

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.057038
pose.position.y = -0.13281
pose.position.z = 0.37172
pose.orientation.x = 0.60863
pose.orientation.y = 0.05909
pose.orientation.z = -0.044238
pose.orientation.w = 0.79001
waypoints.append(pose)

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.08341
pose.position.y = -0.13383
pose.position.z = 0.36473
pose.orientation.x = 0.55
pose.orientation.y = 0.23079
pose.orientation.z = -0.058283
pose.orientation.w = 0.80052
waypoints.append(pose)

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.083263
pose.position.y = -0.10451
pose.position.z = 0.39957
pose.orientation.x = 0.27607
pose.orientation.y = 0.16801
pose.orientation.z = -0.0061039
pose.orientation.w = 0.94632
waypoints.append(pose)

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.067446
pose.position.y = -0.099706
pose.position.z = 0.40685
pose.orientation.x = 0.24168
pose.orientation.y = 0.083281
pose.orientation.z = -0.021483
pose.orientation.w = 0.96654
waypoints.append(pose)

hand_commander.plan_to_waypoints_target(waypoints)
hand_commander.execute()
