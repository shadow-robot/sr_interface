#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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

# This example demonstrates how you can send a partial trajectory to a joint or set of joints.
# The partial trajectory can then be run during an existing motion and define a new goal for
# the joints specified in this sub list


import rospy
from sr_utilities.hand_finder import HandFinder
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("partial_traj_example", anonymous=True)
rospy.sleep(1)  # Do not start at time zero


def construct_trajectory_point(posture, duration):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(posture[key])
    return trajectory_point


hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

hand_mapping = hand_parameters.mapping[hand_serial]

# Hand joints are detected
joints = hand_finder.get_hand_joints()[hand_mapping]

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
             'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
             'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

keys = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_LFJ1', 'rh_LFJ2',
        'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3',
        'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_THJ1',
        'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']

position = [1.07, 0.26, 0.88, -0.34, 0.85, 0.60,
            0.21, -0.23, 0.15, 1.06, 0.16, 1.04,
            0.05, 1.04, 0.34, 0.68, -0.24, 0.35,
            0.69, 0.18, 1.20, -0.11]

grasp_pose = dict(zip(keys, position))

# Adjust poses according to the hand loaded
open_hand_current = {i: open_hand[i] for i in joints if i in open_hand}
grasp_pose_current = {i: grasp_pose[i] for i in joints if i in grasp_pose}

# Partial list of goals
grasp_partial_1 = {'rh_FFJ3': 0.50}

start_time = rospy.Time.now()

# Move hand using move_to_joint_value_target_unsafe to 1st position
hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

rospy.sleep(2)

# Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

trajectory_start_time = 2.0
joint_trajectory = JointTrajectory()

# Construct and send partial trajectory for joint listed in grasp_partial_1
joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(float(trajectory_start_time))
joint_trajectory.joint_names = list(grasp_partial_1.keys())
joint_trajectory.points = []
this_trajectory_point = construct_trajectory_point(grasp_partial_1, 1.0)
joint_trajectory.points.append(this_trajectory_point)

hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)
rospy.sleep(2)
