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

# Example where two joints are specified and move with a sinusoidal trajectory, with a pi/4 phase difference

import rospy
from numpy import sin, cos, pi, arange
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("joint_sine_example", anonymous=True)

# handfinder is used to access the hand parameters
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = hand_parameters.mapping.values()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

# cycles per second of sine wave
frequency = 1
# angular frequency, rads/s
angular_frequency = 2 * pi * frequency
# time for motion to complete
time_to_complete = 20

# specify 2 joints to move
joint_names = [prefix[0] + '_FFJ3', prefix[0] + '_RFJ3']

# set max and min joint positions
min_pos_j3 = 0.0
max_pos_j3 = pi / 2

rospy.sleep(rospy.Duration(2))

hand_joints_goal = {joint_names[0]: 0.0, joint_names[1]: 0.0}

rospy.loginfo("Running joints trajectory")

# initialising the joint trajectory message
joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = rospy.Time.now()
joint_trajectory.joint_names = list(hand_joints_goal.keys())
joint_trajectory.points = []

# generate sinusoidal list of data points, two joints moving out of phase
for t in arange(0.002, time_to_complete, 0.02):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(t))
    trajectory_point.positions = []
    trajectory_point.velocities = []
    trajectory_point.accelerations = []
    trajectory_point.effort = []

    for key in joint_trajectory.joint_names:
        if key in joint_names[0]:  # generate joint positions for first joint
            joint_position = sin(angular_frequency * t) * (max_pos_j3 - min_pos_j3) / 2 + \
                             (max_pos_j3 - min_pos_j3) / 2 + min_pos_j3
            trajectory_point.positions.append(joint_position)
        elif key in joint_names[1]:  # generate joint positions for second joint
            joint_position = cos(angular_frequency * t) * (max_pos_j3 - min_pos_j3) / 2 + \
                             (max_pos_j3 - min_pos_j3) / 2 + min_pos_j3
            trajectory_point.positions.append(joint_position)
        else:
            trajectory_point.positions.append(hand_joints_goal[key])

        trajectory_point.velocities.append(0.0)
        trajectory_point.accelerations.append(0.0)
        trajectory_point.effort.append(0.0)

    joint_trajectory.points.append(trajectory_point)

# Send trajectory to hand_commander
hand_commander.run_joint_trajectory_unsafe(joint_trajectory)

rospy.sleep(rospy.Duration(15))
