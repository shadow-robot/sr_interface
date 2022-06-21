#!/usr/bin/env python3
# Copyright 2019, 2022 Shadow Robot Company Ltd.
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

# Example where two joints are specified and move with a sinusoidal trajectory, with a pi/4 phase difference

from numpy import sin, cos, pi, arange
import rospy
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
cycles = 1
# angular frequency, rads/s
angular_freq = 2 * pi * cycles
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
            joint_position = sin(angular_freq * t) * (max_pos_j3 - min_pos_j3) \
                             / 2 + (max_pos_j3 - min_pos_j3) / 2 + min_pos_j3
            trajectory_point.positions.append(joint_position)
        elif key in joint_names[1]:  # generate joint positions for second joint
            joint_position = cos(angular_freq * t) * (max_pos_j3 - min_pos_j3) \
                             / 2 + (max_pos_j3 - min_pos_j3) / 2 + min_pos_j3
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
