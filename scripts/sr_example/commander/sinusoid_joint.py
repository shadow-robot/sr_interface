#!/usr/bin/env python

# Example where two joints are specified and move with a sinusoidal trajectory, with a pi/2 phase difference

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_hand_commander import SrHandCommander
from numpy import sin, cos, pi, arange

# cycles per second of sine wave
f = 1
# angular frequency, rads/s
w = 2*pi*f

rospy.init_node("hand_sine_example", anonymous=True)

hand_commander = SrHandCommander()


# specify 2 joints to move
joint_names = ['rh_FFJ3', 'rh_MFJ3']
min_pos_J3 = 0.0
max_pos_J3 = pi/2

rospy.sleep(rospy.Duration(2))

hand_joint_states_1 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
                       'rh_MFJ1': 0.35, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                       'rh_RFJ1': 0.35, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                       'rh_LFJ1': 0.35, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                       'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
                       'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}


print("Running joints trajectory")

joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = rospy.Time.now()
joint_trajectory.joint_names = list(hand_joint_states_1.keys())
joint_trajectory.points = []
time_from_start = rospy.Duration(5)

# generate sinusoidal list of data points, two joints moving out of phase
for t in arange(0.002, 20, 0.02):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(t))
    trajectory_point.positions = []
    trajectory_point.velocities = []
    trajectory_point.accelerations = []
    trajectory_point.effort = []
    for key in joint_trajectory.joint_names:
        if key in joint_names[0]:
            joint_position = sin(w * t) * (max_pos_J3 - min_pos_J3)/2 + (max_pos_J3 - min_pos_J3)/2 + min_pos_J3
            trajectory_point.positions.append(joint_position)
        elif key in joint_names[1]:
            joint_position = cos(w * t) * (max_pos_J3 - min_pos_J3)/2 + (max_pos_J3 - min_pos_J3)/2 + min_pos_J3
            trajectory_point.positions.append(joint_position)
        else:
            trajectory_point.positions.append(hand_joint_states_1[key])
        trajectory_point.velocities.append(0.0)
        trajectory_point.accelerations.append(0.0)
        trajectory_point.effort.append(0.0)

    joint_trajectory.points.append(trajectory_point)

hand_commander.run_joint_trajectory_unsafe(joint_trajectory)

rospy.sleep(rospy.Duration(15))
