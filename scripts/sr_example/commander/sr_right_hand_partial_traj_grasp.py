#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("set_traj_goals", anonymous=True)
rospy.sleep(1)  # Do not start with zero


def construct_trajectory_point(posture, duration):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(posture[key])
    return trajectory_point


hand_commander = SrHandCommander()

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
             'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
             'rh_THJ4': 0.0, 'rh_THJ5': 0.0, 'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

keys = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_LFJ1', 'rh_LFJ2',
        'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3',
        'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_THJ1',
        'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 'rh_WRJ1', 'rh_WRJ2']
position = [1.0707232604620422, 0.26175844295000505, 0.883936396387667,
            -0.3434683241082386, 0.8476100321368696, 0.603282427869523,
            0.21068692113415377, -0.2259571234036457, 0.15140092663442495,
            1.0574641003686231, 0.15788941690905922, 1.0361759477073011,
            0.05490553542027321, 1.040703232148128, 0.3401612929181024,
            0.6804364962780189, -0.23700380992456882, 0.3491125761045675,
            0.6901807769501129, 0.17965518799781588, 1.2043661627831659,
            -0.10694821918223951, -0.07495731995088306, -0.21656822158752753]
grasp_pose = dict(zip(keys, position))

grasp_partial_1 = {'rh_FFJ3': 0.50}

start_time = rospy.Time.now()

# Move hand using move_to_joint_value_target_unsafe
hand_commander.move_to_joint_value_target_unsafe(open_hand, 1.0, True)

rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(grasp_pose, 1.0, False)

# Move hand using run_joint_trajectory_unsafe
trajectory_start_time = 2.0
joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = start_time + rospy.Duration.from_sec(
    float(trajectory_start_time))
joint_trajectory.joint_names = list(grasp_partial_1.keys())
joint_trajectory.points = []
trajectory_point = construct_trajectory_point(grasp_partial_1, 1.0)
joint_trajectory.points.append(trajectory_point)
hand_commander.run_joint_trajectory_unsafe(joint_trajectory, True)
