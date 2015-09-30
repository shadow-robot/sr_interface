#!/usr/bin/env python

# This example demonstrates how you can send a trajectory created from named poses.

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("named_traj_example", anonymous=True)
rospy.sleep(1)  # Do not start at time zero


hand_commander = SrHandCommander()

# Define trajectory. Interpolate time (time to move to each point from previous posture)
# must be specified. Pause time is optional. Names are either the default poses defined
# in SRDF or are states stored in the warehouse.

trajectory = [
    {
        'name':'open',
        'interpolate_time':3.0
    },
    {
        'name':'pack',
        'interpolate_time':3.0,
        'pause_time':2
    },
    {
        'name':'open',
        'interpolate_time':3.0
    },
    {
        'name':'pack',
        'interpolate_time':3.0
    }
]

# Run trajectory
hand_commander.run_named_trajectory_unsafe(trajectory,True)
