#!/usr/bin/env python

# This example demonstrates how you can send a trajectory created from named poses.

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
rospy.init_node("named_traj_example", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)


# Define trajectory. Interpolate time (time to move to each point from previous posture)
# must be specified. Pause time is optional. Names are either the default poses defined
# in SRDF or are states stored in the warehouse.

trajectory = [
    {
        'name': 'open',
        'interpolate_time': 3.0
    },
    {
        'name': 'pack',
        'interpolate_time': 3.0,
        'pause_time': 2
    },
    {
        'name': 'open',
        'interpolate_time': 3.0
    },
    {
        'name': 'pack',
        'interpolate_time': 3.0
    }
]


# Run trajectory via moveit
hand_commander.run_named_trajectory(trajectory)

# Run trajectory by sending directly to controllers - faster but no collision checking.
hand_commander.run_named_trajectory_unsafe(trajectory, True)
