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

trajectory = [
    {
        'name': 'open',
        'interpolate_time': 3.0,
        'pause_time': 2
    },
    {
        'name': 'fingers_pack_thumb_open',
        'interpolate_time': 3.0,
        'pause_time': 2
    },
    {
        'name': 'pack',
        'interpolate_time': 3.0
    }
]

# Run trajectory via moveit
hand_commander.run_named_trajectory(trajectory)
