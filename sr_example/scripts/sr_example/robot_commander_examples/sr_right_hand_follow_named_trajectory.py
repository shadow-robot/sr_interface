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


# This example demonstrates how you can send a trajectory created from named poses.


import rospy
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander


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
    },
    {
        'name': 'open',
        'interpolate_time': 3.0
    }
]

# Run trajectory via moveit
hand_commander.run_named_trajectory(trajectory)

# Run trajectory by sending directly to controllers - faster but no collision checking.
hand_commander.run_named_trajectory_unsafe(trajectory, True)
