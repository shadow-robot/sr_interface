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
