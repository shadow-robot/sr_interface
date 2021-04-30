#!/usr/bin/env python3
# Copyright 2019 Shadow Robot Company Ltd.
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

import rospy
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

"""
The following example shows how to use the warehouse state exporter to save
robot states from the warehouse as plain text python files. States can be
extracted one by one, as a list of state names, or all states from a named
trajectory can be extracted together.

Once states have been extracted, the resulting dictionary can be exported to
 a specified file location. The generated file can then be used as a modules
for importing into subsequent scripts.
"""
# Below is a named trajectory, of the sort used by SrRobotCommander. It is
# assumed that the states "state_1" and "state_2" exist in the warehouse.

trajectory = [
    {
        'name': 'state_1',
        'interpolate_time': 3.0
    },
    {
        'name': 'state_2',
        'interpolate_time': 3.0,
        'pause_time': 2
    }
]

state_exporter = SrRobotStateExporter()

# The following three snippets will produce the same effect, i.e. to extract
# two states, named "state_1" and "state_2"

# Extract one at a time:
state_exporter.extract_one_state("state_1")
state_exporter.extract_one_state("state_2")

# Extract a list:
state_exporter.extract_list(['state_1', 'state_2'])

# Extract all states in a trajectory:
state_exporter.extract_from_trajectory(trajectory)

# Alternatively, you could extract all states from the warehouse:
state_exporter.extract_all()

# Once the states you requrie are extracted, they can be exported to a module:
state_exporter.output_module("/tmp/exported_states.py")
