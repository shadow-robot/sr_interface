#!/usr/bin/env python

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

state_extractor = SrRobotStateExtractor()

# The following three snippets will produce the same effect, i.e. to extract
# two states, named "state_1" and "state_2"

# Extract one at a time:
state_extractor.extract_one_state("state_1")
state_extractor.extract_one_state("state_2")

# Extract a list:
state_extractor.extract_list(['state_1', 'state_2'])

# Extract all states in a trajectory:
state_extractor.extract_from_trajetory(trajectory)

# Alternatively, you could extract all states from the warehouse:
state_extractor.extract_all()

# Once the states you requrie are extracted, they can be exported to a module:
state_extractor.output_module("/tmp/exported_states.py")
