#!/usr/bin/env python

import rospy

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

# It's assumed that a module containing states and named named exported_states.py has already
# been exported and is found somewhere on the path (e.g. in the same directory as this script).
# Let's assume it contains two states, 'state_1' and 'state_2'

from exported_states import warehouse_states

"""
Now we have a dictionary of states called warehouse_states, e.g.
warehouse_states = {
  'state_1': {
     'joint_0': 0.00,
     'joint_1': 0.00
  },
  'state_2': {
     'joint_0': 0.00,
     'joint_1': 0.00
  }
}
"""

# Define a hand commander, so we have something to do with the states we extracted.
hand_commander = SrHandCommander()

# You could use the states directly:
hand_commander.move_to_named_target(warehouse_states['state_1'])

# You could translate all the named states in a trajectory:

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

state_exporter = SrRobotStateExporter(warehouse_states))
converted_trajectory = state_exporter.convert_trajectory(trajectory)

hand_commander.run_named_trajectory(converted_trajectory)


# Or we could repopulate the warehouse with the exported states:

state_exporter = SrRobotStateExporter(warehouse_states))
state_exporter.repopulate_warehouse()
