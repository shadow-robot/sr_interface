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

rospy.init_node("use_exported_states")

# Define a hand commander, so we have something to do with the states we extracted.
hand_commander = SrHandCommander()

# You could use the states directly:
hand_commander.move_to_joint_value_target_unsafe(warehouse_states['state_2'])
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(warehouse_states['state_1'])
rospy.sleep(2)
hand_commander.move_to_joint_value_target_unsafe(warehouse_states['state_2'])
rospy.sleep(2)
# You could translate all the named states in a trajectory:

trajectory = [
    {
        'name': 'state_1',
        'interpolate_time': 3.0
    },
    {
        'name': 'state_2',
        'interpolate_time': 3.0,
        'pause_time': 1
    }
]

state_exporter = SrRobotStateExporter(warehouse_states)
converted_trajectory = state_exporter.convert_trajectory(trajectory)

hand_commander.run_named_trajectory(converted_trajectory)


# Or we could repopulate the warehouse with the exported states:

state_exporter = SrRobotStateExporter(warehouse_states)
state_exporter.repopulate_warehouse()
