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


import rospy
from exported_states import warehouse_states
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

# It's assumed that a module containing states and named named exported_states.py has already
# been exported and is found somewhere on the path (e.g. in the same directory as this script).
# Let's assume it contains two states, 'state_1' and 'state_2'

# Now we have a dictionary of states called warehouse_states, e.g.
# warehouse_states = {
#   'state_1': {
#      'joint_0': 0.00,
#      'joint_1': 0.00
#   },
#   'state_2': {
#      'joint_0': 0.00,
#      'joint_1': 0.00
#   }
# }

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
