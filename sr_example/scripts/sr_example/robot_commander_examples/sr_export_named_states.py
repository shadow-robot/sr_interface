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


from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

# The following example shows how to use the warehouse state exporter to save
# robot states from the warehouse as plain text python files. States can be
# extracted one by one, as a list of state names, or all states from a named
# trajectory can be extracted together.

# Once states have been extracted, the resulting dictionary can be exported to
#  a specified file location. The generated file can then be used as a modules
# for importing into subsequent scripts.

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
