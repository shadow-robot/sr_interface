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


import pprint
from copy import deepcopy
import rospy
from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListState
from moveit_msgs.msg import RobotState


class SrRobotStateExporter:
    def __init__(self, start_dictionary=None):
        if start_dictionary is None:
            start_dictionary = {}
        self._get_state = rospy.ServiceProxy("/get_robot_state", GetState)
        self._has_state = rospy.ServiceProxy("/has_robot_state", HasState)
        self._list_states = rospy.ServiceProxy("/list_robot_state", ListState)
        self._save_state = rospy.ServiceProxy("/save_robot_state", SaveState)
        self._dictionary = deepcopy(start_dictionary)

    def extract_list(self, list_of_states):
        for name in list_of_states:
            self.extract_one_state(name)

    def extract_one_state(self, name):
        if self._has_state(name, '').exists:
            state = self._get_state(name, '').state.joint_state
            names = state.name
            position = state.position
            self._dictionary[name] = {name: position[n] for n, name in enumerate(names)}
        else:
            rospy.logerr(f"State {name} not present in the warehouse.")

    def extract_from_trajectory(self, dictionary_trajectory):
        for entry in dictionary_trajectory:
            if 'name' in entry:
                self.extract_one_state(entry['name'])

    def extract_all(self):
        for state in self._list_states("", "").states:
            self.extract_one_state(state)

    def output_module(self, file_name):
        pretty_printer = pprint.PrettyPrinter()
        with open(file_name, "w", encoding="utf-8") as output:
            output.write('warehouse_states = %s\n' % pretty_printer.pformat(self._dictionary))

    def convert_trajectory(self, named_trajectory):
        new_trajectory = []
        for entry in named_trajectory:
            new_entry = deepcopy(entry)
            if 'name' in new_entry:
                if new_entry['name'] in self._dictionary:
                    new_entry['joint_angles'] = self._dictionary[new_entry['name']]
                    new_entry.pop('name')
                else:
                    rospy.logwarn(f"Entry named {new_entry['name']} not present in dictionary. Not replacing.")
            new_trajectory.append(new_entry)
        return new_trajectory

    def repopulate_warehouse(self):
        for name in self._dictionary:
            if self._has_state(name, '').exists:
                rospy.logwarn(f"State named {name} already in warehouse, not re-adding.")
            else:
                state = RobotState()
                state.joint_state.name = self._dictionary[name].keys()
                state.joint_state.position = self._dictionary[name].values()
                self._save_state(name, '', state)
