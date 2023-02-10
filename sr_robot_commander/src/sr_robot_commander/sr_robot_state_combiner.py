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
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import GetRobotStateFromWarehouseResponse as GetStateResp


class SrRobotStateCombiner:

    _arm_joints = ['shoulder_pan_joint', 'elbow_joint', 'shoulder_lift_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'WRJ2', 'WRJ1']

    def __init__(self, arm_state_name, hand_state_name, new_state_name, robot_name="ur10srh"):
        self._save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self._read = rospy.ServiceProxy(
            'get_robot_state', GetState)

        self._arm_state_name = arm_state_name
        self._hand_state_name = hand_state_name
        self._new_state_name = new_state_name
        self._robot_name = robot_name

    def _filter_joints(self, state, remove_arm=True):
        """
        Filters out the joints that we don't want and their positions
        :param state: initial state
        :param remove_arm: If True we remove the arm joints, if False we remove the non-arm joints (hand joints)
        :return: the filtered state
        """
        name = []
        position = []
        for name_index, pos in zip(state.state.joint_state.name, state.state.joint_state.position):
            if (remove_arm and all(s not in name_index for s in self._arm_joints)) \
                    or (not remove_arm and any(s in name_index for s in self._arm_joints)):
                name.append(name_index)
                position.append(pos)
        state.state.joint_state.name = name
        state.state.joint_state.position = position
        return state

    def combine(self):
        if self._arm_state_name == "NONE":
            arm_state = GetStateResp()
        else:
            arm_state = self._read(self._arm_state_name, self._robot_name)

        if self._hand_state_name == "NONE":
            hand_state = GetStateResp()
        else:
            hand_state = self._read(self._hand_state_name, self._robot_name)

        arm_state = self._filter_joints(arm_state, False)
        hand_state = self._filter_joints(hand_state, True)

        combined_state = arm_state
        combined_state.state.joint_state.name += hand_state.state.joint_state.name
        combined_state.state.joint_state.position += hand_state.state.joint_state.position

        self._save(self._new_state_name, self._robot_name, combined_state.state)
