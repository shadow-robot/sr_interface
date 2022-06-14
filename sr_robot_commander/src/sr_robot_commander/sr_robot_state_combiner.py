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
        for name, pos in zip(state.state.joint_state.name, state.state.joint_state.position):
            if (remove_arm and all(s not in name for s in self._arm_joints)) \
                    or (not remove_arm and any(s in name for s in self._arm_joints)):
                name.append(name)
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
