#!/usr/bin/python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.


from sr_robot_commander.sr_robot_commander import SrRobotCommander


class SrArmCommander(SrRobotCommander):
    """
    Commander class for arm
    """

    def __init__(self, name="right_arm"):
        """
        Initialize object
        @param name - name of the MoveIt group
        """
        super(SrArmCommander, self).__init__(name)

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_to_position_target(xyz, end_effector_link, wait_result=wait)

    def move_thought_joint_states(self, joint_states_list):
        """
        Moves robot thought all joint states with specified timeouts
        @param joint_states_list - list of dictionaries of joint states or tuples with joints state dictionary and
        timeout in millisecond  to stay in this state
        e,g [ {"joint1": 10, "joint2": 45}, ({"joint1": 20, "joint2": 10], 2000), {"joint1": 10, "joint2": 45}]
        """
        return self._move_thought_joint_states(joint_states_list)