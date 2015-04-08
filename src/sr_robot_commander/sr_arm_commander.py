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

    def __init__(self, name):
        """
        Initialize object
        @param name - name of the MoveIt group
        """
        super(SrArmCommander, self).__init__(name)

    def set_joint_value_target(self, joint_states):
        """
        Set target of the robot's links
        @param joint_states - dictionary with joint name and value
        """
        self._set_joint_value_target(joint_states)

    def set_position_target(self, xyz, end_effector_link=""):
        """
        Specify a target position for the end-effector.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        """
        self._set_position_target(xyz, end_effector_link)

    def get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        return self._get_joints_position()

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        return self._get_joints_velocity()
