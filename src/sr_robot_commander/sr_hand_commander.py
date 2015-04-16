#!/usr/bin/env python

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

from sr_robot_commander import SrRobotCommander
from sr_hand.shadowhand_ros import ShadowHand_ROS


class SrHandCommander(SrRobotCommander):
    """
    Commander class for hand
    """

    def __init__(self, name="right_hand"):
        """
        Initialize object
        @param name - name of the MoveIt group
        """
        super(SrHandCommander, self).__init__(name)
        self._hand = ShadowHand_ROS()
        self._initialize_joint_prefix(name)

    def get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        return self._fix_joints_names(self._hand.read_all_current_positions())

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        return dict(self._hand.read_all_current_velocities())

    def get_joints_effort(self):
        """
        Returns a dictionary with the effort of each joint. Currently in ADC units, as no calibration is performed on
        the strain gauges.
        """
        return dict(self._hand.read_all_current_efforts())

    def set_max_force(self, value):
        """
        Set maximum force for hand
        @param value - maximum force value
        """
        raise NotImplementedError()

    def get_tactile_type(self):
        """
        Returns a string indicating the type of tactile sensors present. Possible values are: PST, biotac, UBI0 .
        """
        raise NotImplementedError()

    def get_tactile_state(self):
        """
        Returns an object containing tactile data. The structure of the data is different for every tactile_type .
        """
        raise NotImplementedError()

    def _initialize_joint_prefix(self, name):
        """
        Prefix which would be added to every joint name returned from ShadowHand_ROS() object.
        This functionality need to be removed in the future releases
        @param name - hand name
        """
        self._joint_prefix = ""
        if name is not None:
            for word in name.split("_"):
                self._joint_prefix += word[0]
            self._joint_prefix += "_"

    def _fix_joints_names(self, joints_dictionary):
        """
        Correction of the joint names returned by ShadowHand_ROS() object.
        This functionality need to be removed in the future releases
        @param joints_dictionary - input dictionary
        @return dictionary with fixed key names
        """
        return dict((self._joint_prefix + key, value) for (key, value) in joints_dictionary.items())
