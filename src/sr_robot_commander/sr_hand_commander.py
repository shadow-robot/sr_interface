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

import rospy

from sr_robot_commander import SrRobotCommander
from sr_robot_msgs.srv import ForceController
from sr_hand.tactile_receiver import TactileReceiver


class SrHandCommander(SrRobotCommander):
    """
    Commander class for hand
    """

    __set_force_srv = {}

    def __init__(self, name="right_hand", prefix="rh"):
        """
        Initialize object
        @param name - name of the MoveIt group
        @param prefix - prefix used for the tactiles and max_force
        """
        super(SrHandCommander, self).__init__(name)
        self._tactiles = TactileReceiver(prefix)

        # appends trailing slash if necessary
        self._topic_prefix = prefix
        if self._topic_prefix and not self._topic_prefix.endswith("/"):
            self._topic_prefix += "/"

    def get_joints_effort(self):
        """
        Returns joints effort
        @return - dictionary with joints efforts
        """
        return self._get_joints_effort()

    def set_max_force(self, joint_name, value):
        """
        Set maximum force for hand
        @param value - maximum force value
        """

        # This is for a beta version of our firmware.
        # It uses the motor I and Imax to set a max effort.
        if not self.__set_force_srv.get(joint_name):
            service_name = "realtime_loop/" + self._topic_prefix + \
                           "change_force_PID_"+joint_name.upper()
            self.__set_force_srv[joint_name] = \
                rospy.ServiceProxy(service_name,
                                   ForceController)

        # get the current settings for the motor
        motor_settings = None
        try:
            motor_settings = rospy.get_param(self._topic_prefix +
                                             joint_name.lower() + "/pid")
        except KeyError, e:
            rospy.logerr("Couldn't get the motor parameters for joint " +
                         joint_name + " -> " + str(e))

        # imax is used for max force for now.
        motor_settings["imax"] = value

        try:
            # reorder parameters in the expected order since names don't match:
            self.__set_force_srv[joint_name](motor_settings["max_pwm"],
                                             motor_settings["sg_left"],
                                             motor_settings["sg_right"],
                                             motor_settings["f"],
                                             motor_settings["p"],
                                             motor_settings["i"],
                                             motor_settings["d"],
                                             motor_settings["imax"],
                                             motor_settings["deadband"],
                                             motor_settings["sign"])
        except rospy.ServiceException, e:
            rospy.logerr("Couldn't set the max force for joint " +
                         joint_name + ": " + str(e))

    def get_tactile_type(self):
        """
        Returns a string indicating the type of tactile sensors present.
        Possible values are: PST, biotac, UBI0.
        """
        return self._tactiles.get_tactile_type()

    def get_tactile_state(self):
        """
        Returns an object containing tactile data. The structure of the
        data is different for every tactile_type.
        """
        return self._tactiles.get_tactile_state()
