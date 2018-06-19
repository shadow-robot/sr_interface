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

from sr_robot_commander import SrRobotCommander, SrRobotCommanderException
from sr_robot_msgs.srv import ForceController
from sr_hand.tactile_receiver import TactileReceiver
from sr_utilities.hand_finder import HandFinder
from sys import exit


class SrHandCommander(SrRobotCommander):
    """
    Commander class for hand
    """

    __set_force_srv = {}

    def __init__(self, name=None, prefix=None, hand_parameters=None, hand_serial=None, hand_number=0):
        """
        Initialize the hand commander, using either a name + prefix, or the parameters returned by the hand finder.
        @param name - name of the MoveIt group
        @param prefix - prefix used for the tactiles and max_force
        @param hand_parameters - from hand_finder.get_hand_parameters(). Will overwrite the name and prefix
        @param hand_serial - which hand are you using
        @param hand_number - which hand in a multi-hand system to use (starting at 0 and sorted by name/serial).
        """

        self._hand_h = False
        if name is None and prefix is None and hand_parameters is None and hand_serial is None:
            hand_finder = HandFinder()
            if hand_finder.hand_e_available():
                name, prefix, hand_serial = hand_finder.get_hand_e(number=hand_number)
            elif hand_finder.hand_h_available():
                self._hand_h = True
                name, prefix, hand_serial = hand_finder.get_hand_h(number=hand_number)
            else:
                rospy.logfatal("No hands found and no information given to define hand commander.")
                raise SrRobotCommanderException("No hand found.")

        elif hand_parameters is not None:
            # extracting the name and prefix from the hand finder parameters
            if len(hand_parameters.mapping) is 0:
                rospy.logfatal("No hand detected")
                raise SrRobotCommanderException("No hand found.")

            hand_mapping = hand_parameters.mapping[hand_serial]
            prefix = hand_parameters.joint_prefix[hand_serial]

            if name is None:
                if hand_mapping == 'rh':
                    name = "right_hand"
                else:
                    name = "left_hand"
        else:
            if name is None:
                name = "right_hand"
            if prefix is None:
                prefix = "rh_"

        super(SrHandCommander, self).__init__(name)

        if not self._hand_h:
            self._tactiles = TactileReceiver(prefix)

        # appends trailing slash if necessary
        self._topic_prefix = prefix
        if self._topic_prefix and not self._topic_prefix.endswith("/"):
            self._topic_prefix += "/"

        self._hand_serial = hand_serial

    def get_hand_serial(self):
            return self._hand_serial

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
        joint_name = self._strip_prefix(joint_name)

        # This is for a beta version of our firmware.
        # It uses the motor I and Imax to set a max effort.
        if not self.__set_force_srv.get(joint_name):
            service_name = "sr_hand_robot/" + self._topic_prefix + \
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

    def _strip_prefix(self, joint_name):
        """
        Strips the prefix from the joint name (e.g. rh_ffj3 -> ffj3) if present, returns the joint name otherwise.

        We know that all joint names for the shadow hands are 4 char long. So we only keep the last 4 chars.

        @param joint_name the joint name
        @return stripped joint name
        """
        return joint_name[-4:]

    def attach_object(self, object_name):
        self._move_group_commander.attach_object(object_name)

    def detach_object(self, object_name):
        self._move_group_commander.detach_object(object_name)
