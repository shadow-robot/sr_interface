#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2015, 2022-2023 belongs to Shadow Robot Company Ltd.
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
from sr_robot_msgs.srv import ForceController
from sr_hand.tactile_receiver import TactileReceiver
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_robot_commander import SrRobotCommander, SrRobotCommanderException


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
            if len(hand_parameters.mapping) == 0:
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

        super().__init__(name)

        if not self._hand_h:
            self._tactiles = TactileReceiver(prefix)

        # appends trailing slash if necessary
        self._topic_prefix = prefix
        if self._topic_prefix and self._topic_prefix.endswith("_"):
            self._topic_prefix = self._topic_prefix[:-1]  # Remove trailing _

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
        joint_name = self.strip_prefix(joint_name)

        if not self.__set_force_srv.get(joint_name):
            service_name = "sr_hand_robot/" + self._topic_prefix + \
                           "change_force_PID_" + joint_name.upper()
            self.__set_force_srv[joint_name] = \
                rospy.ServiceProxy(service_name,
                                   ForceController)

        # get the current settings for the motor
        motor_settings = None
        try:
            motor_settings = rospy.get_param(self._topic_prefix +
                                             joint_name.lower() + "/pid")
        except KeyError as exception:
            rospy.logerr("Couldn't get the motor parameters for joint " +
                         joint_name + " -> " + str(exception))

        motor_settings["torque_limit"] = value

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
                                             motor_settings["sign"],
                                             motor_settings["torque_limit"],
                                             motor_settings["torque_limiter_gain"])
        except rospy.ServiceException as exception:
            rospy.logerr("Couldn't set the max force for joint " +
                         joint_name + ": " + str(exception))

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

    @staticmethod
    def strip_prefix(joint_name):
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
