#!/usr/bin/env python

from sys import argv

import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from sr_arm_commander import SrArmCommander
from sr_hand_commander import SrHandCommander
from sr_robot_commander import SrRobotCommander
from sr_utilities.hand_finder import HandFinder


class SrStateSaverUnsafe(object):
    def __init__(self, name, hand_or_arm="both", hand_h=False):

        self.__save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self.__name = name

        self.__hand_h = hand_h

        if hand_or_arm == "arm":
            self.__commander = SrArmCommander()

        elif hand_or_arm == 'hand':
            if self.__hand_h:
                self.__commander = SrRobotCommander("hand_h", prefix="H1_")
            else:
                hand_finder = HandFinder()

                hand_parameters = hand_finder.get_hand_parameters()
                hand_serial = hand_parameters.mapping.keys()[0]

                self.__commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)
        else:
            self.__arm_commander = SrArmCommander()

            if self.__hand_h:
                self.__hand_commander = SrRobotCommander("hand_h", prefix="H1_")
            else:
                hand_finder = HandFinder()

                hand_parameters = hand_finder.get_hand_parameters()
                hand_serial = hand_parameters.mapping.keys()[0]

                self.__hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                                        hand_serial=hand_serial)
        self.__hand_or_arm = hand_or_arm

        rs = RobotState()

        current_dict = {}

        if self.__hand_or_arm == "both":
            current_dict = self.__arm_commander.get_robot_state_bounded()
            robot_name = self.__arm_commander.get_robot_name()
        elif self.__hand_or_arm == "arm":
            current_dict = self.__commander.get_current_state_bounded()
            robot_name = self.__commander.get_robot_name()
        elif self.__hand_or_arm == "hand":
            current_dict = self.__commander.get_current_state_bounded()
            robot_name = self.__commander.get_robot_name()
        else:
            rospy.logfatal("Unknown save type")
            exit(-1)

        rospy.loginfo(current_dict)
        rs.joint_state = JointState()
        rs.joint_state.name = current_dict.keys()
        rs.joint_state.position = current_dict.values()
        self.__save(self.__name, robot_name, rs)
