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
    def __init__(self, name, hand_or_arm="both"):

        self.__save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self.__name = name

        if hand_or_arm == "arm":
            self.__commander = SrArmCommander()

        elif hand_or_arm == 'hand':
            self.__commander = SrHandCommander()

        else:
            self.__arm_commander = SrArmCommander()
            self.__hand_commander = SrHandCommander()

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
