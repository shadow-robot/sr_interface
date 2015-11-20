#!/usr/bin/env python

from sys import argv

import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


class SrCrudeGraspSaver:
    def __init__(self, name, hand_or_arm="both"):

        self.__save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self.__name = name
        self.__arm_commander = SrArmCommander()

        hand_finder = HandFinder()

        hand_parameters = hand_finder.get_hand_parameters()
        hand_serial = hand_parameters.mapping.keys()[0]

        self.__hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                                hand_serial=hand_serial)

        self.__hand_or_arm = hand_or_arm

    def __save_out(self):
        rs = RobotState()

        current_dict = {}

        if self.__hand_or_arm == "both":
            current_dict = self.__arm_commander.get_robot_state_bounded()
        elif self.__hand_or_arm == "arm":
            current_dict = self.__arm_commander.get_current_pose_bounded()
        elif self.__hand_or_arm == "hand":
            current_dict = self.__hand_commander.get_current_pose_bounded()
        else:
            rospy.logfatal("Unknown save type")
            exit(-1)

        robot_name = self.__arm_commander.get_robot_name()

        rospy.loginfo(current_dict)
        rs.joint_state = JointState()
        rs.joint_state.name = current_dict.keys()
        rs.joint_state.position = current_dict.values()
        self.__save(self.__name, robot_name, rs)

    def spin(self):
        self.__save_out()


if "__main__" == __name__:
    rospy.init_node("grasp_saver")
    if len(argv) <= 1 or "" == argv[1]:
        rospy.logerr("You didn't enter a name.")
        exit(-1)

    which = 'both'

    if len(argv) > 2:
        which = argv[2]

    gs = SrCrudeGraspSaver(argv[1], which)
    gs.spin()
