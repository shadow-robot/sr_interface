#!/usr/bin/env python

from sys import argv

import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

"""
***DISCLAIMER***

This script provides a command line interface to save the current pose of the robot (made of an arm and hand).
Poses can be composed of either hand joints/arm joints/all joints. This can be useful for making smooth
movements where the hand and arm should move synchronously. However, there are no safeties! In particular,
if a hand and arm aren't both present , its behaviour is undefined. Likewise if more than one hand/arm are
present. There is also no checking to prevent overwriting existing database entries/otherwise damaging the
robot_states database. It is included here as it is a useful tool, but it should be considered a work in
progress and used with care.

You have been warned :)


To use this script:

rosrun sr_robot_commander grasp_saver_unsafe.py NAME_TO_SAVE WHAT_TO_SAVE

NAME_TO_SAVE is the name that will be used for the state in the database.

N.B. IF A STATE ALREADY EXISTS WITH THAT NAME IT WILL BE OVERWRITTEN WITH NO WARNING!

WHAT_TO_SAVE specifies which joints will be included. It can be "arm","hand" or "both".

If WHAT_TO_SAVE is omitted, it defaults to "both".
"""


class SrGraspSaverUnsafe(object):
    def __init__(self, name, hand_or_arm="both"):

        self.__save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self.__name = name + "_" + hand_or_arm

        if hand_or_arm == "arm":
            self.__commander = SrArmCommander()

        elif hand_or_arm == 'hand':
            hand_finder = HandFinder()

            hand_parameters = hand_finder.get_hand_parameters()
            hand_serial = hand_parameters.mapping.keys()[0]

            self.__commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)
        else:
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

    def spin(self):
        self.__save_out()


if "__main__" == __name__:
    rospy.init_node("grasp_saver")
    if len(argv) <= 1 or "" == argv[1]:
        rospy.logerr("You didn't enter a name.")
        exit(-1)

    which = 'all'

    if len(argv) > 2:
        which = argv[2]

    if which == "all":
        gs = SrGraspSaverUnsafe(argv[1], "hand")
        gs.spin()
        gs = SrGraspSaverUnsafe(argv[1], "arm")
        gs.spin()
        gs = SrGraspSaverUnsafe(argv[1], "both")
        gs.spin()
    else:
        gs = SrGraspSaverUnsafe(argv[1], which)
        gs.spin()
