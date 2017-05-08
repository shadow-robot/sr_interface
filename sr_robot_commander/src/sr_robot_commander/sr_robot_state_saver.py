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
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointTrajectoryControllerState
from threading import Lock


class SrStateSaverUnsafe(object):
    def __init__(self, name, hand_or_arm="both", hand_h=False, save_target=False):

        self._save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self._name = name
        self._hand_h = hand_h
        self._save_target = save_target

        self._save_hand = (hand_or_arm == "hand" or hand_or_arm == "both")
        self._save_arm = (hand_or_arm == "arm" or hand_or_arm == "both")

        if self._save_hand:
            rospy.loginfo("Saving hand data")
        if self._save_arm:
            rospy.loginfo("Saving arm data")

        rospy.loginfo("Saving for hand %s" % "h" if self._hand_h else "e")

        if save_target:
            rospy.loginfo("Saving targets instead of current values")
            self._mutex = Lock()
            self._target_values = dict()
            if self._save_hand:
                prefix = "H1" if hand_h else "rh"
                self._hand_subscriber = rospy.Subscriber("/" + prefix + "_trajectory_controller/state",
                                                          JointTrajectoryControllerState, self._target_cb)
            if self._save_arm:
                self._arm_subscriber = rospy.Subscriber("/ra_trajectory_controller/state",
                                                         JointTrajectoryControllerState, self._target_cb)

        rospy.loginfo("Creating commanders")
        if hand_or_arm == 'hand':
            if self._hand_h:
                self._commander = SrRobotCommander("hand_h", prefix="H1_")
            else:
                hand_finder = HandFinder()

                hand_parameters = hand_finder.get_hand_parameters()
                hand_serial = hand_parameters.mapping.keys()[0]

                self._commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)
        else:
            if hand_h:
                self._commander = SrRobotCommander("right_arm", prefix="ra_")
            else:
                self._commander = SrArmCommander()

        self._hand_or_arm = hand_or_arm

        rs = RobotState()

        current_dict = {}

        rospy.loginfo("Getting current")

        if self._hand_or_arm == "both":
            current_dict = self._commander.get_robot_state_bounded()
            robot_name = self._commander.get_robot_name()
        elif self._hand_or_arm == "arm" or self._hand_or_arm == "hand":
            current_dict = self._commander.get_current_state_bounded()
            robot_name = self._commander.get_robot_name()
        else:
            rospy.logfatal("Unknown save type")
            exit(-1)

        if self._save_target:
            rospy.loginfo("Getting targets")
            waiting_for_targets = True
            while waiting_for_targets and not rospy.is_shutdown():
                self._mutex.acquire()
                waiting_for_targets = False
                for joint in current_dict:
                    if joint in self._target_values:
                        current_dict[joint] = self._target_values[joint]
                    else:
                        waiting_for_targets = True
                        rospy.loginfo("Still waiting for %s target" % joint)
                self._mutex.release()
                if waiting_for_targets:
                    rospy.loginfo(self._target_values)
                    rospy.sleep(1)
        if rospy.is_shutdown():
            exit(0)

        rospy.loginfo(current_dict)
        rs.joint_state = JointState()
        rs.joint_state.name = current_dict.keys()
        rs.joint_state.position = current_dict.values()
        rospy.logwarn(rs)
        self._save(self._name, robot_name, rs)

    def _target_cb(self, data):
        self._mutex.acquire()
        for n, joint in enumerate(data.joint_names):
            self._target_values[joint] = data.desired.positions[n]
        self._mutex.release()
