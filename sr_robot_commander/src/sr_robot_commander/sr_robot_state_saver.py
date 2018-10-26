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

        if name is None or name == '':
            raise ValueError("Cannot save with empty name.")

        if hand_or_arm == "arm":
            self._commander = SrArmCommander()
            if not self._commander.arm_found():
                raise ValueError("'No arm found.'")

        elif hand_or_arm == 'hand':
            self._commander = SrHandCommander()
        else:
            double_error = []
            try:
                self._commander = SrHandCommander()
            except Exception as e:
                double_error.append(str(e))

            self._arm_commander = SrArmCommander()

            if not self._arm_commander.arm_found():
                double_error.append("'No arm found.'")

            if len(double_error) != 0:
                raise ValueError(" ".join(double_error))


        self._save_hand = (hand_or_arm == "hand" or hand_or_arm == "both")
        self._save_arm = (hand_or_arm == "arm" or hand_or_arm == "both")

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
