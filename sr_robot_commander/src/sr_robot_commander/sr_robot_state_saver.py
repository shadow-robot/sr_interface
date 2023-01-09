#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
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

from threading import Lock
import sys
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander


class SrStateSaverUnsafe:
    def __init__(self, name, hand_or_arm="both", side="right", save_target=False):

        self._save = rospy.ServiceProxy(
            'save_robot_state', SaveState)

        self._name = name
        self._save_target = save_target

        if name is None or name == '':
            raise ValueError("Cannot save with empty name.")

        prefix = ""
        double_error = []
        if hand_or_arm == "arm":
            if side != "bimanual":
                prefix = "ra" if side == "right" else "la"
                self._commander = SrArmCommander(side + '_arm')
                if not self._commander.arm_found():
                    double_error.append("Group " + side + "_arm not found.")
            else:
                self._commander = SrRobotCommander('two_arms')
        elif hand_or_arm == 'hand':
            if side != "bimanual":
                prefix = "rh" if side == "right" else "lh"
                self._commander = SrHandCommander(side + '_hand')
            else:
                self._commander = SrRobotCommander('two_hands')
        elif hand_or_arm == 'both':
            if side != "bimanual":
                self._commander = SrRobotCommander(side + '_arm_and_hand')
            else:
                self._commander = SrRobotCommander('two_arms_and_hands')

        if len(double_error) != 0:
            raise ValueError(" ".join(double_error))

        self._save_hand = (hand_or_arm in ("hand", "both"))
        self._save_arm = (hand_or_arm in ("arm", "both"))
        self._save_bimanual = (side == 'bimanual')

        if save_target:
            rospy.loginfo("Saving targets instead of current values")
            self._mutex = Lock()
            self._target_values = {}
            if self._save_hand:
                self._hand_subscriber = rospy.Subscriber("/" + prefix + "_trajectory_controller/state",
                                                         JointTrajectoryControllerState, self._target_cb)
            if self._save_arm:
                self._arm_subscriber = rospy.Subscriber("/" + prefix + "_trajectory_controller/state",
                                                        JointTrajectoryControllerState, self._target_cb)
            if self._save_bimanual:
                if self._save_hand:
                    self.r_hand_subscriber = rospy.Subscriber("/ra_trajectory_controller/state",
                                                              JointTrajectoryControllerState, self._target_cb)
                    self.l_hand_subscriber = rospy.Subscriber("/la_trajectory_controller/state",
                                                              JointTrajectoryControllerState, self._target_cb)
                if self._save_arm:
                    self.r_arm_subscriber = rospy.Subscriber("/ra_trajectory_controller/state",
                                                             JointTrajectoryControllerState, self._target_cb)
                    self.l_arm_subscriber = rospy.Subscriber("/la_trajectory_controller/state",
                                                             JointTrajectoryControllerState, self._target_cb)

        current_dict = {}
        rospy.loginfo("Getting current")
        current_dict = self._commander.get_current_state()
        robot_name = self._commander.get_robot_name()

        if self._save_target:
            rospy.loginfo("Getting targets")
            waiting_for_targets = True
            while waiting_for_targets and not rospy.is_shutdown():
                self._mutex.acquire()  # pylint: disable=R1732
                waiting_for_targets = False
                for joint in current_dict:
                    if joint in self._target_values:
                        current_dict[joint] = self._target_values[joint]
                    else:
                        waiting_for_targets = True
                        rospy.loginfo(f"Still waiting for {joint} target")
                if waiting_for_targets:
                    rospy.loginfo(self._target_values)
                    rospy.sleep(1)
        if rospy.is_shutdown():
            sys.exit()

        self.save_state(current_dict, robot_name)

    def _target_cb(self, data):
        self._mutex.acquire()  # pylint: disable=R1732
        for joint_index, joint in enumerate(data.joint_names):
            self._target_values[joint] = data.desired.positions[joint_index]

    def save_state(self, current_dict, robot_name):
        robot_state = RobotState()
        rospy.loginfo(current_dict)
        robot_state.joint_state = JointState()
        robot_state.joint_state.name = current_dict.keys()
        robot_state.joint_state.position = current_dict.values()
        rospy.logwarn(robot_state)
        self._save(self._name, robot_name, robot_state)
