#!/usr/bin/env python3
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
from sys import argv

import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_utilities.hand_finder import HandFinder
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointTrajectoryControllerState
from threading import Lock


class SrStateSaverUnsafe(object):
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

        self._save_hand = (hand_or_arm == "hand" or hand_or_arm == "both")
        self._save_arm = (hand_or_arm == "arm" or hand_or_arm == "both")
        self._save_bimanual = (side == 'bimanual')

        if save_target:
            rospy.loginfo("Saving targets instead of current values")
            self._mutex = Lock()
            self._target_values = dict()
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

        self.save_state(current_dict, robot_name)

    def _target_cb(self, data):
        self._mutex.acquire()
        for n, joint in enumerate(data.joint_names):
            self._target_values[joint] = data.desired.positions[n]
        self._mutex.release()

    def save_state(self, current_dict, robot_name):
        rs = RobotState()
        rospy.loginfo(current_dict)
        rs.joint_state = JointState()
        rs.joint_state.name = current_dict.keys()
        rs.joint_state.position = current_dict.values()
        rospy.logwarn(rs)
        self._save(self._name, robot_name, rs)
