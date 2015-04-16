#!/usr/bin/python

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
import threading
from sr_robot_commander import SrRobotCommander
from sensor_msgs.msg import JointState


class SrArmCommander(SrRobotCommander):
    """
    Commander class for arm
    """

    def __init__(self, name="right_arm"):
        """
        Initialize object
        @param name - name of the MoveIt group
        """
        super(SrArmCommander, self).__init__(name)

        self._joint_states_lock = threading.Lock()
        self._joint_states_listener = rospy.Subscriber("joint_states", JointState, self._joint_states_callback)
        self._joints_position = {}
        self._joints_velocity = {}
        threading.Thread(None, rospy.spin)

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):
        """
        Specify a target position for the end-effector and moves to it.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait - should method wait for movement end or not
        """
        self._move_to_position_target(xyz, end_effector_link, wait_result=wait)

    def run_joint_trajectory(self, joint_trajectory):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents trajectory of the joints which would be
        executed.
        """
        return self._run_joint_trajectory(joint_trajectory)

    def get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        with self._joint_states_lock:
            return self._joints_position

    def get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        with self._joint_states_lock:
            return self._joints_velocity

    def _joint_states_callback(self, joint_state):
        """
        The callback function for the topic joint_states.
        It will store the received joint position and velocity information in two dictionaries
        @param joint_state - the message containing the joints data.
        """
        with self._joint_states_lock:
            self._joints_position = {n: p for n, p in zip(joint_state.name, joint_state.position)}
            self._joints_velocity = {n: v for n, v in zip(joint_state.name, joint_state.velocity)}
