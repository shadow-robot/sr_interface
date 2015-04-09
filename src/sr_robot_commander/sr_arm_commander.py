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


import threading
import rospy
from sr_robot_commander.sr_robot_commander import SrRobotCommander
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

    def move_thought_joint_states(self, joint_states_list):
        """
        Moves robot thought all joint states with specified timeouts
        @param joint_states_list - list of dictionaries of joint states or tuples with joints state dictionary and
        duration in millisecond  for transition between previous state and current (by default duration is 1 second)
        e.g. [{"joint1": 10, "joint2": 45}, ({"joint1": 20, "joint2": 10], 2000), {"joint1": 10, "joint2": 45}]
        """
        return self._move_thought_joint_states(joint_states_list)

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
        It will store the received joint velocity and effort information in two dictionaries
        @param joint_state - the message containing the joints data.
        """
        with self._joint_states_lock:
            self._joints_position = {n:p for n,p in zip(joint_state.name, joint_state.position)}
            self._joints_velocity = {n:v for n,v in zip(joint_state.name, joint_state.velocity)}
