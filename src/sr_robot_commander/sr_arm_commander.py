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


from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory

import threading
import rospy
from sensor_msgs.msg import JointState


class SrRobotCommander(object):
    """
    Base class for hand and arm commanders
    """

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object
        @param name - name of the MoveIt group
        """
        self._move_group_commander = MoveGroupCommander(name)

    def move_to_joint_value_target(self, joint_states, wait_result=True):
        """
        Set target of the robot's links and moves to it
        @param joint_states - dictionary with joint name and value
        @param wait_result - should method wait for movement end or not
        """
        self._move_group_commander.set_joint_value_target(joint_states)
        self._move_group_commander.go(wait=wait_result)

    def _run_joint_trajectory(self, joint_trajectory):
        """
        Moves robot through all joint states with specified timeouts
        @param joint_trajectory - JointTrajectory class object. Represents trajectory of the joints which would be
        executed.
        """
        plan = RobotTrajectory()
        plan.joint_trajectory = joint_trajectory
        self._move_group_commander.execute(plan)

    def _move_to_position_target(self, xyz, end_effector_link="", wait_result=True):
        """
        Specify a target position for the end-effector and moves to it
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        @param wait_result - should method wait for movement end or not
        """
        self._move_group_commander.set_position_target(xyz, end_effector_link)
        self._move_group_commander.go(wait=wait_result)


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
            self._joints_position = {n:p for n,p in zip(joint_state.name, joint_state.position)}
            self._joints_velocity = {n:v for n,v in zip(joint_state.name, joint_state.velocity)}


if __name__ == "__main__":
    rospy.init_node("basic_example", anonymous=True)

    arm = SrArmCommander()

    arm.move_to_position_target([-0.028412, 0.17665, 0.85672])
    rospy.sleep(rospy.Duration(3))

    arm.move_to_position_target([0.25527, 0.86682, 0.5426])
    rospy.sleep(rospy.Duration(3))

    print("Arm joints position\n" + str(arm.get_joints_position()) + "\n")