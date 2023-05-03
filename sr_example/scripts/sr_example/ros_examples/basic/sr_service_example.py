#!/usr/bin/env python3
#
# Copyright 2023 Shadow Robot Company Ltd.
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

"""
This is a simple service client example, calling the service /sr_example/GetJointStates
and printing out the data in a per-joint basis.
To view the joint_states of a right arm in degrees, run this script and then run:
rosservice call /get_joint_states "robot_name: 'right_arm' angle_units: 'degrees'"
"""

from __future__ import absolute_import
from typing import Dict, List
from math import degrees
import rospy
from sr_example.srv import GetJointStates, GetJointStatesResponse
from sensor_msgs.msg import JointState
from sr_example.msg import StringFloat


class ServiceExample():
    def __init__(self):
        """
            Initialise the node and the service to get the joint states from the robot
            in degrees or radians and print them out.
        """
        self.service_name = "/get_joint_states"
        self.service = rospy.Service(self.service_name, GetJointStates, self.handle_service)
        try:
            self.joint_state_msg = rospy.wait_for_message("/joint_states", JointState)
        except rospy.ROSException as err:
            raise Exception(f"Could not find /joint_states topic: {err}")
        self.avaliable_robot_names = self.confirm_robots_available()
        rospy.loginfo("Service %s is ready", self.service_name)

    def confirm_robots_available(self) -> List[str]:
        """
            Confirm that the robots are available by checking the joint states topic.
            @return: a list of the robot names
        """
        avaliable_robot_names: List[str] = []
        robot_names = {"right_arm": "ra_",
                       "left_arm": "la_",
                       "right_hand": "rh_",
                       "left_hand":  "lh_"}
        for robot_name in robot_names.keys():
            if any(robot_names[robot_name] in joint_name for joint_name in self.joint_state_msg.name):
                avaliable_robot_names.append(robot_name)
        return avaliable_robot_names

    def get_joint_states(self, robot_name: str, angle_units: str) -> Dict[str, float]:
        """
            Get the joint states from the robot in degrees or radians.
            @param robot_name: the name of the robot
            @param angle_units: the units of the joint states, either degrees or radians
            @return: a list of JointState messages containing the joint names and joint positions
        """
        joint_states: List[StringFloat] = []
        # get the prefix of the robot name which is the first character of the string
        # and the first character before the underscore of the robot name plus an underscore
        robot_prefix = f"{robot_name[0]}{robot_name.split('_')[-1][0]}_"
        # zip the joint names and joint positions together
        for joint_name, joint_position in zip(self.joint_state_msg.name, self.joint_state_msg.position):
            # if the joint name is in the robot name, add it to the joint states
            if robot_prefix in joint_name:
                # convert the joint position to degrees if the angle units are degrees
                if angle_units == "degrees":
                    joint_position = degrees(joint_position)
                joint_states.append(StringFloat(joint_name, joint_position))
        return joint_states

    def handle_service(self, req: GetJointStates) -> GetJointStatesResponse:
        """
            Handle the service request by retriving the joint states using the robot name
            and angle units and printing them out.
            @param req: the service request
            @return: the service response containing the joint states
        """
        if req.angle_units not in ["degrees", "radians"]:
            raise Exception("Invalid angle units, must be degrees or radians")
        if req.robot_name not in self.avaliable_robot_names:
            raise Exception(f"Invalid robot name, must be one of {self.avaliable_robot_names}")
        rospy.loginfo(f"Received request to get joint states for {req.robot_name} in {req.angle_units}...")
        joint_states = self.get_joint_states(req.robot_name, req.angle_units)
        rospy.loginfo(f"Joint states for {req.robot_name} in {req.angle_units} found!")
        return GetJointStatesResponse(joint_states)


if __name__ == "__main__":
    rospy.init_node("sr_service_example")
    ServiceExample()
    rospy.spin()
