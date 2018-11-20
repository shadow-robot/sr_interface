#!/usr/bin/env python

# Copyright 2018 Shadow Robot Company Ltd.
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
from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_msgs.srv import ListRobotStatesInWarehouse as ListState


def mock_get_state_callback(req):
    resp = RobotState()
    if req.name == "state1":
        resp.joint_state.name = ["joint_test1", "joint_test2"]
        resp.joint_state.position = [1.0, 2.0]
    if req.name == "state2":
        resp.joint_state.name = ["joint_test3", "joint_test4"]
        resp.joint_state.position = [3.0, 4.0]
    return resp


def mock_has_state_callback(req):
    return True


def mock_list_state_callback(req):
    states = {"state1", "state2"}
    return states


def mock_save_state_callback(req):
    if "state1" in req.name:
        return True
    else:
        return False

if __name__ == "__main__":
    rospy.init_node('mock_services', anonymous=True)
    s1 = rospy.Service('/get_robot_state', GetState, mock_get_state_callback)
    s2 = rospy.Service('/has_robot_state', HasState, mock_has_state_callback)
    s3 = rospy.Service('/list_robot_states', ListState, mock_list_state_callback)
    s4 = rospy.Service('/save_robot_state', SaveState, mock_save_state_callback)
    rospy.spin()
