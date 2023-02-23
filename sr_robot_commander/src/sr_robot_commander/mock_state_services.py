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


def mock_has_state_callback(_req):
    return True


def mock_list_state_callback(_req):
    states = {"state1", "state2"}
    return states


def mock_save_state_callback(req):
    return bool("state1" in req.name)


if __name__ == "__main__":
    rospy.init_node('mock_services', anonymous=True)
    service_1 = rospy.Service('/get_robot_state', GetState, mock_get_state_callback)
    service_2 = rospy.Service('/has_robot_state', HasState, mock_has_state_callback)
    service_3 = rospy.Service('/list_robot_state', ListState, mock_list_state_callback)
    service_4 = rospy.Service('/save_robot_state', SaveState, mock_save_state_callback)
    rospy.spin()
