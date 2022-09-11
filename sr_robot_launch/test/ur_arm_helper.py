#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
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

import rospy
from ur_dashboard_msgs.srv import GetSafetyMode, GetProgramState, GetRobotMode, Load, IsProgramRunning
from std_srvs.srv import Trigger


def resend_program():
    try:
        service_call = rospy.ServiceProxy('/la_sr_ur_robot_hw/dashboard/program_running', IsProgramRunning)
        response_left = service_call()
        if response_left.program_running is False:
            print("Resending program for left arm")
            service_call = rospy.ServiceProxy('/la_sr_ur_robot_hw/resend_robot_program', Trigger)
            response_left = service_call()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed for left arm %s", e)
        raise
    try:
        service_call = rospy.ServiceProxy('/ra_sr_ur_robot_hw/dashboard/program_running', IsProgramRunning)
        response_right = service_call()
        if response_right.program_running is False:
            print("Resending program for right arm")
            service_call = rospy.ServiceProxy('/ra_sr_ur_robot_hw/resend_robot_program', Trigger)
            response_right = service_call()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed for right arm %s", e)
        raise


if __name__ == "__main__":
    rospy.init_node("ur_arm_test_helper")
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        resend_program()
        r.sleep()
