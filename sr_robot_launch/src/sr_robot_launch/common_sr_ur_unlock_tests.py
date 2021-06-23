#!/usr/bin/env python

# Copyright 2021 Shadow Robot Company Ltd.
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
import rostest
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase, TestSuite
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsProgramRunning
from sr_robot_launch.sr_ur_arm_unlock import SrUrUnlock
from sr_robot_launch.mock_sr_ur_robot_hw import MockUrRobotHW
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
import sys


class CommonTests:
    def arm_setup(self, side):
        self.press_pedal()
        self.assertTrue(self.get_program_running(side))

    def arm_mock_dashboard_server(self, side):
        self.assertFalse(self.get_program_running(side))

    def e_stop(self, side, release_estop_before_pedal=True):
        self.assertFalse(self.get_program_running(side))
        self.assertFalse(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                         RobotMode.RUNNING)
        self.press_pedal()
        rospy.sleep(0.01)
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.get_program_running(side))
        self.mock_dashboard[side].robot_state.emergency_stop(latch=True)
        self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                        SafetyMode.ROBOT_EMERGENCY_STOP)
        self.assertFalse(self.get_program_running(side))
        self.assertFalse(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                         RobotMode.RUNNING)
        self.mock_dashboard[side].robot_state.emergency_stop(latch=False)
        self.assertFalse(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                         SafetyMode.ROBOT_EMERGENCY_STOP)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                        SafetyMode.NORMAL)
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running(side))

    def fault(self, side):
        self.press_pedal()
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.get_program_running(side))
        self.mock_dashboard[side].robot_state.fault()
        self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                        SafetyMode.FAULT)
        self.assertFalse(self.get_program_running(side))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                        SafetyMode.NORMAL)
        self.assertTrue(self.get_program_running(side))

    def arm_power_cycle(self, side):
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.POWER_OFF)
        self.assertFalse(self.get_program_running(side))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.get_program_running(side))
        self.press_pedal()
        self.assertFalse(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                         RobotMode.RUNNING)
        self.assertFalse(self.get_program_running(side))

    def get_program_running(self, side):
        service_call = rospy.ServiceProxy(self.service_string[side], IsProgramRunning)
        response = service_call()
        return response.program_running

    def press_pedal(self):
        self.sr_ur_arm_unlock.release_or_brake_arm_cb(Bool(True))

    def arm_fault_bimanual(self, sides):
        self.assertFalse(self.get_program_running('right'))
        self.assertFalse(self.get_program_running('left'))
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.POWER_OFF)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
        for side in sides:
            self.mock_dashboard[side].robot_state.fault()
        for side in sides:
            self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                            SafetyMode.FAULT)
        self.press_pedal()
        for side in sides:
            self.assertFalse(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                             SafetyMode.FAULT)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
