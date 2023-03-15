#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
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

from unittest import TestCase
from std_msgs.msg import Bool
from ur_dashboard_msgs.srv import IsProgramRunning
from ur_dashboard_msgs.msg import SafetyMode, RobotMode
import rospy


class CommonTests(TestCase):
    """
    Base test class to test sr_ur_arm_unlock
    """
    mock_dashboard = {}
    service_string = {}
    sr_ur_arm_unlock = None

    def arm_setup(self, side):
        self.press_pedal()
        self.assertTrue(self.get_program_running(side))

    def arm_mock_dashboard_server(self, side):
        self.assertFalse(self.get_program_running(side))

    def e_stop(self, side, _release_estop_before_pedal=True):
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
