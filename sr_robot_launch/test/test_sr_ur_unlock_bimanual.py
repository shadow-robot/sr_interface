#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2022 belongs to Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import rostest
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsProgramRunning
from sr_robot_launch.sr_ur_arm_unlock import SrUrUnlock
from sr_robot_launch.mock_sr_ur_robot_hw import MockUrRobotHW
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
from sr_robot_launch.common_sr_ur_unlock_tests import CommonTests
import sys


class TestSrUrUnlockBimanual(TestCase, CommonTests):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = {}
        cls.namespaces = ['/ra_sr_ur_robot_hw', '/la_sr_ur_robot_hw']
        cls.params = ['/headless_mode']
        for ns in cls.namespaces:
            for param in cls.params:
                rospy.set_param(ns + param, True)
        cls.service_string['right'] = '/ra_sr_ur_robot_hw/dashboard/program_running'
        cls.service_string['left'] = '/la_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = {}
        cls.mock_dashboard['left'] = MockUrRobotHW('left')
        cls.mock_dashboard['right'] = MockUrRobotHW('right')

    def setUp(self):
        for key, value in self.mock_dashboard.items():
            value.reinitialize()

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_arm_fault_bimanual_left(self):
        self.arm_fault_bimanual(['left'])

    def test_arm_fault_bimanual_right(self):
        self.arm_fault_bimanual(['right'])

    def test_arm_fault_bimanual_both(self):
        self.arm_fault_bimanual(['right', 'left'])

    def test_arm_startup_left(self):
        self.arm_setup('left')

    def test_arm_startup_right(self):
        self.arm_setup('right')

    def test_fault_left(self):
        self.fault('left')

    def test_fault_right(self):
        self.fault('right')

    def test_e_stop_right_1(self):
        self.e_stop('right', release_estop_before_pedal=True)

    def test_e_stop_left_1(self):
        self.e_stop('left', release_estop_before_pedal=True)

    def test_e_stop_right_2(self):
        self.e_stop('right', release_estop_before_pedal=False)

    def test_e_stop_left_2(self):
        self.e_stop('left', release_estop_before_pedal=False)

    def test_arm_power_cycle_left(self):
        self.arm_power_cycle('left')

    def test_arm_power_cycle_right(self):
        self.arm_power_cycle('right')

    def test_arm_mock_dashboard_server_left(self):
        self.arm_mock_dashboard_server('left')

    def test_arm_mock_dashboard_server_right(self):
        self.arm_mock_dashboard_server('right')

    def test_arm_startup_left_again(self):
        self.arm_setup('left')

    def test_arm_startup_right_again(self):
        self.arm_setup('right')


if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_sr_ur_unlock_bimanual'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlockBimanual)
