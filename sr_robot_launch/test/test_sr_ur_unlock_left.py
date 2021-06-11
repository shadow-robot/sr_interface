#!/usr/bin/env python3
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


class TestSrUrUnlockLeft(TestCase, CommonTests):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = {}
        cls.namespaces = ['/la_sr_ur_robot_hw']
        cls.params = ['/headless_mode']
        for ns in cls.namespaces:
            for param in cls.params:
                rospy.set_param(ns + param, True)
        cls.service_string['left'] = '/la_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = {}
        cls.mock_dashboard['left'] = MockUrRobotHW('left')

    def setUp(self):
        for key, value in self.mock_dashboard.iteritems():
            value.reinitialize()

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_arm_startup_left(self):
        self.arm_setup('left')

    def test_fault_left(self):
        self.fault('left')

    def test_e_stop_left_1(self):
        self.e_stop('left', release_estop_before_pedal=True)

    def test_e_stop_left_2(self):
        self.e_stop('left', release_estop_before_pedal=False)

    def test_arm_power_cycle_left(self):
        self.arm_power_cycle('left')

    def test_arm_mock_dashboard_server_left(self):
        self.arm_mock_dashboard_server('left')

    def test_arm_startup_left_again(self):
        self.arm_setup('left')


if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_sr_ur_unlock_left'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlockLeft)
