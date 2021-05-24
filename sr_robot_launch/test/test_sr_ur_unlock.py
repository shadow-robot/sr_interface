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
from sr_robot_commander.sr_hand_commander import SrHandCommander
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsProgramRunning
from sr_robot_launch.sr_ur_arm_unlock import SrUrUnlock
from sr_robot_launch.mock_sr_ur_robot_hw import MockUrRobotHW
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode


class TestSrUrUnlock(TestCase):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = '/ra_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = MockUrRobotHW('right')

    def setUp(self):
        self.mock_dashboard.reinitialize()

    def tearDown(self):
        self.mock_dashboard.reinitialize()

    @classmethod
    def tearDownClass(cls):
        pass

    def test_arm_startup_again(self):
        service_call = rospy.ServiceProxy(self.service_string, IsProgramRunning)
        response = service_call()
        self.assertFalse(response.program_running)

    def test_fault(self):
        self.press_pedal()
        self.assertTrue(self.mock_dashboard.robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running())
        self.mock_dashboard.robot_state.fault()
        self.assertTrue(self.mock_dashboard.robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertFalse(self.get_program_running())
        self.press_pedal()
        self.assertTrue(self.mock_dashboard.robot_state.get_safety_mode().safety_mode.mode == SafetyMode.NORMAL)
        self.assertTrue(self.get_program_running())

    def test_arm_power_cycle_robot_mode(self):
        self.assertTrue(self.mock_dashboard.robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard.robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.press_pedal()
        self.assertFalse(self.mock_dashboard.robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)

    def test_arm_power_cycle_program_state(self):
        self.assertFalse(self.get_program_running())
        self.press_pedal()
        self.assertTrue(self.get_program_running())
        self.press_pedal()
        self.assertFalse(self.get_program_running())

    def test_arm_startup(self):
        self.press_pedal()
        self.assertTrue(self.get_program_running())

    def test_arm_mock(self):
        self.assertFalse(self.get_program_running())

    def get_program_running(self):
        service_call = rospy.ServiceProxy(self.service_string, IsProgramRunning)
        response = service_call()
        return response.program_running

    def press_pedal(self):
        self.sr_ur_arm_unlock.release_or_brake_arm_cb(Bool(True))

if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_sr_ur_unlock'
    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlock)

