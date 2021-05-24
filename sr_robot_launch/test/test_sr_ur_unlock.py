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
        cls.service_string = {}
        cls.service_string['right'] = '/ra_sr_ur_robot_hw/dashboard/program_running'
        cls.service_string['left']  = '/la_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = {}
        cls.mock_dashboard['left'] = MockUrRobotHW('left')
        cls.mock_dashboard['right'] = MockUrRobotHW('right')

    def setUp(self):
        for key, value in self.mock_dashboard.iteritems():
            value.reinitialize()

    def tearDown(self):
        for key, value in self.mock_dashboard.iteritems():
            value.reinitialize()

    @classmethod
    def tearDownClass(cls):
        pass

    def test_arm_power_cycle_bimanual(self):
        self.assertFalse(self.get_program_running('right'))
        self.assertFalse(self.get_program_running('left'))
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertFalse(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)

    def test_arm_fault_bimanual_left(self):
        self.assertFalse(self.get_program_running('right'))
        self.assertFalse(self.get_program_running('left'))
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
        self.mock_dashboard['left'].robot_state.fault()
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))

    def test_arm_fault_bimanual_right(self):
        self.assertFalse(self.get_program_running('right'))
        self.assertFalse(self.get_program_running('left'))
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
        self.mock_dashboard['right'].robot_state.fault()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))

    def test_arm_fault_bimanual(self):
        self.assertFalse(self.get_program_running('right'))
        self.assertFalse(self.get_program_running('left'))
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))
        self.mock_dashboard['right'].robot_state.fault()
        self.mock_dashboard['left'].robot_state.fault()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertFalse(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertTrue(self.get_program_running('right'))
        self.assertTrue(self.get_program_running('left'))

    def test_arm_startup_again_right(self):
        self.assertFalse(self.get_program_running('right'))

    def test_fault_right(self):
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.mock_dashboard['right'].robot_state.fault()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertFalse(self.get_program_running('right'))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.NORMAL)
        self.assertTrue(self.get_program_running('right'))

    def test_arm_power_cycle_right(self):
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertFalse(self.get_program_running('right'))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('right'))
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['right'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertFalse(self.get_program_running('right'))

    def test_arm_mock_right(self):
        self.assertFalse(self.get_program_running('right'))

    def test_arm_startup_again_left(self):
        self.assertFalse(self.get_program_running('left'))

    def test_fault_left(self):
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('left'))
        self.mock_dashboard['left'].robot_state.fault()
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.FAULT)
        self.assertFalse(self.get_program_running('left'))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_safety_mode().safety_mode.mode == SafetyMode.NORMAL)
        self.assertTrue(self.get_program_running('left'))

    def test_arm_power_cycle_left(self):
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.POWER_OFF)
        self.assertFalse(self.get_program_running('left'))
        self.press_pedal()
        self.assertTrue(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertTrue(self.get_program_running('left'))
        self.press_pedal()
        self.assertFalse(self.mock_dashboard['left'].robot_state.get_robot_mode().robot_mode.mode == RobotMode.RUNNING)
        self.assertFalse(self.get_program_running('left'))

    def test_arm_startup_left(self):
        self.press_pedal()
        self.assertTrue(self.get_program_running('left'))

    def test_arm_startup_right(self):
        self.press_pedal()
        self.assertTrue(self.get_program_running('right'))

    def test_arm_mock_left(self):
        self.assertFalse(self.get_program_running('left'))

    def get_program_running(self, side):
        service_call = rospy.ServiceProxy(self.service_string[side], IsProgramRunning)
        response = service_call()
        return response.program_running

    def press_pedal(self):
        self.sr_ur_arm_unlock.release_or_brake_arm_cb(Bool(True))

if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_sr_ur_unlock'
    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlock)

