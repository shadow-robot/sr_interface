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
from unittest import TestCase
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
        self.assertTrue(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        self.assertTrue(self.get_program_running(side))
        self.mock_dashboard[side].robot_state.emergency_stop(latch=True)
        self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                        SafetyMode.ROBOT_EMERGENCY_STOP)
        self.assertFalse(self.get_program_running(side))
        self.assertFalse(self.mock_dashboard[side].robot_state.get_robot_mode().robot_mode.mode ==
                        RobotMode.RUNNING)
        if release_estop_before_pedal:
            self.mock_dashboard[side].robot_state.emergency_stop(latch=False)
            self.press_pedal()
        else:
            self.press_pedal()
            self.assertTrue(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                            SafetyMode.ROBOT_EMERGENCY_STOP)
            self.mock_dashboard[side].robot_state.emergency_stop(latch=False)
            self.assertFalse(self.mock_dashboard[side].robot_state.get_safety_mode().safety_mode.mode ==
                            SafetyMode.ROBOT_EMERGENCY_STOP)
        rospy.sleep(1)
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




class TestSrUrUnlockRight(TestCase, CommonTests):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = {}
        rospy.set_param('ra_sr_ur_robot_hw/headless_mode', True)
        cls.service_string['right'] = '/ra_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = {}
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

    def test_arm_startup_right(self):
        self.arm_setup('right')

    def test_fault_right(self):
        self.fault('right')

    def test_e_stop_right_1(self):
        self.e_stop('right', release_estop_before_pedal=True)

    def test_e_stop_right_2(self):
        self.e_stop('right', release_estop_before_pedal=False)

    def test_arm_power_cycle_right(self):
        self.arm_power_cycle('right')

    def test_arm_mock_dashboard_server_right(self):
        self.arm_mock_dashboard_server('right')

    def test_arm_startup_right_again(self):
        self.arm_setup('right')


class TestSrUrUnlockLeft(TestCase, CommonTests):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = {}
        rospy.set_param('la_sr_ur_robot_hw/headless_mode', True)
        cls.service_string['left'] = '/la_sr_ur_robot_hw/dashboard/program_running'
        cls.sr_ur_arm_unlock = SrUrUnlock()
        cls.mock_dashboard = {}
        cls.mock_dashboard['left'] = MockUrRobotHW('left')

    def setUp(self):
        for key, value in self.mock_dashboard.iteritems():
            value.reinitialize()

    def tearDown(self):
        for key, value in self.mock_dashboard.iteritems():
            value.reinitialize()

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


class TestSrUrUnlockBimanual(TestCase, CommonTests):
    """
    Tests sr_ur_arm_unlock
    """
    @classmethod
    def setUpClass(cls):
        cls.service_string = {}
        rospy.set_param('ra_sr_ur_robot_hw/headless_mode', True)
        rospy.set_param('la_sr_ur_robot_hw/headless_mode', True)
        cls.service_string['right'] = '/ra_sr_ur_robot_hw/dashboard/program_running'
        cls.service_string['left'] = '/la_sr_ur_robot_hw/dashboard/program_running'
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
    NODENAME = 'test_sr_ur_unlock'
    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlockLeft)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlockRight)
    rostest.rosrun(PKGNAME, NODENAME, TestSrUrUnlockBimanual)
