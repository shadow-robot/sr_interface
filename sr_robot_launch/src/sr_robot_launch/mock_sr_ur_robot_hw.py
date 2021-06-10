#!/usr/bin/env python3
#
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
import actionlib
import actionlib_tutorials.msg
import control_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from std_msgs.msg import Bool
from ur_dashboard_msgs.srv import (GetSafetyMode, GetSafetyModeResponse, GetProgramState,
                                   GetRobotMode, Load, IsProgramRunning, GetProgramStateResponse,
                                   GetRobotModeResponse, LoadResponse, IsProgramRunningResponse)
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
from std_srvs.srv import Trigger, TriggerResponse


class IllegalArgumentError(ValueError):
    pass


class ArmState(object):
    def __init__(self, arm_prefix):
        self._arm_prefix = arm_prefix
        self._safety_mode_publisher = rospy.Publisher('/' + arm_prefix + '_sr_ur_robot_hw/safety_mode',
                                                      SafetyMode, queue_size=1)
        self._robot_mode = GetRobotModeResponse()
        self._safety_mode = GetSafetyModeResponse()
        self._program_state = GetProgramStateResponse()
        self._program_state.program_name = "<unnamed>"
        self._program_state.success = True
        self._reboot_required = False
        self._e_stop_active = False
        self._program_running = IsProgramRunningResponse()
        self.power_off()

    def _publish_safety_mode(self, e_stop_pressed):
        if e_stop_pressed:
            self._safety_mode_publisher.publish(SafetyMode.ROBOT_EMERGENCY_STOP)
        else:
            self._safety_mode_publisher.publish(SafetyMode.NORMAL)

    def _set_safety_mode(self, mode):
        self._safety_mode.success = True
        if mode == "NORMAL":
            self._safety_mode.safety_mode.mode = SafetyMode.NORMAL
            self._safety_mode.answer = "Safetymode: NORMAL"
        elif mode == "PROTECTIVE_STOP":
            self._safety_mode.safety_mode.mode = SafetyMode.PROTECTIVE_STOP
            self._safety_mode.answer = "Safetymode: PROTECTIVE_STOP"
        elif mode == "ROBOT_EMERGENCY_STOP":
            self._safety_mode.safety_mode.mode = SafetyMode.ROBOT_EMERGENCY_STOP
            self._safety_mode.answer = "Safetymode: ROBOT_EMERGENCY_STOP"
        elif mode == "FAULT":
            self._safety_mode.safety_mode.mode = SafetyMode.FAULT
            self._safety_mode.answer = "Safetymode: FAULT"
        else:
            self._safety_mode.success = False

    def _set_robot_mode(self, mode):
        self._robot_mode.success = True
        if mode == "IDLE":
            self._robot_mode.robot_mode.mode = RobotMode.IDLE
            self._robot_mode.answer = "Robotmode: IDLE"
        elif mode == "POWER_OFF" or mode == "STOPPED":
            self._robot_mode.robot_mode.mode = RobotMode.POWER_OFF
            self._robot_mode.answer = "Robotmode: POWER_OFF"
        elif mode == "RUNNING":
            self._robot_mode.robot_mode.mode = RobotMode.RUNNING
            self._robot_mode.answer = "Robotmode: RUNNING"
        else:
            self._robot_mode.success = False

    def _set_program_running(self, running):
        self._program_running.success = True
        if running:
            self._program_running.program_running = True
            self._program_running.answer = "RUNNING"
        else:
            self._program_running.program_running = False
            self._program_running.answer = "STOPPED"

    def get_robot_mode(self):
        return self._robot_mode

    def get_safety_mode(self):
        return self._safety_mode

    def get_program_running(self):
        return self._program_running

    def get_program_state(self):
        return self._program_state

    def restart_safety(self):
        self._set_safety_mode('NORMAL')
        self._set_robot_mode('POWER_OFF')
        self._set_program_running(False)
        self._reboot_required = False

    def power_on(self):
        if self._e_stop_active:
            self.emergency_stop()
            return
        self._set_safety_mode('NORMAL')
        self._set_robot_mode('IDLE')
        self._set_program_running(False)

    def brake_release(self):
        if self._e_stop_active:
            self.emergency_stop()
            return
        if not self._reboot_required:
            self._set_safety_mode('NORMAL')
            self._set_robot_mode('RUNNING')
            self._set_program_running(False)
        else:
            self.fault()

    def unlock_protective_stop(self):
        self._set_safety_mode('NORMAL')
        self._set_robot_mode('RUNNING')
        self._set_program_running(False)

    def power_off(self):
        self._set_safety_mode('NORMAL')
        self._set_robot_mode('POWER_OFF')
        self._set_program_running(False)
        self._reboot_required = False

    def protective_stop(self):
        self._set_safety_mode('PROTECTIVE_STOP')
        self._set_robot_mode('RUNNING')
        self._set_program_running(False)

    def check_e_stop(self):
        return self._e_stop_active

    def emergency_stop(self, latch=True):
        self._e_stop_active = latch
        if latch:
            self._set_safety_mode('ROBOT_EMERGENCY_STOP')
            self._set_robot_mode('IDLE')
            self._set_program_running(False)
            self._reboot_required = True
            self._publish_safety_mode(True)
        else:
            self._set_safety_mode('NORMAL')
            self._set_robot_mode('IDLE')
            self._set_program_running(False)
            self._reboot_required = True
            self._publish_safety_mode(False)

    def resend_robot_program(self):
        if self._e_stop_active:
            self.emergency_stop()
            return
        if (self.get_safety_mode().safety_mode.mode == SafetyMode.NORMAL and
                self.get_robot_mode().robot_mode.mode == RobotMode.RUNNING):
            self._set_program_running(True)
        else:
            self._set_program_running(False)

    def fault(self):
        self._set_safety_mode('FAULT')
        self._set_robot_mode('STOPPED')
        self._set_program_running(False)


class MockUrRobotHW(object):
    def __init__(self, side='right'):
        if 'left' not in side and 'right' not in side:
            rospy.logerr("side: %s not valid. Valid sides are: 'left, 'right'", side)
            raise IllegalArgumentError
            exit(1)
        self._arm_prefix = side[0] + 'a'
        self.robot_state = ArmState(self._arm_prefix)
        get_safety_mode_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/get_safety_mode',
                                                GetSafetyMode, self.handle_get_safety_mode)
        get_robot_mode_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/get_robot_mode',
                                               GetRobotMode, self.handle_get_robot_mode)
        is_program_running = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/program_running',
                                           IsProgramRunning, self.handle_is_program_running)
        load_program_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/load',
                                             Load, self.handle_load_program)
        program_state_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/program_state',
                                              GetProgramState, self.handle_get_program_state)
        power_on_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/power_on',
                                         Trigger, self.handle_power_on)
        power_off_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/power_off',
                                          Trigger, self.handle_power_off)
        brake_release_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/brake_release',
                                              Trigger, self.handle_brake_release)
        restart_safety_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/restart_safety',
                                               Trigger, self.handle_restart_safety)
        close_safety_popup_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/close_safety_popup',
                                                   Trigger, self.handle_close_safety_popup)
        close_popup_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/dashboard/close_popup',
                                            Trigger, self.handle_close_popup)
        unlock_protective_stop_service = rospy.Service(self._arm_prefix +
                                                       '_sr_ur_robot_hw/dashboard/unlock_protective_stop',
                                                       Trigger, self.handle_unlock_protective_stop)
        resend_robot_program_service = rospy.Service(self._arm_prefix + '_sr_ur_robot_hw/resend_robot_program',
                                                     Trigger, self.handle_resend_robot_program)

    def reinitialize(self):
        self.robot_state = ArmState(self._arm_prefix)

    def handle_get_safety_mode(self, request):
        return self.robot_state.get_safety_mode()

    def handle_get_program_state(self, request):
        return self.robot_state.get_program_state()

    def handle_get_robot_mode(self, request):
        return self.robot_state.get_robot_mode()

    def handle_load_program(self, request):
        response = LoadResponse()
        rospy.loginfo("Loading: %s", request.filename)
        response.answer = request.filename
        response.success = True
        return response

    def handle_is_program_running(self, request):
        return self.robot_state.get_program_running()

    def handle_close_popup(self, request):
        return self.trigger_response()

    def handle_close_safety_popup(self, request):
        return self.trigger_response()

    def trigger_response(self):
        response = TriggerResponse()
        response.success = True
        response.message = "Testing"
        return response

    def handle_power_on(self, request):
        self.robot_state.power_on()
        return self.trigger_response()

    def handle_power_off(self, request):
        self.robot_state.power_off()
        return self.trigger_response()

    def handle_brake_release(self, request):
        self.robot_state.brake_release()
        return self.trigger_response()

    def handle_restart_safety(self, request):
        self.robot_state.restart_safety()
        return self.trigger_response()

    def handle_unlock_protective_stop(self, request):
        self.robot_state.unlock_protective_stop()
        return self.trigger_response()

    def handle_resend_robot_program(self, request):
        self.robot_state.resend_robot_program()
        return self.trigger_response()

    def set_fault(self):
        self.robot_state.fault()
        return self.trigger_response()
