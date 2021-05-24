#!/usr/bin/env python
#
# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

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


class STATE(object):
    def __init__(self, start_state='ROBOT_STOPPED'):
        self.states = ['ROBOT_STOPPED', 'ROBOT_IDLE', 'ROBOT_RUNNING', 'PROTECTIVE_STOP',
                       'EMERGENCY-STOP', 'PROGRAM_RUNNING']
        self.ROBOT_MODE = GetRobotModeResponse()
        self.SAFETY_MODE = GetSafetyModeResponse()
        self.PROGRAM_STATE = GetProgramStateResponse()
        self.PROGRAM_STATE.program_name = "<unnamed>"
        self.PROGRAM_STATE.success = True
        self.PROGRAM_RUNNING = IsProgramRunningResponse()
        self.update_state(start_state)

    def set_safety_mode(self, mode):
        self.SAFETY_MODE.success = True
        if mode == "NORMAL":
            self.SAFETY_MODE.safety_mode.mode = SafetyMode.NORMAL
            self.SAFETY_MODE.answer = "Safetymode: NORMAL"
        if mode == "PROTECTIVE_STOP":
            self.SAFETY_MODE.safety_mode.mode = SafetyMode.PROTECTIVE_STOP
            self.SAFETY_MODE.answer = "Safetymode: PROTECTIVE_STOP"
        if mode == "ROBOT_EMERGENCY_STOP":
            self.SAFETY_MODE.safety_mode.mode = SafetyMode.ROBOT_EMERGENCY_STOP
            self.SAFETY_MODE.answer = "Safetymode: ROBOT_EMERGENCY_STOP"
        if mode == "FAULT":
            self.SAFETY_MODE.safety_mode.mode = SafetyMode.FAULT
            self.SAFETY_MODE.answer = "Safetymode: FAULT"

    def set_robot_mode(self, mode):
        self.ROBOT_MODE.success = True
        if mode == "IDLE":
            self.ROBOT_MODE.robot_mode.mode = RobotMode.IDLE
            self.ROBOT_MODE.answer = "Robotmode: IDLE"
        if mode == "POWER_OFF" or mode == "STOPPED":
            self.ROBOT_MODE.robot_mode.mode = RobotMode.POWER_OFF
            self.ROBOT_MODE.answer = "Robotmode: POWER_OFF"
        if mode == "RUNNING":
            self.ROBOT_MODE.robot_mode.mode = RobotMode.RUNNING
            self.ROBOT_MODE.answer = "Robotmode: RUNNING"

    def set_program_running(self, running):
        self.PROGRAM_RUNNING.success = True
        if running:
            self.PROGRAM_RUNNING.program_running = True
            self.PROGRAM_RUNNING.answer = "RUNNING"
        else:
            self.PROGRAM_RUNNING.program_running = False
            self.PROGRAM_RUNNING.answer = "STOPPED"

    def get_robot_mode(self):
        return self.ROBOT_MODE

    def get_safety_mode(self):
        return self.SAFETY_MODE

    def get_program_running(self):
        return self.PROGRAM_RUNNING

    def get_program_state(self):
        return self.PROGRAM_STATE

    def restart_safety(self):
        self.set_safety_mode('NORMAL')
        self.set_robot_mode('POWER_OFF')
        self.set_program_running(False)

    def power_on(self):
        self.set_safety_mode('NORMAL')
        self.set_robot_mode('IDLE')
        self.set_program_running(False)

    def brake_release(self):
        self.set_safety_mode('NORMAL')
        self.set_robot_mode('RUNNING')
        self.set_program_running(False)

    def unlock_protective_stop(self):
        self.set_safety_mode('NORMAL')
        self.set_robot_mode('RUNNING')
        self.set_program_running(False)

    def power_off(self):
        self.set_safety_mode('NORMAL')
        self.set_robot_mode('POWER_OFF')
        self.set_program_running(False)

    def protective_stop(self):
        self.set_safety_mode('PROTECTIVE_STOP')
        self.set_robot_mode('RUNNING')
        self.set_program_running(False)

    def emergency_stop(self):
        self.set_safety_mode('ROBOT_EMERGENCY_STOP')
        self.set_robot_mode('IDLE')
        self.set_program_running(False)

    def resend_robot_program(self):
        if (self.get_safety_mode().safety_mode.mode == SafetyMode.NORMAL and
                self.get_robot_mode().robot_mode.mode == RobotMode.RUNNING):
            self.set_program_running(True)
        else:
            self.set_program_running(False)

    def fault(self):
        self.set_safety_mode('FAULT')
        self.set_robot_mode('STOPPED')
        self.set_program_running(False)

    def update_state(self, state):
        if state not in self.states:
            rospy.logwarn("Invalid state: %s valid states are: %s", state, str(self.states))
            return
        if state == 'ROBOT_STOPPED':
            self.set_safety_mode('NORMAL')
            self.set_robot_mode('POWER_OFF')
            self.set_program_running(False)
        if state == 'ROBOT_IDLE':
            self.set_safety_mode('NORMAL')
            self.set_robot_mode('IDLE')
            self.set_program_running(False)
        if state == 'ROBOT_RUNNING':
            self.set_safety_mode('NORMAL')
            self.set_robot_mode('RUNNING')
            self.set_program_running(False)
        if state == 'PROTECTIVE_STOP':
            self.set_safety_mode('PROTECTIVE_STOP')
            self.set_robot_mode('RUNNING')
            self.set_program_running(False)
        if state == 'EMERGENCY-STOP':
            self.set_safety_mode('ROBOT_EMERGENCY_STOP')
            self.set_robot_mode('IDLE')
            self.set_program_running(False)
        if state == 'PROGRAM_RUNNING':
            self.set_safety_mode('NORMAL')
            self.set_robot_mode('RUNNING')
            self.set_program_running(True)


class MockUrRobotHW(object):
    def __init__(self, side='right'):
        if 'left' not in side and 'right' not in side:
            rospy.logerr("side: %s not valid. Valid sides are 'left, 'right'", side)
            exit(0)
        self.side = side
        self.arm_prefix = side[0] + 'a'
        self.robot_state = STATE()
        get_safety_mode_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/get_safety_mode',
                                                GetSafetyMode, self.handle_get_safety_mode)
        get_robot_mode_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/get_robot_mode',
                                               GetRobotMode, self.handle_get_robot_mode)
        is_program_running = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/program_running',
                                           IsProgramRunning, self.handle_is_program_running)
        load_program_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/load',
                                             Load, self.handle_load_program)
        program_state_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/program_state',
                                              GetProgramState, self.handle_get_program_state)
        power_on_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/power_on',
                                         Trigger, self.handle_power_on)
        power_off_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/power_off',
                                          Trigger, self.handle_power_off)
        brake_release_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/brake_release',
                                              Trigger, self.handle_brake_release)
        restart_safety_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/restart_safety',
                                               Trigger, self.handle_restart_safety)
        close_safety_popup_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/close_safety_popup',
                                                   Trigger, self.handle_close_safety_popup)
        close_popup_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/dashboard/close_popup',
                                            Trigger, self.handle_close_popup)
        unlock_protective_stop_service = rospy.Service(self.arm_prefix +
                                                       '_sr_ur_robot_hw/dashboard/unlock_protective_stop',
                                                       Trigger, self.handle_unlock_protective_stop)
        resend_robot_program_service = rospy.Service(self.arm_prefix + '_sr_ur_robot_hw/resend_robot_program',
                                                     Trigger, self.handle_resend_robot_program)

    def reinitialize(self):
        self.robot_state = STATE()

    def handle_get_safety_mode(self, request):
        return self.robot_state.get_safety_mode()

    def handle_get_program_state(self, request):
        return self.robot_state.get_program_state()
        response = GetProgramStateResponse()
        if 'RUNNING' in self.state:
            response.state = ProgramState.RUNNING
        if 'PAUSED' in self.state:
            response.state = ProgramState.RUNNING
        if 'STOPPED' in self.state:
            response.state = ProgramState.STOPPED
        return response

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
