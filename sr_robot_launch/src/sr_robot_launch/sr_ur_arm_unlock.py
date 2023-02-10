#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2020-2023 belongs to Shadow Robot Company Ltd.
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

import rospy
from std_msgs.msg import Bool
from ur_dashboard_msgs.srv import GetSafetyMode, GetProgramState, GetRobotMode, Load, IsProgramRunning
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
from std_srvs.srv import Trigger


class RobotSafetyMonitor:
    def __init__(self, name):
        topic_string = '/' + name + '_sr_ur_robot_hw/safety_mode'
        self.estop_pressed = False
        self._subscriber = rospy.Subscriber(topic_string, SafetyMode,
                                            self._safety_mode_callback)

    def press_estop(self):
        self.estop_pressed = True

    def release_estop(self):
        self.estop_pressed = False

    def _safety_mode_callback(self, message):
        if message.mode == SafetyMode.ROBOT_EMERGENCY_STOP:
            self.press_estop()


class SrUrUnlock:
    def __init__(self):
        self._external_control_program_name = rospy.get_param("~urcap_program_name", "external_ctrl.urp")
        self._arms = []
        if rospy.has_param('ra_sr_ur_robot_hw'):
            self._arms.append('ra')
        if rospy.has_param('la_sr_ur_robot_hw'):
            self._arms.append('la')
        if len(self._arms) == 0:
            rospy.logerr("No arms detected, shutting down %s", rospy.get_name())
            rospy.signal_shutdown("No arms detected")
        rospy.Subscriber("/sr_arm/release_or_brake", Bool, self.release_or_brake_arm_cb)
        self._robot_state_monitors = {}
        self.setup_robot_mode_subscribers()

    def setup_robot_mode_subscribers(self):
        for arm in self._arms:
            robot_safety_monitor = RobotSafetyMonitor(arm)
            self._robot_state_monitors[arm] = robot_safety_monitor

    def release_or_brake_arm_cb(self, message):
        if not message.data:
            return

        if self.check_arms_needs_starting():
            self.release_arm()
            return

        self.brake_arm()

    @staticmethod
    def call_arm_service(side, service_name, service_type, dashboard=True, service_data=""):
        if dashboard:
            service_string = "/" + side + "_sr_ur_robot_hw/dashboard/" + service_name
        else:
            service_string = "/" + side + "_sr_ur_robot_hw/" + service_name
        try:
            service_call = rospy.ServiceProxy(service_string, service_type)
            if service_data == "":
                response = service_call()
            else:
                response = service_call(service_data)
            return response
        except rospy.ServiceException as exception:
            rospy.logerr("Service call to '%s' failed for arm %s. %s", service_name, side, exception)
            raise

    def startup_arms(self):
        if self.is_robot_in_mode('robot', RobotMode.RUNNING):
            return
        for arm in self._arms:
            self.call_arm_service(arm, "power_on", Trigger)
        self.wait_for_mode('robot', RobotMode.IDLE)
        for arm in self._arms:
            self.call_arm_service(arm, "brake_release", Trigger)
        self.wait_for_mode('robot', RobotMode.RUNNING)

    def wait_for_mode(self, mode_type, mode, timeout=15):
        now = rospy.rostime.Time().now().secs
        while not self.is_robot_in_mode(mode_type, mode):
            if rospy.rostime.Time().now().secs > now + timeout:
                return False
        return True

    def is_robot_in_mode(self, mode_type, mode):
        if mode_type == 'robot':
            arms_ready = [mode == self.call_arm_service(arm, "get_robot_mode",
                                                        GetRobotMode).robot_mode.mode for arm in self._arms]
        elif mode_type == 'safety':
            arms_ready = [mode == self.call_arm_service(arm, "get_safety_mode",
                                                        GetSafetyMode).safety_mode.mode for arm in self._arms]
        return all(arms_ready)

    def check_arms_e_stops(self):
        for arm in self._arms:
            safety_mode = self.call_arm_service(arm, "get_safety_mode", GetSafetyMode)
            if self._robot_state_monitors[arm].estop_pressed:
                if SafetyMode.ROBOT_EMERGENCY_STOP == safety_mode.safety_mode.mode:
                    while SafetyMode.ROBOT_EMERGENCY_STOP == \
                            self.call_arm_service(arm, "get_safety_mode", GetSafetyMode).safety_mode.mode:
                        rospy.logwarn("Emergency stop button is still pressed for arm %s, please release", arm)
                        rospy.sleep(0.01)
                rospy.loginfo("Past E-stop detected on arm %s, power-cycling that arm...", arm)
                self._robot_state_monitors[arm].release_estop()
                self.call_arm_service(arm, "power_off", Trigger)
                rospy.sleep(0.1)

    def unlock_arms_if_protective_stop(self):
        for arm in self._arms:
            safety_mode = self.call_arm_service(arm, "get_safety_mode", GetSafetyMode)
            if SafetyMode.PROTECTIVE_STOP == safety_mode.safety_mode.mode:
                rospy.loginfo("Protective stop detected on: %s. Unlocking...", arm)
            while SafetyMode.PROTECTIVE_STOP == \
                    self.call_arm_service(arm, "get_safety_mode", GetSafetyMode).safety_mode.mode:
                self.call_arm_service(arm, "unlock_protective_stop", Trigger)
                rospy.sleep(0.01)

    def unlock_arms_if_fault(self):
        fault = False
        for arm in self._arms:
            safety_mode = self.call_arm_service(arm, "get_safety_mode", GetSafetyMode)
            if (SafetyMode.FAULT == safety_mode.safety_mode.mode or
                    SafetyMode.VIOLATION == safety_mode.safety_mode.mode):
                fault = True
                if SafetyMode.FAULT == safety_mode.safety_mode.mode:
                    rospy.loginfo("Fault detected on: %s. This can be caused by using the external e-stop.\
                                   Have you released it?", arm)
                self.call_arm_service(arm, "restart_safety", Trigger)
        return fault

    def check_arms_needs_starting(self):
        for arm in self._arms:
            robot_mode = self.call_arm_service(arm, "get_robot_mode", GetRobotMode)
            safety_mode = self.call_arm_service(arm, "get_safety_mode", GetSafetyMode)
            program_running = self.call_arm_service(arm, "program_running", IsProgramRunning)
            if (RobotMode.IDLE == robot_mode.robot_mode.mode or RobotMode.POWER_OFF == robot_mode.robot_mode.mode or
                    SafetyMode.PROTECTIVE_STOP == safety_mode.safety_mode.mode or
                    not program_running.program_running):
                rospy.loginfo("Starting arm: %s", arm)
                return True
        return False

    def clear_arms_popups(self):
        for arm in self._arms:
            self.call_arm_service(arm, "close_safety_popup", Trigger)
        for arm in self._arms:
            self.call_arm_service(arm, "close_popup", Trigger)

    def load_arms_program_if_unloaded(self):
        sleep_time = False
        for arm in self._arms:
            try:
                headless_mode = rospy.get_param("/" + arm + "_sr_ur_robot_hw/headless_mode")
            except KeyError:
                headless_mode = False
            if not headless_mode:
                play_msg = self.call_arm_service(arm, "program_state", GetProgramState)
                if play_msg.program_name == "null":
                    rospy.loginfo("Not in headless mode. Loading program: %s for arm: %s",
                                  self._external_control_program_name, arm)
                    resp = self.call_arm_service(arm, "load_program", Load,
                                                 service_data=self._external_control_program_name)
                    rospy.loginfo("%s", resp)
                    sleep_time = True
        return sleep_time

    def start_arms_program_if_stopped(self):
        sleep_time = False
        for arm in self._arms:
            try:
                headless_mode = rospy.get_param("/" + arm + "_sr_ur_robot_hw/headless_mode")
            except KeyError:
                headless_mode = False
            if not headless_mode:
                play_msg = self.call_arm_service(arm, "program_state", GetProgramState)
                if ProgramState.STOPPED == play_msg.state.state or ProgramState.PAUSED == play_msg.state.state:
                    rospy.loginfo("Not in headless mode. Starting program: %s for arm: %s",
                                  play_msg.program_name, arm)
                    self.call_arm_service(arm, "play", Trigger)
                    sleep_time = True
            else:
                rospy.loginfo("Headless mode active, resending robot program to arm: %s", arm)
                self.call_arm_service(arm, "resend_robot_program", Trigger, dashboard=False)
                self.call_arm_service(arm, "resend_robot_program", Trigger, dashboard=False)
                sleep_time = True
        return sleep_time

    def release_arm(self):
        try:
            rospy.loginfo("Unlock arm(s) signal received. Checking arms.")
            rospy.loginfo("Checking e-stops ...")
            self.check_arms_e_stops()
            rospy.loginfo("Checking protective stops ...")
            self.unlock_arms_if_protective_stop()
            rospy.loginfo("Checking for faults ...")
            if self.unlock_arms_if_fault():
                rospy.loginfo("Resetting robot safety, please wait approximately 15 seconds...")
                self.wait_for_mode('safety', SafetyMode.NORMAL, timeout=15)
            rospy.loginfo("Checking protective stops again ...")
            self.unlock_arms_if_protective_stop()
            rospy.loginfo("Closing popups ...")
            self.clear_arms_popups()
            rospy.loginfo("Checking robot mode ...")
            if self.check_arms_needs_starting():
                self.startup_arms()
            rospy.loginfo("Checking if program is loaded ...")
            self.load_arms_program_if_unloaded()
            rospy.loginfo("Checking if program is running ...")
            self.start_arms_program_if_stopped()
        except rospy.ServiceException as exception:
            for arm in self._arms:
                rospy.logerr("Arm checking/restarting failed for arm: %s. %s", arm, exception)

    def brake_arm(self):
        rospy.loginfo("Brake arm signal received.")
        for arm in self._arms:
            try:
                self.call_arm_service(arm, "power_off", Trigger)
            except rospy.ServiceException as exception:
                rospy.logerr("Arm braking failed for arm: %s. %s", arm, exception)
