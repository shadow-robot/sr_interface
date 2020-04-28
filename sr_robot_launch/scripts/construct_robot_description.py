#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import os
import rospy
import rospkg
from subprocess import check_output
from sr_ur_arm_calibration_loader.sr_ur_arm_calibration_loader import SrUrLoadCalibration


class SrConstructRobotDescription():
    def __init__(self):
        self.robot_description_file = rospy.get_param("~robot_description_file")

        if 'bimanual' in self.robot_description_file:
            self.bimanual = False

        self.xacro_commands = []
        self.bimanual = False
        self.node_name = rospy.get_name()

        # If multiple arms, specify right arm first!
        self.sr_ur_load_calibration = SrUrLoadCalibration("192.168.1.1")
        self.robot_description_params = self.get_parameters()
        self.kinematics_configs = self.get_kinematics_config()
        if not self.bimanual:
            self.robot_description_params['kinematics_config'] = self.kinematics_configs[0]
        else:
            self.robot_description_params['kinematics_config_right'] = self.kinematics_configs[0]
            self.robot_description_params['kinematics_config_left'] = self.kinematics_configs[1]

        self.urdf = parse_xacro()
        rospy.set_param('robot_description', self.urdf)

    def parse_xacro(self):
        self.xacro_commands.append("xacro")
        self.xacro_commands.append(self.robot_description_file)
        for key, value in self.robot_description_params.items():
            self.xacro_commands.append(key + ':=' + str(value))
        return self.generate_urdf(self.xacro_commands)

    def get_parameters(self):
        robot_description_params = {}
        robot_description_param_names = [param for param in rospy.get_param_names() if self.node_name in param and 'robot_description' not in param]
        for name in robot_description_param_names:
            robot_description_params[name.rsplit('/',1)[1]] = rospy.get_param(name)
        return robot_description_params

    def get_kinematics_config(self):
        kinematics_configs = []
        arms_info = self.sr_ur_load_calibration.get_calibration_files()
        if not self.bimanual:
            kinematics_configs.append(arms_info[0][2])
        else:
            for arm in arms_info:
                kinematics_configs.append(arm[2])
        return kinematics_configs

    def generate_urdf(self, xacro_commands):
        try:
            out = check_output(xacro_commands)
        except:
            rospy.logerr("error")
        return out

    def run(self):
        if not self.check_arm_calibration_exists(arm_serial):
            calibration_generated = self.generate_new_arm_calibration(arm_ip, arm_serial)
            if not self.check_arm_calibration_exists(arm_serial):
                calibration_generated = self.generate_new_arm_calibration(arm_ip, arm_serial)


if __name__ == "__main__":
    rospy.init_node("sr_construct_robot_description")
    sr_construct_robot_description = SrConstructRobotDescription()
