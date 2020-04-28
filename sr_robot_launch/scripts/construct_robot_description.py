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
        self._robot_description_file = rospy.get_param("~robot_description_file")
        self._bimanual = False
        self._xacro_commands = []
        self._node_name = rospy.get_name()

        if 'bimanual' in self._robot_description_file:
            self._bimanual = True

        arm1 = ('ra', '192.168.1.1')
        arm2 = ('la', '192.168.2.1')
        if not self._bimanual:
            self._sr_ur_load_calibration = SrUrLoadCalibration([arm1])
        else:
            self._sr_ur_load_calibration = SrUrLoadCalibration([arm1, arm2])
        self._robot_description_params = self._get_parameters()
        kinematics_configs = self._get_kinematics_config()
        if not self._bimanual:
            self._robot_description_params['kinematics_config'] = kinematics_configs[0]
        else:
            self._robot_description_params['kinematics_config_right'] = kinematics_configs[0]
            self._robot_description_params['kinematics_config_left'] = kinematics_configs[1]

        urdf = self._parse_xacro()
        rospy.set_param('robot_description', urdf)

    def _parse_xacro(self):
        self._xacro_commands.append("xacro")
        self._xacro_commands.append(self._robot_description_file)
        for key, value in self._robot_description_params.items():
            self._xacro_commands.append(key + ':=' + str(value))
        return self._generate_urdf(self._xacro_commands)

    def _get_parameters(self):
        robot_description_params = {}
        robot_description_param_names = [param for param in rospy.get_param_names() 
                                         if self._node_name in param and 'robot_description' not in param]
        for name in robot_description_param_names:
            robot_description_params[name.rsplit('/',1)[1]] = rospy.get_param(name)
        return robot_description_params

    def _get_kinematics_config(self):
        kinematics_configs = []
        arms_info = self._sr_ur_load_calibration.get_calibration_files()
        if not self._bimanual:
            kinematics_configs.append(arms_info[0]['kinematics_config'])
        else:
            for arm in arms_info:
                kinematics_configs.append(arm['kinematics_config'])
        return kinematics_configs

    def _generate_urdf(self, xacro_commands):
        try:
            out = check_output(xacro_commands)
        except:
            rospy.logerr("error")
        return out


if __name__ == "__main__":
    rospy.init_node("sr_construct_robot_description")
    sr_construct_robot_description = SrConstructRobotDescription()
