#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
# Author: Shadow Robot Software Team <software@shadowrobot.com>

import argparse

import re
from copy import deepcopy
import yaml
from urdf_parser_py.urdf import URDF
from srdfdom.srdf import SRDF
import rospkg
import rospy
import rosparam


def yaml_reindent(in_str, numspaces):
    s_indent = "\n".join((numspaces * " ") + i for i in in_str.splitlines())
    return s_indent


def upload_output_params(upload_str, output_path=None, upload=True, ns_=None):
    if upload:
        paramlist = rosparam.load_str(upload_str, "generated",
                                      default_namespace=ns_)
        for params, namespace in paramlist:
            rosparam.upload_params(namespace, params)
    if output_path is not None:
        with open(output_path, "wb", encoding="utf-8") as file_writer:
            file_writer.write(upload_str)


def generate_fake_controllers(robot, robot_config, output_path=None, ns_=None):
    output_str = "controller_list:\n"
    for manipulator in robot_config.manipulators:
        if manipulator.has_arm:
            # Read arm srdf
            arm_yaml_path = manipulator.arm.moveit_path + "/" + "fake_controllers.yaml"
            with open(arm_yaml_path, 'r', encoding="utf-8") as stream:
                arm_yamldoc = yaml.safe_load(stream)

            output_str += "  - name: fake_" + manipulator.arm.prefix + "controller" + "\n"
            output_str += "    joints:\n"
            for joint in arm_yamldoc["controller_list"][0]["joints"]:
                output_str += "     - " + manipulator.arm.prefix + joint + "\n"

        if manipulator.has_hand:
            sh_group = None
            for group in robot.groups:
                if group.name == manipulator.hand.internal_name:
                    sh_group = group
                    break
            output_str += "  - name: fake_" + manipulator.hand.prefix + "controller\n"
            output_str += "    joints:\n"
            if len(sh_group.joints) == 0:
                output_str += "      []\n"
            else:
                for joint in sh_group.joints:
                    if joint.name[-3:] != "tip":
                        output_str += "      - " + joint.name + "\n"

    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_follow_joint_trajectory_controller(prefix, joints):
    output_str = "  - name: " + prefix + "trajectory_controller\n"
    output_str += "    action_ns: follow_joint_trajectory\n"
    output_str += "    type: FollowJointTrajectory\n"
    output_str += "    default: true\n"
    output_str += "    joints:\n"
    if len(joints) == 0:
        output_str += "      []\n"
    else:
        for joint in joints:
            output_str += "      - " + joint + "\n"
    return output_str


def generate_real_controllers(robot, robot_config, output_path=None, ns_=None):
    output_str = "controller_list:\n"
    for manipulator in robot_config.manipulators:
        if manipulator.has_arm:
            # Read arm srdf
            arm_yaml_path = manipulator.arm.moveit_path + "/" + "controllers.yaml"
            with open(arm_yaml_path, 'r', encoding="utf-8") as stream:
                arm_yamldoc = yaml.safe_load(stream)

            arm_joints = [manipulator.arm.prefix + joint for joint in arm_yamldoc["controller_list"][0]["joints"]]
            output_str += generate_follow_joint_trajectory_controller(manipulator.arm.prefix, arm_joints)

        if manipulator.has_hand:
            sh_group = None
            for group in robot.groups:
                if group.name == manipulator.hand.internal_name:
                    sh_group = group
                    break
            hand_joints = []
            wrist_joints = []
            for joint in sh_group.joints:
                name = joint.name
                if name[-3:] != "tip":
                    if name[-4:-2] == "WR":
                        wrist_joints.append(name)
                    else:
                        hand_joints.append(name)
            output_str += generate_follow_joint_trajectory_controller(manipulator.hand.prefix, hand_joints)
            if wrist_joints:
                output_str += generate_follow_joint_trajectory_controller(manipulator.hand.prefix + 'wr_', wrist_joints)

    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_ompl_planning(robot, robot_config,  # pylint: disable=R0915
                           hand_template_path="ompl_planning_template.yaml", output_path=None, ns_=None):
    with open(hand_template_path, 'r', encoding="utf-8") as stream:
        hand_yamldoc = yaml.safe_load(stream)
    output_str = "planner_configs:\n"
    output_str += yaml_reindent(yaml.dump(hand_yamldoc["planner_configs"],
                                          default_flow_style=False, allow_unicode=True), 2) + "\n"

    for manipulator in robot_config.manipulators:  # pylint: disable=R1702
        if manipulator.has_arm:
            with open(f"{manipulator.arm.moveit_path}/ompl_planning.yaml", 'r', encoding="utf-8") as stream:
                arm_yamldoc = yaml.safe_load(stream)
            if manipulator.arm.extra_groups_config_path:
                filename = f"{manipulator.arm.extra_groups_config_path}/ompl_planning_extra_groups.yaml"
                with open(filename, 'r', encoding="utf-8") as stream:
                    arm_yamldoc_extra_groups = yaml.safe_load(stream)
            prefix = manipulator.arm.prefix
            for group in robot.groups:
                group_name = group.name
                if group_name == manipulator.arm.internal_name:
                    group_name = manipulator.arm.main_group
                    group_prefix = prefix
                elif manipulator.arm.internal_name in group_name:
                    group_prefix, group_name = group_name.split("_", 1)
                    group_prefix = prefix
                else:
                    group_name = group.name[len(prefix):]
                    group_prefix = group.name[:len(prefix)]

                if group_name in arm_yamldoc and group_prefix == prefix:
                    output_str += group.name + ":\n"
                    group_config = arm_yamldoc[group_name]
                    # prepend prefix on projection_evaluator
                    if prefix:
                        if "projection_evaluator" in group_config:
                            proj_eval = group_config["projection_evaluator"]
                            proj_eval.strip()
                            joints = [word for word in re.split(r'\W+', proj_eval) if word not in ["joints", ""]]
                            proj_eval_new = "joints("
                            for joint in joints:
                                proj_eval_new = proj_eval_new + prefix + joint + ","
                            group_config["projection_evaluator"] = proj_eval_new[:-1] + ")"
                        group_dump = yaml.dump(group_config,
                                               default_flow_style=False,
                                               allow_unicode=True)
                        output_str += yaml_reindent(group_dump, 2) + "\n"
                elif group_name in arm_yamldoc_extra_groups:
                    output_str += group.name + ":\n"
                    group_config = arm_yamldoc_extra_groups[group_name]
                    # prepend prefix on projection_evaluator
                    if prefix:
                        if "projection_evaluator" in group_config:
                            proj_eval = group_config["projection_evaluator"]
                            proj_eval.strip()
                            joints = [word for word in re.split(r'\W+', proj_eval) if word not in ["joints", ""]]
                            proj_eval_new = "joints("
                            for joint in joints:
                                proj_eval_new = proj_eval_new + prefix + joint + ","
                            proj_eval_new = proj_eval_new[:-1] + ")"
                            group_config["projection_evaluator"] = proj_eval_new
                        group_dump = yaml.dump(group_config,
                                               default_flow_style=False,
                                               allow_unicode=True)
                        output_str += yaml_reindent(group_dump, 2)
                        output_str += "\n"

        if manipulator.has_hand:
            with open(hand_template_path, 'r', encoding="utf-8") as stream:
                hand_yamldoc = yaml.safe_load(stream)
            prefix = manipulator.hand.prefix
            if prefix:
                proj_eval_re = re.compile(r'joints\(([TFMRLW][FHR]J[0-5]),([TFMRLW][FHR]J[0-5])\)')
            for group in robot.groups:
                group_name = group.name
                if group_name == manipulator.hand.internal_name:
                    group_name = "hand"
                    group_prefix = prefix
                else:
                    group_name = group.name[len(prefix):]
                    group_prefix = group.name[:len(prefix)]
                if group_name in hand_yamldoc and group_prefix == prefix:
                    output_str += group.name + ":\n"
                    group_config = hand_yamldoc[group_name]
                    # prepend prefix on projection_evaluator
                    if prefix:
                        if "projection_evaluator" in group_config:
                            proj_eval = group_config["projection_evaluator"]
                            proj_eval.strip()
                            proj_eval_new = proj_eval_re.sub(r'joints(' +
                                                             prefix +
                                                             r'\g<1>,' +
                                                             prefix +
                                                             r'\g<2>)',
                                                             proj_eval)
                            group_config["projection_evaluator"] = proj_eval_new
                    group_dump = yaml.dump(group_config,
                                           default_flow_style=False,
                                           allow_unicode=True)
                    output_str += yaml_reindent(group_dump, 2)
                    output_str += "\n"
            stream.close()

    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_kinematics(robot, robot_config, ns_=None,  # pylint: disable=R0914,R0915
                        hand_template_path="kinematics_template.yaml", output_path=None,
                        kinematics_file="kinematics.yaml", kinematics_extra_file="kinematics_extra_groups.yaml"):
    output_str = ""
    while not rospy.has_param('/robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")

    for manipulator in robot_config.manipulators:  # pylint: disable=R1702
        if manipulator.has_arm:
            with open(f"{manipulator.arm.moveit_path}/{kinematics_file}", 'r', encoding="utf-8") as stream:
                arm_yamldoc = yaml.safe_load(stream)
            if manipulator.arm.extra_groups_config_path:
                filename = f"{manipulator.arm.extra_groups_config_path}/{kinematics_extra_file}"
                with open(filename, 'r', encoding="utf-8") as stream:
                    arm_yamldoc_extra_groups = yaml.safe_load(stream)
            prefix = manipulator.arm.prefix
            for group in robot.groups:
                group_name = group.name
                if group_name == manipulator.arm.internal_name:
                    group_name = manipulator.arm.main_group
                    group_prefix = prefix
                elif manipulator.arm.internal_name in group_name:
                    group_prefix, group_name = group_name.split("_", 1)
                    group_prefix = prefix
                elif group_name.startswith("two"):
                    group_name = group.name
                    group_prefix = prefix
                else:
                    group_name = group.name[len(prefix):]
                    group_prefix = group.name[:len(prefix)]

                if group_name in arm_yamldoc and group_prefix == prefix:
                    kinematics_config = arm_yamldoc[group_name]
                    if prefix:
                        if "tip_name" in kinematics_config:
                            kinematics_config["tip_name"] = prefix + kinematics_config["tip_name"]
                        if "root_name" in kinematics_config:
                            kinematics_config["root_name"] = prefix + kinematics_config["root_name"]

                    output_str += group.name + ":\n"
                    output_str += yaml_reindent(yaml.dump(kinematics_config, default_flow_style=False,
                                                          allow_unicode=True), 2) + "\n"
                elif group_name in arm_yamldoc_extra_groups:
                    kinematics_config = arm_yamldoc_extra_groups[group_name]
                    if prefix:
                        if "tip_name" in kinematics_config:
                            kinematics_config["tip_name"] = prefix + kinematics_config["tip_name"]
                        if "root_name" in kinematics_config:
                            kinematics_config["root_name"] = prefix + kinematics_config["root_name"]

                    output_str += group.name + ":\n"
                    output_str += yaml_reindent(yaml.dump(kinematics_config,
                                                          default_flow_style=False,
                                                          allow_unicode=True), 2) + "\n"
        if manipulator.has_hand:
            # open hand template files
            with open(hand_template_path, 'r', encoding="utf-8") as stream:
                hand_yamldoc = yaml.safe_load(stream)

            if 'kinematics_template' in hand_template_path:
                fixed_joint_template_path = rospkg.RosPack().get_path('sr_moveit_hand_config') + \
                                            "/config/kinematics_trac_ik_template.yaml"
                with open(fixed_joint_template_path, 'r', encoding="utf-8") as stream:
                    hand_yamldoc_fixed_joint = yaml.safe_load(stream)
            else:
                hand_yamldoc_fixed_joint = deepcopy(hand_yamldoc)

            prefix = manipulator.hand.prefix
            finger_prefixes = ["FF", "MF", "RF", "LF", "TH"]

            # Find in any finger has a fix joint apart from the tip as it needs to use a different kinematics
            is_fixed = {"first_finger": False, "middle_finger": False, "ring_finger": False,
                        "little_finger": False, "thumb": False}
            finger_with_fixed_joint = [False, False, False, False, False]
            for joint in URDF.from_xml_string(rospy.get_param('/robot_description')).joints:
                joint_name = joint.name[len(prefix):]
                for index, finger_prefix in enumerate(finger_prefixes):
                    if joint_name[0:2].upper() == finger_prefix and joint_name[-3:] != "tip" and joint.type == "fixed":
                        finger_with_fixed_joint[index] = True
            is_fixed['first_finger'] = finger_with_fixed_joint[0]
            is_fixed['middle_finger'] = finger_with_fixed_joint[1]
            is_fixed['ring_finger'] = finger_with_fixed_joint[2]
            is_fixed['little_finger'] = finger_with_fixed_joint[3]
            is_fixed['thumb'] = finger_with_fixed_joint[4]

            for group in robot.groups:
                kinematics_config = None
                group_name = group.name
                if group_name == manipulator.hand.internal_name:
                    group_name = "hand"
                    group_prefix = prefix
                else:
                    group_name = group.name[len(prefix):]
                    group_prefix = group.name[:len(prefix)]

                if group_name in hand_yamldoc and group_prefix == prefix:
                    if is_fixed.get(group_name):
                        kinematics_config = hand_yamldoc_fixed_joint[group_name]
                    else:
                        kinematics_config = hand_yamldoc[group_name]

                if kinematics_config is not None:
                    if prefix:
                        if "tip_name" in kinematics_config:
                            kinematics_config["tip_name"] = prefix + kinematics_config["tip_name"]
                        if "root_name" in kinematics_config:
                            kinematics_config["root_name"] = prefix + kinematics_config["root_name"]

                    output_str += group.name + ":\n"
                    output_str += yaml_reindent(yaml.dump(kinematics_config, default_flow_style=False,
                                                          allow_unicode=True), 2) + "\n"
    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_joint_limits(robot, robot_config, hand_template_path="joint_limits_template.yaml",
                          output_path=None, ns_=None):
    output_str = ""
    output_str += "joint_limits:\n"
    for manipulator in robot_config.manipulators:
        if manipulator.has_arm:
            # Read arm srdf
            arm_yaml_path = f"{manipulator.arm.moveit_path}/joint_limits.yaml"
            with open(arm_yaml_path, 'r', encoding="utf-8") as stream:
                arm_yamldoc = yaml.safe_load(stream)
            for joint in arm_yamldoc["joint_limits"]:
                joint_limits_config = arm_yamldoc["joint_limits"][joint]
                output_str += "  " + manipulator.arm.prefix + joint + ":\n"
                joint_limits_dump = yaml.dump(
                    joint_limits_config,
                    default_flow_style=False,
                    allow_unicode=True)
                output_str += yaml_reindent(joint_limits_dump, 4)
                output_str += "\n"
        if manipulator.has_hand:
            with open(hand_template_path, 'r', encoding="utf-8") as stream:
                hand_yamldoc = yaml.safe_load(stream)
            group_name = manipulator.hand.internal_name
            if group_name is not None:
                # for each joint in full hand group
                for joint in robot.group_map[group_name].joints:
                    joint_name = joint.name[-4:]
                    if joint_name in hand_yamldoc["joint_limits"]:
                        joint_limits_config = hand_yamldoc["joint_limits"][joint_name]
                        output_str += "  " + joint.name + ":\n"
                        joint_limits_dump = yaml.dump(
                            joint_limits_config,
                            default_flow_style=False,
                            allow_unicode=True)
                        output_str += yaml_reindent(joint_limits_dump, 4)
                        output_str += "\n"
                stream.close()

    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


if __name__ == '__main__':

    PARSER = argparse.ArgumentParser(usage='Load an SRDF file')
    PARSER.add_argument('file', type=argparse.FileType('r'), nargs='?',
                        default=None,
                        help='File to load. Use - for stdin')
    ARGS = PARSER.parse_args()

    if ARGS.file is not None:

        ROBOT = SRDF.from_xml_string(ARGS.file.read())
        generate_fake_controllers(ROBOT,  # pylint: disable=E1120
                                  output_path="fake_controllers.yaml")
        generate_real_controllers(ROBOT,  # pylint: disable=E1120
                                  output_path="controllers.yaml")
        generate_ompl_planning(ROBOT,
                               "ompl_planning_template.yaml",
                               output_path="ompl_planning.yaml")
        generate_kinematics(ROBOT,
                            "kinematics_template.yaml",
                            output_path="kinematics.yaml",
                            kinematics_file="kinematics.yaml",
                            kinematics_extra_file="kinematics_extra_groups.yaml")
        generate_joint_limits(ROBOT,
                              "joint_limits_template.yaml",
                              output_path="joint_limits.yaml")
    else:
        rospy.logerr("No SRDF file provided")
