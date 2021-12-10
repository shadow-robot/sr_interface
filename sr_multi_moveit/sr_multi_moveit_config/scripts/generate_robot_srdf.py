#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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

from __future__ import absolute_import
import sys
import os
from xml.dom.minidom import parse
import rospkg
import xml
import yaml
from copy import deepcopy
from shutil import copy2

import xacro
import rospy

from sr_moveit_hand_config.generate_hand_srdf import SRDFHandGenerator
from urdf_parser_py.urdf import URDF


class SRDFRobotGeneratorException(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


class Subrobot(object):
    def __init__(self, type):
        self.type = type
        self.name = ""
        self.internal_name = ""
        self.main_group = ""
        self.other_groups = []
        self.group_states = []
        self.is_lite = False
        self.prefix = ""
        self.moveit_path = ""
        self.urdf_args = {}


class Manipulator(object):
    def __init__(self, name, side, has_arm, has_hand):
        self.name = name
        self.side = side
        self.has_arm = has_arm
        self.has_hand = has_hand

        if has_arm:
            self.arm = Subrobot("arm")
            if self.side == "right":
                self.arm.prefix = "ra_"
                self.arm.internal_name = "right_arm"
            else:
                self.arm.prefix = "la_"
                self.arm.internal_name = "left_arm"
        if has_hand:
            self.hand = Subrobot("hand")
            if self.side == "right":
                self.hand.prefix = "rh_"
                self.hand.internal_name = "right_hand"
            else:
                self.hand.prefix = "lh_"
                self.hand.internal_name = "left_hand"


def next_element(elt):
    child = xacro.first_child_element(elt)
    if child:
        return child
    while elt and elt.nodeType == xml.dom.Node.ELEMENT_NODE:
        next = xacro.next_sibling_element(elt)
        if next:
            return next
        elt = elt.parentNode
    return None


class Robot(object):
    def __init__(self):
        self.name = ""
        self.manipulators = []

    def set_parameters(self, yamldoc):
        # Read robot description yaml
        if "robot" in yamldoc:
            robot_yaml = yamldoc["robot"]
            self.name = robot_yaml["name"]
            if "manipulators" in robot_yaml:
                manipulators_yaml = robot_yaml["manipulators"]
                for manipulator_name in manipulators_yaml.keys():
                    manipulator_yaml = manipulators_yaml[manipulator_name]
                    side = manipulator_yaml["side"]
                    if side not in ["left", "right"]:
                        raise SRDFRobotGeneratorException("robot description did not specify " +
                                                          "a correct side for a manipulator")
                    has_arm = True if "arm" in manipulator_yaml else False
                    has_hand = True if "hand" in manipulator_yaml else False
                    if not has_hand and not has_arm:
                        raise SRDFRobotGeneratorException("robot description did not specify " +
                                                          "either an arm or hand for a manipulator")
                    # TODO: check that each manipulator do not have more than one arm and hand
                    manipulator = Manipulator(manipulator_name, side, has_arm, has_hand)
                    if has_arm:
                        arm_yaml = manipulator_yaml["arm"]
                        if "name" in arm_yaml:
                            manipulator.arm.name = arm_yaml["name"]
                        else:
                            raise SRDFRobotGeneratorException("arm name should be specified")
                        if "moveit_path" in arm_yaml:
                            package_name = arm_yaml["moveit_path"]["package"]
                            relative_path = arm_yaml["moveit_path"]["relative_path"]
                            manipulator.arm.moveit_path = rospkg.RosPack().get_path(package_name) + relative_path
                        if "extra_groups_config_path" in arm_yaml:
                            relative_path = arm_yaml["extra_groups_config_path"]
                            manipulator.arm.extra_groups_config_path = \
                                rospkg.RosPack().get_path("sr_multi_moveit_config") + relative_path
                        if "main_group" in arm_yaml:
                            manipulator.arm.main_group = arm_yaml["main_group"]
                        else:
                            raise SRDFRobotGeneratorException("arm main_group should be specified")
                        if "other_groups" in arm_yaml:
                            for group in arm_yaml["other_groups"]:
                                manipulator.arm.other_groups.append(group)
                        if "group_states" in arm_yaml:
                            for group_state in arm_yaml["group_states"]:
                                manipulator.arm.group_states.append(group_state)

                    if has_hand:
                        hand_yaml = manipulator_yaml["hand"]
                        if "name" in hand_yaml:
                            manipulator.hand.name = hand_yaml["name"]
                        if "main_group" in hand_yaml:
                            manipulator.hand.main_group = hand_yaml["main_group"]
                        if "other_groups" in hand_yaml:
                            for group in hand_yaml["other_groups"]:
                                manipulator.hand.other_groups.append(group)
                        if "group_states" in hand_yaml:
                            for group_state in hand_yaml["group_states"]:
                                manipulator.hand.group_states.append(group_state)
                        if "is_lite" in hand_yaml:
                            manipulator.hand.is_lite = bool(hand_yaml["is_lite"])
                        if "urdf_args" in hand_yaml:
                            manipulator.hand.urdf_args = hand_yaml["urdf_args"]

                    self.manipulators.append(manipulator)
        else:
            raise SRDFRobotGeneratorException("robot description did not specify a robot")


class SRDFRobotGenerator(object):
    def __init__(self, description_file=None, load=True):
        if description_file is None and len(sys.argv) > 1:
            description_file = sys.argv[1]

        self._save_files = rospy.get_param('~save_files', False)
        self._path_to_save_files = rospy.get_param('~path_to_save_files', "/tmp/")
        self._file_name = rospy.get_param('~file_name', "generated_robot")

        # ARM
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('sr_multi_moveit_config')

        self.robot = Robot()

        with open(description_file, "r") as stream:
            yamldoc = yaml.safe_load(stream)

        self.robot.set_parameters(yamldoc)
        self.arm_srdf_xmls = []
        self.hand_srdf_xmls = []

        new_srdf_file_name = "generated_robot.srdf"
        self.start_new_srdf(new_srdf_file_name)
        for manipulator_id, manipulator in enumerate(self.robot.manipulators):
            if manipulator.has_arm:
                # Read arm srdf
                arm_srdf_path = manipulator.arm.moveit_path + "/" + manipulator.arm.name + ".srdf"
                with open(arm_srdf_path, 'r') as stream:
                    self.arm_srdf_xmls.append(parse(stream))
                xacro.process_doc(self.arm_srdf_xmls[manipulator_id])

            if manipulator.has_hand:
                hand_urdf = rospy.get_param('hand_description')
                srdfHandGenerator = SRDFHandGenerator(hand_urdf, load=False, save=False)
                self.hand_srdf_xmls.append(srdfHandGenerator.get_hand_srdf())

            comment = ["Manipulator:" + manipulator.name]
            self.add_comments(comment)

            # Add groups and group states
            if manipulator.has_arm:
                self.parse_arm_groups(manipulator_id, manipulator)
                self.add_more_arm_groups(manipulator)
            if manipulator.has_hand:
                self.parse_hand_groups(manipulator_id, manipulator)

        # Add groups for bimanual arm and hand systems
        if len(self.robot.manipulators) == 2:
            if self.robot.manipulators[0].has_arm and self.robot.manipulators[1].has_arm:
                if self.robot.manipulators[0].has_hand and self.robot.manipulators[1].has_hand:
                    comment = ["Bimanual groups with hands"]
                    self.add_bimanual_arm_groups(self.robot.manipulators[0].arm.internal_name,
                                                 self.robot.manipulators[1].arm.internal_name)
                else:
                    comment = ["Bimanual groups without hands"]
                    self.add_bimanual_arm_groups(self.robot.manipulators[0].arm.internal_name,
                                                 self.robot.manipulators[1].arm.internal_name,
                                                 False)
                self.add_comments(comment)

            if self.robot.manipulators[0].has_hand and self.robot.manipulators[1].has_hand:
                comment = ["Bimanual groups with hands"]
                self.add_bimanual_hand_groups(self.robot.manipulators[0].hand.internal_name,
                                              self.robot.manipulators[1].hand.internal_name)
                self.add_comments(comment)

        for manipulator_id, manipulator in enumerate(self.robot.manipulators):

            # Add end effectors
            comment = ["END EFFECTOR: Purpose: Represent information about an end effector."]
            self.add_comments(comment)
            if manipulator.has_hand:
                self.parse_hand_end_effectors(manipulator_id, manipulator)
            if manipulator.has_arm:
                self.parse_arm_end_effectors(manipulator_id, manipulator)

            # Add virtual joints
            if manipulator.has_arm:
                pass
                # self.parse_arm_virtual_joint(manipulator)
            else:
                self.parse_hand_virtual_joint(manipulator_id, manipulator)

            # Add disable collisions
            comment = ["DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially " +
                       "come into collision with any other link in the robot. This tag disables collision checking " +
                       "between a specified pair of links."]
            self.add_comments(comment)
            if manipulator.has_arm:
                self.parse_arm_collisions(manipulator_id, manipulator)
            if manipulator.has_hand:
                self.parse_hand_collisions(manipulator_id, manipulator)

        # Finish and close file
        self.new_robot_srdf.write('</robot>\n')
        self.new_robot_srdf.close()
        if load:
            with open(self.package_path + "/config/" + new_srdf_file_name, 'r') as stream:
                srdf = parse(stream)

            rospy.loginfo("Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name('robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            srdf.toprettyxml(indent='  '))

        if self._save_files:
            rospy.loginfo("Robot urdf and srdf have been saved to %s" % self._path_to_save_files)

            # srdf: File is already generated so just need to be copied to specified location
            copy2(self.package_path + "/config/" + new_srdf_file_name, self._path_to_save_files +
                  "/" + self._file_name + ".srdf")

            # urdf: File can be copied from rosparam
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                urdf_file = open(self._path_to_save_files + "/" + self._file_name + ".urdf", "wb")
                urdf_file.write(urdf_str)
                urdf_file.close()

        rospy.loginfo("generated_robot.srdf has been generated and saved.")

    def start_new_srdf(self, file_name):
        # Generate new robot srdf with arm information
        self.new_robot_srdf = open(self.package_path + "/config/" + file_name, 'w')

        self.new_robot_srdf.write('<?xml version="1.0" ?>\n')
        banner = ["This does not replace URDF, and is not an extension of URDF.\n" +
                  "    This is a format for representing semantic information about the robot structure.\n" +
                  "    A URDF file must exist for this robot as well, where the joints and the links that are" +
                  "referenced are defined\n"]
        self.add_comments(banner, "")
        self.new_robot_srdf.write('<robot name="' + self.robot.name + '">\n')
        comments = ["GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to " +
                    "plan for, defining arms, end effectors, etc",
                    "LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically" +
                    "included",
                    "JOINTS: When a joint is specified, the child link of that joint (which will always exist) is" +
                    "automatically included",
                    "CHAINS: When a chain is specified, all the links along the chain (including endpoints) are" +
                    "included in the group. Additionally, all the joints that are parents to included links are " +
                    "also included. This means that joints along the chain and the parent joint of the base link " +
                    "are included in the group",
                    "SUBGROUPS: Groups can also be formed by referencing to already defined group names"]
        self.add_comments(comments)

    def parse_arm_groups(self, manipulator_id, manipulator):
        previous = self.arm_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'group':
                # Check it is not a subgroup
                if len(elt.childNodes) > 0:
                    group_name = elt.getAttribute('name')
                    if group_name == manipulator.arm.main_group or group_name in manipulator.arm.other_groups:
                        if group_name == manipulator.arm.main_group:
                            elt.setAttribute('name', manipulator.arm.internal_name)
                        else:
                            elt.setAttribute('name', manipulator.arm.prefix + group_name)
                        for index, group_element in enumerate(elt.getElementsByTagName("chain")):
                            attributes = ["base_link", "tip_link"]
                            for attribute in attributes:
                                node_attribute = group_element.getAttributeNode(attribute)
                                node_attribute.nodeValue = (manipulator.arm.prefix +
                                                            group_element.getAttribute(attribute))
                        for tagName in ["joint", "link", "group"]:
                            for index, group_element in enumerate(elt.getElementsByTagName(tagName)):
                                attribute_name = group_element.getAttribute("name")
                                node_attribute = group_element.getAttributeNode("name")
                                node_attribute.nodeValue = (manipulator.arm.prefix + attribute_name)

                        elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                    if group_name == manipulator.arm.main_group and (manipulator.has_hand and
                                                                     not manipulator.hand.is_lite):
                        elt.setAttribute('name', manipulator.arm.internal_name + "_and_wrist")
                        for group_element in elt.getElementsByTagName("chain"):
                            node_attribute = group_element.getAttributeNode("tip_link")
                            node_attribute.nodeValue = (manipulator.hand.prefix + "palm")
                        if len(elt.getElementsByTagName("joint")) > 0:
                            node = deepcopy(elt.getElementsByTagName("joint")[0])
                            node_attribute = node.getAttributeNode("name")
                            node_attribute.nodeValue = (manipulator.hand.prefix + "WRJ2")
                            newatt = elt.appendChild(node)
                            node = deepcopy(elt.getElementsByTagName("joint")[0])
                            node_attribute = node.getAttributeNode("name")
                            node_attribute.nodeValue = (manipulator.hand.prefix + "WRJ1")
                            newatt = elt.appendChild(node)
                        elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                    if group_name == manipulator.arm.main_group and (manipulator.has_hand):
                        elt.setAttribute('name', manipulator.arm.internal_name + "_and_manipulator")
                        for group_element in elt.getElementsByTagName("chain"):
                            node_attribute = group_element.getAttributeNode("tip_link")
                            node_attribute.nodeValue = (manipulator.hand.prefix + "manipulator")
                        elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                        new_group = xml.dom.minidom.Document().createElement('group')
                        new_group.setAttribute("name", manipulator.arm.internal_name + "_and_hand")
                        if manipulator.hand.is_lite:
                            arm_group = xml.dom.minidom.Document().createElement('group name="' +
                                                                                 manipulator.arm.internal_name + '"')
                        else:
                            arm_group = xml.dom.minidom.Document().createElement('group name="' +
                                                                                 manipulator.arm.internal_name + '"')
                        new_group.appendChild(arm_group)
                        hand_group = xml.dom.minidom.Document().createElement('group name="' +
                                                                              manipulator.hand.internal_name + '"')
                        new_group.appendChild(hand_group)
                        new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            elif elt.tagName == 'group_state':
                group_state_name = elt.getAttribute('name')
                group_name = elt.getAttribute('group')
                if group_state_name in manipulator.arm.group_states and (group_name == manipulator.arm.main_group or
                                                                         group_name in manipulator.arm.other_groups):
                    elt.setAttribute('name', manipulator.arm.prefix + group_state_name)
                    if group_name == manipulator.arm.main_group:
                        elt.setAttribute('group', manipulator.arm.internal_name)
                    else:
                        elt.setAttribute('group', manipulator.arm.prefix + group_name)
                    for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                        attribute_name = group_element.getAttribute("name")
                        if attribute_name in ["WRJ1", "WRJ2"]:
                            if manipulator.has_hand:
                                if not manipulator.hand.is_lite:
                                    node_attribute = group_element.getAttributeNode("name")
                                    node_attribute.nodeValue = (manipulator.hand.prefix + attribute_name)
                            else:
                                group_element.parentNode.removeChild(group_element)
                        else:
                            node_attribute = group_element.getAttributeNode("name")
                            node_attribute.nodeValue = (manipulator.arm.prefix + attribute_name)
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def add_more_arm_groups(self, manipulator):
        new_group = xml.dom.minidom.Document().createElement('group_state')
        new_group.setAttribute("group", manipulator.arm.internal_name)
        new_group.setAttribute("name", manipulator.arm.prefix + "start")
        if manipulator.side == "right":
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'elbow_joint" value="2.0"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'shoulder_lift_joint" value="-1.25"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'shoulder_pan_joint" value="0.0"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_1_joint" value="-0.733"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_2_joint" value="1.5708"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_3_joint" value="-3.1416"')
            new_group.appendChild(joint)
            new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
        else:
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'elbow_joint" value="-2.0"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'shoulder_lift_joint" value="-1.89"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'shoulder_pan_joint" value="0.0"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_1_joint" value="-2.4"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_2_joint" value="-1.5708"')
            new_group.appendChild(joint)
            joint = xml.dom.minidom.Document().createElement(
                'joint name="' + manipulator.arm.prefix + 'wrist_3_joint" value="3.1416"')
            new_group.appendChild(joint)
            new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def add_bimanual_arm_groups(self, group_1, group_2, hands=True):
        new_group = xml.dom.minidom.Document().createElement('group')
        new_group.setAttribute("name", "two_arms")
        arm_group_1 = xml.dom.minidom.Document().createElement('group name="' + group_1 + '"')
        new_group.appendChild(arm_group_1)
        arm_group_2 = xml.dom.minidom.Document().createElement('group name="' + group_2 + '"')
        new_group.appendChild(arm_group_2)
        new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

        if hands:
            new_group = xml.dom.minidom.Document().createElement('group')
            new_group.setAttribute("name", "two_arms_and_hands")
            arm_group_1 = xml.dom.minidom.Document().createElement('group name="' + group_1 + '_and_hand"')
            arm_group_2 = xml.dom.minidom.Document().createElement('group name="' + group_2 + '_and_hand"')
            new_group.appendChild(arm_group_1)
            new_group.appendChild(arm_group_2)
            new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

            new_group = xml.dom.minidom.Document().createElement('group')
            new_group.setAttribute("name", "two_arms_and_wrists")
            arm_group_1 = xml.dom.minidom.Document().createElement('group name="' + group_1 + '_and_wrist"')
            arm_group_2 = xml.dom.minidom.Document().createElement('group name="' + group_2 + '_and_wrist"')
            new_group.appendChild(arm_group_1)
            new_group.appendChild(arm_group_2)
            new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def add_bimanual_hand_groups(self, group_1, group_2):
        new_group = xml.dom.minidom.Document().createElement('group')
        new_group.setAttribute("name", "two_hands")
        hand_group_1 = xml.dom.minidom.Document().createElement('group name="' + group_1 + '"')
        new_group.appendChild(hand_group_1)
        hand_group_2 = xml.dom.minidom.Document().createElement('group name="' + group_2 + '"')
        new_group.appendChild(hand_group_2)
        new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def parse_hand_groups(self, manipulator_id, manipulator):
        previous = self.hand_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'group':
                # Check it is not a subgroup
                if len(elt.childNodes) > 0:
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            elif elt.tagName == 'group_state':
                for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                    attribute_name = group_element.getAttribute("name")
                    attribute_name = attribute_name.split("_")[1]
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_hand_end_effectors(self, manipulator_id, manipulator):
        previous = self.hand_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_arm_end_effectors(self, manipulator_id, manipulator):
        previous = self.arm_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                if manipulator.has_hand:
                    elt.getAttributeNode("name").nodeValue = manipulator.arm.internal_name + "_ee"
                    elt.getAttributeNode("parent_link").nodeValue = (manipulator.arm.prefix +
                                                                     elt.getAttribute("parent_link"))
                    elt.getAttributeNode("group").nodeValue = manipulator.hand.internal_name
                    elt.setAttribute('parent_group', manipulator.arm.internal_name)
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                    newElement = deepcopy(elt)
                    newElement.getAttributeNode("name").nodeValue = manipulator.arm.prefix + "and_manipulator_ee"
                    newElement.getAttributeNode("parent_link").nodeValue = manipulator.hand.prefix + "manipulator"
                    newElement.getAttributeNode("group").nodeValue = manipulator.hand.prefix + "fingers"
                    newElement.setAttribute('parent_group', manipulator.arm.internal_name + "_and_manipulator")
                    newElement.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                    if not manipulator.hand.is_lite:
                        newElement = deepcopy(elt)
                        newElement.getAttributeNode("name").nodeValue = manipulator.arm.prefix + "and_wrist_ee"
                        newElement.getAttributeNode("parent_link").nodeValue = manipulator.hand.prefix + "palm"
                        newElement.getAttributeNode("group").nodeValue = manipulator.hand.prefix + "fingers"
                        newElement.setAttribute('parent_group', manipulator.arm.internal_name + "_and_wrist")
                        newElement.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                else:
                    elt.getAttributeNode("name").nodeValue = manipulator.arm.internal_name + "_ee"
                    elt.getAttributeNode("parent_link").nodeValue = (manipulator.arm.prefix +
                                                                     elt.getAttribute("parent_link"))
                    elt.getAttributeNode("group").nodeValue = (manipulator.arm.prefix +
                                                               elt.getAttribute("group"))
                    elt.setAttribute('parent_group', manipulator.arm.internal_name)
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_arm_virtual_joint(self, manipulator_id, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an " +
                   "external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.arm_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.getAttributeNode("name").nodeValue = "world_to_" + manipulator.arm.internal_name
                elt.getAttributeNode("child_link").nodeValue = manipulator.arm.prefix + elt.getAttribute("child_link")
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_hand_virtual_joint(self, manipulator_id, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an " +
                   "external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.hand_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_arm_collisions(self, manipulator_id, manipulator):
        comment = [manipulator.arm.internal_name + " collisions"]
        self.add_comments(comment)
        previous = self.arm_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)

        while elt:
            if elt.tagName == 'disable_collisions':
                elt.getAttributeNode("link1").nodeValue = (manipulator.arm.prefix + elt.getAttribute("link1"))
                elt.getAttributeNode("link2").nodeValue = (manipulator.arm.prefix + elt.getAttribute("link2"))
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                newElement = deepcopy(elt)

            previous = elt
            elt = next_element(previous)
        # Add collisions between arm and hand
        if manipulator.has_hand:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                robot = URDF.from_xml_string(urdf_str)
                arm_chain = robot.get_chain("world", manipulator.hand.prefix + "forearm", joints=False, fixed=False)

                newElement.getAttributeNode("link1").nodeValue = arm_chain[-2]
                newElement.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                newElement.getAttributeNode("reason").nodeValue = "Adjacent"
                newElement.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

                newElement.getAttributeNode("link1").nodeValue = arm_chain[-3]
                newElement.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                newElement.getAttributeNode("reason").nodeValue = "Adjacent"
                newElement.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

                newElement.getAttributeNode("link1").nodeValue = arm_chain[-4]
                newElement.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                newElement.getAttributeNode("reason").nodeValue = "Adjacent"
                newElement.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def parse_hand_collisions(self, manipulator_id, manipulator):
        comment = [manipulator.hand.internal_name + " collisions"]
        self.add_comments(comment)
        previous = self.hand_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'disable_collisions':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

            previous = elt
            elt = next_element(previous)

    def add_comments(self, comments, addindent="  "):
        xml_comments = [xml.dom.minidom.Comment(c) for c in comments]
        for comment in xml_comments:
            comment.writexml(self.new_robot_srdf, indent=addindent, addindent=addindent, newl="\n")


if __name__ == '__main__':
    rospy.init_node('robot_srdf_generator', anonymous=True)
    robot_generator = SRDFRobotGenerator()
