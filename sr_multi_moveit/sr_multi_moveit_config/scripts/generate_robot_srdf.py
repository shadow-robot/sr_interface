#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Shadow Robot Company
# All rights reserved.

import sys
import os
from xml.dom.minidom import parse
import rospkg
import xml
import yaml
from copy import deepcopy

import xacro
import rospy

from sr_moveit_hand_config.generate_hand_srdf import SRDFHandGenerator
from sr_utilities.local_urdf_parser_py import URDF


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

                    self.manipulators.append(manipulator)
        else:
            raise SRDFRobotGeneratorException("robot description did not specify a robot")


class SRDFRobotGenerator(object):
    def __init__(self, description_file=None, load=True):
        if description_file is None and len(sys.argv) > 1:
            description_file = sys.argv[1]
        # ARM
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('sr_multi_moveit_config')

        self.robot = Robot()

        with open(description_file, "r") as stream:
            yamldoc = yaml.load(stream)

        self.robot.set_parameters(yamldoc)

        new_srdf_file_name = "generated_robot.srdf"
        self.start_new_srdf(new_srdf_file_name)
        for manipulator in self.robot.manipulators:
            if manipulator.has_arm:
                # Read arm srdf
                arm_srdf_path = manipulator.arm.moveit_path + "/" + manipulator.arm.name + ".srdf"
                with open(arm_srdf_path, 'r') as stream:
                    self.arm_srdf_xml = parse(stream)
                xacro.process_includes(self.arm_srdf_xml, os.path.dirname(sys.argv[0]))
                xacro.eval_self_contained(self.arm_srdf_xml)
            if manipulator.has_hand:
                # Generate and read hand srdf
                hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + manipulator.hand.name
                with open(hand_urdf_path, 'r') as hand_urdf_xacro_file:
                    hand_urdf_xml = parse(hand_urdf_xacro_file)
                xacro.process_includes(hand_urdf_xml, os.path.dirname(sys.argv[0]))
                xacro.eval_self_contained(hand_urdf_xml)
                hand_urdf = hand_urdf_xml.toprettyxml(indent='  ')
                srdfHandGenerator = SRDFHandGenerator(hand_urdf, load=False, save=False)
                self.hand_srdf_xml = srdfHandGenerator.get_hand_srdf()

            comment = ["Manipulator:" + manipulator.name]
            self.add_comments(comment)

            # Add groups and group states
            if manipulator.has_arm:
                self.parse_arm_groups(manipulator)
            if manipulator.has_hand:
                self.parse_hand_groups(manipulator)

            # Add end effectors
            comment = ["END EFFECTOR: Purpose: Represent information about an end effector."]
            self.add_comments(comment)
            if manipulator.has_hand:
                self.parse_hand_end_effectors(manipulator)
            if manipulator.has_arm:
                self.parse_arm_end_effectors(manipulator)

            # Add virtual joints
            if manipulator.has_arm:
                pass
                # self.parse_arm_virtual_joint(manipulator)
            else:
                self.parse_hand_virtual_joint(manipulator)

            # Add disable collisions
            comment = ["DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially " +
                       "come into collision with any other link in the robot. This tag disables collision checking " +
                       "between a specified pair of links."]
            self.add_comments(comment)
            if manipulator.has_arm:
                self.parse_arm_collisions(manipulator)
            if manipulator.has_hand:
                self.parse_hand_collisions(manipulator)

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

    def parse_arm_groups(self, manipulator):
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
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
            elt = xacro.next_element(previous)

    def parse_hand_groups(self, manipulator):
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'group':
                # Check it is not a subgroup
                if len(elt.childNodes) > 0:
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            elif elt.tagName == 'group_state':
                for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                    attribute_name = group_element.getAttribute("name")
                    attribute_name = attribute_name.split("_")[1]
                    if attribute_name in ["WRJ1", "WRJ2"] and manipulator.has_arm:
                        group_element.parentNode.removeChild(group_element)
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_end_effectors(self, manipulator):
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_arm_end_effectors(self, manipulator):
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                elt.getAttributeNode("name").nodeValue = manipulator.arm.prefix + elt.getAttribute("name")
                elt.getAttributeNode("parent_link").nodeValue = (manipulator.arm.prefix +
                                                                 elt.getAttribute("parent_link"))
                elt.getAttributeNode("group").nodeValue = manipulator.arm.internal_name
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_arm_virtual_joint(self, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an " +
                   "external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.getAttributeNode("name").nodeValue = "world_to_" + manipulator.arm.internal_name
                elt.getAttributeNode("child_link").nodeValue = manipulator.arm.prefix + elt.getAttribute("child_link")
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_virtual_joint(self, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an " +
                   "external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_arm_collisions(self, manipulator):
        comment = [manipulator.arm.internal_name + " collisions"]
        self.add_comments(comment)
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'disable_collisions':
                elt.getAttributeNode("link1").nodeValue = (manipulator.arm.prefix + elt.getAttribute("link1"))
                elt.getAttributeNode("link2").nodeValue = (manipulator.arm.prefix + elt.getAttribute("link2"))
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                newElement = deepcopy(elt)

            previous = elt
            elt = xacro.next_element(previous)
        # Add collisions between arm and hand
        if manipulator.has_hand:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                robot_urdf = URDF.from_xml_string(urdf_str)
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

    def parse_hand_collisions(self, manipulator):
        comment = [manipulator.hand.internal_name + " collisions"]
        self.add_comments(comment)
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'disable_collisions':
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

            previous = elt
            elt = xacro.next_element(previous)

    def add_comments(self, comments, addindent="  "):
        xml_comments = [xml.dom.minidom.Comment(c) for c in comments]
        for comment in xml_comments:
            comment.writexml(self.new_robot_srdf, indent=addindent, addindent=addindent, newl="\n")

if __name__ == '__main__':

    rospy.init_node('robot_srdf_generator', anonymous=True)
    robot_generator = SRDFRobotGenerator()
