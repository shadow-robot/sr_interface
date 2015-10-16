#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Shadow Robot Company
# All rights reserved.

import sys
import os
from xml.dom.minidom import parse

import xacro
import rospy
import rospkg
import xml
import yaml
from sr_moveit_hand_config.generate_hand_srdf import SRDFHandGenerator


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


class SRDFRobotGenerator(object):
    def __init__(self, description_file = "/config/description_ur10_sh.yaml", load = True):
        self.manipulators = []


        # ARM
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('sr_multi_moveit_config')

        #description_file = "/config/description_two_sh.yaml"
        description_file = "/config/description_ur10_sh.yaml"
        self.set_parameters(description_file)

        new_srdf_file_name = "generated_robot.srdf"
        self.start_new_srdf(new_srdf_file_name)
        for manipulator in self.manipulators:
            if manipulator.has_arm:
                # Read arm srdf
                arm_srdf_path = self.package_path + "/config/" + manipulator.arm.name + "/" + manipulator.arm.name + ".srdf"
                stream = open(arm_srdf_path, 'r')
                self.arm_srdf_xml = parse(stream)
                stream.close()
                xacro.process_includes(self.arm_srdf_xml, os.path.dirname(sys.argv[0]))
                xacro.eval_self_contained(self.arm_srdf_xml)
            if manipulator.has_hand:
                #Generate and read hand srdf
                hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + manipulator.hand.name
                hand_urdf_xacro_file = open(hand_urdf_path, 'r')
                hand_urdf_xml = parse(hand_urdf_xacro_file)
                hand_urdf_xacro_file.close()
                xacro.process_includes(hand_urdf_xml, os.path.dirname(sys.argv[0]))
                xacro.eval_self_contained(hand_urdf_xml)
                hand_urdf = hand_urdf_xml.toprettyxml(indent='  ')
                srdfHandGenerator = SRDFHandGenerator(hand_urdf, load = False, save = True)
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
            else:
                self.parse_arm_end_effectors(manipulator)

            # Add virtual joints
            if manipulator.has_arm:
                self.parse_arm_virtual_joint(manipulator)
            else:
                self.parse_hand_virtual_joint(manipulator)

            # Add disable collisions
            comment = ["DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links."]
            self.add_comments(comment)
            if manipulator.has_arm:
                self.parse_arm_collisions(manipulator)
            if manipulator.has_hand:
                self.parse_hand_collisions(manipulator)
            # TODO: Add collisions between manipulators: arm and hand

        #Finish and close file
        self.new_robot_srdf.write('</robot>\n')
        self.new_robot_srdf.close()
        if load:
            stream = open(self.package_path + "/config/" + new_srdf_file_name, 'r')
            srdf = parse(stream)
            stream.close()
            rospy.loginfo("Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name('robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            srdf.toprettyxml(indent='  '))

        rospy.loginfo("generated_robot.srdf has been generated and saved.")

    def set_parameters(self, description_file):
        # Read robot description yaml
        stream = open(self.package_path + description_file, 'r')
        yamldoc = yaml.load(stream)
        stream.close()
        if "robot" in yamldoc:
            robot_yaml = yamldoc["robot"]
            self.robot_name = robot_yaml["name"]
            if "manipulators" in robot_yaml:
                manipulators_yaml = robot_yaml["manipulators"]
                for manipulator_name in manipulators_yaml.keys():
                    manipulator_yaml = manipulators_yaml[manipulator_name]
                    side = manipulator_yaml["side"]
                    if side not in ["left", "right"]:
                        raise SRDFRobotGeneratorException("robot description did not specified a correct side for a manipulator")
                    has_arm = True if "arm" in manipulator_yaml else False
                    has_hand = True if "hand" in manipulator_yaml else False
                    if not has_hand and not has_arm:
                        raise SRDFRobotGeneratorException("robot description did not specified either an arm or hand for a manipulator")
                    # TODO: check that each manipulator do not have more than one arm and hand
                    manipulator = Manipulator(manipulator_name, side, has_arm, has_hand)
                    if has_arm:
                        arm_yaml = manipulator_yaml["arm"]
                        if "name" in arm_yaml:
                            manipulator.arm.name = arm_yaml["name"]
                        else:
                            raise SRDFRobotGeneratorException("arm name should be specified")
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
            raise SRDFRobotGeneratorException("robot description did not specified a robot")

    def start_new_srdf(self, file_name):
        # Generate new robot srdf with arm information
        self.new_robot_srdf = open(self.package_path + "/config/" + file_name, 'w')

        self.new_robot_srdf.write('<?xml version="1.0" ?>\n')
        banner = ["This does not replace URDF, and is not an extension of URDF.\n"+
                  "    This is a format for representing semantic information about the robot structure.\n"+
                  "    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined\n"
                 ]
        self.add_comments(banner, "")
        self.new_robot_srdf.write('<robot name="'+self.robot_name+'">\n')
        comments = ["GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc",
                    "LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included",
                    "JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included",
                    "CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group",
                    "SUBGROUPS: Groups can also be formed by referencing to already defined group names"]
        self.add_comments(comments)

    def parse_arm_groups(self, manipulator):
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'group':
                if len(elt.childNodes) > 0: # Check it is not a subgroup
                    group_name = elt.getAttribute('name')
                    if group_name == manipulator.arm.main_group or group_name in manipulator.arm.main_group:
                        if group_name == manipulator.arm.main_group:
                            elt.setAttribute('name',manipulator.arm.internal_name)
                        else:
                            elt.setAttribute('name',manipulator.arm.prefix + group_name)
                        for index, group_element in enumerate(elt.getElementsByTagName("chain")):
                            attributes = ["base_link", "tip_link"]
                            for attribute in attributes:
                                group_element.getAttributeNode(attribute).nodeValue = (manipulator.arm.prefix +
                                                                                       group_element.getAttribute(attribute))
                        for tagName in ["joint", "link", "group"]:
                            for index, group_element in enumerate(elt.getElementsByTagName(tagName)):
                                attribute_name = group_element.getAttribute("name")
                                if attribute_name in ["WRJ1", "WRJ2"]:
                                    if manipulator.has_hand:
                                        if not manipulator.hand.is_lite:
                                            group_element.getAttributeNode("name").nodeValue = (manipulator.hand.prefix + attribute_name)
                                    else:
                                        group_element.parentNode.removeChild(group_element)
                                else:
                                    group_element.getAttributeNode("name").nodeValue = (manipulator.arm.prefix + attribute_name)

                        elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
                        # TODO: ADD hand wrist joints if the srdf does not have them already
            elif elt.tagName == 'group_state':
                group_state_name = elt.getAttribute('name')
                group_name = elt.getAttribute('group')
                if group_state_name in manipulator.arm.group_states and (group_name == manipulator.arm.main_group or group_name in manipulator.arm.other_groups):
                    elt.setAttribute('name',manipulator.arm.prefix + group_state_name)
                    if group_name == manipulator.arm.main_group:
                        elt.setAttribute('group',manipulator.arm.internal_name)
                    else:
                        elt.setAttribute('group',manipulator.arm.prefix + group_name)
                    for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                        attribute_name = group_element.getAttribute("name")
                        if attribute_name in ["WRJ1", "WRJ2"]:
                            if manipulator.has_hand:
                                if not manipulator.hand.is_lite:
                                    group_element.getAttributeNode("name").nodeValue = (manipulator.hand.prefix + attribute_name)
                            else:
                                group_element.parentNode.removeChild(group_element)
                        else:
                            group_element.getAttributeNode("name").nodeValue = (manipulator.arm.prefix + attribute_name)
                    elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_groups(self, manipulator):
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'group':
                if len(elt.childNodes) > 0: # Check it is not a subgroup
                    elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            elif elt.tagName == 'group_state':
                for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                    attribute_name = group_element.getAttribute("name")
                    attribute_name = attribute_name.split("_")[1]
                    if attribute_name in ["WRJ1", "WRJ2"] and manipulator.has_arm:
                        group_element.parentNode.removeChild(group_element)
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_end_effectors(self, manipulator):
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_arm_end_effectors(self, manipulator):
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'end_effector':
                elt.getAttributeNode("name").nodeValue = manipulator.arm.prefix + elt.getAttribute("name")
                elt.getAttributeNode("parent_link").nodeValue = manipulator.arm.prefix + elt.getAttribute("parent_link")
                elt.getAttributeNode("group").nodeValue = manipulator.arm.internal_name
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_arm_virtual_joint(self, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.arm_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.getAttributeNode("name").nodeValue = "world_to_" + manipulator.arm.internal_name
                elt.getAttributeNode("child_link").nodeValue = manipulator.arm.prefix + elt.getAttribute("child_link")
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_virtual_joint(self, manipulator):
        comment = ["VIRTUAL JOINT: Purpose: this element defines a virtual joint between a robot link and an external frame of reference (considered fixed with respect to the robot)"]
        self.add_comments(comment)
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'virtual_joint':
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
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
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")

            previous = elt
            elt = xacro.next_element(previous)

    def parse_hand_collisions(self, manipulator):
        comment = [manipulator.hand.internal_name + " collisions"]
        self.add_comments(comment)
        previous = self.hand_srdf_xml.documentElement
        elt = xacro.next_element(previous)
        while elt:
            if elt.tagName == 'disable_collisions':
                elt.writexml(self.new_robot_srdf,  indent="  ", addindent="  ", newl="\n")

            previous = elt
            elt = xacro.next_element(previous)

    def add_comments(self, comments, addindent = "  "):
        xml_comments = [xml.dom.minidom.Comment(c) for c in comments]
        for comment in xml_comments:
            comment.writexml(self.new_robot_srdf,  indent=addindent, addindent=addindent, newl="\n")

if __name__ == '__main__':

    rospy.init_node('robot_srdf_generator', anonymous=True)
    robot_generator = SRDFRobotGenerator()
