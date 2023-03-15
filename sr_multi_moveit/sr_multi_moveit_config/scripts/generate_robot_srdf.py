#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2022-2023 belongs to Shadow Robot Company Ltd.
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

import sys
import re
from xml.dom.minidom import parse
import xml
from copy import deepcopy
from shutil import copy2
import yaml
import xacro
import rospy
import rospkg
from urdf_parser_py.urdf import URDF
from sr_moveit_hand_config.generate_hand_srdf import SRDFHandGenerator


class SRDFRobotGeneratorException(Exception):
    def __init__(self, msg):
        super().__init__()
        self.msg = msg

    def __str__(self):
        return self.msg


class Subrobot:
    def __init__(self, type_var):
        self.type = type_var
        self.name = ""
        self.internal_name = ""
        self.main_group = ""
        self.other_groups = []
        self.group_states = []
        self.is_lite = False
        self.prefix = ""
        self.moveit_path = ""


class Manipulator:
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
                self.arm.extra_groups_config_path = ""
            else:
                self.arm.prefix = "la_"
                self.arm.internal_name = "left_arm"
                self.arm.extra_groups_config_path = ""
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
        next_xacro = xacro.next_sibling_element(elt)
        if next_xacro:
            return next_xacro
        elt = elt.parentNode
    return None


class Robot:
    def __init__(self):
        self.name = ""
        self.manipulators = []

    def set_parameters(self, yamldoc):
        # Read robot description yaml
        if "robot" in yamldoc:  # pylint: disable=R1702
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
                    has_arm = "arm" in manipulator_yaml
                    has_hand = bool("hand" in manipulator_yaml and manipulator_yaml["hand"])
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

                    self.manipulators.append(manipulator)
        else:
            raise SRDFRobotGeneratorException("robot description did not specify a robot")


class SRDFRobotGenerator:
    def __init__(self, description_file=None, load=True):  # pylint: disable=R0915
        if description_file is None and len(sys.argv) > 1:
            description_file = sys.argv[1]

        self._save_files = rospy.get_param('~save_files', False)
        self._path_to_save_files = rospy.get_param('~path_to_save_files', "/tmp/")
        self._file_name = rospy.get_param('~file_name', "generated_robot")

        # ARM
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('sr_multi_moveit_config')

        self.robot = Robot()

        with open(description_file, "r", encoding="utf-8") as stream:
            yamldoc = yaml.safe_load(stream)

        self.robot.set_parameters(yamldoc)
        self.arm_srdf_xmls = []
        self.hand_srdf_xmls = []

        new_srdf_file_name = "generated_robot.srdf"
        self.start_new_srdf(new_srdf_file_name)
        for manipulator_id, manipulator in enumerate(self.robot.manipulators):
            if manipulator.has_arm:
                # Read arm srdf
                arm_srdf_path = f"{manipulator.arm.moveit_path}/{manipulator.arm.name}.srdf"
                with open(arm_srdf_path, 'r', encoding="utf-8") as stream:
                    self.arm_srdf_xmls.append(parse(stream))
                xacro.process_doc(self.arm_srdf_xmls[manipulator_id])

            if manipulator.has_hand:
                hand_urdf = rospy.get_param(f'{manipulator.side}_hand_description')
                srdf_hand_generator = SRDFHandGenerator(hand_urdf, load=False, save=False)
                self.hand_srdf_xmls.append(srdf_hand_generator.get_hand_srdf())

            comment = ["Manipulator:" + manipulator.name]
            self.add_comments(comment)

            # Add groups and group states
            if manipulator.has_arm:
                self.parse_arm_groups(manipulator_id, manipulator)
            if manipulator.has_hand:
                self.parse_hand_groups(manipulator_id)

        # Add groups for bimanual arm and hand systems
        if len(self.robot.manipulators) == 2:
            if (self.robot.manipulators[0].has_arm and self.robot.manipulators[1].has_arm):
                self.add_bimanual_arm_groups(self.robot.manipulators)

            if self.robot.manipulators[0].has_hand and self.robot.manipulators[1].has_hand:
                comment = ["Bimanual hand groups"]
                self.add_bimanual_hand_groups(self.robot.manipulators[0].hand.internal_name,
                                              self.robot.manipulators[1].hand.internal_name)
                self.add_comments(comment)

        for manipulator_id, manipulator in enumerate(self.robot.manipulators):

            # Add end effectors
            comment = ["END EFFECTOR: Purpose: Represent information about an end effector."]
            self.add_comments(comment)
            if manipulator.has_hand:
                self.parse_hand_end_effectors(manipulator_id)
            if manipulator.has_arm:
                self.parse_arm_end_effectors(manipulator_id, manipulator)

            # Add virtual joints
            if manipulator.has_arm:
                pass
                # self.parse_arm_virtual_joint(manipulator)
            else:
                self.parse_hand_virtual_joint(manipulator_id)

            # Add disable collisions
            comment = ["DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially " +
                       "come into collision with any other link in the robot. This tag disables collision checking " +
                       "between a specified pair of links."]
            self.add_comments(comment)
            if manipulator.has_arm:
                self.parse_arm_collisions(manipulator_id, manipulator)
            if manipulator.has_hand:
                self.parse_hand_collisions(manipulator_id, manipulator)

        # Generate and add the multi-robot move group states
        self.add_multi_robot_move_group_states()

        # Finish and close file
        self.new_robot_srdf.write('</robot>\n')
        self.new_robot_srdf.close()
        if load:
            with open(f"{self.package_path}/config/{new_srdf_file_name}", 'r', encoding="utf-8") as stream:
                srdf = parse(stream)

            rospy.loginfo("Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name('robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            srdf.toprettyxml(indent='  '))

        if self._save_files:
            rospy.loginfo(f"Robot urdf and srdf have been saved to {self._path_to_save_files}")

            # srdf: File is already generated so just need to be copied to specified location
            copy2(self.package_path + "/config/" + new_srdf_file_name, self._path_to_save_files +
                  "/" + self._file_name + ".srdf")

            # urdf: File can be copied from rosparam
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                with open(f"{self._path_to_save_files}/{self._file_name}.urdf", "wb", encoding="utf-8") as urdf_file:
                    urdf_file.write(urdf_str)

        rospy.loginfo("generated_robot.srdf has been generated and saved.")

    def start_new_srdf(self, file_name):
        # Generate new robot srdf with arm information
        self.new_robot_srdf = open(f"{self.package_path}/config/{file_name}", 'w+',  # pylint: disable=R1732
                                   encoding="utf-8")

        self.new_robot_srdf.write('<?xml version="1.0" ?>\n')
        banner = ["This does not replace URDF, and is not an extension of URDF.\n" +
                  "    This is a format for representing semantic information about the robot structure.\n" +
                  "    A URDF file must exist for this robot as well, where the joints and the links that are" +
                  "referenced are defined\n"]
        self.add_comments(banner, "")
        self.new_robot_srdf.write(f'<robot name="{self.robot.name}">\n')
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
        while elt:   # pylint: disable=R1702
            if elt.tagName == 'group':
                # Check it is not a subgroup
                if len(elt.childNodes) > 0:
                    group_name = elt.getAttribute('name')
                    if group_name == manipulator.arm.main_group or group_name in manipulator.arm.other_groups:
                        if group_name == manipulator.arm.main_group:
                            elt.setAttribute('name', manipulator.arm.internal_name)
                        else:
                            elt.setAttribute('name', manipulator.arm.prefix + group_name)
                        for _, group_element in enumerate(elt.getElementsByTagName("chain")):
                            attributes = ["base_link", "tip_link"]
                            for attribute in attributes:
                                node_attribute = group_element.getAttributeNode(attribute)
                                node_attribute.nodeValue = (manipulator.arm.prefix +
                                                            group_element.getAttribute(attribute))
                        for tag_name in ["joint", "link", "group"]:
                            for _, group_element in enumerate(elt.getElementsByTagName(tag_name)):
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
                            elt.appendChild(node)
                            node = deepcopy(elt.getElementsByTagName("joint")[0])
                            node_attribute = node.getAttributeNode("name")
                            node_attribute.nodeValue = (manipulator.hand.prefix + "WRJ1")
                            elt.appendChild(node)
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
                    for _, group_element in enumerate(elt.getElementsByTagName("joint")):
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

    # Get move group states from an XML DOM, returning a dictionary of {move_group: {state: values}
    @staticmethod
    def parse_move_group_states(srdf_xml_dom, group_states):
        for group_state_xml in srdf_xml_dom.getElementsByTagName("group_state"):
            group = group_state_xml.getAttribute("group")
            state_name = group_state_xml.getAttribute("name")
            if group not in group_states:
                group_states[group] = {}
            group_states[group][state_name] = {}
            for group_state_child in group_state_xml.childNodes:
                if group_state_child.localName == "joint":
                    group_states[group][state_name][group_state_child.getAttribute("name")] = \
                        group_state_child.getAttribute("value")
        return group_states

    # Generates states for move groups that span multiple robots, and therefore can't be defined in individual robot
    # SRDFs (xacros). These states can inherit from single-robot move group states, or each other.
    def add_multi_robot_move_group_states(self):
        # The YAML file containing the multi-robot move group state definitions is an optional argument to this script;
        # if it's not provided, no multi-robot move group states will be generated.
        self._multi_robot_move_group_states = {}
        if len(sys.argv) > 2:
            try:
                with open(sys.argv[2], "r", encoding="utf-8") as stream:
                    self._multi_robot_move_group_states = yaml.safe_load(stream)
            except FileNotFoundError:
                rospy.logwarn(f'Could not open the specified multi-robot move group states definition file: '
                              f'"{sys.argv[2]}". No multi-robot group states loaded.')
        # Check which move groups exist by parsing the previously-generated SRDF
        self.new_robot_srdf.seek(0)
        move_group_names = list(set(re.findall(r'<group\s+.*name="([^"]*)"', self.new_robot_srdf.read())))
        # Collect the single-robot move group states defined in the separate robot SRDFs
        self._single_robot_move_group_states = {}
        for hand_xml in self.hand_srdf_xmls:
            SRDFRobotGenerator.parse_move_group_states(hand_xml, self._single_robot_move_group_states)
        for arm_xml in self.arm_srdf_xmls:
            SRDFRobotGenerator.parse_move_group_states(arm_xml, self._single_robot_move_group_states)
        # Put the original move group states in the list of all move group states
        self._all_move_group_states = deepcopy(self._single_robot_move_group_states)
        # Keep track of the combined move group states we are newly generating
        self._new_multi_robot_move_group_states = {}
        # For any existing move groups
        for move_group_name in move_group_names:
            # If there are combined move group states defined
            if move_group_name in self._multi_robot_move_group_states:
                # For each combined move group state
                for move_group_state_name in self._multi_robot_move_group_states[move_group_name]:
                    # If it has not already been defined
                    if (move_group_name not in self._all_move_group_states or
                            move_group_state_name not in self._all_move_group_states[move_group_name]):
                        # Create the move group state, resolving any inherited values
                        self.create_move_group_state(move_group_name, move_group_state_name)
        # Write all of the newly generated move group states to the generated SRDF
        for move_group_name, move_group_states in self._new_multi_robot_move_group_states.items():
            for move_group_state_name, move_group_state in move_group_states.items():
                new_group_state = xml.dom.minidom.Document().createElement('group_state')
                new_group_state.setAttribute("group", move_group_name)
                new_group_state.setAttribute("name", move_group_state_name)
                for joint_name, joint_angle in move_group_state.items():
                    new_group_state.appendChild(xml.dom.minidom.Document().createElement(
                        f'joint name="{joint_name}" value="{joint_angle}"'))
                new_group_state.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    # Creates a new move group state, resolving any values inherited from other move groups
    def create_move_group_state(self, group_name, group_state_name):
        combined_move_group_state = self._multi_robot_move_group_states[group_name][group_state_name]
        if "joint_angles" not in combined_move_group_state:
            combined_move_group_state["joint_angles"] = {}
        # If the new move group state inherits from any others
        if "inherit_from" in combined_move_group_state:
            for ancestor in combined_move_group_state["inherit_from"]:
                ancestor_group = ancestor["move_group"]
                ancestor_group_state = ancestor["move_group_state"]
                # If the ancestor group state is not defined yet
                if (ancestor_group not in self._all_move_group_states or
                        ancestor_group_state not in self._all_move_group_states[ancestor_group]):
                    # If the ancestor group state is not even defined in the combined move group states
                    if (ancestor_group not in self._multi_robot_move_group_states or
                            ancestor_group_state not in self._multi_robot_move_group_states[ancestor_group]):
                        rospy.logwarn(f'Unable to create move group "{group_name}" state "{group_state_name}", as it '
                                      f'inherits from move group "{ancestor_group}" state "{ancestor_group_state}", '
                                      f'which is not defined.')
                        return False
                    # Try to create the ancestor group state, return if failed
                    if not self.create_move_group_state(ancestor_group, ancestor_group_state):
                        return False
                # The ancestor group state either already existed or has been created
                ancestor_move_group_state = self._all_move_group_states[ancestor_group][ancestor_group_state]
                for joint, value in ancestor_move_group_state.items():
                    if joint not in combined_move_group_state["joint_angles"]:
                        combined_move_group_state["joint_angles"][joint] = value
        if group_name not in self._all_move_group_states:
            self._all_move_group_states[group_name] = {}
        self._all_move_group_states[group_name][group_state_name] = combined_move_group_state["joint_angles"]
        if group_name not in self._new_multi_robot_move_group_states:
            self._new_multi_robot_move_group_states[group_name] = {}
        self._new_multi_robot_move_group_states[group_name][group_state_name] = \
            combined_move_group_state["joint_angles"]
        return True

    def add_bimanual_arm_groups(self, manipulators):
        self.add_comments(comments=["Bimanual arm groups without hands"])
        # Add two arms (no hands) group
        self.add_move_group_combining_others(
            'two_arms', [manipulators[0].arm.internal_name, manipulators[1].arm.internal_name])
        if manipulators[0].has_hand or manipulators[1].has_hand:
            self.add_comments(comments=["Bimanual arm groups with hand(s)"])
        if manipulators[0].has_hand:
            # Add two arms and first hand group
            self.add_move_group_combining_others(
                f'two_arms_and_{manipulators[0].hand.internal_name}',
                [f'{manipulators[0].arm.internal_name}_and_hand', f'{manipulators[1].arm.internal_name}'])
            if not manipulators[0].hand.is_lite:
                # Add two arms and first hand wrist group
                self.add_move_group_combining_others(
                    f'two_arms_and_{manipulators[0].side}_wrist',
                    [f'{manipulators[0].arm.internal_name}_and_wrist', f'{manipulators[1].arm.internal_name}'])
        if manipulators[1].has_hand:
            # Add two arms and second hand group
            self.add_move_group_combining_others(
                f'two_arms_and_{manipulators[1].hand.internal_name}',
                [f'{manipulators[1].arm.internal_name}_and_hand', f'{manipulators[0].arm.internal_name}'])
            if not manipulators[1].hand.is_lite:
                # Add two arms and second hand wrist group
                self.add_move_group_combining_others(
                    f'two_arms_and_{manipulators[1].side}_wrist',
                    [f'{manipulators[1].arm.internal_name}_and_wrist', f'{manipulators[0].arm.internal_name}'])
        if manipulators[0].has_hand and manipulators[1].has_hand:
            # Add two arms and two hands group
            self.add_move_group_combining_others(
                'two_arms_and_hands',
                [f'{manipulators[0].arm.internal_name}_and_hand', f'{manipulators[1].arm.internal_name}_and_hand'])
            if (not manipulators[0].hand.is_lite) and (not manipulators[1].hand.is_lite):
                # Add two arms and two wrists group
                self.add_move_group_combining_others(
                    'two_arms_and_wrists',
                    [f'{manipulators[1].arm.internal_name}_and_wrist',
                        f'{manipulators[0].arm.internal_name}_and_wrist'])

    def add_move_group_combining_others(self, new_group_name, existing_group_names=None):
        """ Adds a new move group to the SRDF that includes other, existing move groups. """
        new_group = xml.dom.minidom.Document().createElement('group')
        new_group.setAttribute("name", new_group_name)
        for existing_group_name in existing_group_names:
            new_group.appendChild(xml.dom.minidom.Document().createElement(f'group name="{existing_group_name}"'))
        new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def add_bimanual_hand_groups(self, group_1, group_2):
        new_group = xml.dom.minidom.Document().createElement('group')
        new_group.setAttribute("name", "two_hands")
        hand_group_1 = xml.dom.minidom.Document().createElement('group name="' + group_1 + '"')
        new_group.appendChild(hand_group_1)
        hand_group_2 = xml.dom.minidom.Document().createElement('group name="' + group_2 + '"')
        new_group.appendChild(hand_group_2)
        new_group.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

    def parse_hand_groups(self, manipulator_id):
        previous = self.hand_srdf_xmls[manipulator_id].documentElement
        elt = next_element(previous)
        while elt:
            if elt.tagName == 'group':
                # Check it is not a subgroup
                if len(elt.childNodes) > 0:
                    elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            elif elt.tagName == 'group_state':
                for _, group_element in enumerate(elt.getElementsByTagName("joint")):
                    attribute_name = group_element.getAttribute("name")
                    attribute_name = attribute_name.split("_")[1]
                elt.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
            previous = elt
            elt = next_element(previous)

    def parse_hand_end_effectors(self, manipulator_id):
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
                    new_element = deepcopy(elt)
                    new_element.getAttributeNode("name").nodeValue = manipulator.arm.prefix + "and_manipulator_ee"
                    new_element.getAttributeNode("parent_link").nodeValue = manipulator.hand.prefix + "manipulator"
                    new_element.getAttributeNode("group").nodeValue = manipulator.hand.prefix + "fingers"
                    new_element.setAttribute('parent_group', manipulator.arm.internal_name + "_and_manipulator")
                    new_element.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
                    if not manipulator.hand.is_lite:
                        new_element = deepcopy(elt)
                        new_element.getAttributeNode("name").nodeValue = manipulator.arm.prefix + "and_wrist_ee"
                        new_element.getAttributeNode("parent_link").nodeValue = manipulator.hand.prefix + "palm"
                        new_element.getAttributeNode("group").nodeValue = manipulator.hand.prefix + "fingers"
                        new_element.setAttribute('parent_group', manipulator.arm.internal_name + "_and_wrist")
                        new_element.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")
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

    def parse_hand_virtual_joint(self, manipulator_id):
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
                new_element = deepcopy(elt)

            previous = elt
            elt = next_element(previous)
        # Add collisions between arm and hand
        if manipulator.has_hand:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                robot = URDF.from_xml_string(urdf_str)
                arm_chain = robot.get_chain("world", manipulator.hand.prefix + "forearm", joints=False, fixed=False)

                new_element.getAttributeNode("link1").nodeValue = arm_chain[-2]
                new_element.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                new_element.getAttributeNode("reason").nodeValue = "Adjacent"
                new_element.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

                new_element.getAttributeNode("link1").nodeValue = arm_chain[-3]
                new_element.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                new_element.getAttributeNode("reason").nodeValue = "Adjacent"
                new_element.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

                new_element.getAttributeNode("link1").nodeValue = arm_chain[-4]
                new_element.getAttributeNode("link2").nodeValue = manipulator.hand.prefix + "forearm"
                new_element.getAttributeNode("reason").nodeValue = "Adjacent"
                new_element.writexml(self.new_robot_srdf, indent="  ", addindent="  ", newl="\n")

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
