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
from xacro import set_substitution_args_context
from rosgraph.names import load_mappings
from sr_moveit_hand_config.generate_hand_srdf import SRDFGenerator

from sr_utilities.local_urdf_parser_py import URDF

if __name__ == '__main__':

    rospy.init_node('robot_srdf_generator', anonymous=True)

    robot_name = "ur10sh"
    
    # ARM
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sr_multi_moveit_config')
    arm_name = "ur10"
    arm_srdf_path = package_path + "/config/" + arm_name + "/" + arm_name + ".srdf"
    
    arm_main_group = "right_arm"
    arm_other_groups = []
    arm_group_states = ["hold_bottle", "approach_bottle", "pour_bottle", "victory", "gamma"]
    end_effectors = [] # Ignore the one in the arm
    #Generate the virtual joint and ignore the one available using info in the urdf
    
    arm_side = "right" # get if side is right or left and set prefix accordingly
    if arm_side == "right":
        arm_prefix = "ra_"
        hand_prefix = "rh_"
        arm_name = "right_arm"
        hand_name = "right_hand"
    elif arm_side == "left":
        arm_prefix = "la_"
        hand_prefix = "lh_"
        arm_name = "left_arm"
        hand_name = "left_hand"
    
    print arm_side
    print arm_prefix

    hand_lite = False

    # Parse arm srdf
    stream = open(arm_srdf_path, 'r')
    arm_srdf_xml = parse(stream)
    stream.close()
    xacro.process_includes(arm_srdf_xml, os.path.dirname(sys.argv[0]))
    xacro.eval_self_contained(arm_srdf_xml)
    
    # Generate new robot srdf with arm information
    new_robot_srdf = open(package_path + "/config/generated_robot.srdf", 'w')
    new_robot_srdf.write('<?xml version="1.0" ?>\n')
    banner = [xml.dom.minidom.Comment(c) for c in
              ["This does not replace URDF, and is not an extension of URDF.\n"+
               "    This is a format for representing semantic information about the robot structure.\n"+
               "    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined\n"
               ]]
    for comment in banner:
        new_robot_srdf.write(comment.toprettyxml(indent='  '))
    new_robot_srdf.write('<robot name="'+robot_name+'">\n')

    previous = arm_srdf_xml.documentElement
    elt = xacro.next_element(previous)
    while elt:
        if elt.tagName == 'group':
            group_name = elt.getAttribute('name')
            if group_name == arm_main_group or group_name in arm_other_groups:
                if group_name == arm_main_group:
                    elt.setAttribute('name',arm_name)
                else:
                    elt.setAttribute('name',arm_prefix + group_name)
                for index, group_element in enumerate(elt.getElementsByTagName("chain")):
                    attributes = ["base_link", "tip_link"]
                    for attribute in attributes:
                        group_element.getAttributeNode(attribute).nodeValue = (arm_prefix +
                                                                               group_element.getAttribute(attribute))
                for tagName in ["joint", "link", "group"]:
                    for index, group_element in enumerate(elt.getElementsByTagName(tagName)):
                        attribute_name = group_element.getAttribute("name")
                        if attribute_name in ["WRJ1", "WRJ2"]: 
                            if not hand_lite:
                                group_element.getAttributeNode("name").nodeValue = (hand_prefix + attribute_name)
                            else:
                                group_element.parentNode.removeChild(group_element)
                        else:
                            group_element.getAttributeNode("name").nodeValue = (arm_prefix + attribute_name)

                elt.writexml(new_robot_srdf,  indent="  ", addindent="  ", newl="\n")
                # TODO: ADD hand wrist joints!
        elif elt.tagName == 'group_state':
            print "elt: ", elt.toprettyxml(indent='  ')
            group_state_name = elt.getAttribute('name')
            group_name = elt.getAttribute('group')
            
            print "group_name",group_name
            print "group_state_name",group_state_name
            if group_state_name in arm_group_states and (group_name == arm_main_group or group_name in arm_other_groups):
                elt.setAttribute('name',arm_prefix + group_state_name)
                if group_name == arm_main_group:
                    elt.setAttribute('group',arm_name)
                else:
                    elt.setAttribute('group',arm_prefix + group_name)
                for index, group_element in enumerate(elt.getElementsByTagName("joint")):
                    attribute_name = group_element.getAttribute("name")
                    if attribute_name in ["WRJ1", "WRJ2"]: 
                        if not hand_lite:
                            group_element.getAttributeNode("name").nodeValue = (hand_prefix + attribute_name)
                        else:
                            group_element.parentNode.removeChild(group_element)
                    else:
                        group_element.getAttributeNode("name").nodeValue = (arm_prefix + attribute_name)
                elt.writexml(new_robot_srdf,  indent="  ", addindent="  ", newl="\n")

        previous = elt
        elt = xacro.next_element(previous)
    new_robot_srdf.write('</robot>\n')
    new_robot_srdf.close()

    #arm_srdf = arm_srdf_xml.toprettyxml(indent='  ')
    #FW = open(package_path + "/config/generated_arm.srdf", "wb")
    #FW.write(arm_srdf)
    #FW.close()

#     # HAND
#     robot_description_path = rospack.get_path('sr_description')+"/robots/"
#     hand_urdf_xacro_name = "shadowhand_motor.urdf.xacro"
#     
#     # open and parse the urdf.xacro file
#     hand_urdf_xacro_file = open(robot_description_path + hand_urdf_xacro_name, 'r')
#     hand_urdf_xml = parse(hand_urdf_xacro_file)
#     xacro.process_includes(hand_urdf_xml, os.path.dirname(sys.argv[0]))
#     xacro.eval_self_contained(hand_urdf_xml)
#     hand_urdf = hand_urdf_xml.toprettyxml(indent='  ')
#     srdfGenerator = SRDFGenerator(hand_urdf)
