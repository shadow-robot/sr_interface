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
from xacro import set_substitution_args_context
from rosgraph.names import load_mappings

from sr_utilities.local_urdf_parser_py import URDF

if __name__ == '__main__':
    
    rospy.init_node('srdf_generator', anonymous=True)
    
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('sr_multi_moveit_config')

    arm_name = "ur10"
    arm_srdf_path = package_path + "/config/" + arm_name + "/" + arm_name + ".srdf"
    
    stream = open(arm_srdf_path, 'r')
    srdf_xml = parse(stream)
    stream.close()
    
    print srdf_xml.toprettyxml(indent='  ')

    while not rospy.has_param('robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")

    # load the urdf from the parameter server
    urdf_str = rospy.get_param('robot_description')
    robot = URDF.from_xml_string(urdf_str)
    
    
    OUTPUT_PATH = package_path + "/config/generated_shadowhand.urdf"
    FW = open(OUTPUT_PATH, "wb")
    FW.write(urdf_str)
    FW.close()
