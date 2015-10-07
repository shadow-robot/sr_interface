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

    while not rospy.has_param('robot_description'):
        rospy.sleep(0.5)
        rospy.loginfo("waiting for robot_description")

    # load the urdf from the parameter server
    urdf_str = rospy.get_param('robot_description')
    robot = URDF.from_xml_string(urdf_str)
    
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    package_path = rospack.get_path('sr_multi_moveit_config')

    print "\n package_path: ",package_path
    OUTPUT_PATH = package_path + "/config/generated_shadowhand.urdf"
    FW = open(OUTPUT_PATH, "wb")
    FW.write(urdf_str)
    FW.close()
