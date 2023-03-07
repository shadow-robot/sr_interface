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

"""
    generate the srdf according to the urdf
    syntax  generate_hand_srdf [output filename]
"""

import sys
import os
from xml.dom.minidom import parse
import xacro
import rospy
import rospkg
from rosgraph.names import load_mappings

from urdf_parser_py.urdf import URDF


class SRDFHandGenerator:
    def __init__(self, urdf_str=None, load=True, save=True):
        if urdf_str is None:
            while not rospy.has_param('robot_description'):
                rospy.sleep(0.5)
                rospy.loginfo("waiting for robot_description")

            # load the urdf from the parameter server
            urdf_str = rospy.get_param('robot_description')
        robot = URDF.from_xml_string(urdf_str)

        while not rospy.has_param('tip_sensors'):
            rospy.sleep(0.5)
            rospy.loginfo("waiting for tip_sensors")
        # load the tip_sensors from the parameter server after the hand has been auto-detected
        tip_sensors_param = rospy.get_param('tip_sensors')

        extracted_prefix = False
        prefix = ""
        first_finger = middle_finger = ring_finger = little_finger = thumb = False
        is_lite = True
        tip_sensors = "pst"
        hand_name = "right_hand"

        # Check if hand has biotac 2p sensors
        if tip_sensors_param.find('bt_2p') > -1:
            tip_sensors = "bt_2p"
        elif tip_sensors_param.find('bt_sp') > -1:
            tip_sensors = "bt_sp"

        for key in robot.joint_map:
            # any joint is supposed to have the same prefix and a joint name with 4 chars
            if not extracted_prefix:
                prefix = key.split("_")[0] + "_"
                rospy.loginfo("Found prefix:" + prefix)
                extracted_prefix = True
                if prefix == "lh_":
                    hand_name = "left_hand"

            if not first_finger and key.endswith("FFJ4"):
                first_finger = True
            if not middle_finger and key.endswith("MFJ4"):
                middle_finger = True
            if not ring_finger and key.endswith("RFJ4"):
                ring_finger = True
            if not little_finger and key.endswith("LFJ4"):
                little_finger = True
            if not thumb and key.endswith("THJ4"):
                thumb = True
            if is_lite and key.endswith("WRJ2"):
                is_lite = False

        rospy.logdebug(f"Found fingers (ff mf rf lf th) {str(first_finger)} {str(middle_finger)} " +
                       f"{str(ring_finger)} {str(little_finger)} {str(thumb)}")
        rospy.logdebug(f"is_lite: {str(is_lite)}")
        rospy.loginfo(f"tip_sensors: {str(tip_sensors)}")

        mappings = load_mappings([f'prefix:={str(prefix)}',
                                  f'robot_name:={robot.name}',
                                  f'ff:={str(int(first_finger))}',
                                  f'mf:={str(int(middle_finger))}',
                                  f'rf:={str(int(ring_finger))}',
                                  f'lf:={str(int(little_finger))}',
                                  f'th:={str(int(thumb))}',
                                  f'is_lite:={str(int(is_lite))}',
                                  f'tip_sensors:={str(tip_sensors)}',
                                  f'hand_name:={str(hand_name)}'
                                  ])

        # the prefix version of the srdf_xacro must be loaded
        package_path = rospkg.RosPack().get_path('sr_moveit_hand_config')
        srdf_xacro_filename = f"{package_path}/config/shadowhands_prefix.srdf.xacro"
        rospy.loginfo(f"File loaded {srdf_xacro_filename}")

        # open and parse the xacro.srdf file
        with open(srdf_xacro_filename, 'r', encoding="utf-8") as srdf_xacro_file:
            self.srdf_xacro_xml = parse(srdf_xacro_file)

        # expand the xacro
        xacro.process_doc(self.srdf_xacro_xml, mappings=mappings)

        if len(sys.argv) > 1:
            output_path = sys.argv[1]
            # reject ROS internal parameters and detect termination
            if (output_path.startswith("_") or
                    output_path.startswith("--")):
                output_path = None
        else:
            output_path = None

        if load:
            rospy.loginfo("Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name('robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            self.srdf_xacro_xml.toprettyxml(indent='  '))
        if save:
            output_path = f"{package_path}/config/generated_shadowhand.srdf"
            with open(output_path, "w", encoding="utf-8") as file_to_save:
                file_to_save.write(self.srdf_xacro_xml.toprettyxml(indent='  '))

            output_path = f"{package_path}/config/generated_shadowhand.urdf"
            with open(output_path, "w", encoding="utf-8") as file_to_save:
                file_to_save.write(urdf_str)

        srdf_xacro_file.close()

    def get_hand_srdf(self):
        return self.srdf_xacro_xml


if __name__ == '__main__':
    rospy.init_node('hand_srdf_generator', anonymous=True)
    srdf_generator = SRDFHandGenerator()
