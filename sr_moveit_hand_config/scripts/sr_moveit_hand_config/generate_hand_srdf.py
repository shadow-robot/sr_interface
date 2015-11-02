#!/usr/bin/env python
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
from xacro import set_substitution_args_context
from rosgraph.names import load_mappings

from sr_utilities.local_urdf_parser_py import URDF


class SRDFHandGenerator(object):
    def __init__(self, urdf_str=None, load=True, save=True):
        if urdf_str is None:
            while not rospy.has_param('robot_description'):
                rospy.sleep(0.5)
                rospy.loginfo("waiting for robot_description")

            # load the urdf from the parameter server
            urdf_str = rospy.get_param('robot_description')

        robot = URDF.from_xml_string(urdf_str)

        extracted_prefix = False
        prefix = ""
        ff = mf = rf = lf = th = False
        is_lite = True
        hand_name = "right_hand"

        for key in robot.joint_map:
            # any joint is supposed to have the same prefix and a joint name with 4 chars
            if not extracted_prefix:
                prefix = key.split("_")[0] + "_"
                rospy.loginfo("Found prefix:" + prefix)
                extracted_prefix = True
                if prefix == "lh_":
                    hand_name = "left_hand"

            if not ff and key.endswith("FFJ4"):
                ff = True
            if not mf and key.endswith("MFJ4"):
                mf = True
            if not rf and key.endswith("RFJ4"):
                rf = True
            if not lf and key.endswith("LFJ4"):
                lf = True
            if not th and key.endswith("THJ4"):
                th = True
            if is_lite and key.endswith("WRJ2"):
                is_lite = False

        rospy.logdebug("Found fingers (ff mf rf lf th)" + str(ff) + str(mf) + str(rf) + str(lf) + str(th))
        rospy.logdebug("is_lite: " + str(is_lite))
        rospy.logdebug("Hand name: " + str(hand_name))

        set_substitution_args_context(
            load_mappings(['prefix:=' + str(prefix),
                           'robot_name:=' + robot.name,
                           'ff:=' + str(int(ff)),
                           'mf:=' + str(int(mf)),
                           'rf:=' + str(int(rf)),
                           'lf:=' + str(int(lf)),
                           'th:=' + str(int(th)),
                           'is_lite:=' + str(int(is_lite)),
                           'hand_name:=' + str(hand_name)
                           ]))

        # the prefix version of the srdf_xacro must be loaded
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sr_moveit_hand_config')
        srdf_xacro_filename = package_path + "/config/shadowhands.srdf.xacro"

        srdf_xacro_filename = srdf_xacro_filename.replace(".srdf.xacro", "_prefix.srdf.xacro")
        rospy.loginfo("File loaded " + srdf_xacro_filename)

        # open and parse the xacro.srdf file
        srdf_xacro_file = open(srdf_xacro_filename, 'r')
        self.srdf_xacro_xml = parse(srdf_xacro_file)

        # expand the xacro
        xacro.process_includes(self.srdf_xacro_xml, os.path.dirname(sys.argv[0]))
        xacro.eval_self_contained(self.srdf_xacro_xml)

        if len(sys.argv) > 1:
            OUTPUT_PATH = sys.argv[1]
            # reject ROS internal parameters and detect termination
            if (OUTPUT_PATH.startswith("_") or
                    OUTPUT_PATH.startswith("--")):
                OUTPUT_PATH = None
        else:
            OUTPUT_PATH = None

        if load:
            rospy.loginfo("Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name('robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            self.srdf_xacro_xml.toprettyxml(indent='  '))
        if save:
            OUTPUT_PATH = package_path + "/config/generated_shadowhand.srdf"
            FW = open(OUTPUT_PATH, "wb")
            FW.write(self.srdf_xacro_xml.toprettyxml(indent='  '))
            FW.close()

            OUTPUT_PATH = package_path + "/config/generated_shadowhand.urdf"
            FW = open(OUTPUT_PATH, "wb")
            FW.write(urdf_str)
            FW.close()

        srdf_xacro_file.close()

    def get_hand_srdf(self):
        return self.srdf_xacro_xml

if __name__ == '__main__':
    rospy.init_node('hand_srdf_generator', anonymous=True)
    srdfGenerator = SRDFHandGenerator()
