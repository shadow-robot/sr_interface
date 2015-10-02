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
    syntax  generate_srdf <srdf.xacro filename> [output filename]
    Note : The srdf.xacro filename should be without _prefix,
    as the name with prefix is generated from the one without if needed
"""

import sys
import os
from xml.dom.minidom import parse

import xacro
import rospy
from xacro import set_substitution_args_context
from rosgraph.names import load_mappings

from sr_utilities.local_urdf_parser_py import URDF

if __name__ == '__main__':

    if len(sys.argv) > 1:
        srdf_xacro_filename = sys.argv[1]

        rospy.init_node('srdf_generator', anonymous=True)

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
        srdf_xacro_filename = srdf_xacro_filename.replace(".srdf.xacro", "_prefix.srdf.xacro")
        rospy.loginfo("File loaded " + srdf_xacro_filename)

        # open and parse the xacro.srdf file
        srdf_xacro_file = open(srdf_xacro_filename, 'r')
        srdf_xacro_xml = parse(srdf_xacro_file)

        # expand the xacro
        xacro.process_includes(srdf_xacro_xml,
                               os.path.dirname(sys.argv[0]))
        xacro.eval_self_contained(srdf_xacro_xml)

        if len(sys.argv) > 2:
            OUTPUT_PATH = sys.argv[2]
            # reject ROS internal parameters and detect termination
            if (OUTPUT_PATH.startswith("_") or
                    OUTPUT_PATH.startswith("--")):
                OUTPUT_PATH = None
        else:
            OUTPUT_PATH = None

        # Upload or output the input string on the correct param namespace or file
        if OUTPUT_PATH is None:
            rospy.loginfo(" Loading SRDF on parameter server")
            robot_description_param = rospy.resolve_name(
                'robot_description') + "_semantic"
            rospy.set_param(robot_description_param,
                            srdf_xacro_xml.toprettyxml(indent='  '))

        else:
            rospy.loginfo(" Writing SRDF to file ", OUTPUT_PATH)
            FW = open(OUTPUT_PATH, "wb")
            FW.write(srdf_xacro_xml.toprettyxml(indent=' '))
            FW.close()

        srdf_xacro_file.close()
    else:
        rospy.logerr("No srdf.xacro file provided")
