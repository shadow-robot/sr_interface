#!/usr/bin/env python3

# Copyright 2019, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# Convert the old style XML grasps file to new style YAML.
#
# rosrun sr_grasp convert_xml.py sr_hand/scripts/sr_hand/grasps.xml > converted.yaml
#

import sys
import genpy
from sr_hand.grasps_parser import GraspParser
from sr_robot_msgs.msg import GraspArray
import sr_grasp


parser = GraspParser()
parser.parse_tree(xml_filename=sys.argv[1])

grasps = GraspArray()
for name, g in parser.grasps.items():
    ng = sr_grasp.Grasp()
    ng.id = name
    ng.set_grasp_point(positions=g.joints_and_positions)
    grasps.grasps.append(ng)

# print grasps # oddly fails, but this works.
print(genpy.message.strify_message(grasps.grasps))
