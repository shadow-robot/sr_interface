#!/usr/bin/env python

#
# Convert the old style XML grasps file to new style YAML.
#
# rosrun sr_grasp convert_xml.py sr_hand/scripts/sr_hand/grasps.xml > converted.yaml
#

import sys
import genpy
from sr_hand.grasps_parser import GraspParser
import sr_grasp
from sr_robot_msgs.msg import GraspArray

parser = GraspParser()
parser.parse_tree(xml_filename=sys.argv[1])

grasps = GraspArray()
for name, g in parser.grasps.iteritems():
    ng = sr_grasp.Grasp()
    ng.id = name
    ng.set_grasp_point(positions=g.joints_and_positions)
    grasps.grasps.append(ng)

# print grasps # oddly fails, but this works.
print genpy.message.strify_message(grasps.grasps)
