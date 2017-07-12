#!/usr/bin/env python

import sys
import os
from xml.dom.minidom import parse
from urdf_parser_py.urdf import URDF
import xacro
import rospkg

from unittest import TestCase
#
PKG = "sr_multi_moveit_config"


class TestBiotacColladaName(TestCase):

    """
    Test class for checking that the collada name "biotac_decimated.dae" of the urdf of the
    biotac hands has not been changed so they can be identified
    """

    def setUp(self):
        self.rospack = rospkg.RosPack()

    def check_name(self, hand_urdf_path):
        with open(hand_urdf_path, 'r') as hand_urdf_xacro_file:
            hand_urdf_xml = parse(hand_urdf_xacro_file)
        xacro.process_includes(hand_urdf_xml, os.path.dirname(sys.argv[0]))
        macros = {}
        xacro.grab_macros(hand_urdf_xml, macros)
        symbols = xacro.Table()
        xacro.grab_properties(hand_urdf_xml, symbols)
        xacro.eval_all(hand_urdf_xml.documentElement, macros, symbols)

        hand_urdf = hand_urdf_xml.toprettyxml(indent='  ')
        robot = URDF.from_xml_string(hand_urdf)

        is_biotac = False
        # Check if hand has the old biotac sensors
        for key in robot.link_map:
            link = robot.link_map[key]
            if link.visual:
                if hasattr(link.visual.geometry, 'filename'):
                    filename = os.path.basename(link.visual.geometry.filename)
                    if filename == "biotac_decimated.dae":
                        is_biotac = True
                        break
        return is_biotac

    def test_motor_hand(self):
        hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + "shadowhand_motor.urdf.xacro"
        is_biotac = self.check_name(hand_urdf_path)
        self.assertFalse(is_biotac, msg="Wrong biotac hand")

    def test_motor_biotac_hand(self):
        hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + "shadowhand_motor_biotac.urdf.xacro"
        is_biotac = self.check_name(hand_urdf_path)
        self.assertTrue(is_biotac, msg="No file name is called biotac_decimated.dae")

    def test_motor_ff_biotac_hand(self):
        hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + "shadowhand_motor_ff_biotac.urdf.xacro"
        is_biotac = self.check_name(hand_urdf_path)
        self.assertTrue(is_biotac, msg="No file name is called biotac_decimated.dae")

    def test_muscle_biotac_hand(self):
        hand_urdf_path = self.rospack.get_path('sr_description')+"/robots/" + "shadowhand_muscle_biotac.urdf.xacro"
        is_biotac = self.check_name(hand_urdf_path)
        self.assertTrue(is_biotac, msg="No file name is called biotac_decimated.dae")

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_biotac_collada_name", TestBiotacColladaName)
