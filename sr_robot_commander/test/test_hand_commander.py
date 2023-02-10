#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2015, 2022-2023 belongs to Shadow Robot Company Ltd.
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

from unittest import TestCase
import rospy
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander

PKG = "sr_robot_commander"


class TestSrHandCommander(TestCase):
    """
    Tests the Hand Commander
    """

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)

    def test_strip_prefix(self):
        hand_commander = SrHandCommander()

        self.assertEqual(hand_commander.strip_prefix("rh_ffj3"), "ffj3", msg="Strip failed")
        self.assertEqual(hand_commander.strip_prefix("ffj3"), "ffj3", msg="Strip failed")

    def test_hand_finder_init(self):
        hand_finder = HandFinder()
        hand_parameters = hand_finder.get_hand_parameters()
        hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                         hand_serial=hand_parameters.mapping.keys()[0])
        self.assertGreater(len(hand_commander.get_joints_position()), 0, "No joints found, init must have failed.")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_hand_commander", TestSrHandCommander)
