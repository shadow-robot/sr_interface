#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from unittest import TestCase

PKG = "sr_robot_commander"


class TestSrHandCommander(TestCase):
    """
    Tests the Hand Commander
    """

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)

    def test_strip_prefix(self):
        hand_commander = SrHandCommander()

        self.assertEqual(hand_commander._strip_prefix("rh_ffj3"), "ffj3", msg="Strip failed")
        self.assertEqual(hand_commander._strip_prefix("ffj3"), "ffj3", msg="Strip failed")

    def test_hand_finder_init(self):
        hand_finder = HandFinder()
        hand_parameters = hand_finder.get_hand_parameters()
        hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                         hand_serial=hand_parameters.mapping.keys()[0])
        self.assertGreater(len(hand_commander.get_joints_position()), 0, "No joints found, init must have failed.")

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_hand_commander", TestSrHandCommander)
