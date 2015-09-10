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
from unittest import TestCase

PKG = "sr_robot_commander"


class TestSrHandCommander(TestCase):
    """
    Tests the Hand Commander
    """

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)

        self._hand_commander = SrHandCommander()

    def test_strip_prefix(self):
        self.assertEqual(self._hand_commander._strip_prefix("rh_ffj3"), "ffj3", msg="Strip failed")
        self.assertEqual(self._hand_commander._strip_prefix("ffj3"), "ffj3", msg="Strip failed")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_hand_commander", TestSrHandCommander)
