#!/usr/bin/env python

# Copyright 2018 Shadow Robot Company Ltd.
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
import filecmp
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter
from unittest import TestCase
from std_srvs.srv import SetBool

PKG = "sr_robot_commander"


class TestSrRobotStateExporter(TestCase):
    """
    Tests the Robot State Exporter
    """

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)
        self.expected_state = "{'joint_test2': 1.0, 'joint_test1': 0.0}"

    def test_extract_all(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_all()
        self.assertEqual(str(state_exporter._dictionary.get('state1')),
                         self.expected_state, msg="Export all states failed")

    def test_extract_one_state(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_one_state("state1")
        self.assertEqual(str(state_exporter._dictionary.get('state1')),
                         self.expected_state, msg="Export one state failed")

    def test_extract_list(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_list(["state1"])
        self.assertEqual(str(state_exporter._dictionary.get('state1')),
                         self.expected_state, msg="Export list failed")


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_robot_state_exporter", TestSrRobotStateExporter)
