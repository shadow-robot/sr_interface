#!/usr/bin/env python3

# Copyright 2018 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import sys
import os
import shutil
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter
from unittest import TestCase
from std_srvs.srv import SetBool

PKG = "sr_robot_commander"


class TestSrRobotStateExporter(TestCase):
    """
    Tests the Robot State Exporter
    """

    @classmethod
    def setUpClass(cls):
        if not os.path.exists("/tmp/test_exporter"):
            os.mkdir("/tmp/test_exporter")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree("/tmp/test_exporter", ignore_errors=True)

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)
        self.test_path = "/tmp/test_exporter"
        sys.path.append(self.test_path)
        self.expected_state = {
            'state1': {
                'joint_test1': 1.0,
                'joint_test2': 2.0}
        }
        self.expected_states = {
            'state1': {
                'joint_test1': 1.0,
                'joint_test2': 2.0
            },
            'state2': {
                'joint_test3': 3.0,
                'joint_test4': 4.0
            }
        }

    def test_extract_all(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_all()
        state_exporter.output_module(self.test_path + "/exporter_output_all.py")
        from self.test_path.exporter_output_all import warehouse_states
        self.assertEqual(warehouse_states, self.expected_states, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_states))

    def test_extract_one_state(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_one_state("state1")
        state_exporter.output_module(self.test_path + "/exporter_output_state.py")
        from self.test_path.exporter_output_state import warehouse_states
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_state))

    def test_extract_list(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_list(["state1"])
        state_exporter.output_module(self.test_path + "/exporter_output_list.py")
        from self.test_path.exporter_output_list import warehouse_states
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_state))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_robot_state_exporter", TestSrRobotStateExporter)
