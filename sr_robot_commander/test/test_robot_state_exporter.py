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
import sys
import os
import filecmp
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
        os.mkdir("/tmp/test_exporter")
    
    @classmethod
    def tearDownClass(cls):
        os.system("rm -rf /tmp/test_exporter")

    def setUp(self):
        rospy.init_node('test_hand_commander', anonymous=True)
        self.test_path = "/tmp/test_exporter"
        sys.path.append(self.test_path)
        self.expected_state = {
            'state1': {
                'joint_test1': 0.0,
                'joint_test2': 1.0}
        }

    def test_extract_all(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_all()
        state_exporter.output_module(self.test_path + "/exporter_output.py")
        from exporter_output import warehouse_states
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:" 
                         + str(warehouse_states) + " not " + str(self.expected_state))

    def test_extract_one_state(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_one_state("state1")
        state_exporter.output_module(self.test_path + "/exporter_output.py")
        from exporter_output import warehouse_states
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:"
                         + str(warehouse_states) + " not " + str(self.expected_state))

    def test_extract_list(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_list(["state1"])
        state_exporter.output_module(self.test_path + "/exporter_output.py")
        from exporter_output import warehouse_states
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:"
                         + str(warehouse_states) + " not " + str(self.expected_state))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_robot_state_exporter", TestSrRobotStateExporter)
