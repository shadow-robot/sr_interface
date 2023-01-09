#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2018-2023 belongs to Shadow Robot Company Ltd.
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

import sys
import os
import shutil
from unittest import TestCase
import rospy
from sr_robot_commander.sr_robot_state_exporter import SrRobotStateExporter

PKG = "sr_robot_commander"


class TestSrRobotStateExporter(TestCase):
    """
    Tests the Robot State Exporter
    """

    @classmethod
    def setUpClass(cls):
        if not os.path.exists("/tmp/test_exporter"):
            os.mkdir("/tmp/test_exporter")

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

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree("/tmp/test_exporter", ignore_errors=True)

    def test_extract_all(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_all()
        state_exporter.output_module(self.test_path + "/exporter_output_all.py")
        from exporter_output_all import warehouse_states
        rospy.sleep(1)
        self.assertEqual(warehouse_states, self.expected_states, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_states))

    def test_extract_one_state(self):
        rospy.wait_for_service("/has_robot_state")
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_one_state("state1")
        state_exporter.output_module(self.test_path + "/exporter_output_state.py")
        from exporter_output_state import warehouse_states
        rospy.sleep(1)
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_state))

    def test_extract_list(self):
        rospy.wait_for_service("/has_robot_state")
        rospy.sleep(2)
        state_exporter = SrRobotStateExporter()
        state_exporter.extract_list(["state1"])
        state_exporter.output_module(self.test_path + "/exporter_output_list.py")
        from exporter_output_list import warehouse_states
        rospy.sleep(1)
        self.assertEqual(warehouse_states, self.expected_state, msg="Export all states failed:" +
                         str(warehouse_states) + " not " + str(self.expected_state))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_robot_state_exporter", TestSrRobotStateExporter)
