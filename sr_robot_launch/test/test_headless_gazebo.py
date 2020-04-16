#!/usr/bin/env python

# Copyright 2015 Shadow Robot Company Ltd.
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

import rospy
import rostest
from sr_robot_commander.sr_arm_commander import SrArmCommander
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

PKG = "sr_robot_launch"


class TestHeadlessGazebo(TestCase):
    """
    Tests the Hand Commander
    """

    def setUp(self):
        rospy.init_node('test_headless_gazebo', anonymous=True)
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        self.arm_commander = SrArmCommander(name='right_arm_and_manipulator', set_ground=False)

    def test_headless_gazebo(self):
        expected_joints_target = {'ra_shoulder_pan_joint': 0.43, 'ra_elbow_joint': 2.12, 'ra_wrist_1_joint': -1.71,
                                  'ra_wrist_2_joint': 1.48, 'ra_shoulder_lift_joint': -2.58, 'ra_wrist_3_joint': 1.62}

        self.arm_commander.move_to_joint_value_target(expected_joints_target)
        rospy.sleep(10)
        final_joint_values = self.arm_commander.get_current_state()

        expected_and_final_joint_value_diff = 0
        for expected_value, final_value in zip(expected_joints_target, final_joint_values):
            expected_and_final_joint_value_diff += abs(expected_joints_target[expected_value] -
                                                       final_joint_values[final_value])
        self.assertAlmostEqual(expected_and_final_joint_value_diff, 0, 3)

if __name__ == "__main__":
    rostest.rosrun(PKG, "test_headlessGazebo", TestHeadlessGazebo)
