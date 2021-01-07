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
from moveit_msgs.msg import PlanningScene
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from rospy import get_rostime
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase


class TestHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim 
    """
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message('/move_group/status', GoalStatusArray)
        cls.hand_type = rospy.get_param('~test_hand_and_arm_sim/hand_type')
        cls.scene = rospy.get_param('~test_hand_and_arm_sim/scene')

        #for phantom hand tests use hand finder
        cls.hand_id = rospy.get_param('/hand/mapping/1082')

        if cls.hand_id == 'rh':
            cls.arm_id = 'ra'
            cls.robot_commander = SrRobotCommander(name="right_arm_and_hand")
            cls.hand_commander = SrHandCommander(name='right_hand')
            cls.arm_commander = SrArmCommander(name='right_arm', set_ground=False)
        elif cls.hand_id == 'lh':
            cls.arm_id = 'la'
            cls.robot_commander = SrRobotCommander(name="left_arm_and_hand")
            cls.hand_commander = SrHandCommander(name='left_hand')
            cls.arm_commander = SrArmCommander(name='left_arm', set_ground=False)

        #rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, self.scene_data_cb)

        rospy.sleep(10)

    @classmethod
    def tearDownClass(cls):
        pass

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

    def test_hand(self):
        hand_joints_target = {
         'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                                'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                                'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                                'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                                'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                                'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
         'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                            'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                            'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                            'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0},

         'hand_extra_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                             'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                             'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0}
                    }

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target[self.hand_id + '_' + key] = value

        self.hand_commander.move_to_joint_value_target(hand_joints_target, wait=True)
        rospy.sleep(5)
        final_hand_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff_hand = self.joints_error_check(hand_joints_target, final_hand_joint_values)

        print('diff for hand')
        print(expected_and_final_joint_value_diff_hand)
        self.assertAlmostEqual(expected_and_final_joint_value_diff_hand, 0, delta=0.2)

if __name__ == "__main__":
    PKGNAME = 'sr_robot_launch'
    NODENAME = 'test_hand_and_arm_sim'

    rospy.init_node(NODENAME, anonymous=True)
    rostest.rosrun(PKGNAME, NODENAME, TestHandAndArmSim)
