#!/usr/bin/env python

# ##########################################################################
# Copyright (c) 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
#
# ###########################################################################

PKG = 'sr_grasp'
NAME = 'test_grasp_contact'

import os, unittest;
import rospy, rostest
from actionlib import SimpleActionClient, GoalStatus
from sr_robot_msgs.msg import GraspAction, GraspGoal
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint
from sr_hand.shadowhand_ros import ShadowHand_ROS

class TestGraspContact(unittest.TestCase):
    longMessage = True

    joint_names = [
            'LFJ0', 'LFJ3', 'LFJ4', 'LFJ5',
            'RFJ0', 'RFJ3', 'RFJ4',
            'MFJ0', 'MFJ3', 'MFJ4',
            'FFJ0', 'FFJ3', 'FFJ4',
            'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5',
            'WRJ1', 'WRJ2']

    def mk_grasp(self, joints):
        grasp = Grasp()
        # pre-grasp (just zero all for now)
        grasp.pre_grasp_posture.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        for jname in self.joint_names:
            jtp.positions.append(0.0)
        grasp.pre_grasp_posture.points.append(jtp)
        # grasp
        grasp.grasp_posture.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        for jname in self.joint_names:
            if jname in joints:
                jtp.positions.append(joints[jname])
            else:
                jtp.positions.append(0.0)
        grasp.grasp_posture.points.append(jtp)
        return grasp

    # Tests
    ########

    def test_grasp_contact(self):
        """Test sending a grasp with object in the way"""
        rospy.sleep(2)
        hand = ShadowHand_ROS()

        # Reset hand.
        for j in hand.allJoints:
            hand.sendupdate(j.name, 0.0)
        rospy.sleep(2)

        goal = GraspGoal()
        goal.grasp = self.mk_grasp({
            'LFJ3': 1.4, 'RFJ3': 1.4, 'MFJ3': 1.4, 'FFJ3': 1.4,
            'LFJ0': 2.0, 'RFJ0': 2.0, 'MFJ0': 2.0, 'FFJ0': 2.0,
            })
        goal.pre_grasp = False

        client = SimpleActionClient('grasp', GraspAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(20.0))
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED,
                "Action did not return in SUCCEEDED state.")

        rospy.sleep(2)

        # Reset hand.
        for j in hand.allJoints:
            hand.sendupdate(j.name, 0.0)
        rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestGraspContact)

