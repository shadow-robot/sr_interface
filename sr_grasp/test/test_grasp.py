#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2014, 2022-2023 belongs to Shadow Robot Company Ltd.
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


import unittest
import rospy
import rostest
from actionlib import SimpleActionClient, GoalStatus
from sr_robot_msgs.msg import GraspAction, GraspGoal
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint

PKG = 'sr_grasp'
NAME = 'test_grasp_object'


class TestGrasp(unittest.TestCase):
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

    def test_grasp_action(self):
        """Test sending a grasp."""
        goal = GraspGoal()
        goal.grasp = self.mk_grasp({
            'LFJ3': 1.4, 'RFJ3': 1.4, 'MFJ3': 1.4, 'FFJ3': 1.4,
            'LFJ0': 3.0, 'RFJ0': 3.0, 'MFJ0': 3.0, 'FFJ0': 3.0,
            })

        # Get a action client
        client = SimpleActionClient('grasp', GraspAction)
        client.wait_for_server()

        # Send pre-grasp
        goal.pre_grasp = True
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(20.0))
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED,
                         "Action did not return in SUCCEEDED state.")

        rospy.sleep(2)

        # Send grasp
        goal.pre_grasp = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(20.0))
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED,
                         "Action did not return in SUCCEEDED state.")


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestGrasp)
