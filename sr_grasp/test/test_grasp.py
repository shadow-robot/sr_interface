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
NAME = 'test_grasp_object'

import os, unittest;
import rospy, rostest
from actionlib import SimpleActionClient, GoalStatus
from sr_robot_msgs.msg import GraspAction, GraspGoal
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint

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

