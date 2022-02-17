#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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

import collections
from builtins import round
import copy
import rospy
import rostest
from unittest import TestCase
from sr_robot_commander.sr_robot_commander import SrRobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import FollowJointTrajectoryActionGoal
from moveit_msgs.msg import RobotState, RobotTrajectory, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK, SaveRobotStateToWarehouse
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_commander.exception import MoveItCommanderException
from moveit_commander import conversions
from actionlib_msgs.msg import GoalStatusArray
import tf2_ros
import time
import numpy as np
from math import fmod

# Some of the test cases do not have an assert method. In case of these methods the test verifies if
# the API of moveit_commander changed - i.e. change of methods name, number of arguments, return type

PKG = "sr_robot_commander"
CONST_RA_HOME_ANGLES = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                        'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                        'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 3.1415}

CONST_EXAMPLE_TARGET = {'ra_shoulder_pan_joint': 0.2, 'ra_elbow_joint': 1.80,
                        'ra_shoulder_lift_joint': -1.37, 'ra_wrist_1_joint': -0.52,
                        'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint':3.1415}

TOLERANCE_UNSAFE = 0.04


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message("/move_group/status", GoalStatusArray)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        rospy.sleep(10.0)  # Wait for Gazebo to sort itself out
        cls.robot_commander = SrRobotCommander("right_arm")
        cls.robot_commander.set_planner_id("BiTRRT")
        cls.eef = cls.robot_commander.get_end_effector_link()
        cls.set_ground()

    @classmethod
    def set_ground(cls, height=0.1, z_position=-0.1):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = z_position - (height / 2.0)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = cls.robot_commander._robot_commander.get_root_link()
        cls.robot_commander._planning_scene.add_box("ground", pose, (3, 3, height))

    def reset_to_home(self):
        rospy.sleep(1)
        self.robot_commander._reset_plan()
        self.robot_commander.move_to_joint_value_target(CONST_RA_HOME_ANGLES, wait=True, angle_degrees=False)
        rospy.sleep(1)

    def compare_poses(self, pose1, pose2, tolerance=0.02):
        pose1_list = [pose1.position.x, pose1.position.y, pose1.position.z,
                      pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        pose2_list = [pose2.position.x, pose2.position.y, pose2.position.z,
                      pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        pose1_list = [round(i, 2) for i in pose1_list]
        pose2_list = [round(i, 2) for i in pose2_list]

        for coordinate_1, coordinate_2 in zip(pose1_list, pose2_list):
            if abs(coordinate_1 - coordinate_2) >= tolerance:
                return False
        return True

    def normalize_angle_positive(self, angle):
        pi_2 = 2. * np.pi
        return fmod(fmod(angle, pi_2) + pi_2, pi_2)

    def compare_joint_states_by_common_joints(self, joint_state_1, joint_state_2, tolerance=0.02):
        joint_state_1_cpy = copy.deepcopy(joint_state_1)
        joint_state_2_cpy = copy.deepcopy(joint_state_2)
        common_joint_names = set(joint_state_1_cpy.keys()).intersection(set(joint_state_2_cpy.keys()))
        if len(common_joint_names) == 0:
            return False
        for key in common_joint_names:
            joint_state_1_cpy[key] = self.normalize_angle_positive(round(joint_state_1_cpy[key], 2))
            joint_state_2_cpy[key] = self.normalize_angle_positive(round(joint_state_2_cpy[key], 2))
            if abs(joint_state_1_cpy[key] - joint_state_2_cpy[key]) >= tolerance:
                return False
        return True

    '''
    def test_move_to_pose_target(self):
        self.reset_to_home()
        pose = conversions.list_to_pose([0.7261, 0.1733, 0.4007, -0.9999, 0.0005, 0.0084, 0.0])
        self.robot_commander.move_to_pose_target(pose, self.eef, wait=True)
        time.sleep(5)
        after_pose = self.robot_commander.get_current_pose()
        condition = self.compare_poses(pose, after_pose)
        self.assertTrue(condition)
    '''

    def test_plan_to_pose_target(self):
        self.reset_to_home()       
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.666, 0.174, 0.575, 1.0, 0.0, 0.0, 0.0])

        # Setting constraints
        constraints = Constraints()
        constraints_list = [] 
        for name, position in self.robot_commander.get_joints_position().items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.05
            joint_constraint.tolerance_below = 0.05
            joint_constraint.weight = 1
            constraints_list.append(joint_constraint)
            
        constraints.joint_constraints = constraints_list
        self.robot_commander._move_group_commander.set_path_constraints(constraints)
        rospy.logwarn(self.robot_commander._move_group_commander.get_path_constraints())
        # Constraints set

        expected_joint_state = self.robot_commander.get_ik(pose)
        expected_joint_state = dict(zip(expected_joint_state.name, expected_joint_state.position))
        plan_pose = self.robot_commander.plan_to_pose_target(pose.pose, end_effector_link=self.eef,
                                                             alternative_method=False, custom_start_state=None)
        end_joint_state = dict(zip(plan_pose.joint_trajectory.joint_names,
                                   plan_pose.joint_trajectory.points[-1].positions))
        time.sleep(5) # just to have some time to see things 
        self.assertTrue(self.compare_joint_states_by_common_joints(expected_joint_state, end_joint_state))
    

if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)
