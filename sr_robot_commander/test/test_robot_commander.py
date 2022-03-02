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
import tf
from rosgraph_msgs.msg import Clock

# Some of the test cases do not have an assert method. In case of these methods the test verifies if
# the API of moveit_commander changed - i.e. change of methods name, number of arguments, return type

PKG = "sr_robot_commander"
CONST_RA_HOME_ANGLES = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                        'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                        'ra_wrist_2_joint': 1.5708, 'ra_wrist_3_joint': 3.14}

CONST_EXAMPLE_TARGET = {'ra_shoulder_pan_joint': 0.2, 'ra_elbow_joint': 1.80,
                        'ra_shoulder_lift_joint': -1.37, 'ra_wrist_1_joint': -0.52,
                        'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 0.00}

TOLERANCE_UNSAFE = 0.04
PLANNING_ATTEMPTS = 20


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message("/move_group/status", GoalStatusArray)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        rospy.wait_for_message("/clock", Clock)
        rospy.sleep(10.0)  # Wait for Gazebo to sort itself out
        cls.robot_commander = SrRobotCommander("right_arm")
        cls.robot_commander.set_planner_id("BiTRRT")
        cls.eef = cls.robot_commander.get_end_effector_link()
        cls.add_ground_plane()

    @classmethod
    def add_ground_plane(cls, height=0.05, z_position=0.05):
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
        cls.robot_commander._planning_scene.add_box("ground_plane", pose, (3, 3, height))

    def reset_to_home(self):
        rospy.sleep(1)
        self.robot_commander._reset_plan()
        self.robot_commander.move_to_joint_value_target(CONST_RA_HOME_ANGLES, wait=True, angle_degrees=False)
        self.robot_commander.set_start_state_to_current_state()
        rospy.sleep(1)

    def create_test_pose_rpy_from_current_pose(self):
        current_pose = self.robot_commander.get_current_pose(reference_frame="world")
        euler_pose_rot = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, 
                                                                   current_pose.orientation.z, current_pose.orientation.w])
        test_pose_rpy = [current_pose.position.x, current_pose.position.y, current_pose.position.z + 0.1,
                        euler_pose_rot[0], euler_pose_rot[1], euler_pose_rot[2] + 0.5]
        
        return test_pose_rpy

    def get_pose_msg_from_pose_rpy(self, pose_rpy):
        pose_msg = Pose()
        pose_msg.position.x = pose_rpy[0]
        pose_msg.position.y = pose_rpy[1]
        pose_msg.position.z = pose_rpy[2]

        quaternion = tf.transformations.quaternion_from_euler(pose_rpy[3], pose_rpy[4], pose_rpy[5])

        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]
        return pose_msg

    def compare_position(self, initial_position, desired_position, tolerance):
        initial_position_list = [initial_position.x, initial_position.y, initial_position.z]
        desired_position_list = [initial_position.x, initial_position.y, initial_position.z]

        for coordinate_1, coordinate_2 in zip(initial_position_list, desired_position_list):
            delta_position = abs(coordinate_1 - coordinate_2)
            if round(delta_position, 2) >= tolerance:
                return False
        return True

    def compare_orientation(self, initial_orientation, desired_orientation, tolerance):
        initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y, 
                                                                  initial_orientation.z, initial_orientation.w])
        desired_euler = tf.transformations.euler_from_quaternion([desired_orientation.x, desired_orientation.y,
                                                                  desired_orientation.z, desired_orientation.w])
        euler_delta_roll = desired_euler[0] - initial_euler[0]
        euler_delta_pitch = desired_euler[1] - initial_euler[1]
        euler_delta_yaw = desired_euler[2] - initial_euler[2]

        if (abs(euler_delta_roll) > tolerance and
            abs(euler_delta_pitch) > tolerance and 
            abs(euler_delta_yaw) > tolerance):
            return False
        return True

    def compare_poses(self, pose1, pose2, position_threshold=0.02, orientation_threshold=0.04):
        if (self.compare_position(pose1.position, pose2.position, position_threshold) and
            self.compare_orientation(pose1.orientation, pose2.orientation, orientation_threshold)):
           return True
        return False

    # def normalize_angle_positive(self, angle):
    #     pi_2 = 2. * np.pi
    #     return fmod(fmod(angle, pi_2) + pi_2, pi_2)

    # def compare_joint_states_by_common_joints(self, joint_state_1, joint_state_2, tolerance=0.02):
    #     joint_state_1_cpy = copy.deepcopy(joint_state_1)
    #     joint_state_2_cpy = copy.deepcopy(joint_state_2)
    #     common_joint_names = set(joint_state_1_cpy.keys()).intersection(set(joint_state_2_cpy.keys()))
    #     if len(common_joint_names) == 0:
    #         return False
    #     for key in common_joint_names:
    #         joint_state_1_cpy[key] = self.normalize_angle_positive(round(joint_state_1_cpy[key], 2))
    #         joint_state_2_cpy[key] = self.normalize_angle_positive(round(joint_state_2_cpy[key], 2))
    #         if abs(joint_state_1_cpy[key] - joint_state_2_cpy[key]) >= tolerance:
    #             return False
    #     return True

    # def test_get_and_set_planner_id(self):
    #     prev_planner_id = self.robot_commander._move_group_commander.get_planner_id()
    #     test_planner_id = "RRTstarkConfigDefault"
    #     self.robot_commander.set_planner_id(test_planner_id)
    #     self.assertEqual(test_planner_id, self.robot_commander._move_group_commander.get_planner_id())

    #     self.robot_commander.set_planner_id(prev_planner_id)

    # def test_get_and_set_planning_time(self):
    #     prev_planning_time = self.robot_commander._move_group_commander.get_planning_time()
    #     test_planning_time = 3
    #     self.robot_commander.set_planning_time(test_planning_time)
    #     self.assertEqual(test_planning_time, self.robot_commander._move_group_commander.get_planning_time())

    #     self.robot_commander.set_planning_time(prev_planning_time)

    # def test_set_num_planning_attempts(self):
    #     self.robot_commander.set_num_planning_attempts(3)

    # def test_get_end_effector_pose_from_state(self):
    #     robot_state = RobotState()
    #     for joint_name, angle in CONST_RA_HOME_ANGLES.items():
    #         robot_state.joint_state.name.append(joint_name)
    #         robot_state.joint_state.position.append(angle)
    #     pose = self.robot_commander.get_end_effector_pose_from_state(robot_state)
    #     condition = type(pose) == PoseStamped
    #     self.assertTrue(condition)

    # def test_get_planning_frame(self):
    #     self.robot_commander.get_planning_frame()

    # def test_set_and_get_pose_reference_frame(self):
    #     const_reference_frame = "world"
    #     self.robot_commander._move_group_commander.set_pose_reference_frame(const_reference_frame)
    #     self.assertEqual(self.robot_commander._move_group_commander.get_pose_reference_frame(), const_reference_frame)

    # def test_get_group_name(self):
    #     self.assertEqual(self.robot_commander._name, self.robot_commander.get_group_name())

    # def test_refresh_named_targets(self):
    #     self.robot_commander.refresh_named_targets()
    #     condition_1 = type(self.robot_commander._srdf_names) == list
    #     condition_2 = type(self.robot_commander._warehouse_names) == list
    #     self.assertTrue(condition_1 and condition_2)

    # def test_set_max_velocity_scaling_factor_range_ok(self):
    #     self.robot_commander.set_max_velocity_scaling_factor(0.2)

    # def test_set_max_velocity_scaling_factor_range_not_ok(self):
    #     self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_velocity_scaling_factor, 3)

    # def test_set_max_acceleration_scaling_factor_range_ok(self):
    #     self.robot_commander.set_max_acceleration_scaling_factor(0.2)

    # def test_set_max_acceleration_scaling_factor_range_not_ok(self):
    #     self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_acceleration_scaling_factor, 3)

    # def test_allow_looking(self):
    #     self.robot_commander.allow_looking(True)

    # def test_allow_replanning(self):
    #     self.robot_commander.allow_replanning(True)

    # def test_plan_to_joint_value_target(self):
    #     self.reset_to_home()
    #     plan = self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     tries = 0
    #     while len(plan.joint_trajectory.points) == 0 and tries < PLANNING_ATTEMPTS:
    #         plan = self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
    #                                                                custom_start_state=None)
    #         time.sleep(1)
    #         tries += 1
    #     condition = len(plan.joint_trajectory.points) != 0
    #     if not condition:
    #         rospy.logerr("Method: test_plan_to_joint_value_target")
    #     self.assertTrue(condition)

    # def test_execute(self):
    #     self.reset_to_home()
    #     self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
    #                                                     custom_start_state=None)
    #     self.robot_commander.execute()
    #     executed_joints = self.robot_commander.get_current_state()
    #     condition = self.compare_joint_states_by_common_joints(CONST_EXAMPLE_TARGET, executed_joints)
    #     self.assertTrue(condition)

    # def test_execute_plan(self):
    #     self.reset_to_home()
    #     plan = self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     self.robot_commander.execute_plan(plan)
    #     executed_joints = self.robot_commander.get_current_state()
    #     condition = self.compare_joint_states_by_common_joints(executed_joints, CONST_RA_HOME_ANGLES)
    #     self.assertTrue(condition)

    # def test_check_plan_is_valid_ok(self):
    #     self.reset_to_home()
    #     self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
    #                                                     custom_start_state=None)
    #     self.assertTrue(self.robot_commander.check_plan_is_valid())

    # def test_check_plan_is_valid_not_ok(self):
    #     self.reset_to_home()
    #     self.robot_commander._SrRobotCommander__plan = None
    #     condition = self.robot_commander.check_plan_is_valid()
    #     self.assertFalse(condition)

    # def test_check_given_plan_is_valid_ok(self):
    #     self.reset_to_home()
    #     plan = self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     self.assertTrue(self.robot_commander.check_given_plan_is_valid(plan))

    # def test_check_given_plan_is_valid_not_ok(self):
    #     self.reset_to_home()
    #     not_valid_goal = copy.deepcopy(CONST_RA_HOME_ANGLES)
    #     out_of_range_value = 3.0
    #     not_valid_goal['ra_elbow_joint'] = out_of_range_value
    #     plan = self.robot_commander.plan_to_joint_value_target(not_valid_goal, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     self.assertFalse(self.robot_commander.check_given_plan_is_valid(plan))

    # def test_evaluate_given_plan_none(self):
    #     self.reset_to_home()
    #     plan = None
    #     evaluation = self.robot_commander.evaluate_given_plan(plan)
    #     self.assertIsNone(evaluation)

    # def test_evaluate_given_plan_low_quality(self):
    #     self.reset_to_home()
    #     end_joints = copy.deepcopy(CONST_RA_HOME_ANGLES)
    #     end_joints['ra_shoulder_pan_joint'] += 0.8
    #     end_joints['ra_shoulder_lift_joint'] -= 0.3
    #     end_joints['ra_elbow_joint'] -= 0.6
    #     end_joints['ra_wrist_1_joint'] += 0.4
    #     end_joints['ra_wrist_2_joint'] -= 0.4
    #     end_joints['ra_wrist_3_joint'] += 0.3
    #     plan = self.robot_commander.plan_to_joint_value_target(end_joints, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     evaluation = self.robot_commander.evaluate_given_plan(plan)
    #     self.assertGreater(evaluation, 10)

    # def test_evaluate_given_plan_high_quality(self):
    #     self.reset_to_home()
    #     end_joints = copy.deepcopy(CONST_RA_HOME_ANGLES)
    #     end_joints['ra_shoulder_pan_joint'] += 0.01
    #     plan = self.robot_commander.plan_to_joint_value_target(end_joints, angle_degrees=False,
    #                                                            custom_start_state=None)
    #     evaluation = self.robot_commander.evaluate_given_plan(plan)
    #     self.assertLess(evaluation, 1)

    # def test_evaluate_plan_quality_good(self):
    #     self.assertEqual('good', self.robot_commander.evaluate_plan_quality(7))

    # def test_evaluate_plan_quality_poor(self):
    #     self.assertEqual('poor', self.robot_commander.evaluate_plan_quality(60))

    # def test_evaluate_plan_quality_medium(self):
    #     self.assertEqual('medium', self.robot_commander.evaluate_plan_quality(32))

    # def test_get_robot_name(self):
    #     self.assertTrue(self.robot_commander.get_robot_name() == self.robot_commander._robot_name)

    # # if launched sr_ur_arm_box.launch
    # def test_named_target_in_srdf_exist(self):
    #     const_test_names = ['lifted', 'flat']
    #     for name in const_test_names:
    #         if self.robot_commander.named_target_in_srdf(name) is False:
    #             self.fail()

    # def test_named_target_in_srdf_not_exist(self):
    #     const_test_names = ['non_existing_target_test_name']
    #     for name in const_test_names:
    #         if self.robot_commander.named_target_in_srdf(name) is True:
    #             self.fail()

    # def test_set_named_target_correct_name(self):
    #     test_names = self.robot_commander.get_named_targets()
    #     result = self.robot_commander.set_named_target(test_names[0])
    #     self.assertTrue(result)

    # def test_set_named_target_false_name(self):
    #     result = self.robot_commander.set_named_target("test_false_name")
    #     self.assertFalse(result)

    # def test_get_named_target_joint_values_srdf(self):
    #     test_names = self.robot_commander._srdf_names
    #     if len(test_names) > 0:
    #         output = self.robot_commander.get_named_target_joint_values(test_names[0])
    #         self.assertIsInstance(output, dict)

    # def test_get_named_target_joint_values_warehouse(self):
    #     test_names = self.robot_commander._warehouse_names
    #     if len(test_names) > 0:
    #         output = self.robot_commander.get_named_target_joint_values(test_names[0])
    #         self.assertIsInstance(output, dict)

    # def test_get_named_target_joint_values_no_target(self):
    #     const_test_names = ["no_target"]
    #     output = self.robot_commander.get_named_target_joint_values(const_test_names[0])
    #     self.assertIsNone(output)

    # def test_get_end_effector_link(self):
    #     self.assertIsInstance(self.robot_commander.get_end_effector_link(), str)

    # def test_get_current_pose_frame(self):
    #     pose = self.robot_commander.get_current_pose(reference_frame="world")
    #     self.assertIsInstance(pose, Pose)

    # def test_get_current_pose_frame_none(self):
    #     pose = self.robot_commander.get_current_pose(reference_frame=None)
    #     self.assertIsInstance(pose, Pose)

    # def test_get_current_pose_frame_wrong(self):
    #     pose = self.robot_commander.get_current_pose(reference_frame="test_wrong_frame")
    #     self.assertIsNone(pose)

    # def test_get_current_state(self):
    #     self.assertIsInstance(self.robot_commander.get_current_state(), dict)

    # def test_get_current_state_bounded(self):
    #     self.assertIsInstance(self.robot_commander.get_current_state_bounded(), dict)

    # def test_get_robot_state_bounded(self):
    #     self.assertIsInstance(self.robot_commander.get_current_state_bounded(), dict)

    # def test_plan_to_named_target_custom_start_state_none(self):
    #     self.reset_to_home()
    #     target_names = self.robot_commander.get_named_targets()
    #     if len(target_names) > 0:
    #         self.robot_commander.plan_to_named_target(target_names[0], None)
    #     self.assertIsInstance(self.robot_commander._SrRobotCommander__plan, RobotTrajectory)

    # def test_plan_to_named_target_custom_start_state_exists(self):
    #     self.reset_to_home()
    #     target_names = self.robot_commander.get_named_targets()

    #     if len(target_names) > 0:
    #         robot_state = RobotState()
    #         for key, value in CONST_RA_HOME_ANGLES .items():
    #             robot_state.joint_state.name.append(key)
    #             robot_state.joint_state.position.append(value)
    #         plan = self.robot_commander.plan_to_named_target(target_names[0], robot_state)

    #     self.assertIsInstance(self.robot_commander._SrRobotCommander__plan, RobotTrajectory)

    # def test_plan_to_named_target_target_not_exists(self):
    #     self.reset_to_home()
    #     const_test_name = "test_non_existing_target"
    #     self.robot_commander.plan_to_named_target(const_test_name, None)
    #     condition = self.robot_commander._SrRobotCommander__plan is None
    #     self.assertTrue(condition)

    # def test_get_named_targets(self):
    #     self.assertIsInstance(self.robot_commander.get_named_targets(), list)

    # def test_get_joints_position(self):
    #     ret_val = self.robot_commander.get_joints_position()
    #     self.assertIsInstance(ret_val, dict)

    # def test_get_joints_velocity(self):
    #     ret_val = self.robot_commander.get_joints_velocity()
    #     self.assertIsInstance(ret_val, dict)

    # def test_get_joints_state(self):
    #     ret_val = self.robot_commander.get_joints_state()
    #     self.assertIsInstance(ret_val, JointState)

    def test_run_joint_trajectory_executed(self):
        self.reset_to_home()
        initial_joint_state = self.robot_commander.get_current_state()
        desired_joint_state = {}
        for key in initial_joint_state:
            desired_joint_state[key] = initial_joint_state[key] + 0.05

        # create joint trajectory message
        desired_joint_trajectory = JointTrajectory()
        desired_joint_trajectory.joint_names = list(desired_joint_state.keys())
        point = JointTrajectoryPoint()
        point.positions = list(desired_joint_state.values())
        desired_joint_trajectory.points.append(point)

        self.assertTrue(self.robot_commander.run_joint_trajectory(desired_joint_trajectory))

    # def test_make_named_trajectory(self):
    #     self.reset_to_home()
    #     trajectory = []
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00},
    #                        "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 2.00},
    #                        "interpolate_time": 0.4, "pause_time": 0.1, "degrees": False})
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 1.95},
    #                        "interpolate_time": 0.6, "pause_time": 0.1, "degrees": False})
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.10, 'ra_elbow_joint': 1.90},
    #                        "interpolate_time": 0.7, "pause_time": 0.1, "degrees": False})
    #     named_trajectory = self.robot_commander.make_named_trajectory(trajectory)
    #     all_positions = []

    #     for i in range(0, len(named_trajectory.points)):
    #         all_positions.append(named_trajectory.points[i].positions)
    #     for waypoint in trajectory:
    #         for joint in waypoint["joint_angles"].keys():
    #             if not any(waypoint["joint_angles"][joint] in sublist for sublist in all_positions):
    #                 self.fail()

    #     self.assertIsInstance(self.robot_commander.make_named_trajectory(trajectory), JointTrajectory)

    # def test_send_stop_trajectory_unsafe(self):
    #     self.reset_to_home()
    #     start_joints = self.robot_commander.get_joints_position()
    #     trajectory = self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
    #                                                                  custom_start_state=None).joint_trajectory
    #     self.robot_commander.run_joint_trajectory(trajectory)
    #     time.sleep(0.5)
    #     self.robot_commander.send_stop_trajectory_unsafe()
    #     end_joints = self.robot_commander.get_joints_position()
    #     condition = self.compare_joint_states_by_common_joints(start_joints, end_joints, TOLERANCE_UNSAFE)
    #     self.assertFalse(condition)

    # def test_run_named_trajectory_unsafe_cancelled(self):
    #     self.reset_to_home()
    #     trajectory = []
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.50, 'ra_elbow_joint': 2.3},
    #                        "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
    #     self.robot_commander.run_named_trajectory_unsafe(trajectory)

    #     for client in self.robot_commander._clients:
    #         self.robot_commander._action_running[client] = True
    #         self.robot_commander._clients[client].cancel_goal()

    #     joint_state = self.robot_commander.get_current_state()
    #     expected_joint_state = trajectory[-1]['joint_angles']
    #     condition = self.compare_joint_states_by_common_joints(expected_joint_state, joint_state, TOLERANCE_UNSAFE)
    #     if condition:
    #         rospy.logerr("Method: test_run_named_trajectory_unsafe_cancelled")
    #         rospy.logerr("Expected end joint state: {}".format(expected_joint_state))
    #         rospy.logerr("Actuall end joint state: {}".format(joint_state))
    #     self.assertFalse(condition)

    # def test_run_named_trajectory_unsafe_executed(self):
    #     self.reset_to_home()
    #     trajectory = []
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.0},
    #                        "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
    #     self.robot_commander.run_named_trajectory_unsafe(trajectory)
    #     joint_state = self.robot_commander.get_current_state()
    #     expected_joint_state = trajectory[-1]['joint_angles']
    #     condition = self.compare_joint_states_by_common_joints(expected_joint_state, joint_state, TOLERANCE_UNSAFE)
    #     self.assertTrue(condition)

    # def test_run_named_trajectory(self):
    #     self.reset_to_home()
    #     trajectory = []
    #     trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00},
    #                        "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
    #     self.robot_commander.run_named_trajectory(trajectory)
    #     joint_state = self.robot_commander.get_current_state()
    #     expected_joint_state = trajectory[-1]['joint_angles']
    #     condition = self.compare_joint_states_by_common_joints(expected_joint_state, joint_state)
    #     if not condition:
    #         rospy.logerr("Method: test_run_named_trajectory")
    #         rospy.logerr("Expected end joint state: {}".format(expected_joint_state))
    #         rospy.logerr("Actuall end joint state: {}".format(joint_state))
    #     self.assertTrue(condition)

    def test_move_to_pose_target(self):
        self.reset_to_home()
        desired_pose_rpy = self.create_test_pose_rpy_from_current_pose()
        is_pose_reached = False
        tries = 0
        desired_pose_msg = self.get_pose_msg_from_pose_rpy(desired_pose_rpy)
        while not is_pose_reached and tries < 5:
            tries += 1
            rospy.loginfo("Moving to pose target attempt {}".format(tries))
            self.robot_commander.move_to_pose_target(desired_pose_rpy, self.eef, wait=True)
            current_pose = self.robot_commander.get_current_pose()
            is_pose_reached = self.compare_poses(current_pose, desired_pose_msg)

        self.assertTrue(is_pose_reached)

    def test_plan_to_pose_target(self):
        self.reset_to_home()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        desired_pose_rpy = self.create_test_pose_rpy_from_current_pose()
        pose.pose = self.get_pose_msg_from_pose_rpy(desired_pose_rpy)

        plan = self.robot_commander.plan_to_pose_target(pose.pose, end_effector_link=self.eef,
                                                        alternative_method=False, custom_start_state=None)
        self.assertIsInstance(plan, RobotTrajectory)

    def test_move_to_joint_value_target_unsafe_executed(self):
        self.reset_to_home()
        initial_joint_state = self.robot_commander.get_current_state()
        desired_joint_state = {}
        for key in initial_joint_state:
            desired_joint_state[key] = initial_joint_state[key] + 0.05
        self.robot_commander.move_to_joint_value_target_unsafe(desired_joint_state, time=0.002, wait=True,
                                                               angle_degrees=False)
        executed_joints_list = self.robot_commander.get_current_joint_values()
        self.assertTrue(np.allclose(list(desired_joint_state.values()), executed_joints_list, TOLERANCE_UNSAFE))

    # def test_run_joint_trajectory_unsafe_executed(self):
    #     self.reset_to_home()
    #     initial_joint_state = self.robot_commander.get_current_state()
    #     desired_joint_state = {}
    #     for key in initial_joint_state:
    #         desired_joint_state[key] = initial_joint_state[key] + 0.05

    #     # create joint trajectory message
    #     desired_joint_trajectory = JointTrajectory()
    #     desired_joint_trajectory.joint_names = list(desired_joint_state.keys())
    #     point = JointTrajectoryPoint()
    #     point.positions = list(desired_joint_state.values())
    #     desired_joint_trajectory.points.append(point)

    #     self.assertTrue(self.robot_commander.run_joint_trajectory_unsafe(desired_joint_trajectory))

    # def test_plan_to_waypoints_target(self):
    #     self.reset_to_home()
    #     waypoints = []
    #     waypoints.append(conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1]))
    #     waypoints.append(conversions.list_to_pose([0.71, 0.15, 0.34, 0, 0, 0, 1]))
    #     waypoints.append(conversions.list_to_pose([0.69, 0.15, 0.34, 0, 0, 0, 1]))
    #     waypoints.append(conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1]))
    #     (plan, f) = self.robot_commander.plan_to_waypoints_target(waypoints)
    #     self.robot_commander.run_joint_trajectory_unsafe(plan.joint_trajectory)
    #     actual_pose = self.robot_commander.get_current_pose()
    #     expected_pose = waypoints[-1]
    #     condition = self.compare_poses(actual_pose, expected_pose)
    #     if not condition:
    #         rospy.logerr("Method: test_plan_to_waypoints_target")
    #         rospy.logerr("Expected end joint state: {}".format(actual_pose))
    #         rospy.logerr("Actuall end joint state: {}".format(expected_pose))
    #     self.assertTrue(condition)

    # def test_move_to_trajectory_start_trajecotry_exists(self):
    #     self.reset_to_home()
    #     trajectory = self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
    #                                                                  custom_start_state=None).joint_trajectory
    #     joints_from_trajectory = dict(zip(trajectory.joint_names, trajectory.points[0].positions))
    #     self.robot_commander.move_to_trajectory_start(trajectory)
    #     time.sleep(5)
    #     current_joints = self.robot_commander.get_current_state()
    #     condition = self.compare_joint_states_by_common_joints(joints_from_trajectory, current_joints)

    #     if not condition:
    #         rospy.logerr("Method: test_move_to_trajectory_start_trajecotry_exists")
    #         rospy.logerr("Expected joint states: {}".format(joints_from_trajectory))
    #         rospy.logerr("Actuall joint states: {}".format(current_joints))

    #     self.assertTrue(condition)

    # def test_get_ik(self):
    #     self.reset_to_home()
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.get_rostime()
    #     pose.pose = conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1])
    #     joint_state_from_ik = self.robot_commander.get_ik(pose)
    #     '''
    #     plan = self.robot_commander.plan_to_pose_target(pose.pose, end_effector_link=self.eef,
    #                                                     alternative_method=False, custom_start_state=None)
    #     joint_state_from_ik = dict(zip(joint_state_from_ik.name, joint_state_from_ik.position))
    #     joint_state_plan = dict(zip(plan.joint_trajectory.joint_names, plan.joint_trajectory.points[-1].positions))
    #     condition = self.compare_joint_states_by_common_joints(joint_state_from_ik, joint_state_plan)
    #     '''
    #     condition = type(joint_state_from_ik) == JointState
    #     self.assertTrue(condition)

    # def test_move_to_joint_value_target(self):
    #     self.reset_to_home()
    #     self.robot_commander.move_to_joint_value_target(CONST_EXAMPLE_TARGET, wait=True, angle_degrees=False)
    #     time.sleep(1)
    #     end_state = self.robot_commander.get_current_state()
    #     condition = self.compare_joint_states_by_common_joints(end_state, CONST_EXAMPLE_TARGET)
    #     self.assertTrue(condition)

    # def test_move_to_pose_value_target_unsafe_executed(self):
    #     self.reset_to_home()
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.get_rostime()
    #     pose.pose = conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1])
    #     self.robot_commander.move_to_pose_value_target_unsafe(pose)
    #     rospy.sleep(1)
    #     after_pose = self.robot_commander.get_current_pose()
    #     condition = self.compare_poses(pose.pose, after_pose, TOLERANCE_UNSAFE)
    #     self.assertTrue(condition)

    # def test_move_to_pose_value_target_unsafe_cancelled(self):
    #     self.reset_to_home()
    #     start_pose = self.robot_commander.get_current_pose()
    #     target_pose = PoseStamped()
    #     target_pose.header.stamp = rospy.get_rostime()
    #     target_pose.pose = conversions.list_to_pose([0.91, 0.17, 0.34, 0, 0, 0, 1])

    #     self.robot_commander.move_to_pose_value_target_unsafe(target_pose, time=2, wait=False)
    #     rospy.sleep(1)

    #     for client in self.robot_commander._clients:
    #         self.robot_commander._action_running[client] = True
    #         self.robot_commander._clients[client].cancel_goal()

    #     stopped_pose = self.robot_commander.get_current_pose()

    #     condition_1 = self.compare_poses(start_pose, target_pose.pose, TOLERANCE_UNSAFE)
    #     condition_2 = self.compare_poses(target_pose.pose, stopped_pose, TOLERANCE_UNSAFE)

    #     if not (condition_1 is False and condition_2 is False):
    #         rospy.logerr("Method: test_move_to_pose_value_target_unsafe_cancelled")
    #         rospy.logerr("Starting position: {}".format(start_pose))
    #         rospy.logerr("Target position: {}".format(target_pose.pose))
    #         rospy.logerr("Current position: {}".format(stopped_pose))

    #     self.assertTrue(condition_1 is False and condition_2 is False)

    # def test_move_to_position_target(self):
    #     self.reset_to_home()

    #     xyz = [0.71, 0.17, 0.34]
    #     target_xyz = Pose()
    #     target_xyz.position.x = xyz[0]
    #     target_xyz.position.y = xyz[1]
    #     target_xyz.position.z = xyz[2]
    #     target_xyz.orientation = self.robot_commander.get_current_pose().orientation

    #     self.robot_commander.move_to_position_target(xyz, self.eef)
    #     time.sleep(5)
    #     end_pose = self.robot_commander.get_current_pose()

    #     condition = self.compare_poses(target_xyz, end_pose)
    #     tries = 0
    #     while not condition and tries < 5:
    #         self.reset_to_home()
    #         tries += 1
    #         rospy.logwarn("test_move_to_position_target {}".format(tries))
    #         self.robot_commander.move_to_position_target(xyz, self.eef)
    #         condition = self.compare_poses(target_xyz, end_pose)
    #         end_pose = self.robot_commander.get_current_pose()
    #         rospy.logwarn(end_pose)

    #     if not condition:
    #         rospy.logerr("Method: test_move_to_position_target")
    #         rospy.logerr("Target position: {}".format(target_xyz))
    #         rospy.logerr("End position: {}".format(end_pose))

    #     self.assertTrue(condition)

    # def test_plan_to_position_target(self):
    #     self.reset_to_home()
    #     xyz = [0.71, 0.17, 0.34]
    #     start_pose = PoseStamped()
    #     start_pose.header.stamp = rospy.get_rostime()
    #     start_pose.pose = self.robot_commander.get_current_pose()
    #     start_pose.pose.position.x = xyz[0]
    #     start_pose.pose.position.y = xyz[1]
    #     start_pose.pose.position.z = xyz[2]

    #     plan = self.robot_commander.plan_to_position_target(xyz, self.eef)
    #     '''
    #     last_planned_joint_state = dict(zip(plan.joint_trajectory.joint_names,
    #                                         plan.joint_trajectory.points[-1].positions))
    #     expected_joint_state = self.robot_commander.get_ik(start_pose)
    #     expected_joint_state = dict(zip(expected_joint_state.name, expected_joint_state.position))
    #     condition = self.compare_joint_states_by_common_joints(expected_joint_state, last_planned_joint_state)
    #     '''
    #     tries = 0
    #     while len(plan.joint_trajectory.points) == 0 and tries < PLANNING_ATTEMPTS:
    #         plan = self.robot_commander.plan_to_position_target(xyz, self.eef)
    #         time.sleep(1)
    #         tries += 1
    #     condition = len(plan.joint_trajectory.points) != 0
    #     if not condition:
    #         rospy.logerr("Method: test_plan_to_position_target")
    #     self.assertTrue(condition)

    # def test_move_to_named_target(self):
    #     self.reset_to_home()
    #     named_target = "home"
    #     self.robot_commander.move_to_named_target(named_target)
    #     expected_joint_state = self.robot_commander.get_named_target_joint_values(named_target)
    #     end_joint_state = self.robot_commander.get_current_state()
    #     condition = self.compare_joint_states_by_common_joints(end_joint_state, expected_joint_state)
    #     self.assertTrue(condition)

    # def test_action_is_running(self):
    #     self.reset_to_home()
    #     self.robot_commander.move_to_joint_value_target_unsafe(CONST_EXAMPLE_TARGET, time=1, wait=False,
    #                                                            angle_degrees=False)
    #     condition_1 = (self.robot_commander.action_is_running() is True)

    #     self.reset_to_home()
    #     self.robot_commander.move_to_joint_value_target_unsafe(CONST_EXAMPLE_TARGET, time=1, wait=True,
    #                                                            angle_degrees=False)
    #     condition_2 = (self.robot_commander.action_is_running() is False)
    #     self.assertTrue(condition_1 and condition_2)

    # no working teach mode so far
    # def test_set_teach_mode(self):


if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)
