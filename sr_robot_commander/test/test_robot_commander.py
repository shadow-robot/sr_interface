#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
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


from builtins import round
import copy
from unittest import TestCase
import numpy as np
import tf
import rospy
from rosgraph_msgs.msg import Clock
import rostest
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from moveit_commander.exception import MoveItCommanderException
from moveit_commander import conversions
from actionlib_msgs.msg import GoalStatusArray
from sr_robot_commander.sr_robot_commander import SrRobotCommander


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
PLANNING_ATTEMPTS = 5


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_message("/move_group/status", GoalStatusArray, 60.0)
        rospy.wait_for_service("/gazebo/set_model_configuration", 60.0)
        rospy.wait_for_message("/clock", Clock, 60.0)
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
        pose.header.frame_id = cls.robot_commander.get_moveit_robot_commander().get_root_link()
        cls.robot_commander.get_moveit_planning_scene().add_box("ground_plane", pose, (3, 3, height))

    def reset_to_home(self):
        rospy.sleep(1)
        self.robot_commander.set_max_velocity_scaling_factor(1.0)
        self.robot_commander.set_max_acceleration_scaling_factor(1.0)
        self.robot_commander.reset_plan()
        self.robot_commander.move_to_joint_value_target(CONST_RA_HOME_ANGLES, wait=True, angle_degrees=False)
        self.robot_commander.set_start_state_to_current_state()
        rospy.sleep(1)

    def create_test_pose_rpy_from_current_pose(self):
        current_pose = self.robot_commander.get_current_pose(reference_frame="world")
        euler_pose_rot = tf.transformations.euler_from_quaternion(
            [current_pose.orientation.x, current_pose.orientation.y,
             current_pose.orientation.z, current_pose.orientation.w])
        test_pose_rpy = [current_pose.position.x, current_pose.position.y, current_pose.position.z + 0.1,
                         euler_pose_rot[0], euler_pose_rot[1], euler_pose_rot[2] + 0.5]
        return test_pose_rpy

    def get_pose_msg_from_pose_rpy(self, pose_rpy):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.get_rostime()
        pose_msg.header.frame_id = self.robot_commander.get_moveit_robot_commander().get_root_link()
        pose_msg.pose.position.x = pose_rpy[0]
        pose_msg.pose.position.y = pose_rpy[1]
        pose_msg.pose.position.z = pose_rpy[2]

        quaternion = tf.transformations.quaternion_from_euler(pose_rpy[3], pose_rpy[4], pose_rpy[5])

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg

    @staticmethod
    def compare_position(initial_position, desired_position, tolerance):
        initial_position_list = [initial_position.x, initial_position.y, initial_position.z]
        desired_position_list = [desired_position.x, desired_position.y, desired_position.z]

        for coordinate_1, coordinate_2 in zip(initial_position_list, desired_position_list):
            delta_position = abs(coordinate_1 - coordinate_2)
            if round(delta_position, 2) >= tolerance:
                return False
        return True

    @staticmethod
    def compare_orientation(initial_orientation, desired_orientation, tolerance):
        initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y,
                                                                  initial_orientation.z, initial_orientation.w])
        desired_euler = tf.transformations.euler_from_quaternion([desired_orientation.x, desired_orientation.y,
                                                                  desired_orientation.z, desired_orientation.w])
        euler_delta_roll = abs(desired_euler[0] - initial_euler[0])
        euler_delta_pitch = abs(desired_euler[1] - initial_euler[1])
        euler_delta_yaw = abs(desired_euler[2] - initial_euler[2])

        if (euler_delta_roll > tolerance and euler_delta_pitch > tolerance and euler_delta_yaw > tolerance):
            return False
        return True

    def compare_poses(self, pose1, pose2, position_threshold=0.02, orientation_threshold=0.04):
        if (self.compare_position(pose1.position, pose2.position, position_threshold) and
                self.compare_orientation(pose1.orientation, pose2.orientation, orientation_threshold)):
            return True
        return False

    def test_get_and_set_planner_id(self):
        prev_planner_id = self.robot_commander.get_move_group_commander().get_planner_id()
        test_planner_id = "RRTstarkConfigDefault"
        self.robot_commander.set_planner_id(test_planner_id)
        self.assertEqual(test_planner_id, self.robot_commander.get_move_group_commander().get_planner_id())
        self.robot_commander.set_planner_id(prev_planner_id)

    def test_get_and_set_planning_time(self):
        prev_planning_time = self.robot_commander.get_move_group_commander().get_planning_time()
        test_planning_time = 3
        self.robot_commander.set_planning_time(test_planning_time)
        self.assertEqual(test_planning_time, self.robot_commander.get_move_group_commander().get_planning_time())
        self.robot_commander.set_planning_time(prev_planning_time)

    def test_set_num_planning_attempts(self):
        self.robot_commander.set_num_planning_attempts(PLANNING_ATTEMPTS)

    def test_get_end_effector_pose_from_state(self):
        robot_state = RobotState()
        for joint_name, angle in CONST_RA_HOME_ANGLES.items():
            robot_state.joint_state.name.append(joint_name)
            robot_state.joint_state.position.append(angle)
        pose = self.robot_commander.get_end_effector_pose_from_state(robot_state)
        self.assertIsInstance(pose, PoseStamped)

    def test_get_planning_frame(self):
        self.robot_commander.get_planning_frame()

    def test_set_and_get_pose_reference_frame(self):
        const_reference_frame = "world"
        self.robot_commander.get_move_group_commander().set_pose_reference_frame(const_reference_frame)
        self.assertEqual(self.robot_commander.get_move_group_commander().get_pose_reference_frame(),
                         const_reference_frame)

    def test_set_max_velocity_scaling_factor_range_ok(self):
        self.robot_commander.set_max_velocity_scaling_factor(0.2)

    def test_set_max_velocity_scaling_factor_range_not_ok(self):
        self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_velocity_scaling_factor, 3)

    def test_set_max_acceleration_scaling_factor_range_ok(self):
        self.robot_commander.set_max_acceleration_scaling_factor(0.2)

    def test_set_max_acceleration_scaling_factor_range_not_ok(self):
        self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_acceleration_scaling_factor, 3)

    def test_allow_looking(self):
        self.robot_commander.allow_looking(True)

    def test_allow_replanning(self):
        self.robot_commander.allow_replanning(True)

    def test_plan_to_joint_value_target(self):
        plan = self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertIsInstance(plan, RobotTrajectory)

    def test_execute(self):
        self.reset_to_home()
        self.robot_commander.plan_to_joint_value_target(CONST_EXAMPLE_TARGET, angle_degrees=False,
                                                        custom_start_state=None)
        self.assertTrue(self.robot_commander.execute())

    def test_execute_plan(self):
        self.reset_to_home()
        plan = self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertTrue(self.robot_commander.execute_plan(plan))

    def test_check_plan_is_valid_ok(self):
        self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
                                                        custom_start_state=None)
        self.assertTrue(self.robot_commander.check_plan_is_valid())

    def test_check_plan_is_valid_not_ok(self):
        self.robot_commander.reset_plan()
        condition = self.robot_commander.check_plan_is_valid()
        self.assertFalse(condition)

    def test_check_given_plan_is_valid_ok(self):
        plan = self.robot_commander.plan_to_joint_value_target(CONST_RA_HOME_ANGLES, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertTrue(self.robot_commander.check_given_plan_is_valid(plan))

    def test_check_given_plan_is_valid_not_ok(self):
        not_valid_goal = copy.deepcopy(CONST_RA_HOME_ANGLES)
        out_of_range_value = 3.0
        not_valid_goal['ra_elbow_joint'] = out_of_range_value
        plan = self.robot_commander.plan_to_joint_value_target(not_valid_goal, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertFalse(self.robot_commander.check_given_plan_is_valid(plan))

    def test_evaluate_given_plan_none(self):
        plan = None
        evaluation = self.robot_commander.evaluate_given_plan(plan)
        self.assertIsNone(evaluation)

    def test_evaluate_given_plan_low_quality(self):
        end_joints = copy.deepcopy(CONST_RA_HOME_ANGLES)
        end_joints['ra_shoulder_pan_joint'] += 0.8
        end_joints['ra_shoulder_lift_joint'] -= 0.3
        end_joints['ra_elbow_joint'] -= 0.6
        end_joints['ra_wrist_1_joint'] += 0.4
        end_joints['ra_wrist_2_joint'] -= 0.4
        end_joints['ra_wrist_3_joint'] += 0.3
        plan = self.robot_commander.plan_to_joint_value_target(end_joints, angle_degrees=False,
                                                               custom_start_state=None)
        evaluation = self.robot_commander.evaluate_given_plan(plan)
        self.assertGreater(evaluation, 10)

    def test_evaluate_given_plan_high_quality(self):
        self.reset_to_home()
        end_joints = copy.deepcopy(CONST_RA_HOME_ANGLES)
        end_joints['ra_shoulder_pan_joint'] += 0.01
        plan = self.robot_commander.plan_to_joint_value_target(end_joints, angle_degrees=False,
                                                               custom_start_state=None)
        evaluation = self.robot_commander.evaluate_given_plan(plan)
        self.assertLess(evaluation, 1)

    def test_evaluate_plan_quality_good(self):
        self.assertEqual('good', self.robot_commander.evaluate_plan_quality(7))

    def test_evaluate_plan_quality_poor(self):
        self.assertEqual('poor', self.robot_commander.evaluate_plan_quality(60))

    def test_evaluate_plan_quality_medium(self):
        self.assertEqual('medium', self.robot_commander.evaluate_plan_quality(32))

    # It shouldn't be tested like this but assigning a robot name
    # def test_get_robot_name(self):
    #     self.assertTrue(self.robot_commander.get_robot_name() == self.robot_commander._robot_name)

    # if launched sr_ur_arm_box.launch
    def test_named_target_in_srdf_exist(self):
        const_test_names = ['ra_home', 'ra_up']
        for name in const_test_names:
            if self.robot_commander.named_target_in_srdf(name) is False:
                self.fail()

    def test_named_target_in_srdf_not_exist(self):
        const_test_names = ['non_existing_target_test_name']
        for name in const_test_names:
            if self.robot_commander.named_target_in_srdf(name) is True:
                self.fail()

    def test_set_named_target_correct_name(self):
        test_names = self.robot_commander.get_named_targets()
        result = self.robot_commander.set_named_target(test_names[0])
        self.assertTrue(result)

    def test_set_named_target_false_name(self):
        result = self.robot_commander.set_named_target("test_false_name")
        self.assertFalse(result)

    def test_get_named_target_joint_values_srdf(self):
        test_names = self.robot_commander.get_srdf_names()
        if len(test_names) > 0:
            output = self.robot_commander.get_named_target_joint_values(test_names[0])
            self.assertIsInstance(output, dict)

    def test_get_named_target_joint_values_warehouse(self):
        test_names = self.robot_commander.get_warehouse_names()
        if len(test_names) > 0:
            output = self.robot_commander.get_named_target_joint_values(test_names[0])
            self.assertIsInstance(output, dict)

    def test_get_named_target_joint_values_no_target(self):
        const_test_names = ["no_target"]
        output = self.robot_commander.get_named_target_joint_values(const_test_names[0])
        self.assertIsNone(output)

    def test_get_end_effector_link(self):
        self.assertIsInstance(self.robot_commander.get_end_effector_link(), str)

    def test_get_current_pose_frame(self):
        pose = self.robot_commander.get_current_pose(reference_frame="world")
        self.assertIsInstance(pose, Pose)

    def test_get_current_pose_frame_none(self):
        pose = self.robot_commander.get_current_pose(reference_frame=None)
        self.assertIsInstance(pose, Pose)

    def test_get_current_pose_frame_wrong(self):
        pose = self.robot_commander.get_current_pose(reference_frame="test_wrong_frame")
        self.assertIsNone(pose)

    def test_get_current_state(self):
        self.assertIsInstance(self.robot_commander.get_current_state(), dict)

    def test_get_current_state_bounded(self):
        self.assertIsInstance(self.robot_commander.get_current_state_bounded(), dict)

    def test_get_robot_state_bounded(self):
        self.assertIsInstance(self.robot_commander.get_current_state_bounded(), dict)

    def test_plan_to_named_target_custom_start_state_none(self):
        target_names = self.robot_commander.get_named_targets()
        if len(target_names) > 0:
            self.robot_commander.plan_to_named_target(target_names[0], None)
        self.assertIsInstance(self.robot_commander.get_plan(), RobotTrajectory)

    def test_plan_to_named_target_custom_start_state_exists(self):
        target_names = self.robot_commander.get_named_targets()
        if len(target_names) > 0:
            robot_state = RobotState()
            for key, value in CONST_RA_HOME_ANGLES .items():
                robot_state.joint_state.name.append(key)
                robot_state.joint_state.position.append(value)
            self.robot_commander.plan_to_named_target(target_names[0], robot_state)
        self.assertIsInstance(self.robot_commander.get_plan(), RobotTrajectory)

    def test_plan_to_named_target_target_not_exists(self):
        const_test_name = "test_non_existing_target"
        self.assertFalse(self.robot_commander.plan_to_named_target(const_test_name, None))

    def test_get_named_targets(self):
        self.assertIsInstance(self.robot_commander.get_named_targets(), list)

    def test_get_joints_position(self):
        ret_val = self.robot_commander.get_joints_position()
        self.assertIsInstance(ret_val, dict)

    def test_get_joints_velocity(self):
        ret_val = self.robot_commander.get_joints_velocity()
        self.assertIsInstance(ret_val, dict)

    def test_get_joints_state(self):
        ret_val = self.robot_commander.get_joints_state()
        self.assertIsInstance(ret_val, JointState)

    def test_run_joint_trajectory_unsafe_executed(self):
        self.reset_to_home()
        initial_joint_state = self.robot_commander.get_current_state()
        desired_joint_state = {}
        for key in initial_joint_state:
            # goal traj delta needs to be smaller than initial_start_state_threshold
            # or it will fail since it's a traj state
            desired_joint_state[key] = initial_joint_state[key] + 0.01

        # create joint trajectory message
        desired_joint_trajectory = JointTrajectory()
        desired_joint_trajectory.header.stamp = rospy.Time.now()
        desired_joint_trajectory.joint_names = list(desired_joint_state.keys())
        point = JointTrajectoryPoint()
        point.positions = list(desired_joint_state.values())
        point.time_from_start = rospy.Duration(0.5)
        desired_joint_trajectory.points.append(point)

        self.robot_commander.run_joint_trajectory_unsafe(desired_joint_trajectory)
        executed_joints_list = list(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(list(desired_joint_state.values()), executed_joints_list,
                                   TOLERANCE_UNSAFE, TOLERANCE_UNSAFE)

    def test_run_joint_trajectory_executed(self):
        self.robot_commander.set_start_state_to_current_state()
        initial_joint_state = self.robot_commander.get_current_state()
        desired_joint_state = {}
        for key in initial_joint_state:
            # goal traj delta needs to be smaller than initial_start_state_threshold
            # or it will fail since it's a traj state
            desired_joint_state[key] = initial_joint_state[key] + 0.01

        # create joint trajectory message
        desired_joint_trajectory = JointTrajectory()
        desired_joint_trajectory.header.stamp = rospy.Time.now()
        desired_joint_trajectory.joint_names = list(desired_joint_state.keys())
        point = JointTrajectoryPoint()
        point.positions = list(desired_joint_state.values())
        point.time_from_start = rospy.Time.now()
        desired_joint_trajectory.points.append(point)
        self.robot_commander.set_start_state_to_current_state()
        self.assertTrue(self.robot_commander.run_joint_trajectory(desired_joint_trajectory))

    def test_make_named_trajectory(self):
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 2.00},
                           "interpolate_time": 0.4, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 1.95},
                           "interpolate_time": 0.6, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.10, 'ra_elbow_joint': 1.90},
                           "interpolate_time": 0.7, "pause_time": 0.1, "degrees": False})
        named_trajectory = self.robot_commander.make_named_trajectory(trajectory)
        all_positions = []

        for i in range(len(named_trajectory.points)):
            all_positions.append(named_trajectory.points[i].positions)
        for waypoint in trajectory:
            for joint in waypoint["joint_angles"].keys():
                if not any(waypoint["joint_angles"][joint] in sublist for sublist in all_positions):
                    self.fail()

        self.assertIsInstance(self.robot_commander.make_named_trajectory(trajectory), JointTrajectory)

    def test_send_stop_trajectory_unsafe(self):
        start_joints = sorted(list(self.robot_commander.get_current_state().values()))
        self.robot_commander.send_stop_trajectory_unsafe()
        end_joints = sorted(list(self.robot_commander.get_current_state().values()))
        np.testing.assert_allclose(start_joints, end_joints, 0.001, 0.001)

    def test_run_named_trajectory_unsafe_executed(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                                            'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                            'ra_wrist_2_joint': 1.3, 'ra_wrist_3_joint': 3.0},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        self.robot_commander.run_named_trajectory_unsafe(trajectory, wait=True)
        desired_joint_state = sorted(list(trajectory[0]['joint_angles'].values()))
        executed_joints_list = sorted(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(desired_joint_state, executed_joints_list, TOLERANCE_UNSAFE, TOLERANCE_UNSAFE)

    def test_run_named_trajectory(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                                            'ra_shoulder_lift_joint': -1.25, 'ra_wrist_1_joint': -0.733,
                                            'ra_wrist_2_joint': 1.3, 'ra_wrist_3_joint': 3.0},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        self.robot_commander.run_named_trajectory(trajectory)
        desired_joint_state = sorted(list(trajectory[0]['joint_angles'].values()))
        executed_joints_list = sorted(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(desired_joint_state, executed_joints_list, 0.01, 0.01)

    def test_plan_to_pose_target(self):
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
        executed_joints_list = list(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(list(desired_joint_state.values()), executed_joints_list,
                                   TOLERANCE_UNSAFE, TOLERANCE_UNSAFE)

    def test_plan_to_waypoints_target(self):
        waypoints = []
        waypoints.append(conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.71, 0.15, 0.34, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.69, 0.15, 0.34, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1]))
        (plan, _) = self.robot_commander.plan_to_waypoints_target(waypoints)
        self.assertIsInstance(plan, RobotTrajectory)

    def test_get_ik(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.71, 0.17, 0.34, 0, 0, 0, 1])
        joint_state_from_ik = self.robot_commander.get_ik(pose)
        self.assertIsInstance(joint_state_from_ik, dict)

    def test_move_to_joint_value_target_executed(self):
        self.reset_to_home()
        initial_joint_state = self.robot_commander.get_current_state()
        desired_joint_state = {}
        for key in initial_joint_state:
            desired_joint_state[key] = initial_joint_state[key] + 0.05
        self.robot_commander.move_to_joint_value_target(desired_joint_state, wait=True,
                                                        angle_degrees=False)
        executed_joints_list = list(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(list(desired_joint_state.values()), executed_joints_list, 0.01, 0.01)

    def test_move_to_pose_target(self):
        self.reset_to_home()
        desired_pose_rpy = self.create_test_pose_rpy_from_current_pose()
        is_pose_reached = False
        tries = 0
        desired_pose_msg = self.get_pose_msg_from_pose_rpy(desired_pose_rpy)
        while not is_pose_reached and tries < PLANNING_ATTEMPTS:
            tries += 1
            self.robot_commander.move_to_pose_target(desired_pose_rpy, self.eef, wait=True)
            current_pose = self.robot_commander.get_current_pose()
            is_pose_reached = self.compare_poses(current_pose, desired_pose_msg.pose)
        self.assertTrue(is_pose_reached)

    def test_move_to_pose_target_unsafe(self):
        self.reset_to_home()
        desired_pose_rpy = self.create_test_pose_rpy_from_current_pose()
        is_pose_reached = False
        tries = 0
        desired_pose_msg = self.get_pose_msg_from_pose_rpy(desired_pose_rpy)
        while not is_pose_reached and tries < PLANNING_ATTEMPTS:
            tries += 1
            self.robot_commander.move_to_pose_value_target_unsafe(desired_pose_msg, wait=True)
            current_pose = self.robot_commander.get_current_pose()
            is_pose_reached = self.compare_poses(current_pose, desired_pose_msg.pose)
        self.assertTrue(is_pose_reached)

    def test_move_to_position_target(self):
        self.reset_to_home()
        desired_pose_rpy = self.create_test_pose_rpy_from_current_pose()
        desired_pose_msg = self.get_pose_msg_from_pose_rpy(desired_pose_rpy)
        xyz = [desired_pose_msg.pose.position.x, desired_pose_msg.pose.position.y, desired_pose_msg.pose.position.z]
        is_position_reached = False
        tries = 0
        while not is_position_reached and tries < PLANNING_ATTEMPTS:
            tries += 1
            rospy.loginfo(f"test_move_to_position_target {tries}")
            self.robot_commander.move_to_position_target(xyz, self.eef, wait=True)
            current_pose = self.robot_commander.get_current_pose()
            is_position_reached = self.compare_poses(current_pose, desired_pose_msg.pose)
        self.assertTrue(is_position_reached)

    def test_plan_to_position_target(self):
        xyz = [0.71, 0.17, 0.34]
        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.get_rostime()
        start_pose.pose = self.robot_commander.get_current_pose()
        start_pose.pose.position.x = xyz[0]
        start_pose.pose.position.y = xyz[1]
        start_pose.pose.position.z = xyz[2]
        plan = self.robot_commander.plan_to_position_target(xyz, self.eef)
        self.assertIsInstance(plan, RobotTrajectory)

    def test_move_to_named_target(self):
        self.robot_commander.set_start_state_to_current_state()
        named_target = "ra_start"
        desired_joint_state = sorted(self.robot_commander.get_named_target_joint_values(named_target).values())
        self.robot_commander.move_to_named_target(named_target, wait=True)
        executed_joints_list = sorted(self.robot_commander.get_current_state().values())
        np.testing.assert_allclose(desired_joint_state, executed_joints_list, 0.01, 0.01)

    def test_action_is_running(self):
        self.reset_to_home()
        self.robot_commander.move_to_joint_value_target_unsafe(CONST_EXAMPLE_TARGET, time=1, wait=False,
                                                               angle_degrees=False)
        self.assertTrue(self.robot_commander.action_is_running())

    # no working teach mode so far
    # def test_set_teach_mode(self):


if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)
