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
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionFK
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_commander.exception import MoveItCommanderException
from moveit_commander import conversions
import tf2_ros
import time

PKG = "sr_robot_commander"
RA_HOME_ANGLES = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00,
                  'ra_shoulder_lift_joint': -1.57, 'ra_wrist_1_joint': -0.73,
                  'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 0.00}

RA_EXAMPLE_TARGET = {'ra_shoulder_pan_joint': 0.2, 'ra_elbow_joint': 1.80,
                     'ra_shoulder_lift_joint': -1.37, 'ra_wrist_1_joint': -0.52,
                     'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 0.00}


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot_commander = SrRobotCommander("right_arm")
        cls.robot_commander.set_planner_id("RRTstarkConfigDefault")
        cls.robot_commander.set_planning_time(3)
        cls.eef = cls.robot_commander.get_end_effector_link()

    def reset_to_home(self):
        self.robot_commander._reset_plan()
        plan = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False,
                                                               custom_start_state=None)
        self.robot_commander.execute_plan(plan)

    '''
    def compare_poses(self, pose1, pose2, tolerance=0.01):
        p1_list = [pose1.position.x, pose1.position.y, pose1.position.z,
                   pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        p2_list = [pose2.position.x, pose2.position.y, pose2.position.z,
                   pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        digit = 2
        p1_list = [round(i, digit) for i in p1_list]
        p2_list = [round(i, digit) for i in p2_list]

        for idx, vals in enumerate(zip(p1_list, p2_list)):
            if vals[0] == -0.0:
                p1_list[idx] = 0
            if vals[1] == -0.0:
                p2_list[idx] = 0
            #rospy.logwarn("{} {}".format(p1_list[idx], p2_list[idx]))
            if abs(p1_list[idx]) - abs(p2_list[idx]) > tolerance:
                return False
        return True

    def compare_joint_states(self, js1, js2, tolerance=0.01):
        common_keys = set(js1.keys()).intersection(set(js2.keys()))
        if len(common_keys) == 0:
            return False
        else:
            digit = 2
            for key in common_keys:
                js1[key] = round(js1[key], digit)
                js2[key] = round(js2[key], digit)
                if abs(js1[key]) - abs(js2[key]) > tolerance:
                    return False
        return True

    def test_get_and_set_planner_id(self):
        planner_id = "RRTstarkConfigDefault"
        self.robot_commander.set_planner_id(planner_id)
        self.assertEqual(planner_id, self.robot_commander._move_group_commander.get_planner_id())

    def test_get_and_set_planning_time(self):
        time_test_value = 3
        self.robot_commander.set_planning_time(time_test_value)
        self.assertEqual(time_test_value, self.robot_commander._move_group_commander.get_planning_time())

    def test_set_num_planning_attempts(self):
        raised = False
        try:
            self.robot_commander.set_num_planning_attempts(3)
        except (NameError, TypeError):
            raised = True
        self.assertFalse(raised)

    def test_get_end_effector_pose_from_state(self):
        rs = RobotState()

        for key, value in RA_HOME_ANGLES.items():
            rs.joint_state.name.append(key)
            rs.joint_state.position.append(value)

        pose = self.robot_commander.get_end_effector_pose_from_state(rs)
        self.assertIsInstance(pose, PoseStamped)

    def test_get_planning_frame(self):
        raised = False
        try:
            self.robot_commander.get_planning_frame()
        except (NameError, TypeError):
            raised = True
        self.assertFalse(raised)

    def test_set_and_get_pose_reference_frame(self):
        reference_frame = "world"
        self.robot_commander._move_group_commander.set_pose_reference_frame(reference_frame)
        self.assertEqual(self.robot_commander._move_group_commander.get_pose_reference_frame(), reference_frame)

    def test_get_group_name(self):
        self.assertEqual(self.robot_commander._name, self.robot_commander.get_group_name())

    def test_refresh_named_targets(self):
        self.robot_commander.refresh_named_targets()
        condition_1 = type(self.robot_commander._srdf_names) == list
        condition_2 = type(self.robot_commander._warehouse_names) == list
        self.assertTrue(condition_1 and condition_2)

    def test_set_max_velocity_scaling_factor__range_ok(self):
        self.robot_commander.set_max_velocity_scaling_factor(0.2)

    def test_set_max_velocity_scaling_factor__range_not_ok(self):
        self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_velocity_scaling_factor, 3)

    def test_set_max_acceleration_scaling_factor__range_ok(self):
        self.robot_commander.set_max_acceleration_scaling_factor(0.2)

    def test_set_max_acceleration_scaling_factor__range_not_ok(self):
        self.assertRaises(MoveItCommanderException, self.robot_commander.set_max_acceleration_scaling_factor, 3)

    def test_allow_looking(self):
        self.robot_commander.allow_looking(True)

    def test_allow_replanning(self):
        self.robot_commander.allow_replanning(True)

    def test_plan_to_joint_value_target(self):
        self.reset_to_home()
        plan = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                               custom_start_state=None)
        last_point = list(plan.joint_trajectory.points[-1].positions)

        for i in range(0, len(last_point)):
            last_point[i] = round(last_point[i], 2)
            if last_point[i] == -0.00:
                last_point[i] = 0.00
        last_point.sort()

        arm_home_joints_goal_list = list(RA_EXAMPLE_TARGET.values())
        arm_home_joints_goal_list.sort()
        condition_1 = type(plan) == RobotTrajectory
        condition_2 = last_point == arm_home_joints_goal_list
        self.assertTrue(condition_1 and condition_2)

    def test_execute(self):
        self.reset_to_home()
        self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False, custom_start_state=None)
        self.robot_commander.execute()
        executed_joints = self.robot_commander.get_current_state()
        diffs = [RA_EXAMPLE_TARGET[key] - executed_joints[key] for key in RA_EXAMPLE_TARGET
                 if key in executed_joints]
        condition = all(diff < 0.01 for diff in diffs)
        self.assertTrue(condition)

    def test_execute_plan(self):
        self.reset_to_home()
        plan = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False,
                                                               custom_start_state=None)
        self.robot_commander.execute_plan(plan)
        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(executed_joints, RA_HOME_ANGLES)
        self.assertTrue(condition)

    def test_move_to_joint_value_target(self):
        self.reset_to_home()
        self.robot_commander.move_to_joint_value_target(RA_EXAMPLE_TARGET, wait=True, angle_degrees=False)
        end_state = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(end_state, RA_EXAMPLE_TARGET)
        self.assertTrue(condition)

    def test_check_plan_is_valid__ok(self):
        self.reset_to_home()
        self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False, custom_start_state=None)
        self.assertTrue(self.robot_commander.check_plan_is_valid())

    def test_check_plan_is_valid__not_ok(self):
        self.reset_to_home()
        self.robot_commander._SrRobotCommander__plan = None
        self.assertFalse(self.robot_commander.check_plan_is_valid())

    def test_check_given_plan_is_valid__ok(self):
        self.reset_to_home()
        plan = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertTrue(self.robot_commander.check_given_plan_is_valid(plan))

    def test_check_given_plan_is_valid__not_ok(self):
        self.reset_to_home()
        not_valid_goal = copy.deepcopy(RA_HOME_ANGLES)
        out_of_range_value = 3.0
        not_valid_goal['ra_elbow_joint'] = out_of_range_value
        plan = self.robot_commander.plan_to_joint_value_target(not_valid_goal, angle_degrees=False,
                                                               custom_start_state=None)
        self.assertFalse(self.robot_commander.check_given_plan_is_valid(plan))

    def test_evaluate_given_plan__none(self):
        self.reset_to_home()
        plan = None
        evaluation = self.robot_commander.evaluate_given_plan(plan)
        self.assertTrue(evaluation is None)

    def test_evaluate_given_plan__low_quality(self):
        self.reset_to_home()
        end_joints = copy.deepcopy(RA_HOME_ANGLES)
        end_joints['ra_shoulder_pan_joint'] += 0.8
        end_joints['ra_shoulder_lift_joint'] += 0.4
        end_joints['ra_elbow_joint'] += 0.6
        end_joints['ra_wrist_1_joint'] += 0.4
        plan = self.robot_commander.plan_to_joint_value_target(end_joints, angle_degrees=False,
                                                               custom_start_state=None)
        evaluation = self.robot_commander.evaluate_given_plan(plan)
        self.assertGreater(evaluation, 10)

    def test_evaluate_given_plan__high_quality(self):
        self.reset_to_home()
        end_joints = copy.deepcopy(RA_HOME_ANGLES)
        end_joints['ra_shoulder_pan_joint'] += 0.1
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

    def test_get_robot_name(self):
        self.assertTrue(self.robot_commander.get_robot_name() == self.robot_commander._robot_name)

    # if launched sr_ur_arm_box.launch
    def test_named_target_in_srdf__exist(self):
        test_names = ['lifted', 'flat']
        failed = False
        for name in test_names:
            if self.robot_commander.named_target_in_srdf(name) is False:
                failed = True
        self.assertFalse(failed)

    def test_named_target_in_srdf__not_exist(self):
        test_names = ['non_existing_target_test_name']
        failed = False
        for name in test_names:
            if self.robot_commander.named_target_in_srdf(name) is False:
                failed = True
        self.assertTrue(failed)

    def test_set_named_target__correct_name(self):
        test_names = self.robot_commander.get_named_targets()
        result = self.robot_commander.set_named_target(test_names[0])
        self.assertTrue(result)

    def test_set_named_target__false_name(self):
        result = self.robot_commander.set_named_target("test_false_name")
        self.assertFalse(result)

    def test_get_named_target_joint__values_srdf(self):
        test_names = self.robot_commander._srdf_names
        if len(test_names) > 0:
            output = self.robot_commander.get_named_target_joint_values(test_names[0])
            self.assertTrue(type(output) == dict)

    def test_get_named_target_joint__values_warehouse(self):
        test_names = self.robot_commander._warehouse_names
        if len(test_names) > 0:
            output = self.robot_commander.get_named_target_joint_values(test_names[0])
            self.assertTrue(type(output) == dict)

    def test_get_named_target_joint__values_no_target(self):
        test_names = "no_target"
        output = self.robot_commander.get_named_target_joint_values(test_names[0])
        self.assertTrue(output is None)

    def test_get_end_effector_link(self):
        self.assertTrue(type(self.robot_commander.get_end_effector_link()) == str)

    def test_get_current_pose_frame(self):
        pose = self.robot_commander.get_current_pose(reference_frame="world")
        self.assertTrue(type(pose) == Pose)

    def test_get_current_pose_frame__none(self):
        pose = self.robot_commander.get_current_pose(reference_frame=None)
        self.assertTrue(type(pose) == Pose)

    def test_get_current_pose_frame__wrong(self):
        pose = self.robot_commander.get_current_pose(reference_frame="test_wrong_frame")
        self.assertTrue(pose is None)

    def test_get_current_state(self):
        self.assertTrue(type(self.robot_commander.get_current_state()) == dict)

    def test_get_current_state_bounded(self):
        self.assertTrue(type(self.robot_commander.get_current_state_bounded()) == dict)

    def test_get_robot_state_bounded(self):
        self.assertTrue(type(self.robot_commander.get_current_state_bounded()) == dict)

    def test_plan_to_named_target__custom_start_state_none(self):
        self.reset_to_home()
        target_names = self.robot_commander.get_named_targets()
        if len(target_names) > 0:
            self.robot_commander.plan_to_named_target(target_names[0], None)
        self.assertTrue(type(self.robot_commander._SrRobotCommander__plan) == RobotTrajectory)

    def test_plan_to_named_target__custom_start_state_exists(self):
        self.reset_to_home()
        target_names = self.robot_commander.get_named_targets()

        if len(target_names) > 1:
            rs = RobotState()
            for key, value in RA_HOME_ANGLES.items():
                rs.joint_state.name.append(key)
                rs.joint_state.position.append(value)
            plan = self.robot_commander.plan_to_named_target(target_names[0], rs)

        self.assertTrue(type(self.robot_commander._SrRobotCommander__plan) == RobotTrajectory)

    def test_plan_to_named_target__target_not_exists(self):
        self.reset_to_home()
        self.robot_commander.plan_to_named_target("test_non_existing_target", None)
        condition = self.robot_commander._SrRobotCommander__plan is None
        self.assertTrue(condition)

    def test_get_named_targets(self):
        self.assertTrue(type(self.robot_commander.get_named_targets()) == list)

    def test_get_joints_position(self):
        ret_val = self.robot_commander.get_joints_position()
        self.assertTrue(type(ret_val) == dict)

    def test_get_joints_velocity(self):
        ret_val = self.robot_commander.get_joints_velocity()
        self.assertTrue(type(ret_val) == dict)

    def test_get_joints_state(self):
        ret_val = self.robot_commander.get_joints_state()
        self.assertTrue(type(ret_val) == JointState)

    def test_run_joint_trajectory(self):
        self.reset_to_home()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        self.robot_commander.run_joint_trajectory(trajectory)
        end_state = copy.deepcopy(self.robot_commander.get_current_state())
        condition = self.compare_joint_states(RA_HOME_ANGLES, end_state)
        self.assertTrue(condition)

    def test_make_named_trajectory(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 2.00},
                           "interpolate_time": 0.4, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.05, 'ra_elbow_joint': 1.95},
                           "interpolate_time": 0.6, "pause_time": 0.1, "degrees": False})
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.10, 'ra_elbow_joint': 1.90},
                           "interpolate_time": 0.7, "pause_time": 0.1, "degrees": False})
        t = self.robot_commander.make_named_trajectory(trajectory)
        all_positions = []

        for i in range(0, len(t.points)):
            all_positions.append(t.points[i].positions)
        for wp in trajectory:
            for key in wp["joint_angles"].keys():
                if not any(wp["joint_angles"][key] in sublist for sublist in all_positions):
                    self.fail()

        self.assertTrue(type(self.robot_commander.make_named_trajectory(trajectory)) == JointTrajectory)

    def test_send_stop_trajectory_unsafe(self):
        self.reset_to_home()
        start_joints = self.robot_commander.get_joints_position()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        self.robot_commander.run_joint_trajectory(trajectory)
        time.sleep(0.5)
        self.robot_commander.send_stop_trajectory_unsafe()
        end_joints = self.robot_commander.get_joints_position()
        condition = self.compare_joint_states(start_joints, end_joints)
        self.assertFalse(condition)

    def test_run_named_trajectory_unsafe__cancelled(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.50, 'ra_elbow_joint': 2.3},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        self.robot_commander.run_named_trajectory_unsafe(trajectory)

        for client in self.robot_commander._clients:
            self.robot_commander._action_running[client] = True
            self.robot_commander._clients[client].cancel_goal()

        js = self.robot_commander.get_current_state()
        expected_js = trajectory[-1]['joint_angles']
        condition = self.compare_joint_states(expected_js, js)
        self.assertFalse(condition)

    def test_run_named_trajectory_unsafe__executed(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.0},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        self.robot_commander.run_named_trajectory_unsafe(trajectory)
        js = self.robot_commander.get_current_state()
        expected_js = trajectory[-1]['joint_angles']
        condition = self.compare_joint_states(expected_js, js)
        self.assertTrue(condition)

    def test_run_named_trajectory(self):
        self.reset_to_home()
        trajectory = []
        trajectory.append({"joint_angles": {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00},
                           "interpolate_time": 0.5, "pause_time": 0.1, "degrees": False})
        self.robot_commander.run_named_trajectory(trajectory)
        js = self.robot_commander.get_current_state()
        expected_js = trajectory[-1]['joint_angles']
        condition = self.compare_joint_states(expected_js, js)
        self.assertTrue(condition)

    def test_move_to_pose_target(self):
        self.reset_to_home()
        curr_pose = self.robot_commander.get_current_pose()
        pose = conversions.list_to_pose([0.4, 0.2, 0.3, 0, 0, 0, 1])
        self.robot_commander.move_to_pose_target(pose, self.eef)
        after_pose = self.robot_commander.get_current_pose()
        condition = self.compare_poses(pose, after_pose)
        self.assertTrue(condition)

    def test_plan_to_pose_target(self):
        self.reset_to_home()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.4, 0.2, 0.3, 0, 0, 0, 1])
        expected_js = self.robot_commander.get_ik(pose)
        expected_js = dict(zip(expected_js.name, expected_js.position))
        plan_pose = self.robot_commander.plan_to_pose_target(pose.pose, end_effector_link=self.eef,
                                                             alternative_method=False, custom_start_state=None)
        end_js = dict(zip(plan_pose.joint_trajectory.joint_names, plan_pose.joint_trajectory.points[-1].positions))
        self.assertTrue(True)

    def test_move_to_joint_value_target_unsafe__executed(self):
        self.reset_to_home()
        self.robot_commander.move_to_joint_value_target_unsafe(RA_EXAMPLE_TARGET, time=0.002, wait=True,
                                                               angle_degrees=False)
        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        self.assertTrue(condition)

    def test_move_to_joint_value_target_unsafe__cancelled(self):
        self.reset_to_home()
        self.robot_commander.move_to_joint_value_target_unsafe(RA_EXAMPLE_TARGET, time=0.002, wait=False,
                                                               angle_degrees=False)
        for client in self.robot_commander._clients:
            self.robot_commander._action_running[client] = True
            self.robot_commander._clients[client].cancel_goal()

        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        self.assertFalse(condition)

    def test_run_joint_trajectory_unsafe__executed(self):
        self.reset_to_home()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        self.robot_commander.run_joint_trajectory_unsafe(trajectory)
        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        self.assertTrue(condition)

    def test_run_joint_trajectory_unsafe__cancelled(self):
        self.reset_to_home()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        self.robot_commander.run_joint_trajectory_unsafe(trajectory, wait=False)

        for client in self.robot_commander._clients:
            self.robot_commander._action_running[client] = True
            self.robot_commander._clients[client].cancel_goal()

        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        self.assertFalse(condition)

    def test_plan_to_waypoints_target(self):
        self.reset_to_home()
        waypoints = []
        waypoints.append(conversions.list_to_pose([0.4, 0.1, 0.3, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.4, -0.1, 0.3, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.5, -0.1, 0.3, 0, 0, 0, 1]))
        waypoints.append(conversions.list_to_pose([0.5, 0.1, 0.3, 0, 0, 0, 1]))
        (plan, f) = self.robot_commander.plan_to_waypoints_target(waypoints)
        self.robot_commander.run_joint_trajectory_unsafe(plan.joint_trajectory)
        after_pose = self.robot_commander.get_current_pose()
        condition = self.compare_poses(after_pose, waypoints[-1])
        self.assertTrue(condition)

    def test_move_to_trajectory_start__trajecotry_exists(self):
        self.reset_to_home()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        joints_from_trajectory = dict(zip(trajectory.joint_names, trajectory.points[0].positions))
        self.robot_commander.move_to_trajectory_start(trajectory)
        current_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(joints_from_trajectory, current_joints)
        self.assertTrue(condition)

    def test_get_ik(self):
        self.reset_to_home()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.4, 0.2, 0.3, 0, 0, 0, 1])
        js_from_ik = self.robot_commander.get_ik(pose)
        js = dict(zip(js_from_ik.name, js_from_ik.position))
        self.robot_commander.move_to_joint_value_target(js)
        end_js = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(js, end_js)
        self.assertTrue(condition)

    def test_move_to_pose_value_target_unsafe__executed(self):
        self.reset_to_home()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.4, 0.2, 0.3, 0, 0, 0, 1])
        self.robot_commander.move_to_pose_value_target_unsafe(pose)
        after_pose = self.robot_commander.get_current_pose()
        condition = self.compare_poses(pose.pose, after_pose)
        self.assertTrue(condition)

    def test_move_to_pose_value_target_unsafe__cancelled(self):
        self.reset_to_home()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.pose = conversions.list_to_pose([0.4, 0.2, 0.3, 0, 0, 0, 1])
        self.robot_commander.move_to_pose_value_target_unsafe(pose)

        for client in self.robot_commander._clients:
            self.robot_commander._action_running[client] = True
            self.robot_commander._clients[client].cancel_goal()

        after_pose = self.robot_commander.get_current_pose()
        condition = self.compare_poses(pose.pose, after_pose)
        self.assertFalse(condition) 
    
    def test_move_to_position_target(self):
        self.reset_to_home()
        xyz = [0.5, 0.3, 0.4]
        self.robot_commander.move_to_position_target(xyz, self.eef)
        time.sleep(1)
        end_pose = self.robot_commander.get_current_pose()

        target_xyz = Pose()
        target_xyz.position.x = xyz[0]
        target_xyz.position.y = xyz[1]
        target_xyz.position.z = xyz[2]
        target_xyz.orientation = end_pose.orientation

        condition = self.compare_poses(target_xyz, end_pose)
        self.assertTrue(condition)

    def test_plan_to_position_target(self):
        self.reset_to_home()
        xyz = [0.5, 0.3, 0.4]
        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.get_rostime()
        start_pose.pose = self.robot_commander.get_current_pose()
        start_pose.pose.position.x = xyz[0]
        start_pose.pose.position.y = xyz[1]
        start_pose.pose.position.z = xyz[2]

        plan = self.robot_commander.plan_to_position_target(xyz, self.eef)
        plan = self.robot_commander._SrRobotCommander__plan

        last_planned_js = dict(zip(plan.joint_trajectory.joint_names, plan.joint_trajectory.points[-1].positions))
        expected_js = self.robot_commander.get_ik(start_pose)
        expected_js = dict(zip(expected_js.name, expected_js.position))

        condition = self.compare_joint_states(expected_js, last_planned_js)
        self.assertTrue(condition)
    '''
    
    # this depends on launch file for the test
    def test_get_end_effector_pose_from_named_state(self):
        '''
        rs = RobotState()
        for key, value in RA_HOME_ANGLES.items():
            rs.joint_state.name.append(key)
            rs.joint_state.position.append(value)
        '''
        name = 'lifted'
        self.robot_commander.move_to_named_target(name)
        #js_start = self.robot_commander.get_end_effector_pose_from_named_state(name)
        js_stop = self.robot_commander.get_named_target_joint_values(name)

        #rospy.logwarn(js_start)
        rospy.logwarn(js_stop)
        
    #this won't work?
    #def move_to_named_target(self, name, wait=True):
    
    # TODO
    #def action_is_running(self, controller=None):
    
    '''
    # no working teach mode so far?
    def test_set_teach_mode(self):
    '''


if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)
