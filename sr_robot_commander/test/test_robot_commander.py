#!/usr/bin/env python3

# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import

import collections
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
RA_HOME_ANGLES = {'ra_shoulder_pan_joint': 0.00, 'ra_elbow_joint': 2.00, # 0 , 114
                    'ra_shoulder_lift_joint': -1.57, 'ra_wrist_1_joint': -0.73,  # 
                    'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 0.00}

RA_EXAMPLE_TARGET = {'ra_shoulder_pan_joint': 0.3, 'ra_elbow_joint': 1.70, # 0 , 114
                     'ra_shoulder_lift_joint': -1.10, 'ra_wrist_1_joint': -0.32,  # 
                     'ra_wrist_2_joint': 1.57, 'ra_wrist_3_joint': 0.00}


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot_commander = SrRobotCommander("right_arm")
        cls.robot_commander.set_planner_id("RRTstarkConfigDefault")
        cls.robot_commander.set_planning_time(3)
        
    def reset_to_home(self):
        rospy.logerr("RESETTING")          
        self.robot_commander._reset_plan()
        plan = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False, 
                                                               custom_start_state=None)
        self.robot_commander.execute_plan(plan)
        time.sleep(2)

    def compare_poses(self, pose1, pose2, tolerance = 0.0):
        digit = 3
        p1_list = [pose1.position.x, pose1.position.y, pose1.position.z,
                   pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        p2_list = [pose2.position.x, pose2.position.y, pose2.position.z,
                   pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]

        p1_list = [round(i, digit) for i in p1_list]
        p2_list = [round(i, digit) for i in p2_list]

        for idx, vals in enumerate(zip(p1_list, p2_list)):
            if vals[0] == -0.0:
                p1_list[idx] = 0
            if vals[1] == -0.0:
                p2_list[idx] = 0

            if p1_list[idx] != p2_list[idx]:
                return False
        #rospy.logwarn(p1_list)
        #rospy.logwarn(p2_list)
        return True

    def compare_joint_states(self, js1, js2, tolerance = 0.01):

        if set(js1.keys()) != set(js2.keys()):
            return False 
        else:                
            digit = 2
            for key in js1.keys():
                js1[key] = round(js1[key], digit)
                js2[key] = round(js2[key], digit)
                if abs(js1[key] - js2[key]) > tolerance:
                    return False

        rospy.logwarn(js1.items())
        rospy.logwarn(js2.items())
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

    # def get_end_effector_pose_from_named_state(self, name):

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
        plan  = self.robot_commander.plan_to_joint_value_target(RA_EXAMPLE_TARGET, angle_degrees=False,
                                                                custom_start_state=None)   
        last_point = list(plan.joint_trajectory.points[-1].positions)

        for i in range(0,len(last_point)):
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
        rospy.logwarn("test-test_execute")
        rospy.logwarn(diffs)
        self.assertTrue(condition)

    def test_execute_plan(self):
        self.reset_to_home()
        plan = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False, 
                                                               custom_start_state=None)
        self.robot_commander.execute_plan(plan)
        executed_joints = self.robot_commander.get_current_state()
        diffs = [RA_HOME_ANGLES[key] - executed_joints[key] for key in RA_HOME_ANGLES
                 if key in executed_joints]
        condition = all(diff < 0.01 for diff in diffs)
        self.assertTrue(condition)
 
    def test_move_to_joint_value_target(self):
        self.reset_to_home()
        self.robot_commander.move_to_joint_value_target(RA_EXAMPLE_TARGET, wait=True, angle_degrees=False)
        end_state = self.robot_commander.get_current_state()
        rospy.logwarn("test_move_to_joint_value_target")
        rospy.logwarn(RA_EXAMPLE_TARGET)
        rospy.logwarn(end_state)
        condition = self.compare_joint_states(end_state, RA_EXAMPLE_TARGET)
        rospy.logwarn(condition)
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
        end_joints['ra_shoulder_lift_joint'] += 0.6
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

    # to test with sr_ur_arm_box.launch
    #def test_named_target_in_srdf__exist(self):
    #    # if launched sr_ur_arm_box.launch
    #    test_names = ['lifted', 'flat']
    #    # if launch sr_ur_arm_hand.launch
    #    failed = False
    #    for name in test_names:
    #        if self.robot_commander.named_target_in_srdf(name) is False:
    #            failed = True
    #    self.assertFalse(failed)

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

    #def move_to_named_target(self, name, wait=True):
        
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
    
        #self.reset_to_home()
        #plan_before = copy.deepcopy(self.robot_commander._SrRobotCommander__plan)
        #rospy.logwarn("PLAN BEFORE")
        #rospy.logwarn(plan_before)
        
        #plan_after = copy.deepcopy(self.robot_commander._SrRobotCommander__plan)

        #points_count = len(plan_before.joint_trajectory.points)
        #i = 0
        #for i in range(0, len(plan_before.joint_trajectory.points)):
        #    for j in range(0, len(plan_before.joint_trajectory.points[0].positions)):
        #        pb = plan_before.joint_trajectory.points[i].positions[j]
        #        pa = plan_after.joint_trajectory.points[i].positions[j]
        #        if abs(pb - pa) > 0.1:
        #            break
        #plans_no_changed = points_count == (i + 1)
        #self.assertTrue(plans_no_changed)
    
        condition = self.robot_commander._SrRobotCommander__plan == None
        self.assertTrue(condition)

    def get_named_targets(self):
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

    # whats the point of this?
    def test_run_joint_trajectory(self):
        self.reset_to_home()
        trajectory = self.robot_commander.plan_to_joint_value_target(RA_HOME_ANGLES, angle_degrees=False,
                                                                     custom_start_state=None).joint_trajectory
        self.robot_commander.run_joint_trajectory(trajectory)
        end_state = copy.deepcopy(self.robot_commander.get_current_state())
        diffs = [RA_HOME_ANGLES[key] - end_state[key] for key in RA_HOME_ANGLES if key in end_state]
        condition = all(abs(diff) < 0.1 for diff in diffs)
        self.assertTrue(condition)

    # this isnt used and is
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
        # this should check if all waypoints are in the trajectory
        for wp in trajectory:
            for key in wp["joint_angles"].keys():
                if not any(wp["joint_angles"][key] in sublist for sublist in all_positions):
                    self.fail()
        self.assertTrue(type(self.robot_commander.make_named_trajectory(trajectory)) == JointTrajectory)

    '''
    def test_send_stop_trajectory_unsafe(self):
        self.reset_to_home()

        current_joints = copy.deepcopy(self.robot_commander.get_joints_position())
        end_joints = copy.deepcopy(self.robot_commander.get_joints_position())

        js = JointState()
        js.header = Header()
        js.name = list(current_joints.keys())
        js.position = list(current_joints.values())
        rs = RobotState()
        rs.joint_state = js

        end_joints['ra_shoulder_pan_joint'] += 0.14
        end_joints['ra_elbow_joint'] += -0.1

        plan = self.robot_commander.plan_to_joint_value_target(end_joints,
                                                               angle_degrees=False, custom_start_state=rs)
        self.robot_commander.execute_plan(plan)
        rospy.sleep(1)
        self.robot_commander.send_stop_trajectory_unsafe()
        current_joints = self.robot_commander.get_current_state()

        diffs = [end_joints[key] - current_joints[key] for key in end_joints
                 if key in current_joints]

        rospy.logwarn("DIFFS ")
        rospy.logwarn(diffs)
        rospy.logwarn(list(abs(diff) < 0.01 for diff in diffs))

        self.assertTrue(any(abs(diff) < 0.01 for diff in diffs))

    def test_run_named_trajectory_unsafe(self):
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
              
        self.robot_commander.run_named_trajectory_unsafe(trajectory, wait=False)
    '''
    #def run_named_trajectory(self, trajectory):

    #def move_to_position_target(self, xyz, end_effector_link="", wait=True):

    #def plan_to_position_target(self, xyz, end_effector_link="", custom_start_state=None):
    
    def test_move_to_pose_target(self):
        self.reset_to_home()

        eef = self.robot_commander.get_end_effector_link()
        curr_pose = self.robot_commander.get_current_pose()        

        pose = conversions.list_to_pose([0.5, 0.10, 0.40, 0.00, 0.00, 0.00, 1.00])

        '''
        pose = Pose()
        pose.position.x = 0.50
        pose.position.y = 0.10
        pose.position.z = 0.40
        pose.orientation.x = 0.00
        pose.orientation.y = 0.00
        pose.orientation.z = 0.00
        pose.orientation.w = 1.00
        '''
        self.robot_commander.move_to_pose_target(pose, eef)
        time.sleep(2)
        after_pose = self.robot_commander.get_current_pose()
        #rospy.logwarn("XXXtest_move_to_pose_target")
        #rospy.logwarn(pose)
        #rospy.logwarn(after_pose)
        condition = self.compare_poses(pose, after_pose)

        self.assertTrue(condition)
       
    def test_plan_to_pose_target(self):
        self.reset_to_home()

        eef = self.robot_commander.get_end_effector_link()

        pose = conversions.list_to_pose([0.5, -0.10, 0.40, 0.00, 0.00, 0.00, 1.00])
        '''
        pose = Pose()
        pose.position.x = 0.50
        pose.position.y = -0.10
        pose.position.z = 0.40
        pose.orientation.x = 0.00
        pose.orientation.y = 0.00
        pose.orientation.z = 0.00
        pose.orientation.w = 1.00
        '''
        
        plan_pose = self.robot_commander.plan_to_pose_target(pose, end_effector_link=eef, alternative_method=False, 
                                                             custom_start_state=None)
        plan_joint = self.robot_commander.plan_to_pose_target(pose, end_effector_link=eef, alternative_method=True, 
                                                              custom_start_state=None)

        
        #rospy.logwarn("PLAN TO POSE TARGET")
        #rospy.logwarn(plan_pose)
        #rospy.logwarn(plan_joint)
        #rospy.logwarn("JOINT TARGET")
        #rospy.logwarn(self.robot_commander._move_group_commander.get_joint_value_target())
        '''
        #self.robot_commander.execute_plan(plan)

        end_joints = self.robot_commander.get_current_state()
        rs = RobotState()
        for key, value in end_joints.items():
            rs.joint_state.name.append(key)
            rs.joint_state.position.append(value)

        after_pose = self.robot_commander.get_current_pose()     
        rospy.logwarn("xxx-test_plan_to_pose_target")     
        rospy.logwarn(pose)     
        rospy.logwarn(after_pose)

        self.assertTrue(self.compare_poses(pose, after_pose))
        '''
    
    def test_move_to_joint_value_target_unsafe__executed(self):
        self.reset_to_home()

        self.robot_commander.move_to_joint_value_target_unsafe(RA_EXAMPLE_TARGET, time=0.002, wait=True, angle_degrees=False)
        time.sleep(3)
        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        rospy.logwarn("UNSAFE_EXECUTED")
        rospy.logwarn(RA_EXAMPLE_TARGET)
        rospy.logwarn(executed_joints)
        rospy.logerr(condition) 
        self.assertTrue(condition)

    def test_move_to_joint_value_target_unsafe__cancelled(self):
        self.reset_to_home()

        self.robot_commander.move_to_joint_value_target_unsafe(RA_EXAMPLE_TARGET, time=0.002, wait=False, angle_degrees=False)
        for client in self.robot_commander._clients:
            self.robot_commander._action_running[client] = True
            self.robot_commander._clients[client].cancel_goal()
        executed_joints = self.robot_commander.get_current_state()
        condition = self.compare_joint_states(RA_EXAMPLE_TARGET, executed_joints)
        rospy.logwarn("UNSAFE_CANCELLED")
        rospy.logwarn(RA_EXAMPLE_TARGET)
        rospy.logwarn(executed_joints)
        rospy.logwarn(condition)

        #rospy.logerr(condition) 
        self.assertFalse(condition)


        
  
        #rospy.Subscriber("/ra_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.mycb)
        


    '''
    def action_is_running(self, controller=None):

    '''

    def test_run_joint_trajectory_unsafe____executed(self):
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

    
    def test_zplan_to_waypoints_target(self):
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

    ''' # no teach mode
    def test_set_teach_mode(self):
        self.robot_commander.set_teach_mode(False)

    '''

    def test_zzzget_ik(self):
        self.reset_to_home()

        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.2
        pose.pose.position.z = 0.3
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        joint_state_from_pose = self.robot_commander.get_ik(pose)
        
        eef = self.robot_commander.get_end_effector_link()
        self.robot_commander.move_to_joint_value_target(joint_state_from_pose)
        #time.sleep(2)
        after_js = self.robot_commander.get_current_state()

        rospy.logwarn("GET IK")
        rospy.logwarn(joint_state_from_pose)
        rospy.logwarn(after_js)


        #curr_pose = self.robot_commander.get_current_pose()
        


    '''
    def move_to_pose_value_target_unsafe(self, target_pose, avoid_collisions=False, time=0.002, wait=True,
                                         ik_constraints=None):


    '''


if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)
