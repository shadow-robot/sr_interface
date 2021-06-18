#!/usr/bin/env python3

# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import

import rospy
import rostest
from unittest import TestCase
from sr_robot_commander.sr_robot_commander import SrRobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

PKG = "sr_robot_commander"


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot_commander = SrRobotCommander("right_arm")
        print("start")
    
    def test_mvgr_get_and_set_planner_id(self):
        planner_list = ["BKPIECEkConfigDefault", "ESTkConfigDefault", "KPIECEkConfigDefault", "LBKPIECEkConfigDefault",
                       "PRMkConfigDefault", "RRTkConfigDefault", "SBLkConfigDefault", "PRMstarkConfigDefault",
                       "RRTstarkConfigDefault"]

        for id in planner_list:
            self.robot_commander.set_planner_id(id) # set_planner_id()
            self.assertTrue(id == self.robot_commander._move_group_commander.get_planner_id()) # get_planner_id()

    def test_mvgr_get_and_set_planning_time(self):
        time_test_values = [1,2,3,4,5]
        for time in time_test_values:
            self.robot_commander.set_planning_time(time) # set_planning_time()
            self.assertTrue(time == self.robot_commander._move_group_commander.get_planning_time()) # get_planning_time()

    
    def test_mvgr_set_num_planning_attempts(self):
        raised = False
        try:
            self.robot_commander.set_num_planning_attempts(1)
        except (NameError, TypeError):
            raised = True
        self.assertFalse(raised)


    #def get_end_effector_pose_from_named_state(self, name):

    
    def test_mvgr_get_end_effector_pose_from_state(self):   

        d = {'ra_shoulder_pan_joint': 0.5157461682721474,
                 'ra_elbow_joint': 0.6876824920327893,
                 'ra_wrist_1_joint': -0.7695210732233582,
                 'ra_wrist_2_joint': 0.2298871642157314,
                 'ra_shoulder_lift_joint': -0.9569080092786892,
                 'ra_wrist_3_joint': -0.25991215955733704} 

        #rospy.logwarn(self.robot_commander._move_group_commander.get_end_effector_link())
        #rospy.logwarn(self.robot_commander._move_group_commander.get_pose_reference_frame())

        rs = RobotState()        
        for key, value in d.items():
            rs.joint_state.name.append(key)
            rs.joint_state.position.append(value)

        pose = self.robot_commander.get_end_effector_pose_from_state(rs)

    def test_mvgr_get_planning_frame(self):
        raised = False
        try:
            self.robot_commander.get_planning_frame()
        except (NameError, TypeError):
            raised = True
        self.assertFalse(raised)

    def test_mvgr_set_and_get_pose_reference_frame(self):
        reference_frame = "world"
        self.robot_commander._move_group_commander.set_pose_reference_frame(reference_frame)
        self.assertTrue(self.robot_commander._move_group_commander.get_pose_reference_frame() == reference_frame)

    def test_get_group_name(self):   
        self.assertTrue(self.robot_commander._name == self.robot_commander.get_group_name())

    '''
    def refresh_named_targets(self):

    def set_max_velocity_scaling_factor(self, value):

    def set_max_acceleration_scaling_factor(self, value):

    def allow_looking(self, value):

    def allow_replanning(self, value):

    def execute(self):

    def execute_plan(self, plan):

    def move_to_joint_value_target(self, joint_states, wait=True, angle_degrees=False):

    def plan_to_joint_value_target(self, joint_states, angle_degrees=False, custom_start_state=None):

    def check_plan_is_valid(self):

    def check_given_plan_is_valid(self, plan):

    def evaluate_given_plan(self, plan):

    def evaluate_plan(self):

    def evaluate_plan_quality(self, plan_quality, good_threshold=20, medium_threshold=50):

    def get_robot_name(self):

    def named_target_in_srdf(self, name):
  
    def set_named_target(self, name):

    def get_named_target_joint_values(self, name):
       
    def get_end_effector_link(self):

    def get_current_pose(self, reference_frame=None):
       
    def get_current_state(self):

    def get_current_state_bounded(self):

    def get_robot_state_bounded(self):

    def move_to_named_target(self, name, wait=True):

    def plan_to_named_target(self, name, custom_start_state=None):

    def get_named_targets(self):
    '''
    def test_get_joints_position(self):
        ret_val = self.robot_commander.get_joints_position()
        self.assertTrue(type(ret_val) == dict)

    def test_get_joints_velocity(self):
        ret_val = self.robot_commander.get_joints_velocity()
        self.assertTrue(type(ret_val) == dict)

    def test_get_joints_state(self):
        ret_val = self.robot_commander.get_joints_state()
        self.assertTrue(type(ret_val) == JointState)
    '''
    def run_joint_trajectory(self, joint_trajectory):

    def make_named_trajectory(self, trajectory):

    def send_stop_trajectory_unsafe(self):

    def run_named_trajectory_unsafe(self, trajectory, wait=False):

    def run_named_trajectory(self, trajectory):

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):

    def plan_to_position_target(self, xyz, end_effector_link="", custom_start_state=None):

    def move_to_pose_target(self, pose, end_effector_link="", wait=True):

    def plan_to_pose_target(self, pose, end_effector_link="", alternative_method=False, custom_start_state=None):

    def move_to_joint_value_target_unsafe(self, joint_states, time=0.002, wait=True, angle_degrees=False):

    def action_is_running(self, controller=None):

    def run_joint_trajectory_unsafe(self, joint_trajectory, wait=True):

    def plan_to_waypoints_target(self, waypoints, reference_frame=None, eef_step=0.005, jump_threshold=0.0, custom_start_state=None):   

    def set_teach_mode(self, teach):

    def move_to_trajectory_start(self, trajectory, wait=True):

    @staticmethod
    def change_teach_mode(mode, robot):
        teach_mode_client = rospy.ServiceProxy('/teach_mode', RobotTeachMode)

        req = RobotTeachModeRequest()
        req.teach_mode = mode
        req.robot = robot
        try:
            resp = teach_mode_client(req)
            if resp.result == RobotTeachModeResponse.ERROR:
                rospy.logerr("Failed to change robot %s to mode %d", robot,
                             mode)
            else:
                rospy.loginfo("Changed robot %s to mode %d Result = %d", robot,
                              mode, resp.result)
        except rospy.ServiceException:
            rospy.logerr("Failed to call service teach_mode")

    def get_ik(self, target_pose, avoid_collisions=False, joint_states=None, ik_constraints=None):

    def move_to_pose_value_target_unsafe(self, target_pose, avoid_collisions=False, time=0.002, wait=True, ik_constraints=None):
    '''

if __name__ == "__main__":
    rospy.init_node('test_sr_robot_commander', anonymous=True)
    rostest.rosrun(PKG, 'test_sr_robot_commander', TestSrRobotCommander)