#!/usr/bin/env python3

# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import

import rospy
import rostest
from unittest import TestCase
from sr_robot_commander.sr_robot_commander import SrRobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

PKG = "sr_robot_commander"


class TestSrRobotCommander(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot_commander = SrRobotCommander("right_hand")
        print("start")

    @classmethod
    def setUp(cls):
        cls._last_request = None
        rospy.sleep(1)
    
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

        d  = {'rh_WRJ2': 0.1745340287580568,
                'rh_WRJ1': 0.488692040221113,
                'rh_FFJ4': -0.12370829534925232,
                'rh_FFJ3': 0.04498799811400733,
                'rh_FFJ2': 0.016694409745289462,
                'rh_FFJ1': 0.00012700937002119161,
                'rh_LFJ5': 0.08430645146960725,
                'rh_LFJ4': 0.12166385051603079,
                'rh_LFJ3': 0.05330174206790339,
                'rh_LFJ2': 0.019320504252301696,
                'rh_LFJ1': 0.00013192817694029912,
                'rh_MFJ4': -0.12241554547398614,
                'rh_MFJ3': 0.04350528336593573,
                'rh_MFJ2': 0.016265098966647074,
                'rh_MFJ1': -6.230829582953845e-06,
                'rh_RFJ4': 0.12232543595297685,
                'rh_RFJ3': 0.04469699140779948,
                'rh_RFJ2': 0.016575470880908938,
                'rh_RFJ1': 0.0001891606132202739,
                'rh_THJ5': -0.008792838897692334,
                'rh_THJ4': 0.25618694160585687,
                'rh_THJ3': 0.013972368249145717,
                'rh_THJ2': -0.6981317670242566,
                'rh_THJ1': -0.07812434355592579}   

        rospy.logwarn(self.robot_commander._move_group_commander.get_end_effector_link())
        rospy.logwarn(self.robot_commander._move_group_commander.get_pose_reference_frame())

        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.header = Header()
        rs.joint_state.name = d.keys()
        rs.joint_state.position = d.values()


        x = self.robot_commander.get_end_effector_pose_from_state(rs)
        rospy.logwarn(x)

    '''
    def test_mvgr_get_planning_frame(self):
        raised = False
        try:
            self.robot_commander.get_planning_frame()
        except (NameError, TypeError):
            raised = True
        self.assertFalse(raised)

    def set_pose_reference_frame(self, reference_frame):

    def get_group_name(self):

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

    def get_joints_position(self):

    def get_joints_velocity(self):

    def get_joints_state(self):

    def run_joint_trajectory(self, joint_trajectory):

    def make_named_trajectory(self, trajectory):

    def send_stop_trajectory_unsafe(self):

    def run_named_trajectory_unsafe(self, trajectory, wait=False):

    def run_named_trajectory(self, trajectory):

    def move_to_position_target(self, xyz, end_effector_link="", wait=True):

    def plan_to_position_target(self, xyz, end_effector_link="", custom_start_state=None):

    def move_to_pose_target(self, pose, end_effector_link="", wait=True):

    def plan_to_pose_target(self, pose, end_effector_link="", alternative_method=False, custom_start_state=None):

    def _joint_states_callback(self, joint_state):

    def _set_up_action_client(self, controller_list):

    def move_to_joint_value_target_unsafe(self, joint_states, time=0.002, wait=True, angle_degrees=False):

    def action_is_running(self, controller=None):

    def _action_done_cb(self, controller, terminal_state, result):

    def _call_action(self, goals):

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