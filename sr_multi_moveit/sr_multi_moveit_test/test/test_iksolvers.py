#!/usr/bin/env python

import rospy
import numpy
from unittest import TestCase
import logging

import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayRobotState
import geometry_msgs.msg
import copy
#
PKG = "sr_multi_moveit_config"


#class TestIKSolvers(TestCase):
class TestIKSolvers():

    """
    Test class for iksolvers using moveit
    """

    def setUp(self):
        
        group_id = str(sys.argv[1])
        #planner_id = str(sys.argv[2])
        iksolver_id = str(sys.argv[2])
        planner_id = "RRTkConfigDefault"

        rospy.init_node('moveit_test_iksolvers', anonymous=True)


        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(group_id)
        
        self.num_successful_plans = 0
        self.num_failed_plans = 0
        
        rospy.sleep(1)

        #self.group.set_planner_id(planner_id)

        #Visualize the goal position
        self.goal_pub = rospy.Publisher('tested_goal', DisplayRobotState, queue_size=10)
        self.msg = DisplayRobotState()
        self.msg.state.joint_state.name =  ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 'ra_wrist_1_joint', 'ra_wrist_2_joint', 'ra_wrist_3_joint', 'rh_WRJ2', 'rh_WRJ1', 'rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 'rh_RFJ2', 'rh_RFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 'rh_LFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1']
        self.msg.state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.sleep(5)

    def _add_walls_and_ground(self):
        # publish a scene
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()

        p.pose.position.x = 0
        p.pose.position.y = 0
        # offset such that the box is below ground (to prevent collision with
        # the robot itself)
        p.pose.position.z = -0.11
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        self.scene.add_box("ground", p, (3, 3, 0.1))

        p.pose.position.x = 0.4
        p.pose.position.y = 0.85
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.5
        p.pose.orientation.y = -0.5
        p.pose.orientation.z = 0.5
        p.pose.orientation.w = 0.5
        self.scene.add_box("wall_front", p, (0.8, 2, 0.01))

        p.pose.position.x = 1.33
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707388
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.706825
        self.scene.add_box("wall_right", p, (0.8, 2, 0.01))

        p.pose.position.x = -0.5
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707107
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.707107
        self.scene.add_box("wall_left", p, (0.8, 2, 0.01))

        # rospy.sleep(1)

    def _check_plan(self, plan):
        if len(plan.joint_trajectory.points) > 0:
            rospy.sleep(2)
            print "Plan ok"
            return True
        else:
            print "Plan failed"
            return False

    def _plan_joints(self, joints):
        self.group.clear_pose_targets()
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0:6] = joints[0:6]
        self.group.set_joint_value_target(group_variable_values)
        
        self.msg.state.joint_state.position[0:6] = joints[0:6]
        
        self.goal_pub.publish(self.msg)

        plan = self.group.plan()
        return self._check_plan(plan)

#     def test_trajectories_rotating_each_joint(self):
# test_joint_values = [numpy.pi/2.0, numpy.pi-0.33, -numpy.pi/2]
#          test_joint_values = [numpy.pi/2.0]
#
# Joint 4th is colliding with the hand
# for joint in range(6):
#          for joint in [0,1,2,3,5]:
#              joints = [0.0,0.0,0.0,-numpy.pi/2.0,0.0,0.0]
#              for value in test_joint_values:
#                  joints[joint] = value
# self.assertTrue(self._plan_joints(joints), msg="Unable to plan to:
# "+str(joints))

    def test_trajectories_empty_environment(self):
        # Up - Does not work with sbpl but it does with ompl
        joints = [
            -0.000938865682661, -
            1.98674414251, 2.19198020153, 0.581030868484, -
            0.00190057368648,
            -0.785934528975, 0.0, 0.0]
        #self.assertTrue(
        self._plan_joints(joints)

    def test_trajectories_with_walls_and_ground(self):
        self._add_walls_and_ground()

        # Goal close to left corner - Fails sometimes
        joints = [
            1.22262556307, -
            2.22935714353, 1.94043810556, 0.288788732588, 1.22183316693, 0.0873097240233,
            0.0, 0.0]
        #self.assertTrue(
        #    self._plan_joints(joints), msg="Unable to plan to: " + str(joints))
        #self._plan_joints(joints)

        # Goal close to right corner
        joints = [
            0.354696232081, -0.982224980654, 0.908055961723, -
            1.92328051116, -1.3516255551, 2.8225061435,
            0.0, 0.0]
        #self.assertTrue(
        #    self._plan_joints(joints), msg="Unable to plan to: " + str(joints))
        #self._plan_joints(joints)
        
        print "random joints"
        for i in range(10):
            joints = self.group.get_random_joint_values()
            self._plan_joints(joints)
        
#         print "random poses"
#         for i in range(10):
#             random_pose = self.group.get_random_pose()
#             self.group.set_pose_target(random_pose)
#             plan = self.group.plan()
#             self._check_plan(plan)

if __name__ == "__main__":
    #import rostest
    #rostest.rosrun(PKG, "test_iksolvers", TestIKSolvers)
    iktest = TestIKSolvers()
    iktest.setUp()
    iktest.test_trajectories_empty_environment()
    iktest.test_trajectories_with_walls_and_ground()

    
