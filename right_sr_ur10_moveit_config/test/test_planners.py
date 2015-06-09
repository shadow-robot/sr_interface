#!/usr/bin/env python

import rospy
import numpy
from unittest import TestCase
import logging
 
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayRobotState
import geometry_msgs.msg

PKG = "right_sr_ur10_moveit_config"
 
class TestPlanners(TestCase):
    """
    Test class for motion planners using moveit
    """
 
    def setUp(self):      
         rospy.init_node('moveit_test_planners', anonymous=True)
           
         self.scene = PlanningSceneInterface()
         self.robot = RobotCommander()
         self.group = MoveGroupCommander("right_arm")
         rospy.sleep(1)
          
         planner_id = "LBKPIECEkConfigDefault"
         self.group.set_planner_id(planner_id)    
         
         #Visualize the goal position
         self.goal_pub = rospy.Publisher('tested_goal', DisplayRobotState, queue_size=10)
         self.msg = DisplayRobotState()
         self.msg.state.joint_state.name =  ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 'ra_wrist_1_joint', 'ra_wrist_2_joint', 'ra_wrist_3_joint', 'rh_WRJ2', 'rh_WRJ1', 'rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 'rh_RFJ2', 'rh_RFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 'rh_LFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1']
         self.msg.state.joint_state.position = [1.57542451065, 3.01734161219, 2.01043257686, -1.14647092839, 0.694689321451, -0.390769365032, 0.0, 0.0, 0.0, 0.0, 0.0, 0.349938515025, 0.0, 0.0, 0.0, 0.349938515025, 0.0, 0.0, 0.0, 0.349938515025, 0.0, 0.0, 0.0, 0.0, 0.349938515025, 0.0, 0.0, 0.0, 0.0, 0.349938515025]
         rospy.sleep(5)
         
         #For debugging purposes
         logging.basicConfig(filename='example.log',level=logging.DEBUG)
          
    def _add_walls_and_ground(self):       
         # publish a scene
         p = geometry_msgs.msg.PoseStamped()
         p.header.frame_id = self.robot.get_planning_frame()
         
         p.pose.position.x = 0
         p.pose.position.y = 0
         # offset such that the box is below ground (to prevent collision with the robot itself)
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
         self.scene.add_box("wall_left", p, (0.8, 2, 0.01))
         
         p.pose.position.x = -0.5
         p.pose.position.y = 0.4
         p.pose.position.z = 0.4
         p.pose.orientation.x = 0.0
         p.pose.orientation.y = -0.707107
         p.pose.orientation.z = 0.0
         p.pose.orientation.w = 0.707107
         self.scene.add_box("wall_right", p, (0.8, 2, 0.01))
         
         rospy.sleep(1)
         
    def _check_plan(self, plan):
         if len(plan.joint_trajectory.points) > 0:
             rospy.sleep(1)
             return True
         else:
             return False
      
    def _plan_joints(self, joints):
         self.group.clear_pose_targets()
         group_variable_values = self.group.get_current_joint_values()
         group_variable_values[0:6] = joints[0:6]
         self.group.set_joint_value_target(group_variable_values)
         
         self.msg.state.joint_state.position[0:6] = joints
         self.goal_pub.publish(self.msg)
         
         plan = self.group.plan()
         return self._check_plan(plan)
 
    def test_trajectories_empty_environment(self):
         #test_joint_values = [numpy.pi/2.0, numpy.pi, -numpy.pi/2]
         test_joint_values = [ -numpy.pi/2]
         
         #for joint in range(6):
         for joint in [3]:
             joints = self.group.get_current_joint_values()[0:6]
             for value in test_joint_values:
                 joints[joint] = value
                 self.assertTrue(self._plan_joints(joints),msg="Unable to plan to: "+str(joints))
         
         
         
#          joints = [-1.67232, -2.39104, 0.264862, 0.43346, 2.44148, 2.48026]
#          self.assertTrue(self._plan_joints(joints))
#          
#          #Does not work with sbpl but it does with ompl
#          joints = [-0.000938865682661, -1.98674414251, 2.19198020153, 0.581030868484, -0.00190057368648, -0.785934528975]
#          self.assertTrue(self._plan_joints(joints),msg="")
#     
#          joints = [ 1.425279839, -0.110370375874, -1.52548746261, -1.50659865247, -1.42700242769, 3.1415450794]
#          self.assertTrue(self._plan_joints(joints))
#     
#          joints = [1.57542451065, 3.01734161219, 2.01043257686, -1.14647092839, 0.694689321451, -0.390769365032]
#          self.assertTrue(self._plan_joints(joints))
#           
#     def test_trajectory_with_walls_and_ground(self):
#          self._add_walls_and_ground()
#           
#          #Goal is in collision with the wall_front
#          joints = [0.302173213174, 0.192487443763, -1.94298265002, 1.74920382275, 0.302143499777, 0.00130280337897]
#          self.assertFalse(self._plan_joints(joints))
 
#     def test_one(self):
#         a= 15
#         
#         logging.debug('This message should go to the log file')
#         logging.info("Distance error is not close to zero: "+str(a))
#         logging.warning('And this, bla')
#         #self.assertEqual(1, 2,msg="Distance error is not close to zero: "+str(a))
#         try:
#             self.assertEqual(1, 2)
#         except AssertionError as e:      
#             logging.debug("\nMESSENGER OUTPUT: %s" % str(e))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_planners", TestPlanners)
    
