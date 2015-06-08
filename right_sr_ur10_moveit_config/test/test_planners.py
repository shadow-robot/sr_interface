#!/usr/bin/env python

import rospy
import numpy
from unittest import TestCase

import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayRobotState
import geometry_msgs.msg

PKG = "right_sr_ur10_moveit_config"

class TestPlanners(TestCase):
    """
    Test class for motion planners using moveit
    """

    def setUp(self):      
        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_test_planners', anonymous=True)
        
        scene = PlanningSceneInterface()
        robot = RobotCommander()
        rospy.sleep(1)
         
        # clean the scene
        scene.remove_world_object("ground")
        scene.remove_world_object("wall_front")
        scene.remove_world_object("wall_left")
        scene.remove_world_object("wall_right")
        
        # publish a scene
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        
        p.pose.position.x = 0
        p.pose.position.y = 0
        # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
        p.pose.position.z = -0.11
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        scene.add_box("ground", p, (3, 3, 0.1))
        
        p.pose.position.x = 0.4
        p.pose.position.y = 0.85 #1.40
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.5
        p.pose.orientation.y = -0.5
        p.pose.orientation.z = 0.5
        p.pose.orientation.w = 0.5
        scene.add_box("wall_front", p, (0.8, 2, 0.01))
        
        p.pose.position.x = 1.33
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707388
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.706825
        scene.add_box("wall_left", p, (0.8, 2, 0.01))
        
        p.pose.position.x = -0.5
        p.pose.position.y = 0.4
        p.pose.position.z = 0.4
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = -0.707107
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.707107
        scene.add_box("wall_right", p, (0.8, 2, 0.01))
        
        rospy.sleep(1)
        
        # plan to a random location 
        #a = robot.right_arm
        #a.set_start_state(RobotState())
        #r = a.get_random_joint_values()
        #print "Planning to random joint position: "
        #print r
        #p = a.plan(r)
        #print "Solution:"
        #print p
        
        #print "Current state:"
        #print robot.get_current_state()
        
        group = MoveGroupCommander("right_arm")
        #print group.get_end_effector_link()
        #print "pose reference frame: ", group.get_pose_reference_frame()
        
        group_variable_values = group.get_current_joint_values()


    def test_simple_trajectory(self):
        self.assertEqual('FOO'.upper(), 'FOO')

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "test_planners", TestPlanners)