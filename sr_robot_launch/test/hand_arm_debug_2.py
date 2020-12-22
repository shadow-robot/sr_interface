#!/usr/bin/env python
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.
import rospy
import rostest
from moveit_msgs.msg import PlanningScene
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from geometry_msgs.msg import PoseStamped, Pose
from rospy import get_rostime
from actionlib_msgs.msg import GoalStatusArray
from unittest import TestCase

hand_type = ()
launch_file = ()
arm_id = ()
hand_id = ()
robot_commander = ()
hand_commander = ()
arm_commander = ()
current_value = ()
scene_value = ()
expected_home_angles = ()
hand_side = ()
#hand_side = left

class TestHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim
    """
    def __init__(self):
        #self.hand_side = rospy.get_param('~test_sim/side')
        hand_side == 'right'
        if hand_side == 'right':
            self.hand_id = 'rh'
            self.arm_id = 'ra'
        elif hand_side == 'left':
            self.hand_id = 'lh'
            self.arm_id = 'la'

        self.hand_type = rospy.get_param('test_sim/hand_type')
        self.scene = rospy.get_param('/test_sim/scene')

        print('hand id')
        print(hand_id)
        print('arm id')
        print(arm_id)
        print('hand type')
        print(self.hand_type)
        print('scene')
        print(self.scene)

if __name__ == '__main__':
     PKG = "sr_robot_launch"
     rospy.init_node("hand_and_arm_test", anonymous=True)
     test = TestHandAndArmSim()
     rospy.sleep(10)
     