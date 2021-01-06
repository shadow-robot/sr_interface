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
hand_side_from_param = ()
robot_name = ()

class TestHandAndArmSim(TestCase):
    """
    Tests the Hand and Arm in Sim
    """
    def __init__(self):
        self.hand_type = rospy.get_param('test_sim/hand_type')
        self.scene = rospy.get_param('/test_sim/scene')

        #for phantom hand tests use hand finder
        self.hand_id = rospy.get_param('/hand/mapping/1082')

        rospy.wait_for_message('/move_group/status', GoalStatusArray)

        if self.hand_id == 'rh':
            self.arm_id = 'ra'
            self.robot_commander = SrRobotCommander(name="right_arm_and_hand")
            self.hand_commander = SrHandCommander(name='right_hand')
            self.arm_commander = SrArmCommander(name='right_arm', set_ground=False)
        elif self.hand_id == 'lh':
            self.arm_id = 'la'
            self.robot_commander = SrRobotCommander(name="left_arm_and_hand")
            self.hand_commander = SrHandCommander(name='left_hand')
            self.arm_commander = SrArmCommander(name='left_arm', set_ground=False)

        rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, self.scene_data_cb)

        print('hand id')
        print(self.hand_id)
        print('arm id')
        print(self.arm_id)
        print('hand type')
        print(self.hand_type)
        print('scene')
        print(self.scene)

    def joints_error_check(self, expected_joint_values, recieved_joint_values):
        expected_and_final_joint_value_diff = 0
        for expected_value, recieved_value in zip(sorted(expected_joint_values), sorted(recieved_joint_values)):
            expected_and_final_joint_value_diff += abs(expected_joint_values[expected_value] -
                                                       recieved_joint_values[recieved_value])
        return expected_and_final_joint_value_diff

    def scene_data_cb(self, result):
        scene_data = ()
        self.scene_data = result.world.collision_objects

    def wait_for_topic_with_scene(self, timeout=50):
        counter = 0
        while counter < timeout:
            current_value = self.scene_data
            print(self.scene_data)
            if len(current_value) != 0:
                return current_value
            rospy.sleep(0.1)
            counter += 1

    def test_home_position(self):
        # self.start_home = rospy.get_param('/test_sim/start_home')
        start_arm_angles = self.arm_commander.get_current_state()
        print('start angles')
        print(start_arm_angles)

        if self.arm_id == 'ra':
            #right arm home
            self.expected_home_angles = {'shoulder_pan_joint': 0.00, 'elbow_joint': 2.0,
                                             'shoulder_lift_joint': -1.25,'wrist_1_joint': -0.733, 
                                             'wrist_2_joint': 1.578, 'wrist_3_joint': -3.1416}

        elif self.arm_id == 'la':
            #left arm home
            self.expected_home_angles = {'shoulder_pan_joint': 0.0, 'elbow_joint': -2.0,
                                             'shoulder_lift_joint': -1.89,'wrist_1_joint': -2.4, 
                                             'wrist_2_joint': -1.5708, 'wrist_3_joint': 3.1416}

        # use if the start_home param is relevant. We don't use this for the UR10/5 specific launch files.

        # if self.start_home == False:
        #     self.expected_home_angles = {'shoulder_pan_joint': 0.00, 'elbow_joint': 0.00,
        #                                      'shoulder_lift_joint': 0.00, 'wrist_3_joint': 0.00,
        #                                      'wrist_1_joint': 0.00, 'wrist_2_joint': 0.00, 
        #                                      'wrist_3_joint': 0.00}
        # elif self.start_home == True:
        #     self.expected_home_angles = self.expected_home_angles

        home_angles_no_id = self.expected_home_angles
        expected_start_angles = {}
        for key, value in home_angles_no_id.items():
            expected_start_angles[self.arm_id + '_' + key] = value

        print('expected angles')
        print(expected_start_angles)

        expected_and_actual_home_angles = self.joints_error_check(expected_start_angles, start_arm_angles)

        self.assertAlmostEqual(expected_and_actual_home_angles, 0, delta=0.2)

    def test_scene(self):
        scene = ()
        self.scene = rospy.get_param('/test_sim/scene')
        self.scene_value = self.wait_for_topic_with_scene()
        print('scene value from topic')
        print(self.scene_value)
        if self.scene == True:
            self.assertNotEqual(len(self.scene_value), 0)
        elif self.scene == False:
            self.assertTrue(self.scene_value == None)
        print('scene check complete')
        
    def test_hand(self):
        hand_joints_target = {
         'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                                'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                                'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                                'RFJ1': 1.5707, 'RFJ2': 1.5707, 'RFJ3': 1.5707, 'RFJ4': 0.0,
                                'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                                'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
         'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                            'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                            'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                            'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0},

         'hand_extra_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                             'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                             'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0}
                    }

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in hand_joints_target_no_id.items():
            hand_joints_target[self.hand_id + '_' + key] = value

        self.hand_commander.move_to_joint_value_target(hand_joints_target, wait=True)
        rospy.sleep(5)
        final_hand_joint_values = self.hand_commander.get_current_state()

        expected_and_final_joint_value_diff_hand = self.joints_error_check(hand_joints_target, final_hand_joint_values)

        print('diff for hand')
        print(expected_and_final_joint_value_diff_hand)
        self.assertAlmostEqual(expected_and_final_joint_value_diff_hand, 0, delta=0.2)
    
    def test_arm(self):

        if self.arm_id == 'la':
            #left arm near home
            arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': -1.43,
                                'shoulder_lift_joint': -1.82, 'wrist_1_joint': 3.24,
                                'wrist_2_joint': -1.57, 'wrist_3_joint': 3.13}
        elif self.arm_id == 'ra':
            #right arm near home
            arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 1.43,
                                'shoulder_lift_joint': -1.27, 'wrist_1_joint': -0.1,
                                'wrist_2_joint': 1.57, 'wrist_3_joint': 3.13}

        arm_joints_target_no_id = arm_joints_target
        arm_joints_target = {}
        for key, value in arm_joints_target_no_id.items():
            arm_joints_target[self.arm_id + '_' + key] = value

        self.arm_commander.move_to_joint_value_target(arm_joints_target, wait=True)
        rospy.sleep(5)
        final_arm_joint_values = self.arm_commander.get_current_state()

        expected_and_final_joint_value_diff_arm = self.joints_error_check(arm_joints_target, final_arm_joint_values)

        print('arm diff')
        print(expected_and_final_joint_value_diff_arm)

        self.assertAlmostEqual(expected_and_final_joint_value_diff_arm, 0, delta=0.2)

    def test_hand_and_arm(self):
        hand_joints_target = {
            'hand_e': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ3': 0.0, 'THJ4': 1.20, 'THJ5': 0.17,
                             'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                             'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                             'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0,
                             'LFJ1': 0.0, 'LFJ2': 0.0, 'LFJ3': 0.0, 'LFJ4': 0.0,
                             'LFJ5': 0.0, 'WRJ1': 0.0, 'WRJ2': 0.0},
            'hand_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                           'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 0.0, 'FFJ4': 0.0,
                           'MFJ1': 1.5707, 'MFJ2': 1.5707, 'MFJ3': 1.5707, 'MFJ4': 0.0,
                           'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0},
            'hand_extra_lite': {'THJ1': 0.52, 'THJ2': 0.61, 'THJ4': 1.20, 'THJ5': 0.17,
                           'FFJ1': 1.5707, 'FFJ2': 1.5707, 'FFJ3': 1.5707, 'FFJ4': 0.0,
                           'RFJ1': 0.0, 'RFJ2': 0.0, 'RFJ3': 0.0, 'RFJ4': 0.0}
                 }

        hand_joints_target_no_id = hand_joints_target[self.hand_type]
        hand_joints_target = {}
        for key, value in hand_joints_target_no_id.items():
             hand_joints_target[self.hand_id + '_' + key] = value

        if self.arm_id == 'ra':
            #right arm home
            arm_joints_target = {'shoulder_pan_joint': 0.00, 'elbow_joint': 2.0,
                                             'shoulder_lift_joint': -1.25,'wrist_1_joint': -0.733, 
                                             'wrist_2_joint': 1.578, 'wrist_3_joint': -3.1416}

        elif self.arm_id == 'la':
            #left arm home
            arm_joints_target = {'shoulder_pan_joint': 0.0, 'elbow_joint': -2.0,
                                             'shoulder_lift_joint': -1.89,'wrist_1_joint': 3.8, 
                                             'wrist_2_joint': -1.5708, 'wrist_3_joint': 3.1416}

        arm_joints_target_no_id = arm_joints_target
        arm_joints_target = {}
        for key, value in arm_joints_target_no_id.items():
             arm_joints_target[self.arm_id + '_' + key] = value
        
        hand_and_arm_joints_target = dict(hand_joints_target.items() + arm_joints_target.items())
        print('hand and arm targets')
        print(hand_and_arm_joints_target)
        self.robot_commander.move_to_joint_value_target_unsafe(hand_and_arm_joints_target, 10.0, True)

        rospy.sleep(5)

        final_hand_and_arm_joint_values = self.robot_commander.get_current_state()
        print('hand and arm actual')
        print(final_hand_and_arm_joint_values)

        joint_value_diff_arm_and_hand = self.joints_error_check(hand_and_arm_joints_target, final_hand_and_arm_joint_values)

        self.assertAlmostEqual(joint_value_diff_arm_and_hand, 0, delta=0.4)

        print('arm and hand diff')
        print(joint_value_diff_arm_and_hand)

if __name__ == '__main__':
     PKG = "sr_robot_launch"
     rospy.init_node("hand_and_arm_test", anonymous=True)
     test = TestHandAndArmSim()
    #  rospy.sleep(10)
    #  test.test_home_position()
     rospy.sleep(10)
     test.test_scene()
     rospy.sleep(10)
     test.test_hand()
     rospy.sleep(10)
     test.test_arm()
     rospy.sleep(10)
     test.test_hand_and_arm()