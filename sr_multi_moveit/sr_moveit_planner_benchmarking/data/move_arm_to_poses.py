#!/usr/bin/env python

# This example demonstrates some of the functions of the arm commander.
# The arm is moved through a sequence of goals generated via different functions in the commander.

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("basic_right_arm_example", anonymous=True)

arm_commander = SrArmCommander(set_ground=False)

rospy.sleep(rospy.Duration(2))

# # The arm commander generates a plan to a new pose before the pose is executed.
# pose = [0.70, -0.19, 0.65, -0.62, -0.08, -0.65, 0.42]
# print("Planning the move to the first pose:\n" + str(pose) + "\n")
# arm_commander.plan_to_pose_target(pose)
# print("Finished planning, moving the arm now.")
# arm_commander.execute()
# rospy.sleep(6.0)
# raw_input("press key after pose")
# print("Arm joints position:\n" + str(arm_commander.get_joints_position()) + "\n")
# raw_input("press a key to continue")


keys = ['ra_shoulder_pan_joint', 'ra_shoulder_lift_joint', 'ra_elbow_joint', 'ra_wrist_1_joint', 'ra_wrist_2_joint',
        'ra_wrist_3_joint']

# pose = [0.11, 0.001, -0.93, -2.22, -1.71, -1.68]
# joints_goal = dict(zip(keys, pose))
# print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joints_goal)
# rospy.sleep(rospy.Duration(5))
# print("Arm joints position:\n" + str(arm_commander.get_current_state()) + "\n")
# raw_input("press a key to continue")
#
# pose = [1.55, -1.15, 2.01, 2.39, -1.55, -1.58]
# joints_goal = dict(zip(keys, pose))
# print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joints_goal)
# rospy.sleep(rospy.Duration(5))
# print("Arm joints position:\n" + str(arm_commander.get_joints_position()) + "\n")
# raw_input("press a key to continue")

# pose = [2.0124379710802414, -0.6395552894291645, 1.8272080603542877, 2.0191359302159064, -1.3904105764847612,
#         -3.0564905988137565]
# joints_goal = dict(zip(keys, pose))
# print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joints_goal)
# rospy.sleep(rospy.Duration(5))
# print("Arm joints position:\n" + str(arm_commander.get_current_state()) + "\n")
# raw_input("press a key to continue")
#
# pose = [2.6752085499210883, -0.8522038622523826, 1.201729214706459, 2.821729335878766, -1.7238947911545262,
#         -1.6146325266562542]
# joints_goal = dict(zip(keys, pose))
# print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joints_goal)
# rospy.sleep(rospy.Duration(5))
# print("Arm joints position:\n" + str(arm_commander.get_joints_position()) + "\n")
# raw_input("press a key to continue")

pose = [-1.4888676313634834, -2.032021937909764, 2.0806817862208606, -2.7766676701654585, -1.5224358139380132,
        3.1386020362120806]
joints_goal = dict(zip(keys, pose))
print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joints_goal)
rospy.sleep(rospy.Duration(5))
print("Arm joints position:\n" + str(arm_commander.get_current_state()) + "\n")
raw_input("press a key to continue")

pose = [3.0990236703835805, -1.648608875775305, -1.618753950152506, -3.015160603087673, -1.6143316921599373, 3.142204119951054]
joints_goal = dict(zip(keys, pose))
print("Moving arm to joints position:\n" + str(joints_goal) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joints_goal)
rospy.sleep(rospy.Duration(5))
print("Arm joints position:\n" + str(arm_commander.get_joints_position()) + "\n")
raw_input("press a key to continue")
print "Finished"

