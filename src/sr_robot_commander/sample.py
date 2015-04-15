#!/usr/bin/python
import rospy
# from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander

rospy.init_node("basic_example", anonymous=True)

# hand = SrHandCommander()
# hand.move_to_joint_value_target({
#     "THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
#     "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
#     "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
#     "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
#     "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10
# })
# rospy.sleep(rospy.Duration(3))
#
# hand_joint_states = hand.get_joints_position()
#
# print("Hand joints positions \n " + str(hand_joint_states) + "\n")

arm = SrArmCommander()
arm.move_to_position_target([0.5, 0.5, 1.0])

rospy.sleep(rospy.Duration(3))

print("Arm joints position\n" + str(arm.get_joints_position()) + "\n")



