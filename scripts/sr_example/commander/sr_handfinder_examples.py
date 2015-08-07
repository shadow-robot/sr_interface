#!/usr/bin/env python
import sys
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("basic_hand_examples", anonymous=True)
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
if len(hand_parameters.mapping) is 0:
    print("No hand detected")
    sys.exit("no hand detected")

hand_serial = hand_parameters.mapping.keys()[0]
hand_mapping = hand_parameters.mapping[hand_serial]
prefix = hand_parameters.joint_prefix[hand_serial]
if hand_mapping == 'rh':
    hand_commander = SrHandCommander()
else:
    hand_commander = SrHandCommander(name="left_hand", prefix="lh")
joints = hand_finder.get_hand_joints()[hand_mapping]
if len(joints) is not 24:
    print("Joints are less than 24")
    sys.exit("no hand detected")
print("Moving to hand position defined by joint values")

position_1 = {prefix + 'FFJ1': 0.3490658767850654,
              prefix + 'FFJ2': 0.1747066021773609,
              prefix + 'FFJ3': 0.3773716583863109,
              prefix + 'FFJ4': -0.0353585782262833,
              prefix + 'THJ4': 0.01741447758271608,
              prefix + 'THJ5': -0.0044660151203368414,
              prefix + 'THJ1': 0.34905554413970474,
              prefix + 'THJ2': 0.19665410818337659,
              prefix + 'THJ3': -0.094030693144318,
              prefix + 'LFJ2': 0.1689616221460435,
              prefix + 'LFJ3': 0.27430689938959674,
              prefix + 'LFJ1': 0.3508110105031834,
              prefix + 'LFJ4': 0.02130857476409176,
              prefix + 'LFJ5': 0.03232033355620789,
              prefix + 'RFJ4': 0.03538037757354218,
              prefix + 'RFJ1': 0.350811000266134,
              prefix + 'RFJ2': 0.1754494541236511,
              prefix + 'RFJ3': 0.3761746978350553,
              prefix + 'MFJ1': 0.349075214851851,
              prefix + 'MFJ3': 0.41179341920347934,
              prefix + 'MFJ2': 0.1749198992229255,
              prefix + 'MFJ4': -0.03698607363380546,
              prefix + 'WRJ2': 0.020004520938839754,
              prefix + 'WRJ1': 2.8093405901152835e-05}

hand_commander.move_to_joint_value_target(position_1)

named_target_1 = "pack"
print("Moving to hand named target " + named_target_1)
hand_commander.move_to_named_target(named_target_1)

named_target_2 = "open"
print("Moving to hand named target " + named_target_2)
hand_commander.move_to_named_target(named_target_2)

hand_joints_state = hand_commander.get_joints_position()
hand_joints_velocity = hand_commander.get_joints_velocity()
hand_joints_effort = hand_commander.get_joints_effort()

print("Hand joints position \n " + str(hand_joints_state) + "\n")
print("Hand joints velocity \n " + str(hand_joints_velocity) + "\n")
print("Hand joints effort \n " + str(hand_joints_effort) + "\n")

tactile_type = hand_commander.get_tactile_type()
tactile_state = hand_commander.get_tactile_state()
print("Tactile type \n " + str(tactile_type) + "\n")
print("Tactile state \n " + str(tactile_state) + "\n")
