#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_left_examples", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")

hand_joint_states_1 = {'lh_FFJ1': 0.35, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0, 'lh_FFJ4': 0.0,
                       'lh_MFJ1': 0.35, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0, 'lh_MFJ4': 0.0,
                       'lh_RFJ1': 0.35, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0, 'lh_RFJ4': 0.0,
                       'lh_LFJ1': 0.35, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0, 'lh_LFJ4': 0.0,
                       'lh_THJ1': 0.35, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0,
                       'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}
hand_joint_states_2 = {'lh_FFJ1': 0.35, 'lh_FFJ2': 1.5707, 'lh_FFJ3': 1.5707, 'lh_FFJ4': 0.0,
                       'lh_MFJ1': 0.35, 'lh_MFJ2': 1.5707, 'lh_MFJ3': 1.5707, 'lh_MFJ4': 0.0,
                       'lh_RFJ1': 0.35, 'lh_RFJ2': 1.5707, 'lh_RFJ3': 1.5707, 'lh_RFJ4': 0.0,
                       'lh_LFJ1': 0.35, 'lh_LFJ2': 1.5707, 'lh_LFJ3': 1.5707, 'lh_LFJ4': 0.0,
                       'lh_THJ1': 0.35, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0,
                       'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}
hand_joint_states_3 = {'lh_FFJ1': 0.35, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0}

# Move hand
joint_states = hand_joint_states_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 1.0, False)
rospy.sleep(1)

# Move hand
joint_states = hand_joint_states_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)
rospy.sleep(1)

# Move hand
joint_states = hand_joint_states_3
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False)
rospy.sleep(1)
