#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("store_left_hand", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")

open_hand = {'lh_FFJ1': 0.0, 'lh_FFJ2': 0.0, 'lh_FFJ3': 0.0, 'lh_FFJ4': 0.0,
                       'lh_MFJ1': 0.0, 'lh_MFJ2': 0.0, 'lh_MFJ3': 0.0, 'lh_MFJ4': 0.0,
                       'lh_RFJ1': 0.0, 'lh_RFJ2': 0.0, 'lh_RFJ3': 0.0, 'lh_RFJ4': 0.0,
                       'lh_LFJ1': 0.0, 'lh_LFJ2': 0.0, 'lh_LFJ3': 0.0, 'lh_LFJ4': 0.0,
                       'lh_THJ1': 0.0, 'lh_THJ2': 0.0, 'lh_THJ3': 0.0, 'lh_THJ4': 0.0, 'lh_THJ5': 0.0,
                       'lh_WRJ1': 0.0, 'lh_WRJ2': 0.0}
pack_hand = {'lh_RFJ2': 1.5913206690439587, 'lh_RFJ3': 1.5301542471460914, 'lh_RFJ1': 1.514606265265658,
                     'lh_RFJ4': -0.004058584682882777, 'lh_LFJ4': 0.0075105064037285126, 'lh_LFJ5': 0.05658738641706916,
                     'lh_LFJ1': 1.4735637326698765, 'lh_LFJ2': 1.6004770713418932, 'lh_LFJ3': 1.5360892129922588,
                     'lh_THJ2': 0.6617049492258753, 'lh_THJ3': -0.10063535555027057, 'lh_THJ1': 0.43651492781440776,
                     'lh_THJ4': 0.49568178734759333, 'lh_THJ5': 0.36066791862916064, 'lh_FFJ4': -0.013247430697175228,
                     'lh_FFJ2': 1.7185088254155176, 'lh_FFJ3': 1.5369430550231247, 'lh_FFJ1': 1.4161209742074399,
                     'lh_MFJ3': 1.5312562463861372, 'lh_MFJ2': 1.6668522172823543, 'lh_MFJ1': 1.4413350910700424,
                     'lh_MFJ4': -0.008121358479877746, 'lh_WRJ2': -0.01325827302846633, 'lh_WRJ1': 0.02205477172306269}

# Move hand
joint_states = open_hand
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(1)

joint_states = pack_hand
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, False)
rospy.sleep(1)



