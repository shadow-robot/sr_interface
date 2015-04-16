#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("basic_hand_examples", anonymous=True)

hand_commander = SrHandCommander()
position_1 = {'rh_FFJ1': 0.3490658767850654, 'rh_FFJ2': 0.1747066021773609, 'rh_FFJ3': 0.3773716583863109,
              'rh_FFJ4': -0.0353585782262833, 'rh_THJ4': 0.01741447758271608, 'rh_THJ5': -0.0044660151203368414,
              'rh_THJ1': 0.34905554413970474, 'rh_THJ2': 0.19665410818337659, 'rh_THJ3': -0.094030693144318,
              'rh_LFJ2': 0.1689616221460435, 'rh_LFJ3': 0.27430689938959674, 'rh_LFJ1': 0.3508110105031834,
              'rh_LFJ4': 0.02130857476409176, 'rh_LFJ5': 0.03232033355620789, 'rh_RFJ4': 0.03538037757354218,
              'rh_RFJ1': 0.350811000266134, 'rh_RFJ2': 0.1754494541236511, 'rh_RFJ3': 0.3761746978350553,
              'rh_MFJ1': 0.349075214851851, 'rh_MFJ3': 0.41179341920347934, 'rh_MFJ2': 0.1749198992229255,
              'rh_MFJ4': -0.03698607363380546, 'rh_WRJ2': 0.020004520938839754, 'rh_WRJ1': 2.8093405901152835e-05}

hand_commander.move_to_joint_value_target(position_1)
rospy.sleep(rospy.Duration(5))

hand_joint_states = hand_commander.get_joints_position()

print("Hand joints positions \n " + str(hand_joint_states) + "\n")
