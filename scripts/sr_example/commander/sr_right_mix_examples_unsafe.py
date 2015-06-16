#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_arm_examples", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

rospy.loginfo("Set arm teach mode ON")
arm_commander.set_teach_mode(True)
# sleep for some time during which the arm can be moved around by pushing it
# but be careful to get away before the time runs out. You are warned
rospy.sleep(20.0)
rospy.loginfo("Set arm teach mode OFF")
arm_commander.set_teach_mode(False)

hand_joint_states_1 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
                       'rh_MFJ1': 0.35, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                       'rh_RFJ1': 0.35, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                       'rh_LFJ1': 0.35, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
                       'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
                       'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
hand_joint_states_2 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.0,
                       'rh_MFJ1': 0.35, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0,
                       'rh_RFJ1': 0.35, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0,
                       'rh_LFJ1': 0.35, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0,
                       'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
                       'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
hand_joint_states_3 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_MFJ3': 0.0}


joint_states_1 = {'ra_shoulder_pan_joint': 0.43221632746577665, 'ra_elbow_joint': 2.118891128999479,
                   'ra_wrist_1_joint': -1.711370650686752, 'ra_wrist_2_joint': 1.4834244535003318,
                   'ra_shoulder_lift_joint': -2.5813317754982474, 'ra_wrist_3_joint': 1.6175960918705412}

joint_states_2 = {'ra_shoulder_pan_joint': 0.4225743596855942, 'ra_elbow_joint': 1.9732180863151747,
                   'ra_wrist_1_joint': -0.8874321427449576, 'ra_wrist_2_joint': -0.9214312892819567,
                   'ra_shoulder_lift_joint': -1.9299519748391978, 'ra_wrist_3_joint': 0.7143446787498702}
# Move hand
joint_states = hand_joint_states_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 1.0, False)




# Move arm
joint_states = joint_states_1
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move hand
joint_states = hand_joint_states_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)


# Move hand
joint_states = hand_joint_states_3
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False)

# Move arm
joint_states = joint_states_2
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, wait= True)


hand_commander.set_teach_mode(True)
# sleep for some time during which the hand joints can be moved manually
rospy.sleep(20.0)
rospy.loginfo("Set hand teach mode OFF")
hand_commander.set_teach_mode(False)




