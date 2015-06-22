#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_arm_examples", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")
arm_commander = SrArmCommander(name="left_arm", set_ground=False)

rospy.loginfo("Set arm teach mode ON")
arm_commander.set_teach_mode(True)
# sleep for some time during which the arm can be moved around by pushing it
# but be careful to get away before the time runs out. You are warned
rospy.sleep(20.0)
rospy.loginfo("Set arm teach mode OFF")
arm_commander.set_teach_mode(False)

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


joint_states_1 = {'la_shoulder_pan_joint': 0.43221632746577665, 'la_elbow_joint': 2.118891128999479,
                   'la_wrist_1_joint': -1.711370650686752, 'la_wrist_2_joint': 1.4834244535003318,
                   'la_shoulder_lift_joint': -2.5813317754982474, 'la_wrist_3_joint': 1.6175960918705412}

joint_states_2 = {'la_shoulder_pan_joint': 0.4225743596855942, 'la_elbow_joint': 1.9732180863151747,
                   'la_wrist_1_joint': -0.8874321427449576, 'la_wrist_2_joint': -0.9214312892819567,
                   'la_shoulder_lift_joint': -1.9299519748391978, 'la_wrist_3_joint': 0.7143446787498702}
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




