#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_arm_examples", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

# Move hand
named_target_1 = "open"
print("Moving to hand named target " + named_target_1)
hand_commander.move_to_named_target(named_target_1, False)


joint_states_1 = {'ra_shoulder_pan_joint': 0.43221632746577665, 'ra_elbow_joint': 2.118891128999479,
                   'ra_wrist_1_joint': -1.711370650686752, 'ra_wrist_2_joint': 1.4834244535003318,
                   'ra_shoulder_lift_joint': -2.5813317754982474, 'ra_wrist_3_joint': 1.6175960918705412}

joint_states_2 = {'ra_shoulder_pan_joint': 0.4225743596855942, 'ra_elbow_joint': 1.9732180863151747,
                   'ra_wrist_1_joint': -0.8874321427449576, 'ra_wrist_2_joint': -0.9214312892819567,
                   'ra_shoulder_lift_joint': -1.9299519748391978, 'ra_wrist_3_joint': 0.7143446787498702}

# Move arm
joint_states = joint_states_1
print("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target(joint_states)

named_target_1 = "pack"
print("Moving to hand named target " + named_target_1)
hand_commander.move_to_named_target(named_target_1, True)



hand_joint_states_1 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
                       'rh_MFJ1': 0.35, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
                       'rh_RFJ1': 0.35, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
                       'rh_LFJ1': 0.35, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
                       'rh_THJ1': 0.35, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
                       'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
hand_joint_states_2 = {'rh_FFJ1': 0.35, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0}

# Move hand
joint_states = hand_joint_states_2
print("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target(joint_states, True)

# Move arm
joint_states = joint_states_2
print("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target(joint_states)


#rospy.sleep(5.0)



