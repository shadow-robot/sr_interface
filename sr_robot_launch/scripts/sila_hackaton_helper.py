#!/usr/bin/env python3


from __future__ import absolute_import
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_utilities_common_private.acm_utils import AcmUtils
from actionlib_msgs.msg import GoalStatusArray


if __name__ == "__main__":
    rospy.init_node("sila_hackaton_helper", anonymous=True)

    start = {'ra_elbow_joint': 1.9999073349184213,
            'ra_shoulder_lift_joint': -1.2500806615435351,
            'ra_shoulder_pan_joint': 0.00018118389901911058,
            'ra_wrist_1_joint': -0.7329827331459604,
            'ra_wrist_2_joint': 1.570861742479413,
            'ra_wrist_3_joint': -3.141590934827494,
            'rh_FFJ1': 1.5698862699023888,
            'rh_FFJ2': 1.573139743745771,
            'rh_FFJ3': 1.572241466385437,
            'rh_FFJ4': -0.00543973278641019,
            'rh_LFJ1': 1.5712633535863718,
            'rh_LFJ2': 1.5699406996524674,
            'rh_LFJ3': 1.5703635725833358,
            'rh_LFJ4': 0.0016334617740012547,
            'rh_LFJ5': 0.00044875525609455735,
            'rh_MFJ1': 1.5715300176117015,
            'rh_MFJ2': 1.5701701768171406,
            'rh_MFJ3': 1.5704653335794152,
            'rh_MFJ4': -0.0011836010127712626,
            'rh_RFJ1': 1.5701212376853082,
            'rh_RFJ2': 1.5704116196328641,
            'rh_RFJ3': 1.570576351457027,
            'rh_RFJ4': 0.0010461939346626536,
            'rh_THJ1': 0.5162443082692887,
            'rh_THJ2': 0.6094010152987215,
            'rh_THJ3': 0.0014926925212055409,
            'rh_THJ4': 1.2211349179533366,
            'rh_THJ5': 0.17470433124917584,
            'rh_WRJ1': 0.0,
            'rh_WRJ2': 0.0}

    rospy.wait_for_service('/get_planning_scene')
    acm_utils = AcmUtils()
    acm_utils.allow_collisions_between_links(['ball__link', 'rh_ffdistal', 'rh_ffmiddle'], allow=True)

    rospy.wait_for_message("/move_group/status", GoalStatusArray)
    rospy.sleep(3)
    arm_and_hand_commander = SrArmCommander(name='right_arm_and_hand', set_ground=False)
    arm_and_hand_commander.move_to_joint_value_target_unsafe(start)
    rospy.sleep(2)
    rospy.loginfo("Ready to start.")

