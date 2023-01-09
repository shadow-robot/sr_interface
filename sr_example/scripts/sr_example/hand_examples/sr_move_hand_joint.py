#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.


from copy import deepcopy
from builtins import input
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

####################
# POSE DEFINITIONS #
####################

# starting position for the hand
start_pos = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0,
             "rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0,
             "rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0,
             "rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0,
             "rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0,
             "rh_WRJ1": 0, "rh_WRJ2": 0}

# flex first finger
flex_ff = {"rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0}
# extend first finger
ext_ff = {"rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0}

flex_mf = {"rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0}
ext_mf = {"rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0}

flex_rf = {"rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0}
ext_rf = {"rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0}

flex_lf = {"rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0}
ext_lf = {"rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0}

flex_th = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}
ext_th = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}


def select_finger():
    flex = {}
    extend = {}
    finger = input("select which finger to move (options: ff, mf, rf, lf, th  default: ff): ")
    if finger == "ff":
        finger = "FF"
        flex = deepcopy(flex_ff)
        extend = deepcopy(ext_ff)
    elif finger == "mf":
        finger = "MF"
        flex = deepcopy(flex_mf)
        extend = deepcopy(ext_mf)
    elif finger == "rf":
        finger = "RF"
        flex = deepcopy(flex_rf)
        extend = deepcopy(ext_rf)
    elif finger == "lf":
        finger = "LF"
        flex = deepcopy(flex_lf)
        extend = deepcopy(ext_lf)
    elif finger == "th":
        finger = "TH"
        flex = deepcopy(flex_th)
        extend = deepcopy(ext_th)
    else:
        rospy.loginfo("Default finger 'ff' will be used.")
        finger = "FF"
        flex = deepcopy(flex_ff)
        extend = deepcopy(ext_ff)

    correct_joint_number = False
    while not correct_joint_number:
        correct_joint_number = True
        joint_number_str = input("select joint number to move: ")
        joint_number_int = int(joint_number_str)
        if finger in ('FF', 'MF', 'RF'):
            if joint_number_int not in range(5):
                rospy.logerr(f"The finger you selected doesn't have joint {joint_number_int}")
                correct_joint_number = False
        elif finger in ('LF', 'TH'):
            if joint_number_int not in range(6):
                rospy.logerr(f"The finger you selected doesn't have joint {joint_number_int}")
                correct_joint_number = False

    return finger, flex, extend, joint_number_str, joint_number_int


def sequence_ff():
    rospy.sleep(1.2)
    while True:
        finger, flex, extend, joint_number_str, joint_number_int = select_finger()
        if joint_number_int == 0:
            joint_1 = "rh_" + finger + "J1"
            joint_2 = "rh_" + finger + "J2"
        else:
            joint = "rh_" + finger + "J" + joint_number_str
        flex_degrees = input("Select flex position in degrees (default 180): ")
        try:
            if flex_degrees:
                flex_degrees_float = float(flex_degrees)
                if joint_number_int == 0:
                    flex[joint_1] = flex_degrees_float / 2.0
                    flex[joint_2] = flex_degrees_float / 2.0
                else:
                    flex[joint] = flex_degrees_float
            else:
                if joint_number_int == 0:
                    flex[joint_1] = 90
                    flex[joint_2] = 90
                else:
                    flex[joint] = 90
        except ValueError:
            rospy.logerr("You didn't give a valid value")
            continue

        ext_degrees = input("Select extension position in degrees (default 0): ")
        try:
            if ext_degrees:
                ext_degrees_float = float(ext_degrees)
                if joint_number_int == 0:
                    extend[joint_1] = ext_degrees_float / 2.0
                    extend[joint_2] = ext_degrees_float / 2.0
                else:
                    extend[joint] = ext_degrees_float
            else:
                if joint_number_int == 0:
                    extend[joint_1] = 0
                    extend[joint_2] = 0
                else:
                    extend[joint] = 0
        except ValueError:
            rospy.logerr("You didn't give a valid value")
            continue

        time_raw = input("Select the time for each movement in seconds (default 1.0): ")
        try:
            if time_raw:
                time = float(time_raw)
            else:
                time = 1.0
        except ValueError:
            rospy.logerr("You didn't give a valid value")
            continue

        while True:
            hand_commander.move_to_joint_value_target_unsafe(flex, time, True, angle_degrees=True)
            rospy.sleep(1.0)
            hand_commander.move_to_joint_value_target_unsafe(extend, time, True, angle_degrees=True)

            user_input = input(
                "Press return to run again, 'change' to change parameters or 'exit' to exit the program: ")
            if user_input == 'exit':
                return
            if user_input == 'change':
                break


if __name__ == "__main__":
    rospy.init_node("right_hand_demo", anonymous=True)
    hand_commander = SrHandCommander(name="right_hand")

    sequence_ff()
    rospy.loginfo("Demo finished!")
