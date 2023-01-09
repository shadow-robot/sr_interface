#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2021-2023 belongs to Shadow Robot Company Ltd.
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

import argparse
import yaml
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


def burn_in_demo(hand_commander, burn_in_config):
    for iteration in range(100):
        rospy.loginfo(f"We're on iteration number {iteration}")
        rospy.sleep(1)
        execute_command_check(hand_commander, burn_in_config, 'store_3', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'start_pos', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'flex_ff', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ext_ff', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'flex_mf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ext_mf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'flex_rf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ext_rf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'flex_lf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ext_lf', 1.1, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'flex_th_1', 1.0, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'flex_th_2', 1.0, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'ext_th_1', 2.0, 2.0)
        execute_command_check(hand_commander, burn_in_config, 'ext_th_2', 0.7, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'l_ext_lf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_ext_rf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_ext_mf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_ext_ff', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_int_all', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_ext_all', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_int_ff', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_int_mf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_int_rf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_int_lf', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_zero_all', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_spock', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'l_zero_all', 0.5, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'pre_ff_ok', 1.0, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ff_ok', 0.9, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'ff2mf_ok', 0.4, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'mf_ok', 0.9, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'mf2rf_ok', 0.4, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'rf_ok', 0.9, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'rf2lf_ok', 0.4, 0.5)
        execute_command_check(hand_commander, burn_in_config, 'lf_ok', 0.9, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'start_pos', 1.0, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'flex_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'flex_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_ff', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_mf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_rf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'ext_lf', 0.2, 0.2)
        execute_command_check(hand_commander, burn_in_config, 'pre_ff_ok', 1.0, 1.0)
        execute_command_check(hand_commander, burn_in_config, 'ff_ok', 1.3, 1.3)
        execute_command_check(hand_commander, burn_in_config, 'ne_wr', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'nw_wr', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'sw_wr', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'se_wr', 1.1, 1.1)
        execute_command_check(hand_commander, burn_in_config, 'ne_wr', 0.7, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'nw_wr', 0.7, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'sw_wr', 0.7, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'se_wr', 0.7, 0.7)
        execute_command_check(hand_commander, burn_in_config, 'zero_wr', 0.4, 0.4)
        execute_command_check(hand_commander, burn_in_config, 'start_pos', 1.5, 1.5)


def correct_joint_states_for_hand_type(joint_states_config, hand_type):
    hand_type_joints_filename = '/home/user/projects/shadow_robot/base/src/'\
                       'sr_interface/sr_demos/config/joints_in_hand.yaml'
    with open(hand_type_joints_filename, encoding="utf-8") as joint_file:
        hand_type_joints = yaml.load(joint_file, Loader=yaml.FullLoader)

    for joint_state_dicts_no_id in joint_states_config.keys():
        for key in list(joint_states_config[joint_state_dicts_no_id]):
            if key not in hand_type_joints[hand_type]:
                joint_states_config[joint_state_dicts_no_id].pop(key)

    return joint_states_config


def add_prefix_to_joint_states(corrected_joint_states_config, joint_prefix):
    demo_states = {}
    for joint_state_dicts_no_id in corrected_joint_states_config.keys():
        joints_target = {}
        for key, value in corrected_joint_states_config[joint_state_dicts_no_id].items():
            if joint_prefix == 'both':
                joints_target['rh_' + key] = value
                joints_target['lh_' + key] = value
            else:
                joints_target[joint_prefix + key] = value
            demo_states[joint_state_dicts_no_id] = joints_target
    return demo_states


def execute_command_check(hand_commander, joint_states_config, joint_states,
                          sleep, time, wait=False, angle_degrees=True):
    if joint_states in joint_states_config.keys():
        hand_commander.move_to_joint_value_target_unsafe(joint_states_config[joint_states], time, wait, angle_degrees)
        rospy.sleep(sleep)


if __name__ == "__main__":
    rospy.init_node("burn_in_demo", anonymous=True)

    parser = argparse.ArgumentParser(description="Hand side")
    parser.add_argument("-s", "--side",
                        dest="side",
                        type=str,
                        required=False,
                        help="Please select hand side, can be 'right', 'left' or 'both'.",
                        default=None,
                        choices=["right", "left", "both"])
    parser.add_argument("-ht", "--hand_type",
                        dest="hand_type",
                        type=str,
                        required=True,
                        help="Please select hand type, can be 'hand_e', 'hand_lite', 'hand_extra_lite'.",
                        default="hand_e",
                        choices=["hand_e", "hand_lite", "hand_extra_lite"])

    args = parser.parse_args(rospy.myargv()[1:])

    if args.side == 'right':
        joint_prefix_name = 'rh_'
    elif args.side == 'left':
        joint_prefix_name = 'lh_'
    else:
        joint_prefix_name = 'both'

    if joint_prefix_name == 'rh_':
        hand_name = 'right_hand'
    elif joint_prefix_name == 'lh_':
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    hand_commander_instance = SrHandCommander(name=hand_name)

    # Get joint states for burn in demo
    burn_in_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                              'sr_interface/sr_demos/config/burn_in_states.yaml'
    with open(burn_in_config_filename, encoding="utf-8") as config_file:
        burn_in_config_file = yaml.load(config_file, Loader=yaml.FullLoader)

    corrected_burn_in_config = correct_joint_states_for_hand_type(burn_in_config_file, args.hand_type)

    # Add prefix to joint states
    burn_in_states = add_prefix_to_joint_states(corrected_burn_in_config, joint_prefix_name)

    burn_in_demo(hand_commander_instance, burn_in_states)
