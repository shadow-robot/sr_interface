#!/usr/bin/env python3

# Copyright 2021-2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import argparse
import yaml
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


def burn_in_demo(hand_commander, burn_in_config):
    for iteration in range(0, 100):
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
    with open(hand_type_joints_filename) as hand_type_file:
        hand_type_joints = yaml.load(hand_type_file, Loader=yaml.FullLoader)

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
        joint_prefix_main = 'rh_'
    elif args.side == 'left':
        joint_prefix_main = 'lh_'
    else:
        joint_prefix_main = 'both'

    if joint_prefix_main == 'rh_':
        hand_name = 'right_hand'
    elif joint_prefix_main == 'lh_':
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    hand_commander_class = SrHandCommander(name=hand_name)

    # Get joint states for burn in demo
    burn_in_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                              'sr_interface/sr_demos/config/burn_in_states.yaml'
    with open(burn_in_config_filename) as burn_in_file:
        burn_in_config_yaml = yaml.load(burn_in_file, Loader=yaml.FullLoader)

    corrected_burn_in_config = correct_joint_states_for_hand_type(burn_in_config_yaml, args.hand_type)

    # Add prefix to joint states
    burn_in_states = add_prefix_to_joint_states(corrected_burn_in_config, joint_prefix_main)

    burn_in_demo(hand_commander_class, burn_in_states)
