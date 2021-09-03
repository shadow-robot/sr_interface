#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import random
import time
import yaml
import termios
import tty
import sys
import argparse
from math import degrees
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


def sequence_th(hand_commander, joint_states_config):
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'],
                                                     1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    return


def sequence_ff(hand_commander, joint_states_config):
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['store_3'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_ff'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_ff'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_mf'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_mf'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_rf'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_rf'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_lf'], 1.0, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_th_1'], 0.7, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_th_2'], 0.7, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_th_1'], 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_th_2'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_ext_lf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_ext_rf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_ext_mf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_ext_ff'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_int_all'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_ext_all'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_int_ff'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_int_mf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_int_rf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_int_lf'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_zero_all'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_spock'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['l_zero_all'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['pre_ff_ok'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ff_ok'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ff2mf_ok'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['mf_ok'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['mf2rf_ok'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['rf_ok'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['rf2lf_ok'], 0.5, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['lf_ok'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.9)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['flex_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_ff'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_mf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_rf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ext_lf'], 0.2, False, angle_degrees=True)
    rospy.sleep(0.2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['pre_ff_ok'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ff_ok'], 1.3, False, angle_degrees=True)
    rospy.sleep(3.3)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ne_wr'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['nw_wr'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['sw_wr'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['se_wr'], 1.1, False, angle_degrees=True)
    rospy.sleep(1.1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['ne_wr'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['nw_wr'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['sw_wr'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['se_wr'], 0.7, False, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['zero_wr'], 0.4, False, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'], 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)
    return


def sequence_mf(hand_commander, joint_states_config, inter_time_max, sim, force_zero):
    rospy.sleep(0.5)
    # Initialize wake time
    wake_time = time.time()

    while True:
        touched = None
        if sim is False:
            # Check if any of the tactile senors have been triggered
            # If so, send the Hand to its start position
            tactile_values = read_tactile_values(hand_commander, hand_commander.get_tactile_type())
            for finger in ["FF", "MF", "RF", "LF", "TH"]:
                if tactile_values[finger] > force_zero[finger]:
                    touched = finger
        if touched is not None:
            hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'],
                                                             2.0, False, angle_degrees=True)
            rospy.loginfo('{} touched!'.format(finger))
            rospy.sleep(2.0)
            if touched == "TH":
                break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in joint_states_config['rand_pos']:
                    joint_states_config['rand_pos'][i] =\
                        random.randrange(joint_states_config['min_range'][i],
                                         joint_states_config['max_range'][i])

                joint_states_config['rand_pos']['rh_FFJ4'] =\
                    random.randrange(joint_states_config['min_range']['rh_FFJ4'],
                                     joint_states_config['rand_pos']['rh_MFJ4'])
                joint_states_config['rand_pos']['rh_LFJ4'] =\
                    random.randrange(joint_states_config['min_range']['rh_LFJ4'],
                                     joint_states_config['rand_pos']['rh_RFJ4'])
                inter_time = inter_time_max * random.random()
                hand_commander.move_to_joint_value_target_unsafe(joint_states_config['rand_pos'],
                                                                 inter_time, False, angle_degrees=True)
                wake_time = time.time() + inter_time * 0.9
    return


def sequence_rf(hand_commander, joint_states_config):
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_pre_zero'], 2.0, False, angle_degrees=True)
    rospy.sleep(2)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_zero'], 1.0, False, angle_degrees=True)
    rospy.sleep(4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_1'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_2'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_3'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_4'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_5'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_6'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_7'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_8'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_9'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_11'], 1.0, False, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['bc_12'], 3.0, False, angle_degrees=True)
    rospy.sleep(4)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'], 1.5, False, angle_degrees=True)
    rospy.sleep(1.5)

    return


def sequence_lf(hand_commander, joint_states_config, sim, force_zero):
    # Trigger flag array
    trigger = [0, 0, 0, 0, 0]

    # Move Hand to zero position
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'],
                                                     2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to starting position
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['pregrasp_pos'],
                                                     2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to close position
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['grasp_pos'],
                                                     11.0, False, angle_degrees=True)
    offset1 = 3

    # Initialize end time
    end_time = time.time() + 11

    while True and sim is False:
        # Check  the state of the tactile senors
        tactile_values = read_tactile_values(hand_commander, hand_commander.get_tactile_type())

        # Record current joint positions
        hand_pos = {joint: degrees(i) for joint, i in hand_commander.get_joints_position().items()}

        # If any tacticle sensor has been triggered, send
        # the corresponding digit to its current position
        if (tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0):
            hand_pos_incr_f = {"rh_FFJ1": hand_pos['rh_FFJ1'] + offset1, "rh_FFJ3": hand_pos['rh_FFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_f, 0.5, False, angle_degrees=True)
            rospy.loginfo('First finger contact')
            trigger[0] = 1

        if (tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0):
            hand_pos_incr_m = {"rh_MFJ1": hand_pos['rh_MFJ1'] + offset1, "rh_MFJ3": hand_pos['rh_MFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_m, 0.5, False, angle_degrees=True)
            rospy.loginfo('Middle finger contact')
            trigger[1] = 1

        if (tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0):
            hand_pos_incr_r = {"rh_RFJ1": hand_pos['rh_RFJ1'] + offset1, "rh_RFJ3": hand_pos['rh_RFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_r, 0.5, False, angle_degrees=True)
            rospy.loginfo('Ring finger contact')
            trigger[2] = 1

        if (tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0):
            hand_pos_incr_l = {"rh_LFJ1": hand_pos['rh_LFJ1'] + offset1, "rh_LFJ3": hand_pos['rh_LFJ3'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_l, 0.5, False, angle_degrees=True)
            rospy.loginfo('Little finger contact')
            trigger[3] = 1

        if (tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0):
            hand_pos_incr_th = {"rh_THJ2": hand_pos['rh_THJ2'] + offset1, "rh_THJ5": hand_pos['rh_THJ5'] + offset1}
            hand_commander.move_to_joint_value_target_unsafe(hand_pos_incr_th, 0.5, False, angle_degrees=True)
            rospy.loginfo('Thumb contact')
            trigger[4] = 1

        if (trigger[0] == 1 and trigger[1] == 1 and trigger[2] == 1 and trigger[3] == 1 and trigger[4] == 1):
            break

        if time.time() > end_time:
            break

    # Send all joints to current position to compensate
    # for minor offsets created in the previous loop
    hand_pos = {joint: degrees(i) for joint, i in hand_commander.get_joints_position().items()}
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    # Generate new values to squeeze object slightly
    offset2 = 3
    squeeze = hand_pos.copy()
    squeeze.update({"rh_THJ5": hand_pos['rh_THJ5'] + offset2, "rh_THJ2": hand_pos['rh_THJ2'] + offset2,
                    "rh_FFJ3": hand_pos['rh_FFJ3'] + offset2, "rh_FFJ1": hand_pos['rh_FFJ1'] + offset2,
                    "rh_MFJ3": hand_pos['rh_MFJ3'] + offset2, "rh_MFJ1": hand_pos['rh_MFJ1'] + offset2,
                    "rh_RFJ3": hand_pos['rh_RFJ3'] + offset2, "rh_RFJ1": hand_pos['rh_RFJ1'] + offset2,
                    "rh_LFJ3": hand_pos['rh_LFJ3'] + offset2, "rh_LFJ1": hand_pos['rh_LFJ1'] + offset2})

    # Squeeze object gently
    hand_commander.move_to_joint_value_target_unsafe(squeeze, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(squeeze, 0.5, False, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, False, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['pregrasp_pos'],
                                                     2.0, False, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'],
                                                     2.0, False, angle_degrees=True)
    rospy.sleep(2.0)

    return


def zero_tactile_sensors(hand_commander, joint_states_config):
    # Move Hand to zero position
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(joint_states_config['start_pos'],
                                                     1.0, False, angle_degrees=True)

    rospy.logwarn('\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n')
    # raw_input ('Press ENTER to continue')
    rospy.sleep(1.0)

    for x in range(1, 1000):
        # Read current state of tactile sensors to zero them
        read_tactile_values(hand_commander, hand_commander.get_tactile_type())

        if tactile_values['FF'] > force_zero['FF']:
            force_zero['FF'] = tactile_values['FF']
        if tactile_values['MF'] > force_zero['MF']:
            force_zero['MF'] = tactile_values['MF']
        if tactile_values['RF'] > force_zero['RF']:
            force_zero['RF'] = tactile_values['RF']
        if tactile_values['LF'] > force_zero['LF']:
            force_zero['LF'] = tactile_values['LF']
        if tactile_values['TH'] > force_zero['TH']:
            force_zero['TH'] = tactile_values['TH']

    force_zero['FF'] = force_zero['FF'] + 5
    force_zero['MF'] = force_zero['MF'] + 5
    force_zero['RF'] = force_zero['RF'] + 5
    force_zero['LF'] = force_zero['LF'] + 5
    force_zero['TH'] = force_zero['TH'] + 5

    rospy.loginfo('Force Zero: ' + force_zero)

    return force_zero


def read_tactile_values(hand_commander, tactile_type):
    # Read current state of tactile sensors
    tactile_state = hand_commander.get_tactile_state()

    if tactile_type == "biotac":
        tactile_values['FF'] = tactile_state.tactiles[0].pdc
        tactile_values['MF'] = tactile_state.tactiles[1].pdc
        tactile_values['RF'] = tactile_state.tactiles[2].pdc
        tactile_values['LF'] = tactile_state.tactiles[3].pdc
        tactile_values['TH'] = tactile_state.tactiles[4].pdc

    elif tactile_type == "PST":
        tactile_values['FF'] = tactile_state.pressure[0]
        tactile_values['MF'] = tactile_state.pressure[1]
        tactile_values['RF'] = tactile_state.pressure[2]
        tactile_values['LF'] = tactile_state.pressure[3]
        tactile_values['TH'] = tactile_state.pressure[4]

    return tactile_values


def get_input():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_tactiles(hand_commander, joint_states_config):
    # Read tactile type
    tactile_type = hand_commander.get_tactile_type()
    # Zero values in dictionary for tactile sensors (initialized at 0)
    force_zero = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
    # Initialize values for current tactile values
    tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}

    if tactile_type is None:
        rospy.loginfo("You don't have tactile sensors. " +
                      "Talk to your Shadow representative to purchase some " +
                      "or use the keyboard to access this demo.")
    else:
        # Zero tactile sensors
        force_zero = zero_tactile_sensors(hand_commander, joint_states_config)

    return force_zero, tactile_values, tactile_type

def confirm_touched(tactile_values, force_zero):
    touched = None
    for finger in ["FF", "MF", "RF", "LF", "TH"]:
        if tactile_values[finger] > force_zero[finger]:
            touched = finger
            rospy.loginfo("{} contact".format(touched))
    return touched

if __name__ == "__main__":

    rospy.init_node("right_hand_demo", anonymous=True)

    parser = argparse.ArgumentParser(description="Hand side")
    parser.add_argument("-s", "--side",
                        dest="side",
                        type=str,
                        required=False,
                        help="Please select hand side, can be 'right', 'left' or 'both'.",
                        default=None,
                        choices=["right", "left", "both"])

    args = parser.parse_args(rospy.myargv()[1:])

    # Search for gazebo to confirm if in simulation or not
    sim = rospy.search_param('gazebo')

    if args.side is None:
        rospy.loginfo("Hand side not specified, defaulting to first hand avalliable.")
        if sim is None:
            hand_finder = HandFinder()
            joint_prefix = hand_finder.get_hand_parameters().joint_prefix['1']
        else:
            # Default parameter for simulated hand
            joint_prefix = rospy.get_param('/hand/joint_prefix/0')
    else:
        if args.side == 'right':
            joint_prefix = 'rh_'
        elif args.side == 'left':
            joint_prefix = 'lh_'
        else:
            joint_prefix = 'both'

    if 'rh_' == joint_prefix:
        hand_name = 'right_hand'
    elif 'lh_' == joint_prefix:
        hand_name = 'left_hand'
    else:
        hand_name = 'two_hands'

    hand_commander = SrHandCommander(name=hand_name)

    # Get joint states for demo from yaml
    joint_states_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                                   'sr_interface/sr_demos/config/demo_joint_states.yaml'
    with open(joint_states_config_filename) as f:
        joint_states_config = yaml.load(f, Loader=yaml.FullLoader)

    # Add prefix to joint states
    demo_states = {}
    for joint_state_dicts_no_id in joint_states_config.keys():
        for key, value in joint_states_config[joint_state_dicts_no_id].items():
            joints_target = {}
            if joint_prefix == 'both':
                joints_target['rh_' + key] = value
                joints_target['lh_' + key] = value
            else:
                joints_target[joint_prefix + key] = value
            demo_states[joint_state_dicts_no_id] = joints_target


    if joint_prefix == 'both':
        hand_commander_right = SrHandCommander(name='right_hand')
        hand_commander_left = SrHandCommander(name='left_hand')
        force_zero_right, tactile_values_right, tactile_type_right = get_tactiles(hand_commander_right, joint_states_config)
        force_zero_left, tactile_values_left, tactile_type_left = get_tactiles(hand_commander_left, joint_states_config)
    else:
        force_zero, tactile_values, tactile_type = get_tactiles(hand_commander, joint_states_config)

    rospy.loginfo("\nPRESS ONE OF THE TACTILES or 1-5 ON THE KEYBOARD TO START A DEMO:\
                   \n   TH or 1: Open Hand\
                   \n   FF or 2: Standard Demo\
                   \n   MF or 3: Shy Hand Demo\
                   \n   RF or 4: Card Trick Demo\
                   \n   LF or 5: Grasp Demo\
                   \n   ESC to exit")

    while not rospy.is_shutdown():
        # Check the state of the tactile senors
        touched = None

        if tactile_type is not None:
            if joint_prefix == 'both':
                tactile_values_right = read_tactile_values(hand_commander_right, tactile_type_right)
                tactile_values_left = read_tactile_values(hand_commander_left, tactile_type_left)
                touched_right = confirm_touched(tactile_values_right, force_zero_right)
                touched_left = confirm_touched(tactile_values_left, force_zero_left)
                if touched_right is not None:
                    touched = touched_right
                    force_zero = force_zero_right
                elif touched_left is not None:
                    touched = touched_left
                    force_zero = force_zero_left
                elif touched_right is not None and touched_left is not None:
                    rospy.loginfo("You touched fingers on both hands at the same time. Defaulting to right touch")
                    touched = touched_right
                    force_zero = force_zero_right
            else:
                tactile_values = read_tactile_values(hand_commander, tactile_type)
                touched = confirm_touched(tactile_values, force_zero)

        # Get input from the keyboard
        input_val = get_input()

        # If the tactile is touched, trigger the corresponding function
        if touched == "TH" or "1" == input_val:
            sequence_th(hand_commander, demo_states)
        elif touched == "FF" or "2" == input_val:
            sequence_ff(hand_commander, demo_states)
        elif touched == "MF" or "3" == input_val:
            sequence_mf(hand_commander, demo_states, 4.0, sim, force_zero)
        elif touched == "RF" or "4" == input_val:
            sequence_rf(hand_commander, demo_states)
        elif touched == "LF" or "5" == input_val:
            sequence_lf(hand_commander, demo_states, sim, force_zero)

        rospy.loginfo("Demo completed")

        if tactile_type is not None:
            force_zero = zero_tactile_sensors(hand_commander, demo_states)

        CONST_ESC_KEY_HEX_VALUE = '0x1b'
        if CONST_ESC_KEY_HEX_VALUE == hex(ord(input_val)):
            sys.exit(0)
