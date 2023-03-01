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

import random
import time
from builtins import input
import termios
import tty
import sys
import argparse
from threading import Thread
from math import degrees
import yaml
import rospy
from sr_hand.tactile_receiver import TactileReceiver
from sr_robot_commander.sr_hand_commander import SrHandCommander

SAMPLES_TO_COLLECT = 50
TOUCH_THRESHOLD = 75
CONST_TIME_TO_COMPLETE_DEMO = 15


class TactileReading:
    def __init__(self, prefix):
        self.prefix = prefix

        # Read tactile type
        self.tactile_receiver = TactileReceiver(prefix)
        self.tactile_type = self.tactile_receiver.get_tactile_type()

        if self.get_tactiles() is None:
            rospy.loginfo("You don't have tactile sensors. " +
                          "Talk to your Shadow representative to purchase some " +
                          "or use the keyboard to access this demo.")

        # Initialize values for current tactile values
        self.tactile_values = {"FF": 0, "MF": 0, "RF": 0, "LF": 0, "TH": 0}
        # Zero values in dictionary for tactile sensors (initialized at 0)
        self.reference_tactile_values = dict(self.tactile_values)
        self.zero_tactile_sensors()

    def zero_tactile_sensors(self):
        if self.get_tactiles() is not None:
            rospy.logwarn('\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED')
            input('\nPress ENTER to continue...')

            # Collect SAMPLES_TO_COLLECT next samples and average them to filter out possible noise
            accumulator = []
            for _ in range(SAMPLES_TO_COLLECT):
                self.read_tactile_values()
                accumulator.append(self.tactile_values)

            for key in ["FF", "MF", "RF", "LF", "TH"]:
                self.reference_tactile_values[key] = sum(entry[key] for entry in accumulator) / len(accumulator)

            rospy.loginfo('Reference values: ' + str(self.reference_tactile_values))

    def read_tactile_values(self):
        if self.get_tactiles() is not None:
            # Read current state of tactile sensors
            tactile_state = self.tactile_receiver.get_tactile_state()

            if self.tactile_type == "biotac":
                self.tactile_values['FF'] = tactile_state.tactiles[0].pdc
                self.tactile_values['MF'] = tactile_state.tactiles[1].pdc
                self.tactile_values['RF'] = tactile_state.tactiles[2].pdc
                self.tactile_values['LF'] = tactile_state.tactiles[3].pdc
                self.tactile_values['TH'] = tactile_state.tactiles[4].pdc
            elif self.tactile_type == "PST":
                self.tactile_values['FF'] = tactile_state.pressure[0]
                self.tactile_values['MF'] = tactile_state.pressure[1]
                self.tactile_values['RF'] = tactile_state.pressure[2]
                self.tactile_values['LF'] = tactile_state.pressure[3]
                self.tactile_values['TH'] = tactile_state.pressure[4]

    def get_tactiles(self):
        return self.tactile_type

    def confirm_touched(self):
        touched = None
        if self.get_tactiles() is not None:
            self.read_tactile_values()
            for finger in ["FF", "MF", "RF", "LF", "TH"]:
                if self.tactile_values[finger] > self.reference_tactile_values[finger] + TOUCH_THRESHOLD:
                    touched = finger
                    rospy.loginfo(f"{touched} contact")
        return touched


class KeyboardPressDetector:
    def __init__(self, robot_type):
        self.keyboard_pressed = False
        self.robot = robot_type

    @staticmethod
    def _get_input():
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)
        try:
            tty.setraw(sys.stdin.fileno())
            first_char = sys.stdin.read(1)
        finally:
            termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)
        return first_char

    def run(self):
        while not rospy.is_shutdown():
            input_val = self._get_input()
            if input_val == "1":
                robot.sequence_th()
            elif input_val == "2":
                robot.sequence_ff()
            elif input_val == "3":
                robot.sequence_mf()
            elif input_val == "4":
                robot.sequence_rf()
            elif input_val == "5":
                if robot.hand_type == 'hand_e':
                    robot.sequence_lf()
                else:
                    rospy.logerr("This demo only works for a 5-fingered Hand E. Please try demos 1-4")
            elif input_val == "6":
                rospy.signal_shutdown("Ending demo as key 6 has been pressed.")
                sys.exit(0)
            rospy.sleep(0.05)


class Robot:
    '''
        This class is used to store a robot object
    '''
    def __init__(self, robot_type, hand_type):
        self.robot = robot_type
        self.hand_type = hand_type
        self.commander = SrHandCommander(name=robot)
        if robot == "two_hands":
            self.prefixes = ["rh_", "lh_"]
        else:
            self.prefixes = [f"{robot_type[0]}h_"]
        self.tactiles = self._has_tactiles()

        # Get joint states for demo from yaml
        joint_states_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                                       'sr_interface/sr_demos/config/demo_joint_states.yaml'
        with open(joint_states_config_filename, encoding="utf-8") as joint_state_file:
            joint_states_config_yaml = yaml.load(joint_state_file, Loader=yaml.FullLoader)

        self.demo_joint_states = self._get_joint_states_for_robot(joint_states_config_yaml)
        self.execute_command_check('start_pos', 0.0, 1.0)

    def _has_tactiles(self):
        """
            TactileReading is going to search for any available sensors (Biotact, PST, etc)
        """
        tactiles = []
        if len(self.prefixes) == 2:
            tactile_right = TactileReading('rh_')
            tactile_left = TactileReading('lh_')
            if tactile_left.get_tactiles() is not None and tactile_right.get_tactiles() is not None:
                tactiles.extend([tactile_right, tactile_left])
        else:
            tactile = TactileReading(self.prefixes[0])
            if tactile.get_tactiles() is not None:
                tactiles.append(tactile)
        return tactiles

    def _get_joint_states_for_robot(self, joint_states_config_yaml):
        joint_states_config = self._correct_joint_states_for_hand_type(joint_states_config_yaml)
        # Add prefix to joint states
        joint_states = self._add_prefix_to_joint_states(joint_states_config)
        return joint_states

    def _correct_joint_states_for_hand_type(self, joint_states_config):
        hand_type_joints_filename = '/home/user/projects/shadow_robot/base/src/'\
                        'sr_interface/sr_demos/config/joints_in_hand.yaml'
        with open(hand_type_joints_filename, encoding="utf-8") as hand_type_joints_file:
            hand_type_joints = yaml.load(hand_type_joints_file, Loader=yaml.FullLoader)

        for joint_state_dicts_no_id in joint_states_config.keys():
            for key in list(joint_states_config[joint_state_dicts_no_id]):
                if key not in hand_type_joints[self.hand_type]:
                    joint_states_config[joint_state_dicts_no_id].pop(key)
        return joint_states_config

    def _add_prefix_to_joint_states(self, joint_states_config):
        demo_states = {}
        for joint_state_dicts_no_id in joint_states_config.keys():
            joints_target = {}
            for key, value in joint_states_config[joint_state_dicts_no_id].items():
                if len(self.prefixes) == 2:
                    joints_target['rh_' + key] = value
                    joints_target['lh_' + key] = value
                else:
                    joints_target[self.prefixes[0] + key] = value
                demo_states[joint_state_dicts_no_id] = joints_target
        return demo_states

    def execute_command_check(self, joint_states, sleep, time_to_execute,
                              wait=True, angle_degrees=True):
        if joint_states in self.demo_joint_states.keys():
            self.commander.move_to_joint_value_target_unsafe(self.demo_joint_states[joint_states],
                                                             time_to_execute, wait,
                                                             angle_degrees)
            rospy.sleep(sleep)

    def check_touched_finger(self):
        """
            Check if any tactile sensors are being touched
        """
        touched_finger = None
        if len(self.tactiles) == 2:
            # confirm_touched() will return None if no sensors are found
            touched_right = robot.tactiles[0].confirm_touched()
            touched_left = robot.tactiles[1].confirm_touched()
            if touched_right is not None and touched_left is not None:
                rospy.loginfo("You touched fingers on both hands at the same time. Defaulting to right touch")
                touched_finger = touched_right
            elif touched_right is not None:
                touched_finger = touched_right
            elif touched_left is not None:
                touched_finger = touched_left
        # check if tactile sensors have been previously found
        elif len(robot.tactiles) == 1:  # Unimanual mode
            touched_finger = robot.tactiles[0].confirm_touched()
        return touched_finger

    def sequence_th(self):
        rospy.loginfo("TH demo started")

        rospy.sleep(0.5)
        self.execute_command_check('start_pos', 1.5, 1.5)

        rospy.loginfo("TH demo completed")

    def sequence_ff(self):
        rospy.loginfo("FF demo started")

        rospy.sleep(1)
        self.sequence_ff_commands()

        rospy.loginfo("FF demo completed")

    def sequence_ff_commands(self):
        self.execute_command_check('store_3', 1.1, 1.1)
        self.execute_command_check('start_pos', 1.1, 1.1)
        self.execute_command_check('flex_ff', 1.1, 1.0)
        self.execute_command_check('ext_ff', 1.1, 1.0)
        self.execute_command_check('flex_mf', 1.1, 1.0)
        self.execute_command_check('ext_mf', 1.1, 1.0)
        self.execute_command_check('flex_rf', 1.1, 1.0)
        self.execute_command_check('ext_rf', 1.1, 1.0)
        self.execute_command_check('flex_lf', 1.1, 1.0)
        self.execute_command_check('ext_lf', 1.1, 1.0)
        self.execute_command_check('flex_th_1', 1, 0.7)
        self.execute_command_check('flex_th_2', 1, 0.7)
        self.execute_command_check('ext_th_1', 1.5, 1.5)
        self.execute_command_check('ext_th_2', 0.5, 0.5)
        self.execute_command_check('l_ext_lf', 0.5, 0.5)
        self.execute_command_check('l_ext_rf', 0.5, 0.5)
        self.execute_command_check('l_ext_mf', 0.5, 0.5)
        self.execute_command_check('l_ext_ff', 0.5, 0.5)
        self.execute_command_check('l_int_all', 0.5, 0.5)
        self.execute_command_check('l_ext_all', 0.5, 0.5)
        self.execute_command_check('l_int_ff', 0.5, 0.5)
        self.execute_command_check('l_int_mf', 0.5, 0.5)
        self.execute_command_check('l_int_rf', 0.5, 0.5)
        self.execute_command_check('l_int_lf', 0.5, 0.5)
        self.execute_command_check('l_zero_all', 0.5, 0.5)
        self.execute_command_check('l_spock', 0.5, 0.5)
        self.execute_command_check('l_zero_all', 0.5, 0.5)
        self.execute_command_check('pre_ff_ok', 1.0, 1.0)
        self.execute_command_check('ff_ok', 0.9, 0.7)
        self.execute_command_check('ff2mf_ok', 0.4, 0.5)
        self.execute_command_check('mf_ok', 0.9, 0.7)
        self.execute_command_check('mf2rf_ok', 0.4, 0.5)
        self.execute_command_check('rf_ok', 0.9, 0.7)
        self.execute_command_check('rf2lf_ok', 0.4, 0.5)
        self.execute_command_check('lf_ok', 0.9, 0.7)
        self.execute_command_check('start_pos', 1.0, 1.0)
        self.execute_command_check('flex_ff', 0.2, 0.2)
        self.execute_command_check('flex_mf', 0.2, 0.2)
        self.execute_command_check('flex_rf', 0.2, 0.2)
        self.execute_command_check('flex_lf', 0.2, 0.2)
        self.execute_command_check('ext_ff', 0.2, 0.2)
        self.execute_command_check('ext_mf', 0.2, 0.2)
        self.execute_command_check('ext_rf', 0.2, 0.2)
        self.execute_command_check('ext_lf', 0.2, 0.2)
        self.execute_command_check('flex_ff', 0.2, 0.2)
        self.execute_command_check('flex_mf', 0.2, 0.2)
        self.execute_command_check('flex_rf', 0.2, 0.2)
        self.execute_command_check('flex_lf', 0.2, 0.2)
        self.execute_command_check('ext_ff', 0.2, 0.2)
        self.execute_command_check('ext_mf', 0.2, 0.2)
        self.execute_command_check('ext_rf', 0.2, 0.2)
        self.execute_command_check('ext_lf', 0.2, 0.2)
        self.execute_command_check('flex_ff', 0.2, 0.2)
        self.execute_command_check('flex_mf', 0.2, 0.2)
        self.execute_command_check('flex_rf', 0.2, 0.2)
        self.execute_command_check('flex_lf', 0.2, 0.2)
        self.execute_command_check('ext_ff', 0.2, 0.2)
        self.execute_command_check('ext_mf', 0.2, 0.2)
        self.execute_command_check('ext_rf', 0.2, 0.2)
        self.execute_command_check('ext_lf', 0.2, 0.2)
        self.execute_command_check('pre_ff_ok', 1.0, 1.0)
        self.execute_command_check('ff_ok', 3.3, 1.3)
        if self.hand_type == 'hand_e':
            self.execute_command_check('ne_wr', 1.1, 1.1)
            self.execute_command_check('nw_wr', 1.1, 1.1)
            self.execute_command_check('sw_wr', 1.1, 1.1)
            self.execute_command_check('se_wr', 1.1, 1.1)
            self.execute_command_check('ne_wr', 0.7, 0.7)
            self.execute_command_check('nw_wr', 0.7, 0.7)
            self.execute_command_check('sw_wr', 0.7, 0.7)
            self.execute_command_check('se_wr', 0.7, 0.7)
            self.execute_command_check('zero_wr', 0.4, 0.4)
        self.execute_command_check('start_pos', 1.5, 1.5)

    def sequence_mf(self):
        rospy.loginfo("MF demo started")
        rospy.sleep(0.5)
        self.execute_command_check('start_pos', 1.0, 1.0)
        self.execute_command_check('bc_pre_zero', 2.0, 2.0)
        self.execute_command_check('bc_zero', 4.0, 1.0)
        self.execute_command_check('bc_1', 1.0, 1.0)
        self.execute_command_check('bc_2', 1.0, 1.0)
        self.execute_command_check('bc_3', 1.0, 1.0)
        self.execute_command_check('bc_4', 1.0, 1.0)
        self.execute_command_check('bc_5', 1.0, 1.0)
        self.execute_command_check('bc_6', 1.0, 1.0)
        self.execute_command_check('bc_7', 1.0, 1.0)
        self.execute_command_check('bc_8', 1.0, 1.0)
        self.execute_command_check('bc_9', 1.0, 1.0)
        self.execute_command_check('bc_11', 1.0, 1.0)
        self.execute_command_check('bc_12', 4.0, 3.0)
        self.execute_command_check('start_pos', 1.5, 1.5)
        rospy.loginfo("MF demo completed")

    def sequence_rf(self):
        rospy.loginfo("RF demo started")

        # Move Hand to zero position
        self.execute_command_check('start_pos', 2.0, 2.0)

        # Move Hand to starting position
        self.execute_command_check('pregrasp_pos', 2.0, 2.0)

        # Move Hand to close position
        self.execute_command_check('grasp_pos', 0.0, 11.0)

        # Send all joints to current position to compensate
        # for minor offsets created in the previous loop
        hand_pos = {joint: degrees(i) for joint, i in robot.commander.get_joints_position().items()}
        robot.commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, wait=False, angle_degrees=True)
        rospy.sleep(2.0)

        # Generate new values to squeeze object slightly
        offset = 5
        squeeze = hand_pos.copy()
        for prefix in self.prefixes:
            squeeze.update({f"{prefix}THJ5": hand_pos[f'{prefix}THJ5'] + offset,
                            f"{prefix}THJ2": hand_pos[f'{prefix}THJ2'] + offset,
                            f"{prefix}FFJ3": hand_pos[f'{prefix}FFJ3'] + offset,
                            f"{prefix}FFJ1": hand_pos[f'{prefix}FFJ1'] + offset,
                            f"{prefix}RFJ3": hand_pos[f'{prefix}RFJ3'] + offset,
                            f"{prefix}RFJ1": hand_pos[f'{prefix}RFJ1'] + offset})
        if robot.hand_type == 'hand_lite' or robot.hand_type == 'hand_e':
            for prefix in self.prefixes:
                squeeze.update({f"{prefix}MFJ3": hand_pos[f'{prefix}MFJ3'] + offset,
                                f"{prefix}MFJ1": hand_pos[f'{prefix}MFJ1'] + offset})
        if robot.hand_type == 'hand_e':
            for prefix in self.prefixes:
                squeeze.update({f"{prefix}LFJ3": hand_pos[f'{prefix}LFJ3'] + offset,
                                f"{prefix}LFJ1": hand_pos[f'{prefix}LFJ1'] + offset})

        # Squeeze object gently
        self.commander.move_to_joint_value_target_unsafe(squeeze, 0.5, wait=False, angle_degrees=True)
        rospy.sleep(0.5)
        self.commander.move_to_joint_value_target_unsafe(hand_pos, 0.5, wait=False, angle_degrees=True)
        rospy.sleep(0.5)
        self.commander.move_to_joint_value_target_unsafe(squeeze, 0.5, wait=False, angle_degrees=True)
        rospy.sleep(0.5)
        self.commander.move_to_joint_value_target_unsafe(hand_pos, 2.0, wait=False, angle_degrees=True)
        rospy.sleep(2.0)
        self.execute_command_check('pregrasp_pos', 2.0, 2.0)
        self.execute_command_check('start_pos', 2.0, 2.0)

        rospy.loginfo("RF demo completed")

    def sequence_lf(self):
        rospy.loginfo("LF demo started")

        rospy.sleep(0.5)
        # Initialize wake time
        wake_time = time.time()
        tactiles = bool(self.tactiles)
        while True:
            if tactiles:
                # Check if any of the tactile senors have been triggered
                # If so, send the Hand to its start position
                touched_finger = self.check_touched_finger()

                if touched_finger is not None:
                    self.execute_command_check('start_pos', 0.0, 2.0)
                    rospy.loginfo(f'{touched_finger} touched!')
                    rospy.sleep(2.0)
                    if touched_finger == "TH":
                        break

                # If the tactile sensors have not been triggered and the Hand
                # is not in the middle of a movement, generate a random position
                # and interpolation time
                else:
                    if time.time() < wake_time + CONST_TIME_TO_COMPLETE_DEMO:
                        self.complete_random_sequence()
                    else:
                        break
            else:
                if time.time() < wake_time + CONST_TIME_TO_COMPLETE_DEMO:
                    self.complete_random_sequence()
                else:
                    break

        self.execute_command_check('start_pos', 2.0, 2.0)

        rospy.loginfo("LF demo completed")

    def complete_random_sequence(self):
        prefix = "lh_"
        for joint_state in self.demo_joint_states['start_pos'].keys():
            if "rh_" in joint_state:
                prefix = "rh_"
                break

        for i in self.demo_joint_states['rand_pos']:
            self.demo_joint_states['rand_pos'][i] =\
                random.randrange(self.demo_joint_states['min_range'][i],
                                 self.demo_joint_states['max_range'][i])
        self.demo_joint_states['rand_pos'][f'{prefix}FFJ4'] =\
            random.randrange(self.demo_joint_states['min_range'][f'{prefix}FFJ4'],
                             self.demo_joint_states['rand_pos'][f'{prefix}MFJ4'])
        self.demo_joint_states['rand_pos'][f'{prefix}LFJ4'] =\
            random.randrange(self.demo_joint_states['min_range'][f'{prefix}LFJ4'],
                             self.demo_joint_states['rand_pos'][f'{prefix}RFJ4'])
        inter_time = 4.0 * random.random()
        self.execute_command_check('rand_pos', 0.2, inter_time)


if __name__ == "__main__":

    rospy.init_node("right_hand_demo", anonymous=True)

    parser = argparse.ArgumentParser(description="Hand side")
    parser.add_argument("-s", "--side",
                        dest="side",
                        type=str,
                        required=False,
                        help="Please select hand side, can be 'right', 'left' or 'both'.",
                        default=True,
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
        robot = Robot("right_hand", args.hand_type)
    elif args.side == 'left':
        robot = Robot("left_hand", args.hand_type)
    else:
        robot = Robot("two_hands", args.hand_type)

    rospy.loginfo("\nPRESS ONE OF THE TACTILES or 1-5 ON THE KEYBOARD TO START A DEMO:\
                   \nTH or 1: Open Hand\
                   \nFF or 2: Standard Demo\
                   \nMF or 3: Card Trick Demo\
                   \nRF or 4: Grasp Demo\
                   \nLF or 5: Shy Hand Demo (only works with Hand E).\
                   \nPRESS 6 TO END THE PROGRAM")

    # Keyboard thread for input
    kpd = KeyboardPressDetector(robot)
    keyboard_thread = Thread(target=kpd.run)
    keyboard_thread.start()

    while not rospy.is_shutdown():
        # Check the state of the tactile sensors
        if robot.tactiles:
            finger_touched = robot.check_touched_finger()
            # If the tactile is touched, trigger the corresponding function
            if finger_touched == "TH":
                robot.sequence_th()
            elif finger_touched == "FF":
                robot.sequence_ff()
            elif finger_touched == "MF":
                robot.sequence_mf()
            elif finger_touched == "RF":
                robot.sequence_rf()
            elif finger_touched == "LF":
                robot.sequence_lf()

        rospy.sleep(0.1)
