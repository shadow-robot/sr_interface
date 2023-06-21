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

# Reading the tactiles from the hand.


from threading import Thread
import termios
import sys
import tty
import yaml
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


class GraspExecution:
    def __init__(self):
        self.keyboard_pressed = False
        self.hand_commander = SrHandCommander(name='right_hand')
        self.grasp_yaml = {}

    def _open_yaml(self):
        grasp_config_filename = '/home/user/projects/shadow_robot/base/src/'\
                                'sr_interface/sr_example/config/demo_grasps.yaml'
        with open(grasp_config_filename, encoding="utf-8") as grasp_config_file:
            self.grasp_yaml = yaml.load(grasp_config_file, Loader=yaml.FullLoader)

    @staticmethod
    def _get_input():
        file_description = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_description)
        try:
            tty.setraw(sys.stdin.fileno())
            char_read = sys.stdin.read(1)
        finally:
            termios.tcsetattr(file_description, termios.TCSADRAIN, old_settings)
        return char_read

    def run(self):
        self._open_yaml()

        while True:
            input_val = self._get_input()
            if input_val == "1":
                self.execute_grasp("open_hand")
            elif input_val == "2":
                self.execute_grasp("pre_grasp")
            elif input_val == "3":
                self.execute_grasp("grasp_soft")
            elif input_val == "4":
                self.execute_grasp("grasp_hard")

            if hex(ord(input_val)) == '0x1b':
                sys.exit(0)

    def execute_grasp(self, grasp):
        rospy.loginfo(f"Grasp {grasp} started.")
        grasp_dict = dict(zip(self.grasp_yaml['joint_names'], self.grasp_yaml['grasps'][grasp]))
        self.hand_commander.move_to_joint_value_target_unsafe(grasp_dict, 5.0, True)
        rospy.sleep(2.0)
        rospy.loginfo(f"Grasp {grasp} completed.")


if __name__ == "__main__":

    rospy.init_node("right_hand_demo", anonymous=True)

    # Keyboard thread for input
    kpd = GraspExecution()
    keyboard_thread = Thread(target=kpd.run)
    keyboard_thread.start()

    rospy.loginfo("\nCUP GRASP DEMO.\
                \n PLEASE USING THE FOLLOWING TO PRE-GRASP THEN PLACE CUP IN HAND AND GRASP:\
                \n   1: Open Hand\
                \n   2: Pre-Grasp\
                \n   3: Grasp Soft\
                \n   4: Grasp Hard")
